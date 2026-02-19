// ============================================================================
// Boreal SoC - Decision VM
// ============================================================================
// Deterministic micro-VM that reads AI mailbox inputs, executes a small
// program, and produces action requests for the Gate.  Fixed instruction
// set with cycle-budget enforcement ensures deterministic execution.
//
// Instruction format (32-bit):
//   [31:28] opcode  [27:24] dst_reg  [23:20] src_a  [19:16] src_b  [15:0] imm16
//
// Opcodes:
//   0 = NOP          4 = SUB       8 = SHL       C = BEQ (branch if equal)
//   1 = LOAD_IMM     5 = AND       9 = SHR       D = BNE
//   2 = LOAD_MB      6 = OR        A = CMP       E = EMIT (action request)
//   3 = ADD          7 = XOR       B = JMP       F = HALT
// ============================================================================
`include "boreal_pkg.v"

module boreal_decision_vm #(
    parameter PMEM_DEPTH     = 256,
    parameter PMEM_DEPTH_LOG = 8,
    parameter CYCLE_BUDGET   = 1024
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- MMIO slave interface ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- Mailbox read port ---
    output reg  [ 3:0] mb_rd_idx,
    output reg         mb_rd_slot,
    input  wire [31:0] mb_rd_data,
    input  wire        mb_slot0_valid,
    input  wire        mb_slot1_valid,
    output reg         mb_slot0_ack,
    output reg         mb_slot1_ack,

    // --- Action request output (to Gate) ---
    output reg         act_valid,
    output reg  [31:0] act_opcode,
    output reg  [31:0] act_target,
    output reg  [31:0] act_arg0,
    output reg  [31:0] act_arg1,
    output reg  [31:0] act_context_hash,
    output reg  [31:0] act_policy_hash,
    output reg  [31:0] act_bounds,
    output reg  [31:0] act_nonce,
    input  wire        act_ready
);

    // -----------------------------------------------------------------------
    // MMIO register map (base 0x1003_0000)
    // -----------------------------------------------------------------------
    // 0x00 = VM_CTRL (W: bit0=start, bit1=reset)
    // 0x04 = VM_STATUS (R: bit0=running, bit1=halted, bit2=budget_exceeded)
    // 0x08 = VM_PC (R: current PC)
    // 0x0C = VM_BUDGET (R/W: cycle budget)
    // 0x100-0x4FC = Program memory (256 x 32-bit instructions)
    localparam OFF_CTRL   = 8'h00;
    localparam OFF_STATUS = 8'h04;
    localparam OFF_PC     = 8'h08;
    localparam OFF_BUDGET = 8'h0C;

    wire [11:0] reg_off = addr[11:0];
    wire        pmem_access = (reg_off >= 12'h100) && (reg_off < 12'h500);

    // -----------------------------------------------------------------------
    // Program memory
    // -----------------------------------------------------------------------
    reg [31:0] pmem [0:PMEM_DEPTH-1];
    wire [PMEM_DEPTH_LOG-1:0] pmem_wr_idx = reg_off[PMEM_DEPTH_LOG+1:2] - 8'h40;

    // -----------------------------------------------------------------------
    // Register file (16 x 32-bit)
    // -----------------------------------------------------------------------
    reg [31:0] rf [0:15];

    // -----------------------------------------------------------------------
    // VM state
    // -----------------------------------------------------------------------
    reg [PMEM_DEPTH_LOG-1:0] pc;
    reg [31:0] cycle_count;
    reg [31:0] budget;
    reg        running;
    reg        halted;
    reg        budget_exceeded;

    // -----------------------------------------------------------------------
    // Instruction decode
    // -----------------------------------------------------------------------
    wire [31:0] instr    = pmem[pc];
    wire [ 3:0] i_op     = instr[31:28];
    wire [ 3:0] i_dst    = instr[27:24];
    wire [ 3:0] i_src_a  = instr[23:20];
    wire [ 3:0] i_src_b  = instr[19:16];
    wire [15:0] i_imm    = instr[15: 0];

    wire [31:0] va = rf[i_src_a];
    wire [31:0] vb = rf[i_src_b];

    // -----------------------------------------------------------------------
    // MMIO write
    // -----------------------------------------------------------------------
    wire start_pulse;
    wire reset_pulse;

    reg mmio_start, mmio_reset;
    assign start_pulse = mmio_start;
    assign reset_pulse = mmio_reset;

    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            budget     <= CYCLE_BUDGET;
            mmio_start <= 1'b0;
            mmio_reset <= 1'b0;
        end else begin
            mmio_start <= 1'b0;
            mmio_reset <= 1'b0;
            if (sel && wr) begin
                if (pmem_access) begin
                    pmem[pmem_wr_idx] <= wdata;
                end else begin
                    if (reg_off[7:0] == OFF_CTRL) begin
                        mmio_start <= wdata[0];
                        mmio_reset <= wdata[1];
                    end
                    if (reg_off[7:0] == OFF_BUDGET)
                        budget <= wdata;
                end
            end
        end
    end

    // MMIO read
    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr) begin
            case (reg_off[7:0])
                OFF_STATUS: rdata = {29'b0, budget_exceeded, halted, running};
                OFF_PC:     rdata = {{(32-PMEM_DEPTH_LOG){1'b0}}, pc};
                OFF_BUDGET: rdata = budget;
                default: begin
                    if (pmem_access)
                        rdata = pmem[pmem_wr_idx];
                end
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Execution engine
    // -----------------------------------------------------------------------
    // EMIT state machine
    reg        emit_pending;
    reg [1:0]  emit_phase;   // 0=assert valid, 1=wait accept, 2=wait done
    reg [31:0] emit_opcode_r, emit_target_r, emit_arg0_r, emit_arg1_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc              <= {PMEM_DEPTH_LOG{1'b0}};
            cycle_count     <= 32'd0;
            running         <= 1'b0;
            halted          <= 1'b0;
            budget_exceeded <= 1'b0;
            act_valid       <= 1'b0;
            emit_pending    <= 1'b0;
            emit_phase      <= 2'd0;
            mb_rd_idx       <= 4'd0;
            mb_rd_slot      <= 1'b0;
            mb_slot0_ack    <= 1'b0;
            mb_slot1_ack    <= 1'b0;
            act_opcode      <= 32'd0;
            act_target      <= 32'd0;
            act_arg0        <= 32'd0;
            act_arg1        <= 32'd0;
            act_context_hash<= 32'd0;
            act_policy_hash <= 32'd0;
            act_bounds      <= 32'd0;
            act_nonce       <= 32'd0;
            emit_opcode_r   <= 32'd0;
            emit_target_r   <= 32'd0;
            emit_arg0_r     <= 32'd0;
            emit_arg1_r     <= 32'd0;
            for (j = 0; j < 16; j = j + 1)
                rf[j] <= 32'd0;
        end else begin
            mb_slot0_ack <= 1'b0;
            mb_slot1_ack <= 1'b0;

            if (reset_pulse) begin
                pc              <= {PMEM_DEPTH_LOG{1'b0}};
                cycle_count     <= 32'd0;
                running         <= 1'b0;
                halted          <= 1'b0;
                budget_exceeded <= 1'b0;
                act_valid       <= 1'b0;
                emit_pending    <= 1'b0;
                emit_phase      <= 2'd0;
                for (j = 0; j < 16; j = j + 1)
                    rf[j] <= 32'd0;
            end else if (start_pulse && !running) begin
                running     <= 1'b1;
                halted      <= 1'b0;
                cycle_count <= 32'd0;
            end else if (running) begin
                // Budget enforcement
                if (cycle_count >= budget) begin
                    running         <= 1'b0;
                    halted          <= 1'b1;
                    budget_exceeded <= 1'b1;
                end
                // Handle pending EMIT – two-phase handshake with Gate
                else if (emit_pending) begin
                    cycle_count <= cycle_count + 1;
                    case (emit_phase)
                        2'd0: begin
                            // Phase 0: Assert valid and drive action fields
                            act_valid  <= 1'b1;
                            act_opcode <= emit_opcode_r;
                            act_target <= emit_target_r;
                            act_arg0   <= emit_arg0_r;
                            act_arg1   <= emit_arg1_r;
                            emit_phase <= 2'd1;
                        end
                        2'd1: begin
                            // Phase 1: Wait for Gate to accept (act_ready goes low)
                            if (!act_ready) begin
                                act_valid  <= 1'b0;
                                emit_phase <= 2'd2;
                            end
                        end
                        2'd2: begin
                            // Phase 2: Wait for Gate to finish (act_ready goes high)
                            if (act_ready) begin
                                emit_pending <= 1'b0;
                                emit_phase   <= 2'd0;
                            end
                        end
                        default: emit_phase <= 2'd0;
                    endcase
                end else begin
                    cycle_count <= cycle_count + 1;

                    case (i_op)
                        4'h0: begin // NOP
                            pc <= pc + 1;
                        end

                        4'h1: begin // LOAD_IMM: dst = sign-extend(imm16)
                            rf[i_dst] <= {{16{i_imm[15]}}, i_imm};
                            pc <= pc + 1;
                        end

                        4'h2: begin // LOAD_MB: dst = mailbox[src_a[3:0]], slot=src_b[0]
                            mb_rd_idx  <= va[3:0];
                            mb_rd_slot <= vb[0];
                            rf[i_dst]  <= mb_rd_data;
                            pc <= pc + 1;
                        end

                        4'h3: begin // ADD
                            rf[i_dst] <= va + vb;
                            pc <= pc + 1;
                        end

                        4'h4: begin // SUB
                            rf[i_dst] <= va - vb;
                            pc <= pc + 1;
                        end

                        4'h5: begin // AND
                            rf[i_dst] <= va & vb;
                            pc <= pc + 1;
                        end

                        4'h6: begin // OR
                            rf[i_dst] <= va | vb;
                            pc <= pc + 1;
                        end

                        4'h7: begin // XOR
                            rf[i_dst] <= va ^ vb;
                            pc <= pc + 1;
                        end

                        4'h8: begin // SHL
                            rf[i_dst] <= va << vb[4:0];
                            pc <= pc + 1;
                        end

                        4'h9: begin // SHR
                            rf[i_dst] <= va >> vb[4:0];
                            pc <= pc + 1;
                        end

                        4'hA: begin // CMP: dst = (va == vb) ? 1 : 0
                            rf[i_dst] <= (va == vb) ? 32'd1 : 32'd0;
                            pc <= pc + 1;
                        end

                        4'hB: begin // JMP: pc = imm16
                            pc <= i_imm[PMEM_DEPTH_LOG-1:0];
                        end

                        4'hC: begin // BEQ: if (va == vb) pc = imm16
                            pc <= (va == vb) ? i_imm[PMEM_DEPTH_LOG-1:0] : pc + 1;
                        end

                        4'hD: begin // BNE: if (va != vb) pc = imm16
                            pc <= (va != vb) ? i_imm[PMEM_DEPTH_LOG-1:0] : pc + 1;
                        end

                        4'hE: begin // EMIT: action request
                            // r0=opcode, r1=target, r2=arg0, r3=arg1
                            emit_opcode_r <= rf[0];
                            emit_target_r <= rf[1];
                            emit_arg0_r   <= rf[2];
                            emit_arg1_r   <= rf[3];
                            emit_pending  <= 1'b1;
                            pc <= pc + 1;
                        end

                        4'hF: begin // HALT
                            running <= 1'b0;
                            halted  <= 1'b1;
                        end

                        default: pc <= pc + 1;
                    endcase
                end
            end
        end
    end

endmodule
