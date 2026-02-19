// ============================================================================
// Boreal SoC - AI Mailbox
// ============================================================================
// Dual-port mailbox for passing inference results from the AI accelerator
// (or external host) to the Decision-VM.  Two independent 64-byte slots
// with valid/ready handshake.
// ============================================================================

module boreal_ai_mailbox (
    input  wire        clk,
    input  wire        rst_n,

    // --- MMIO slave interface (AI-side write, VM-side read) ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- VM read port (direct, low-latency) ---
    input  wire [ 3:0] vm_rd_idx,     // word index within slot (0-15)
    input  wire        vm_rd_slot,     // slot select (0 or 1)
    output wire [31:0] vm_rd_data,
    output wire        vm_slot0_valid,
    output wire        vm_slot1_valid,
    input  wire        vm_slot0_ack,   // VM acknowledges slot 0
    input  wire        vm_slot1_ack    // VM acknowledges slot 1
);

    // Two slots, 16 words each (64 bytes = 512 bits per slot)
    reg [31:0] slot0 [0:15];
    reg [31:0] slot1 [0:15];
    reg        valid0, valid1;

    // MMIO address decode
    // 0x00       = SLOT0_VALID (R/W: write 1 to mark valid, read status)
    // 0x04       = SLOT1_VALID
    // 0x40-0x7C  = SLOT0 data words (16 x 32-bit)
    // 0x80-0xBC  = SLOT1 data words (16 x 32-bit)
    wire [7:0] reg_off = addr[7:0];

    wire slot0_data_wr = sel && wr && (reg_off >= 8'h40) && (reg_off < 8'h80);
    wire slot1_data_wr = sel && wr && (reg_off >= 8'h80) && (reg_off < 8'hC0);

    wire [3:0] slot0_word_idx = reg_off[5:2] - 4'd0;  // offset from 0x40
    wire [3:0] slot1_word_idx = reg_off[5:2] - 4'd0;  // offset from 0x80

    integer i;

    // -----------------------------------------------------------------------
    // Write logic
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid0 <= 1'b0;
            valid1 <= 1'b0;
            for (i = 0; i < 16; i = i + 1) begin
                slot0[i] <= 32'h0;
                slot1[i] <= 32'h0;
            end
        end else begin
            // VM acknowledgement clears valid flags
            if (vm_slot0_ack) valid0 <= 1'b0;
            if (vm_slot1_ack) valid1 <= 1'b0;

            if (sel && wr) begin
                case (reg_off)
                    8'h00: valid0 <= wdata[0];
                    8'h04: valid1 <= wdata[0];
                    default: ;
                endcase

                if (slot0_data_wr)
                    slot0[addr[5:2]] <= wdata;

                if (slot1_data_wr)
                    slot1[addr[5:2]] <= wdata;
            end
        end
    end

    // -----------------------------------------------------------------------
    // MMIO read
    // -----------------------------------------------------------------------
    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr) begin
            case (reg_off)
                8'h00:   rdata = {31'b0, valid0};
                8'h04:   rdata = {31'b0, valid1};
                default: begin
                    if (reg_off >= 8'h40 && reg_off < 8'h80)
                        rdata = slot0[addr[5:2]];
                    else if (reg_off >= 8'h80 && reg_off < 8'hC0)
                        rdata = slot1[addr[5:2]];
                end
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // VM direct-read port
    // -----------------------------------------------------------------------
    assign vm_rd_data    = vm_rd_slot ? slot1[vm_rd_idx] : slot0[vm_rd_idx];
    assign vm_slot0_valid = valid0;
    assign vm_slot1_valid = valid1;

endmodule
