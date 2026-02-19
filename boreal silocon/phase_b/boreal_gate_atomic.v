`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_gate_atomic.v  (Phase-B Gate with mailbox-based req/resp)
// Same policy logic as Phase-A gate, but reads from mailbox req words
// and writes response atomically back through mailbox interface.
// ============================================================================
`include "boreal_pkg.vh"

module boreal_gate_atomic (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] cycle_ctr,

    // Mailbox request interface (from mbox_block)
    input  wire        mb_req_valid,
    input  wire [31:0] mb_req_w0,   // opcode
    input  wire [31:0] mb_req_w1,   // target
    input  wire [31:0] mb_req_w2,   // arg0
    input  wire [31:0] mb_req_w3,   // arg1
    input  wire [31:0] mb_req_w4,   // context_hash
    input  wire [31:0] mb_req_w5,   // policy_hash
    input  wire [31:0] mb_req_w6,   // bounds
    input  wire [31:0] mb_req_w7,   // nonce
    output reg         mb_req_consume,

    // Mailbox response interface (to mbox_block)
    output reg         mb_resp_we,
    output reg  [2:0]  mb_resp_widx,
    output reg  [31:0] mb_resp_wdata,
    output reg         mb_resp_valid_set,

    // Commit intent (future: privileged IO bus)
    output reg         io_we,
    output reg  [31:0] io_addr,
    output reg  [31:0] io_wdata,

    // Ledger append
    output reg         led_wr,
    output reg [255:0] led_event,

    // MMIO policy regs
    input  wire        mmio_we,
    input  wire [7:0]  mmio_addr,
    input  wire [31:0] mmio_wdata,
    input  wire [3:0]  mmio_wstrb,
    output reg  [31:0] mmio_rdata
);
    // Policy state
    reg [63:0] allow_mask;
    reg [31:0] rate_limit;
    reg [31:0] rate_window;
    reg [31:0] policy_hash;
    reg [31:0] clamp_min [0:3];
    reg [31:0] clamp_max [0:3];
    reg [31:0] window_ctr;
    reg [31:0] commit_ctr;
    reg [31:0] nonce_expected;

    // Decode from mailbox request words
    wire [31:0] opcode       = mb_req_w0;
    wire [31:0] target       = mb_req_w1;
    wire [31:0] arg0         = mb_req_w2;
    wire [31:0] arg1         = mb_req_w3;
    wire [31:0] context_hash = mb_req_w4;
    wire [31:0] req_pol_hash = mb_req_w5;
    wire [31:0] bounds       = mb_req_w6;
    wire [31:0] nonce        = mb_req_w7;

    wire allowed   = allow_mask[target[5:0]];
    wire policy_ok = (req_pol_hash == policy_hash);
    wire rate_ok   = (commit_ctr < rate_limit);
    wire nonce_ok  = (nonce == nonce_expected);

    wire [1:0]  clamp_class = bounds[1:0];
    wire [31:0] cmin = clamp_min[clamp_class];
    wire [31:0] cmax = clamp_max[clamp_class];
    wire [31:0] clamped0    = (arg0 < cmin) ? cmin : (arg0 > cmax) ? cmax : arg0;
    wire        was_clamped = (clamped0 != arg0);

    reg        committed;
    reg [31:0] reason;

    // Response write sequencer state
    reg [2:0] resp_seq;
    reg [31:0] resp_w [0:4];

    integer j;

    always @(posedge clk) begin
        if (!rst_n) begin
            allow_mask     <= 64'h0;
            rate_limit     <= 32'd0;
            rate_window    <= 32'd1;
            policy_hash    <= 32'h0;
            window_ctr     <= 32'h0;
            commit_ctr     <= 32'h0;
            nonce_expected <= 32'h0;
            for (j = 0; j < 4; j = j + 1) begin
                clamp_min[j] <= 32'h0;
                clamp_max[j] <= 32'hFFFF_FFFF;
            end
            io_we      <= 1'b0;
            io_addr    <= 32'h0;
            io_wdata   <= 32'h0;
            led_wr     <= 1'b0;
            led_event  <= 256'h0;
            mmio_rdata <= 32'h0;
            mb_req_consume   <= 1'b0;
            mb_resp_we       <= 1'b0;
            mb_resp_widx     <= 3'd0;
            mb_resp_wdata    <= 32'h0;
            mb_resp_valid_set<= 1'b0;
            resp_seq <= 3'd0;
            for (j = 0; j < 5; j = j + 1) resp_w[j] <= 32'h0;
        end else begin
            // defaults
            io_we            <= 1'b0;
            led_wr           <= 1'b0;
            mb_req_consume   <= 1'b0;
            mb_resp_we       <= 1'b0;
            mb_resp_valid_set<= 1'b0;

            // rate window
            window_ctr <= window_ctr + 1;
            if (window_ctr >= rate_window) begin
                window_ctr <= 0;
                commit_ctr <= 0;
            end

            // MMIO writes
            if (mmio_we) begin
                case (mmio_addr)
                    8'h00: allow_mask[31:0]  <= mmio_wdata;
                    8'h01: allow_mask[63:32] <= mmio_wdata;
                    8'h02: rate_limit        <= mmio_wdata;
                    8'h03: rate_window       <= (mmio_wdata == 0) ? 32'd1 : mmio_wdata;
                    8'h04: policy_hash       <= mmio_wdata;
                    8'h07: nonce_expected    <= mmio_wdata;
                    8'h08: clamp_min[0]      <= mmio_wdata;
                    8'h09: clamp_max[0]      <= mmio_wdata;
                    8'h0A: clamp_min[1]      <= mmio_wdata;
                    8'h0B: clamp_max[1]      <= mmio_wdata;
                    8'h0C: clamp_min[2]      <= mmio_wdata;
                    8'h0D: clamp_max[2]      <= mmio_wdata;
                    8'h0E: clamp_min[3]      <= mmio_wdata;
                    8'h0F: clamp_max[3]      <= mmio_wdata;
                    default: ;
                endcase
            end

            // MMIO reads
            case (mmio_addr)
                8'h00: mmio_rdata <= allow_mask[31:0];
                8'h01: mmio_rdata <= allow_mask[63:32];
                8'h02: mmio_rdata <= rate_limit;
                8'h03: mmio_rdata <= rate_window;
                8'h04: mmio_rdata <= policy_hash;
                8'h06: mmio_rdata <= nonce_expected;
                default: mmio_rdata <= 32'h0;
            endcase

            // Response write sequencer (writes 5 words then sets valid)
            if (resp_seq > 0) begin
                if (resp_seq <= 3'd5) begin
                    mb_resp_we    <= 1'b1;
                    mb_resp_widx  <= resp_seq - 1;
                    mb_resp_wdata <= resp_w[resp_seq - 1];
                    resp_seq      <= resp_seq + 1;
                end else begin
                    mb_resp_valid_set <= 1'b1;
                    resp_seq <= 3'd0;
                end
            end

            // Handle request (1-cycle decision)
            if (mb_req_valid && resp_seq == 0) begin
                committed = 1'b0;
                reason    = `GATE_INVALID;

                if (!policy_ok)      begin committed = 1'b0; reason = `GATE_POLICY_HASHERR; end
                else if (!allowed)   begin committed = 1'b0; reason = `GATE_POLICY_DENIED;  end
                else if (!nonce_ok)  begin committed = 1'b0; reason = `GATE_NONCE_ERR;      end
                else if (!rate_ok)   begin committed = 1'b0; reason = `GATE_RATE_LIMITED;   end
                else                 begin committed = 1'b1; reason = was_clamped ? `GATE_CLAMPED : `GATE_OK; end

                // Consume request
                mb_req_consume <= 1'b1;

                // Build response words
                resp_w[0] <= 32'h0;
                resp_w[1] <= arg1;
                resp_w[2] <= clamped0;
                resp_w[3] <= reason;
                resp_w[4] <= committed ? 32'd1 : 32'd0;
                resp_seq  <= 3'd1;

                // Commit intent
                if (committed) begin
                    io_we    <= 1'b1;
                    io_addr  <= `BASE_PRIV_IO | {16'h0000, target[15:0]};
                    io_wdata <= clamped0;
                    commit_ctr     <= commit_ctr + 1;
                    nonce_expected <= nonce_expected + 1;
                end

                // Ledger (always)
                led_wr <= 1'b1;
                led_event <= {
                    32'h0, 32'h0,
                    opcode,
                    target,
                    clamped0,
                    {(committed ? 1'b1 : 1'b0), 31'h0},
                    cycle_ctr,
                    policy_hash
                };
            end
        end
    end
endmodule
`default_nettype wire
