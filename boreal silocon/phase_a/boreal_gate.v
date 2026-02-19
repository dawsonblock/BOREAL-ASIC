`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_gate.v  (hardened Phase-A Gate)
// - Validates: policy hash, allowlist, rate window, clamp class, nonce
// - Emits: priv write intent (io_we/io_addr/io_wdata)
// - Always appends to ledger for every req_valid.
// - Deterministic 1-cycle decision and response.
// ============================================================================
`include "boreal_pkg.vh"

module boreal_gate (
    input  wire         clk,
    input  wire         rst_n,

    // action request (from VM)
    input  wire         req_valid,
    input  wire [511:0] req_data,

    output reg          resp_valid,
    output reg  [159:0] resp_data,

    // commit intent (future: privileged IO bus)
    output reg          io_we,
    output reg  [31:0]  io_addr,
    output reg  [31:0]  io_wdata,

    // ledger append
    output reg          led_wr,
    output reg  [255:0] led_event,

    input  wire [31:0]  cycle_ctr,

    // MMIO policy regs
    input  wire         mmio_we,
    input  wire [7:0]   mmio_addr,
    input  wire [31:0]  mmio_wdata,
    input  wire [3:0]   mmio_wstrb,
    output reg  [31:0]  mmio_rdata
);
    // Policy state
    reg [63:0] allow_mask;
    reg [31:0] rate_limit;
    reg [31:0] rate_window;
    reg [31:0] policy_hash;

    // 4 clamp classes (min/max)
    reg [31:0] clamp_min [0:3];
    reg [31:0] clamp_max [0:3];

    // Rate window counters
    reg [31:0] window_ctr;
    reg [31:0] commit_ctr;

    // Nonce (monotonic expected for committed actions)
    reg [31:0] nonce_expected;

    // Decode request fields
    wire [31:0] opcode       = req_data[31:0];
    wire [31:0] target       = req_data[63:32];
    wire [31:0] arg0         = req_data[95:64];
    wire [31:0] arg1         = req_data[127:96];
    wire [31:0] context_hash = req_data[159:128];
    wire [31:0] req_pol_hash = req_data[191:160];
    wire [31:0] bounds       = req_data[223:192];
    wire [31:0] nonce        = req_data[255:224];

    wire allowed   = allow_mask[target[5:0]];
    wire policy_ok = (req_pol_hash == policy_hash);
    wire rate_ok   = (commit_ctr < rate_limit);
    wire nonce_ok  = (nonce == nonce_expected);

    wire [1:0]  clamp_class = bounds[1:0];
    wire [31:0] cmin = clamp_min[clamp_class];
    wire [31:0] cmax = clamp_max[clamp_class];

    wire [31:0] clamped0    = (arg0 < cmin) ? cmin : (arg0 > cmax) ? cmax : arg0;
    wire        was_clamped = (clamped0 != arg0);

    // Decision temporaries (blocking assignment in clocked block)
    reg        committed;
    reg [31:0] reason;

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

            resp_valid <= 1'b0;
            resp_data  <= 160'h0;
            io_we      <= 1'b0;
            io_addr    <= 32'h0;
            io_wdata   <= 32'h0;
            led_wr     <= 1'b0;
            led_event  <= 256'h0;
            mmio_rdata <= 32'h0;
        end else begin
            // defaults
            resp_valid <= 1'b0;
            io_we      <= 1'b0;
            led_wr     <= 1'b0;

            // rate window management (deterministic)
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
                8'h08: mmio_rdata <= clamp_min[0];
                8'h09: mmio_rdata <= clamp_max[0];
                8'h0A: mmio_rdata <= clamp_min[1];
                8'h0B: mmio_rdata <= clamp_max[1];
                8'h0C: mmio_rdata <= clamp_min[2];
                8'h0D: mmio_rdata <= clamp_max[2];
                8'h0E: mmio_rdata <= clamp_min[3];
                8'h0F: mmio_rdata <= clamp_max[3];
                default: mmio_rdata <= 32'h0;
            endcase

            // Handle request
            if (req_valid) begin
                committed = 1'b0;
                reason    = `GATE_INVALID;

                if (!policy_ok)      begin committed = 1'b0; reason = `GATE_POLICY_HASHERR; end
                else if (!allowed)   begin committed = 1'b0; reason = `GATE_POLICY_DENIED;  end
                else if (!nonce_ok)  begin committed = 1'b0; reason = `GATE_NONCE_ERR;      end
                else if (!rate_ok)   begin committed = 1'b0; reason = `GATE_RATE_LIMITED;   end
                else                 begin committed = 1'b1; reason = was_clamped ? `GATE_CLAMPED : `GATE_OK; end

                // Commit intent (Phase-A)
                if (committed) begin
                    io_we    <= 1'b1;
                    io_addr  <= `BASE_PRIV_IO | {16'h0000, target[15:0]};
                    io_wdata <= clamped0;

                    commit_ctr     <= commit_ctr + 1;
                    nonce_expected <= nonce_expected + 1;
                end

                // Response
                resp_valid <= 1'b1;
                resp_data  <= {committed ? 32'd1 : 32'd0,
                               reason,
                               clamped0,
                               arg1,
                               32'h0};

                // Ledger emit (always, for every request)
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
