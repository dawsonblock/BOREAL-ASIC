// ============================================================================
// Boreal SoC - Gate Policy Engine
// ============================================================================
// MMIO-configurable policy registers: target allowlists, rate limiter,
// active policy hash, and debug override.
// ============================================================================
`include "boreal_pkg.v"

module boreal_gate_policy (
    input  wire        clk,
    input  wire        rst_n,

    // --- MMIO slave interface ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- Policy outputs (active configuration) ---
    output reg  [31:0] allow0,        // target allow mask 0-31
    output reg  [31:0] allow1,        // target allow mask 32-63
    output reg  [31:0] rate_limit,    // max commits per window
    output reg  [31:0] rate_window,   // window length in cycles
    output reg  [31:0] policy_hash,   // active policy hash
    output reg  [31:0] override_reg,  // debug override (fuse-lockable)

    // --- Nonce (read-only monotonic counter) ---
    input  wire [31:0] nonce_val
);

    // Register offsets within Gate address space (0x1004_0000 base)
    localparam OFF_ALLOW0   = 8'h00;
    localparam OFF_ALLOW1   = 8'h04;
    localparam OFF_RATE_LIM = 8'h08;
    localparam OFF_RATE_WIN = 8'h0C;
    localparam OFF_POLICY   = 8'h10;
    localparam OFF_OVERRIDE = 8'h14;
    localparam OFF_NONCE    = 8'h18;

    wire [7:0] reg_off = addr[7:0];

    // MMIO write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            allow0      <= 32'h0;
            allow1      <= 32'h0;
            rate_limit  <= 32'h0000_000A;   // default 10
            rate_window <= 32'h0000_03E8;   // default 1000 cycles
            policy_hash <= 32'h0;
            override_reg<= 32'h0;
        end else if (sel && wr) begin
            case (reg_off)
                OFF_ALLOW0:   allow0      <= wdata;
                OFF_ALLOW1:   allow1      <= wdata;
                OFF_RATE_LIM: rate_limit  <= wdata;
                OFF_RATE_WIN: rate_window <= wdata;
                OFF_POLICY:   policy_hash <= wdata;
                OFF_OVERRIDE: override_reg<= wdata;
                default: ;
            endcase
        end
    end

    // MMIO read
    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr) begin
            case (reg_off)
                OFF_ALLOW0:   rdata = allow0;
                OFF_ALLOW1:   rdata = allow1;
                OFF_RATE_LIM: rdata = rate_limit;
                OFF_RATE_WIN: rdata = rate_window;
                OFF_POLICY:   rdata = policy_hash;
                OFF_OVERRIDE: rdata = override_reg;
                OFF_NONCE:    rdata = nonce_val;
                default:      rdata = 32'h0;
            endcase
        end
    end

endmodule
