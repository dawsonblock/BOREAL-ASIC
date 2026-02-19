// ============================================================================
// Formal properties for boreal_gate
// ============================================================================
`include "boreal_pkg.v"

module fv_gate_props (
    input wire        clk,
    input wire        rst_n,
    input wire [2:0]  state,
    input wire        committed,
    input wire        priv_req,
    input wire        priv_wr,
    input wire        ledger_wr_en,
    input wire        act_ready,
    input wire        resp_valid,
    input wire [31:0] resp_committed,
    input wire [31:0] nonce_counter,
    input wire [31:0] r_opcode,
    input wire [31:0] r_nonce,
    input wire [31:0] r_policy_hash,
    input wire [31:0] policy_hash,
    input wire [31:0] override_reg,
    input wire [31:0] rate_count,
    input wire [31:0] rate_limit
);

    localparam S_IDLE    = 3'd0;
    localparam S_CHECK   = 3'd1;
    localparam S_CLAMP   = 3'd2;
    localparam S_COMMIT  = 3'd3;
    localparam S_LEDGER  = 3'd4;
    localparam S_RESPOND = 3'd5;

    reg [31:0] prev_nonce;
    reg        past_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_nonce <= 32'd0;
            past_valid <= 1'b0;
        end else begin
            prev_nonce <= nonce_counter;
            past_valid <= 1'b1;
        end
    end

    // P1: Denied action in COMMIT → no priv_req
    always @(posedge clk)
        if (rst_n && state == S_COMMIT && !committed) assert (!priv_req);

    // P5: Nonce counter never decreases
    always @(posedge clk)
        if (rst_n && past_valid) assert (nonce_counter >= prev_nonce);

    // P6: In LEDGER state, ledger_wr_en is asserted
    always @(posedge clk)
        if (rst_n && state == S_LEDGER) assert (ledger_wr_en);

    // P7: In RESPOND state, act_ready re-asserted
    always @(posedge clk)
        if (rst_n && state == S_RESPOND) assert (act_ready);

endmodule
