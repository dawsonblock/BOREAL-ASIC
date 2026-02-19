// ============================================================================
// Formal properties for boreal_ledger
// ============================================================================

module fv_ledger_props (
    input wire        clk,
    input wire        rst_n,
    input wire        wr_en,
    input wire [31:0] idx,
    input wire        sel,
    input wire        ack
);

    reg [31:0] prev_idx;
    reg        past_valid;
    reg        prev_wr_en;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_idx   <= 32'd0;
            past_valid <= 1'b0;
            prev_wr_en <= 1'b0;
        end else begin
            prev_idx   <= idx;
            past_valid <= 1'b1;
            prev_wr_en <= wr_en;
        end
    end

    // P1: Index never decreases
    always @(posedge clk)
        if (rst_n && past_valid) assert (idx >= prev_idx);

    // P2: Previous-cycle wr_en → index incremented by exactly 1
    always @(posedge clk)
        if (rst_n && past_valid && prev_wr_en) assert (idx == prev_idx + 1);

    // P3: Previous-cycle !wr_en → index stable
    always @(posedge clk)
        if (rst_n && past_valid && !prev_wr_en) assert (idx == prev_idx);

    // P4: sel → ack (combinational slave)
    always @(*) if (rst_n && sel) assert (ack);

endmodule
