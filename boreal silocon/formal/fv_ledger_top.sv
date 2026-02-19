// ============================================================================
// Formal wrapper for boreal_ledger
// ============================================================================

module fv_ledger_top (
    input wire        clk,
    input wire        rst_n,

    input wire        wr_en,
    input wire [255:0] wr_data,

    input wire        sel,
    input wire        wr,
    input wire [31:0] addr,
    input wire [31:0] wdata
);

    // Reset assumption
    reg init_done = 0;
    always @(posedge clk) begin
        if (!init_done) begin
            assume (!rst_n);
            init_done <= 1;
        end else begin
            assume (rst_n);
        end
    end

    wire [31:0] idx;
    wire [31:0] rdata;
    wire        ack;

    boreal_ledger u_dut (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en), .wr_data(wr_data),
        .idx(idx),
        .sel(sel), .wr(wr), .addr(addr), .wdata(wdata),
        .rdata(rdata), .ack(ack)
    );

    fv_ledger_props u_props (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en),
        .idx(idx),
        .sel(sel),
        .ack(ack)
    );

endmodule
