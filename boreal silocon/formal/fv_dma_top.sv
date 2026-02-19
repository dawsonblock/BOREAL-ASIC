// ============================================================================
// Formal wrapper for boreal_dma_ring
// ============================================================================

module fv_dma_top (
    input wire        clk,
    input wire        rst_n,
    input wire        sel,
    input wire        wr,
    input wire [31:0] addr,
    input wire [31:0] wdata,
    input wire [31:0] mem_rdata,
    input wire        mem_ack
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

    wire [31:0] rdata;
    wire        ack;
    wire        mem_sel, mem_wr;
    wire [9:0]  mem_addr;
    wire [31:0] mem_wdata;

    boreal_dma_ring u_dut (
        .clk(clk), .rst_n(rst_n),
        .sel(sel), .wr(wr), .addr(addr), .wdata(wdata),
        .rdata(rdata), .ack(ack),
        .mem_sel(mem_sel), .mem_wr(mem_wr),
        .mem_addr(mem_addr), .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata), .mem_ack(mem_ack)
    );

    // Observable: read DMA status register to get busy/done/head/tail
    // But for formal, we assert output-level properties only:

    // P1: mem_wr implies mem_sel
    always @(posedge clk)
        if (rst_n && mem_wr) assert (mem_sel);

    // P2: mem_sel deasserts cleanly (not stuck high when idle)
    // Read status: when sel && !wr && addr[7:0]==0x08, rdata = {29'b0,error,done,busy}
    // We can check: if status shows not-busy, mem_sel should be low
    // (Can't easily do this without hierarchical refs, so keep it simple)

    // P3: ack always responds to sel
    always @(*) if (rst_n && sel) assert (ack);

endmodule
