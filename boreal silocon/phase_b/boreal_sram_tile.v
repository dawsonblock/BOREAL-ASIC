`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_sram_tile.v  (copy from Phase-A for Phase-B build)
// ============================================================================
module boreal_sram_tile #(
    parameter WORDS    = 1024,
    parameter INIT_HEX = ""
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req_valid,
    input  wire        req_we,
    input  wire [31:0] req_addr,
    input  wire [31:0] req_wdata,
    input  wire [3:0]  req_wstrb,
    output reg         resp_valid,
    output reg  [31:0] resp_rdata,
    output reg         resp_err
);
    localparam AW = $clog2(WORDS);
    reg [31:0] mem [0:WORDS-1];
    wire [AW-1:0] widx = req_addr[2 +: AW];

    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1) mem[i] = 32'h0;
        if (INIT_HEX != "") $readmemh(INIT_HEX, mem);
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            resp_valid <= 1'b0; resp_rdata <= 32'h0; resp_err <= 1'b0;
        end else begin
            resp_valid <= req_valid; resp_err <= 1'b0;
            if (req_valid) begin
                if (req_we) begin
                    if (req_wstrb[0]) mem[widx][ 7: 0] <= req_wdata[ 7: 0];
                    if (req_wstrb[1]) mem[widx][15: 8] <= req_wdata[15: 8];
                    if (req_wstrb[2]) mem[widx][23:16] <= req_wdata[23:16];
                    if (req_wstrb[3]) mem[widx][31:24] <= req_wdata[31:24];
                end
                resp_rdata <= mem[widx];
            end
        end
    end
endmodule
`default_nettype wire
