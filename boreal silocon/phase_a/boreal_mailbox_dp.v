`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_mailbox_dp.v  (simple dual-port mailbox: CPU port + internal port)
// ============================================================================
module boreal_mailbox_dp #(
    parameter WORDS = 256
) (
    input  wire        clk,

    // CPU/MMIO port
    input  wire        cpu_we,
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_wdata,
    input  wire [3:0]  cpu_wstrb,
    output reg  [31:0] cpu_rdata,

    // Internal port A (read-only)
    input  wire [7:0]  a_ridx,
    output reg  [31:0] a_rdata,

    // Internal port B (write-only)
    input  wire        b_we,
    input  wire [7:0]  b_widx,
    input  wire [31:0] b_wdata
);
    localparam AW = $clog2(WORDS);
    reg [31:0] mem [0:WORDS-1];

    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1) mem[i] = 32'h0;
    end

    wire [AW-1:0] cpu_idx = cpu_addr[AW-1:0];

    always @(posedge clk) begin
        if (cpu_we) begin
            if (cpu_wstrb[0]) mem[cpu_idx][ 7: 0] <= cpu_wdata[ 7: 0];
            if (cpu_wstrb[1]) mem[cpu_idx][15: 8] <= cpu_wdata[15: 8];
            if (cpu_wstrb[2]) mem[cpu_idx][23:16] <= cpu_wdata[23:16];
            if (cpu_wstrb[3]) mem[cpu_idx][31:24] <= cpu_wdata[31:24];
        end
        if (b_we) mem[b_widx] <= b_wdata;
        cpu_rdata <= mem[cpu_idx];
        a_rdata   <= mem[a_ridx];
    end
endmodule
`default_nettype wire
