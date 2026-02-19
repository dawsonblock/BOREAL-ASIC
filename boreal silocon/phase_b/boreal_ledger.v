`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_ledger.v  (copy from Phase-A for Phase-B build)
// ============================================================================
module boreal_ledger #(
    parameter DEPTH = 1024
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         wr,
    input  wire [255:0] event_in,
    input  wire [31:0]  cycle_in,
    input  wire         rd_req,
    input  wire [9:0]   rd_addr,
    output reg  [255:0] rd_data,
    output reg  [31:0]  idx
);
    reg [255:0] mem [0:DEPTH-1];
    reg [63:0] run_hash;

    function [63:0] fnv1a64_step;
        input [63:0] h;
        input [7:0]  b;
        reg [63:0] x;
        begin
            x = h ^ {56'h0, b};
            fnv1a64_step = x * 64'h0000_0100_0000_01B3;
        end
    endfunction

    function [63:0] fnv1a64_blob;
        input [63:0]  h0;
        input [255:0] blob;
        integer k;
        reg [63:0] h;
        begin
            h = h0;
            for (k = 0; k < 32; k = k + 1)
                h = fnv1a64_step(h, blob[8*k +: 8]);
            fnv1a64_blob = h;
        end
    endfunction

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) mem[i] = 256'h0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            idx      <= 32'h0;
            run_hash <= 64'hCBF2_9CE4_8422_2325;
        end else begin
            if (wr) begin
                mem[idx[9:0]] <= {run_hash[63:32], run_hash[31:0], event_in[191:0]};
                run_hash <= fnv1a64_blob(run_hash, {run_hash[63:32], run_hash[31:0], event_in[191:0]});
                idx <= idx + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (rd_req) rd_data <= mem[rd_addr];
    end
endmodule
`default_nettype wire
