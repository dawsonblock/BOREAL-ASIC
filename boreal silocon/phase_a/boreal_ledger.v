`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_ledger.v  (append-only, dev rolling hash chain)
// - Only Gate can append (wr/event_in).
// - Public can read entries via rd_req/rd_addr/rd_data.
// - 64-bit FNV-1a style running hash (NOT cryptographic — dev only).
// ============================================================================
module boreal_ledger #(
    parameter DEPTH = 1024
) (
    input  wire         clk,
    input  wire         rst_n,

    // Gate append port
    input  wire         wr,
    input  wire [255:0] event_in,
    input  wire [31:0]  cycle_in,

    // Public read MMIO port
    input  wire         rd_req,
    input  wire [9:0]   rd_addr,
    output reg  [255:0] rd_data,

    output reg  [31:0]  idx
);
    reg [255:0] mem [0:DEPTH-1];

    // 64-bit running hash (dev)
    reg [63:0] run_hash;

    // FNV-1a step: XOR byte then multiply by FNV prime
    function [63:0] fnv1a64_step;
        input [63:0] h;
        input [7:0]  b;
        reg [63:0] x;
        begin
            x = h ^ {56'h0, b};
            fnv1a64_step = x * 64'h0000_0100_0000_01B3;
        end
    endfunction

    // FNV-1a over a 256-bit blob (32 bytes)
    function [63:0] fnv1a64_blob;
        input [63:0]  h0;
        input [255:0] blob;
        integer k;
        reg [63:0] h;
        begin
            h = h0;
            for (k = 0; k < 32; k = k + 1) begin
                h = fnv1a64_step(h, blob[8*k +: 8]);
            end
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
            run_hash <= 64'hCBF2_9CE4_8422_2325; // FNV offset basis
        end else begin
            if (wr) begin
                // Build chained event: overwrite prev_hash fields from running hash
                mem[idx[9:0]] <= {run_hash[63:32], run_hash[31:0], event_in[191:0]};
                // Update running hash over full stored entry
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
