`timescale 1ns / 1ps
// ============================================================================
// Boreal SoC - SHA-256 Stub
// ============================================================================
// Lightweight SHA-256 stub for simulation and early FPGA bring-up.
// Implements a simplified running hash (XOR-fold + rotate) that is NOT
// cryptographically secure.  Replace with a real SHA-256 core for
// production silicon.
// ============================================================================

module boreal_sha256_stub (
    input  wire        clk,
    input  wire        rst_n,

    // --- Control interface ---
    input  wire        start,       // pulse to initialise
    input  wire        update,      // pulse to feed a word
    input  wire [31:0] data_in,     // word to hash
    output reg  [31:0] hash_out,    // current hash (truncated to 32 bits)
    output reg         ready        // ready for next operation
);

    // Internal state
    reg [255:0] h_state;
    reg         busy;
    reg [3:0]   mix_cnt;

    // SHA-256 initial hash values (first 32 bits of fractional parts of sqrt(2..9))
    localparam [255:0] H_INIT = {
        32'h6a09e667, 32'hbb67ae85, 32'h3c6ef372, 32'ha54ff53a,
        32'h510e527f, 32'h9b05688c, 32'h1f83d9ab, 32'h5be0cd19
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_state  <= 256'h0;
            hash_out <= 32'h0;
            ready    <= 1'b1;
            busy     <= 1'b0;
            mix_cnt  <= 4'd0;
        end else begin
            if (start) begin
                h_state <= H_INIT;
                ready   <= 1'b0;
                busy    <= 1'b1;
                mix_cnt <= 4'd1;
            end else if (busy && mix_cnt > 0) begin
                mix_cnt <= mix_cnt - 1;
                if (mix_cnt == 1) begin
                    ready <= 1'b1;
                    busy  <= 1'b0;
                end
            end else if (update && ready) begin
                // Simplified mixing: XOR data into state and rotate
                h_state[255:224] <= h_state[255:224] ^ data_in;
                h_state[223:192] <= h_state[223:192] ^ {data_in[15:0], data_in[31:16]};
                h_state[191:160] <= h_state[191:160] + data_in;
                h_state[159:128] <= h_state[159:128] ^ (data_in << 3);
                h_state[127: 96] <= h_state[127: 96] + (data_in >> 5);
                h_state[ 95: 64] <= h_state[ 95: 64] ^ {data_in[23:0], data_in[31:24]};
                h_state[ 63: 32] <= h_state[ 63: 32] + data_in;
                h_state[ 31:  0] <= h_state[ 31:  0] ^ ~data_in;

                // Output is XOR-fold of all 8 state words
                hash_out <= h_state[255:224] ^ h_state[223:192] ^
                            h_state[191:160] ^ h_state[159:128] ^
                            h_state[127: 96] ^ h_state[ 95: 64] ^
                            h_state[ 63: 32] ^ h_state[ 31:  0] ^
                            data_in;
            end
        end
    end

endmodule
