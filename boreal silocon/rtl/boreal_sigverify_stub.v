// ============================================================================
// Boreal SoC - Signature Verification Stub
// ============================================================================
// Placeholder signature verification module.  In production, this would
// implement Ed25519 or ECDSA-P256 verification.  The stub compares
// the computed hash against an expected hash burned into a register
// (simulating a fused public key / expected digest).
// ============================================================================

module boreal_sigverify_stub (
    input  wire        clk,
    input  wire        rst_n,

    // --- Control interface ---
    input  wire        start,
    input  wire [31:0] hash_in,     // hash to verify
    output reg         pass,        // 1 = signature valid
    output reg         ready,       // operation complete

    // --- Configuration (set before boot) ---
    input  wire [31:0] expected_hash,
    input  wire [31:0] min_version,
    input  wire [31:0] image_version
);

    reg [1:0] state;
    localparam S_IDLE  = 2'd0;
    localparam S_CHECK = 2'd1;
    localparam S_DONE  = 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            pass  <= 1'b0;
            ready <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        ready <= 1'b0;
                        pass  <= 1'b0;
                        state <= S_CHECK;
                    end
                end

                S_CHECK: begin
                    // Development mode: fuses unprogrammed (all zeros) = always pass
                    if (expected_hash == 32'h0)
                        pass <= 1'b1;
                    // Production: hash match AND version rollback protection
                    else if (hash_in == expected_hash && image_version >= min_version)
                        pass <= 1'b1;
                    else
                        pass <= 1'b0;
                    state <= S_DONE;
                end

                S_DONE: begin
                    ready <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
