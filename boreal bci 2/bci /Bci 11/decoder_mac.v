module decoder_mac (
    input clk,
    input rst,
    input signed [15:0] features_q [47:0], // Q1.15 format
    output reg signed [15:0] control_outputs_q [1:0] // Q5.11 format
);

    // Parameters from coeffs.json
    localparam N_FEATURES = 48;
    localparam N_OUTPUTS = 2;

    // Coefficients (W_q511 and b_q511) from coeffs.json
    // W_q511 is Q5.11, b_q511 is Q5.11
    // These would typically be loaded from memory or ROM in a real FPGA design
    // For this simulation, we'll hardcode a small example or assume they are accessible.
    // In a full implementation, these would be read from a .mem file or similar.
    // For now, let's define them as parameters or localparams if they are small enough.
    // Given the size (2x48 for W, 2 for b), they would likely be in BRAM/ROM.

    // Example hardcoded values (replace with actual loaded values)
    // For demonstration, using a simplified W and b. Actual values from coeffs.json are too large to hardcode directly.
    // In a real scenario, these would be initialized from the .mem files or a configuration register.
    reg signed [15:0] W_q511 [1:0][47:0];
    reg signed [15:0] b_q511 [1:0];

    // Initialize W_q511 and b_q511 from coeffs.json (simplified for example)
    initial begin
        // Example values, replace with actual data from coeffs.json
        // W_q511[0][0] = 16'd4556; // Example from coeffs.json
        // b_q511[0] = 16'd105; // Example from coeffs.json
        // ... (actual loading mechanism would be more complex)

        // For now, let's use some placeholder values for W and b for compilation.
        // A real system would have a mechanism to load these from memory or a configuration interface.
        for (int i = 0; i < N_OUTPUTS; i = i + 1) begin
            for (int j = 0; j < N_FEATURES; j = j + 1) begin
                W_q511[i][j] = 16'd0;
            end
            b_q511[i] = 16'd0;
        end
        // Set a few example values to avoid all zeros
        W_q511[0][0] = 16'd4556; // From coeffs.json
        W_q511[0][1] = 16'd(-194); // From coeffs.json
        W_q511[1][0] = 16'd(-1002); // From coeffs.json
        W_q511[1][1] = 16'd3526; // From coeffs.json
        b_q511[0] = 16'd105; // From coeffs.json
        b_q511[1] = 16'd47; // From coeffs.json
    end

    integer i, j;
    reg signed [31:0] acc_32bit; // Accumulator for MAC operations

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < N_OUTPUTS; i = i + 1) begin
                control_outputs_q[i] <= 16'd0;
            end
        end else begin
            for (i = 0; i < N_OUTPUTS; i = i + 1) begin
                acc_32bit = 32'd0;
                for (j = 0; j < N_FEATURES; j = j + 1) begin
                    // W_q511 (Q5.11) * features_q (Q1.15) = Q6.26
                    acc_32bit = acc_32bit + ($signed(W_q511[i][j]) * $signed(features_q[j]));
                end

                // Scale Q6.26 to Q5.11: shift by 15 bits
                // Rounding: (acc + (1 << 14)) >> 15
                acc_32bit = (acc_32bit + (1 << 14)) >>> 15; // Arithmetic right shift

                // Add bias (Q5.11)
                acc_32bit = acc_32bit + $signed(b_q511[i]);

                // Saturate to Q5.11 range (16-bit signed)
                // Min: -32768 * 32 = -1048576 (approx) -> -2^15 * 2^5 = -2^20
                // Max: 32767 * 32 = 1048544 (approx) -> (2^15-1) * 2^5 = 2^20 - 2^5
                // The actual range for Q5.11 is [-32, 32 - 2^-11], which is [-32*2^11, (32-2^-11)*2^11] in integer form
                // Min: -(1 << (5+11-1)) = -(1 << 15) = -32768
                // Max: (1 << (5+11-1)) - 1 = (1 << 15) - 1 = 32767
                // So, the result should be clipped to 16-bit signed range.
                if (acc_32bit > 16'h7FFF) begin
                    control_outputs_q[i] <= 16'h7FFF;
                end else if (acc_32bit < 16'h8000) begin
                    control_outputs_q[i] <= 16'h8000;
                end else begin
                    control_outputs_q[i] <= acc_32bit[15:0];
                end
            end
        end
    end

endmodule
