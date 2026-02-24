/*
 * Boreal BCI Least Mean Squares (LMS) Adaptive Decoder
 * 
 * Purpose:
 * Dynamically updates its internal decoding weights on a cycle-by-cycle basis
 * to minimize the residual error signal. Compensates for biological signal drift
 * and neuroplasticity in real-time.
 * 
 * Architecture:
 * 1D Linear Adaptive Combiner
 * Inputs:  N spatial components (e.g., from CSP filter/Kalman)
 * Outputs: 1 Decoded state value
 * 
 * Equation (Standard LMS):
 * y(n) = w(n) * x(n) 
 * e(n) = d(n) - y(n)   [d(n) is the desired target/error feedback]
 * w(n+1) = w(n) + 2 * mu * e(n) * x(n)
 * 
 * Note: Fixed-point Q1.15 arithmetic implies `mu` is a bit logical right-shift
 * (Division by powers of 2) for maximum DSP slice throughput.
 */

module boreal_lms_decoder #(
    parameter NUM_COMPONENTS = 4,

    // Learning Rate (mu). Implemented as a right bit shift.
    // E.g., MU_SHIFT = 8 -> mu = 1/256 ≈ 0.0039
    // A smaller mu prevents weight explosion (divergence) from biological spikes.
    parameter MU_SHIFT = 10 
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        valid_in,

    // Inputs (from Kalman/CSP)
    input  wire signed [15:0] comp_in_0,
    input  wire signed [15:0] comp_in_1,
    input  wire signed [15:0] comp_in_2,
    input  wire signed [15:0] comp_in_3,

    // Error Feedback (e.g., Target Pos - Current Pos)
    // Positive error = output needs to be more positive
    input  wire signed [15:0] error_in,

    // Adaptive Output
    output reg signed [15:0]  state_out,
    output reg                valid_out
);

    // Q1.15 Fixed Point Scaling
    localparam FP_SHIFT = 15;

    // Weight Registers (Dynamic)
    reg signed [15:0] w_0;
    reg signed [15:0] w_1;
    reg signed [15:0] w_2;
    reg signed [15:0] w_3;

    // Combinatorial Multiplier Pipelines (Stage 1: Output Calculation)
    reg signed [31:0] mul_x0, mul_x1, mul_x2, mul_x3;
    reg signed [31:0] sum_x;
    
    // Combinatorial Multiplier Pipelines (Stage 2: Weight Update 'Delta')
    reg signed [31:0] dw_0, dw_1, dw_2, dw_3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset weights to a neutral or slight positive initial prior
            w_0 <= 16'h0000;
            w_1 <= 16'h0000;
            w_2 <= 16'h0000;
            w_3 <= 16'h0000;
            
            state_out <= 0;
            valid_out <= 0;
        end else if (enable) begin
            if (valid_in) begin
                
                // ------------------------------------------------------------
                // 1. Output Calculation (y(n) = W * X)
                // ------------------------------------------------------------
                // Must pad to 32-bits BEFORE multiplying to prevent intermediate 16-bit overflow
                mul_x0 = $signed({{16{w_0[15]}}, w_0}) * $signed({{16{comp_in_0[15]}}, comp_in_0});
                mul_x1 = $signed({{16{w_1[15]}}, w_1}) * $signed({{16{comp_in_1[15]}}, comp_in_1});
                mul_x2 = $signed({{16{w_2[15]}}, w_2}) * $signed({{16{comp_in_2[15]}}, comp_in_2});
                mul_x3 = $signed({{16{w_3[15]}}, w_3}) * $signed({{16{comp_in_3[15]}}, comp_in_3});

                sum_x = (mul_x0 >>> FP_SHIFT) + (mul_x1 >>> FP_SHIFT) + 
                        (mul_x2 >>> FP_SHIFT) + (mul_x3 >>> FP_SHIFT);

                // Saturation arithmetic guard for output
                if (sum_x > 32767) state_out <= 32767;
                else if (sum_x < -32768) state_out <= -32768;
                else state_out <= sum_x[15:0];

                // ------------------------------------------------------------
                // 2. Weight Update (w_new = w_old + (mu * e * x))
                // ------------------------------------------------------------
                
                // Calculate weight deltas based on backpropagated error
                // The 2* factor from standard LMS is absorbed into the mu_shift.
                // Shift by FP_SHIFT removes the fixed-point Q1.15 multiplication overhead.
                // Shift by MU_SHIFT applies the learning rate scalar.
                dw_0 = ($signed({{16{error_in[15]}}, error_in}) * $signed({{16{comp_in_0[15]}}, comp_in_0})) >>> (FP_SHIFT + MU_SHIFT);
                dw_1 = ($signed({{16{error_in[15]}}, error_in}) * $signed({{16{comp_in_1[15]}}, comp_in_1})) >>> (FP_SHIFT + MU_SHIFT);
                dw_2 = ($signed({{16{error_in[15]}}, error_in}) * $signed({{16{comp_in_2[15]}}, comp_in_2})) >>> (FP_SHIFT + MU_SHIFT);
                dw_3 = ($signed({{16{error_in[15]}}, error_in}) * $signed({{16{comp_in_3[15]}}, comp_in_3})) >>> (FP_SHIFT + MU_SHIFT);

                // Update physical registers with saturation
                // To prevent DSP runaway upon noisy disconnected electrodes, clip weights
                
                if (w_0 + dw_0 > 32767) w_0 <= 32767;
                else if (w_0 + dw_0 < -32768) w_0 <= -32768;
                else w_0 <= w_0 + dw_0[15:0];

                if (w_1 + dw_1 > 32767) w_1 <= 32767;
                else if (w_1 + dw_1 < -32768) w_1 <= -32768;
                else w_1 <= w_1 + dw_1[15:0];

                if (w_2 + dw_2 > 32767) w_2 <= 32767;
                else if (w_2 + dw_2 < -32768) w_2 <= -32768;
                else w_2 <= w_2 + dw_2[15:0];

                if (w_3 + dw_3 > 32767) w_3 <= 32767;
                else if (w_3 + dw_3 < -32768) w_3 <= -32768;
                else w_3 <= w_3 + dw_3[15:0];

                valid_out <= 1;
            end else begin
                valid_out <= 0;
            end
        end else begin
            valid_out <= 0;
        end
    end

endmodule
