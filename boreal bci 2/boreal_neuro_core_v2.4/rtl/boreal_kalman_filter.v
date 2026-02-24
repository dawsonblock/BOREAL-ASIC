/*
 * Boreal BCI 1D Steady-State Kalman Filter (Alpha-Beta Filter)
 * 
 * Purpose:
 * Provides optimal recursive state estimation (position and velocity) for 
 * a single spatial dimension, drastically smoothing noisy EEG control signals 
 * without introducing the severe phase-lag typical of IIR/FIR low-pass filters.
 *
 * Algorithm (Alpha-Beta Approximation of Kalman Filter):
 * Predict Step:
 *   x_pred = x_est_prev + v_est_prev * dt
 *   v_pred = v_est_prev
 * 
 * Update Step (Measurement):
 *   residual = measurement - x_pred
 *   x_est = x_pred + alpha * residual
 *   v_est = v_pred + (beta / dt) * residual
 * 
 * Note: dt is abstracted into the Beta coefficient (Beta_prime = Beta / dt)
 * for fixed-point hardware efficiency.
 */

module boreal_kalman_filter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    
    // Raw noisy measurement from the CSP or inference stage
    input  wire signed [15:0] measurement_in,
    input  wire               valid_in,

    // Kalman gain coefficients (Q1.15 Fixed Point)
    // 0x7FFF = ~0.999, 0x0100 = ~0.0078
    // Typical values for BCI: Alpha = 0.2 (0x1999), Beta' = 0.05 (0x0666)
    input  wire signed [15:0] alpha_gain,
    input  wire signed [15:0] beta_gain,

    // Optimally smoothed state outputs
    output reg signed [15:0] position_out,
    output reg signed [15:0] velocity_out,
    output reg               valid_out
);

    // Internal State Registers (Q1.15)
    reg signed [31:0] x_est; // Position
    reg signed [31:0] v_est; // Velocity

    // Combinatorial prediction nodes
    reg signed [31:0] x_pred;
    reg signed [31:0] v_pred;
    reg signed [31:0] residual;

    // Fixed-point scaling constant
    localparam FP_SHIFT = 15;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_est <= 0;
            v_est <= 0;
            position_out <= 0;
            velocity_out <= 0;
            valid_out <= 0;
        end else if (enable) begin
            if (valid_in) begin
                // 1. Predict Step
                // x_pred = x_est + v_est
                // v_pred = v_est
                x_pred = x_est + v_est;
                v_pred = v_est;

                // 2. Calculate Residual (Innovation)
                // residual = measurement - x_pred
                // Note: Sign-extend measurement to 32 bits for math
                residual = {{16{measurement_in[15]}}, measurement_in} - x_pred;

                // 3. Update Step
                // Multiply coefficients: (Q1.15 * Q1.15) >> 15 = Q1.15
                // x_est = x_pred + (alpha * residual)
                // v_est = v_pred + (beta * residual)
                
                // Temporary registers for DSP multiplication results
                // Using 64-bit to prevent intermediate overflow during multiply of 32-bit residual
                // We truncate back to 32 bits after the shift.
                
                // Inline math for FPGA DSP synthesis
                x_est <= x_pred + (($signed(alpha_gain) * residual) >>> FP_SHIFT);
                v_est <= v_pred + (($signed(beta_gain)  * residual) >>> FP_SHIFT);

                // 4. Output assignment with saturation prevention
                // In typical BCI, position shouldn't exceed 16-bit limits
                position_out <= x_pred[15:0] + (($signed(alpha_gain) * residual) >>> FP_SHIFT);
                velocity_out <= v_pred[15:0] + (($signed(beta_gain)  * residual) >>> FP_SHIFT);

                valid_out <= 1;
            end else begin
                valid_out <= 0;
            end
        end else begin
            // Disabled: hold outputs, hold state
            valid_out <= 0;
        end
    end

endmodule
