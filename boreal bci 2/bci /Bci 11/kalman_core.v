module kalman_core (
    input clk,
    input rst,
    input signed [15:0] u_q [1:0], // Control input Q5.11 [vel_x, vel_y]
    input signed [15:0] z_q [1:0], // Measurement Q5.11 [pos_x, pos_y]
    output reg signed [15:0] x_q [3:0] // Predicted state Q5.11 [pos_x, pos_y, vel_x, vel_y]
);

    // Parameters from coeffs.json (Kalman section)
    // A_q214, B_q214, H_q214 are Q2.14
    // Q_q610, R_q610 are Q6.10

    // State: [pos_x, pos_y, vel_x, vel_y]
    // x_q is Q5.11

    // Hardcoded example matrices (replace with actual loaded values from coeffs.json)
    // In a real FPGA, these would be loaded from .mem files or configuration registers.
    reg signed [15:0] A_q214 [3:0][3:0];
    reg signed [15:0] B_q214 [3:0][1:0];
    reg signed [15:0] H_q214 [1:0][3:0];
    reg signed [15:0] Q_q610 [3:0][3:0];
    reg signed [15:0] R_q610 [1:0][1:0];

    // Initial state (x_q) and covariance (P_q) - these would be resetable or loaded
    reg signed [15:0] P_q [3:0][3:0]; // Q6.10

    initial begin
        // Initialize A_q214 from coeffs.json
        A_q214[0][0] = 16'd16384; A_q214[0][1] = 16'd0; A_q214[0][2] = 16'd131; A_q214[0][3] = 16'd0;
        A_q214[1][0] = 16'd0; A_q214[1][1] = 16'd16384; A_q214[1][2] = 16'd0; A_q214[1][3] = 16'd131;
        A_q214[2][0] = 16'd0; A_q214[2][1] = 16'd0; A_q214[2][2] = 16'd16384; A_q214[2][3] = 16'd0;
        A_q214[3][0] = 16'd0; A_q214[3][1] = 16'd0; A_q214[3][2] = 16'd0; A_q214[3][3] = 16'd16384;

        // Initialize B_q214 from coeffs.json
        B_q214[0][0] = 16'd0; B_q214[0][1] = 16'd0;
        B_q214[1][0] = 16'd0; B_q214[1][1] = 16'd0;
        B_q214[2][0] = 16'd131; B_q214[2][1] = 16'd0;
        B_q214[3][0] = 16'd0; B_q214[3][1] = 16'd131;

        // Initialize H_q214 from coeffs.json
        H_q214[0][0] = 16'd16384; H_q214[0][1] = 16'd0; H_q214[0][2] = 16'd0; H_q214[0][3] = 16'd0;
        H_q214[1][0] = 16'd0; H_q214[1][1] = 16'd16384; H_q214[1][2] = 16'd0; H_q214[1][3] = 16'd0;

        // Initialize Q_q610 from coeffs.json
        Q_q610[0][0] = 16'd10; Q_q610[0][1] = 16'd0; Q_q610[0][2] = 16'd0; Q_q610[0][3] = 16'd0;
        Q_q610[1][0] = 16'd0; Q_q610[1][1] = 16'd10; Q_q610[1][2] = 16'd0; Q_q610[1][3] = 16'd0;
        Q_q610[2][0] = 16'd0; Q_q610[2][1] = 16'd0; Q_q610[2][2] = 16'd102; Q_q610[2][3] = 16'd0;
        Q_q610[3][0] = 16'd0; Q_q610[3][1] = 16'd0; Q_q610[3][2] = 16'd0; Q_q610[3][3] = 16'd102;

        // Initialize R_q610 from coeffs.json
        R_q610[0][0] = 16'd1024; R_q610[0][1] = 16'd0;
        R_q610[1][0] = 16'd0; R_q610[1][1] = 16'd1024;

        // Initial state x_q (all zeros)
        x_q[0] = 16'd0; x_q[1] = 16'd0; x_q[2] = 16'd0; x_q[3] = 16'd0;

        // Initial covariance P_q (eye(4) * 100.0 in Q6.10)
        P_q[0][0] = 16'd(100 << 10); P_q[0][1] = 16'd0; P_q[0][2] = 16'd0; P_q[0][3] = 16'd0;
        P_q[1][0] = 16'd0; P_q[1][1] = 16'd(100 << 10); P_q[1][2] = 16'd0; P_q[1][3] = 16'd0;
        P_q[2][0] = 16'd0; P_q[2][1] = 16'd0; P_q[2][2] = 16'd(100 << 10); P_q[2][3] = 16'd0;
        P_q[3][0] = 16'd0; P_q[3][1] = 16'd0; P_q[3][2] = 16'd0; P_q[3][3] = 16'd(100 << 10);
    end

    integer i, j, k;
    reg signed [31:0] acc_32bit;

    // Internal state for prediction and update
    reg signed [15:0] x_q_predict [3:0];
    reg signed [15:0] P_q_predict [3:0][3:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x_q[0] <= 16'd0; x_q[1] <= 16'd0; x_q[2] <= 16'd0; x_q[3] <= 16'd0;
            // Reset P_q as well if needed, or load initial value
        end else begin
            // --- Prediction Step (x = A*x + B*u) ---
            for (i = 0; i < 4; i = i + 1) begin
                acc_32bit = 32'd0;
                // A*x term (A_q214 (Q2.14) * x_q (Q5.11) -> Q7.25)
                for (j = 0; j < 4; j = j + 1) begin
                    acc_32bit = acc_32bit + ($signed(A_q214[i][j]) * $signed(x_q[j]));
                end
                // B*u term (B_q214 (Q2.14) * u_q (Q5.11) -> Q7.25)
                for (j = 0; j < 2; j = j + 1) begin
                    acc_32bit = acc_32bit + ($signed(B_q214[i][j]) * $signed(u_q[j]));
                end
                // Scale Q7.25 to Q5.11: shift by 14 bits (25 - 11 = 14)
                // Rounding: (acc + (1 << 13)) >> 14
                x_q_predict[i] = (acc_32bit + (1 << 13)) >>> 14;
                // Saturate to Q5.11 range (16-bit signed)
                x_q_predict[i] = (x_q_predict[i] > 16'h7FFF) ? 16'h7FFF : (x_q_predict[i] < 16'h8000) ? 16'h8000 : x_q_predict[i];
            end

            // --- Simplified Covariance Prediction (P = A*P*A' + Q) ---
            // In the Python twin, this step is simplified for FPGA speed.
            // We will also simplify it here, assuming P_q is updated less frequently or by a separate module.
            // For now, we'll just pass through P_q or apply a simple update.
            // A full implementation would involve matrix multiplication of Q6.10 matrices.
            // For this exercise, we'll assume P_q_predict is a direct copy or a simplified update of P_q.
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    P_q_predict[i][j] = P_q[i][j]; // Placeholder: no actual matrix multiplication
                end
            end

            // --- Update Step (simplified for position-only measurement) ---
            // y = z - H*x_predict (Innovation)
            // H (Q2.14) * x_predict (Q5.11) -> Q7.25, then shift to Q5.11
            reg signed [15:0] y_q [1:0]; // Q5.11
            for (i = 0; i < 2; i = i + 1) begin
                acc_32bit = 32'd0;
                for (j = 0; j < 4; j = j + 1) begin
                    acc_32bit = acc_32bit + ($signed(H_q214[i][j]) * $signed(x_q_predict[j]));
                end
                acc_32bit = (acc_32bit + (1 << 13)) >>> 14; // Scale Q7.25 to Q5.11
                y_q[i] = $signed(z_q[i]) - acc_32bit; // z_q is Q5.11, so y_q is Q5.11
            end

            // S = H*P*H' + R (Innovation Covariance)
            // This is skipped in the Python twin for FPGA speed. We will also skip it here.
            // Instead, we use a fixed gain for simplicity, as in the Python twin's simplified update.

            // K = P*H'*inv(S) (Kalman Gain)
            // x = x + K*y (State Update)
            // Simplified: x_q[0] += (gain_q * y[0]) >> 15
            // gain_q is Q1.15 (0.3 * 32768 = 9830)
            localparam signed [15:0] GAIN_Q115 = 16'd9830; // 0.3 in Q1.15

            x_q[0] <= x_q_predict[0] + (($signed(GAIN_Q115) * $signed(y_q[0])) >>> 15); // Q1.15 * Q5.11 -> Q6.26, then shift by 15 to Q5.11
            x_q[1] <= x_q_predict[1] + (($signed(GAIN_Q115) * $signed(y_q[1])) >>> 15); // Q1.15 * Q5.11 -> Q6.26, then shift by 15 to Q5.11
            x_q[2] <= x_q_predict[2]; // Velocity components are not directly updated by position measurement in this simplified model
            x_q[3] <= x_q_predict[3];

            // Saturate final x_q to Q5.11 range
            for (i = 0; i < 4; i = i + 1) begin
                x_q[i] <= (x_q[i] > 16'h7FFF) ? 16'h7FFF : (x_q[i] < 16'h8000) ? 16'h8000 : x_q[i];
            end

            // P_q update (simplified or skipped)
            // For now, P_q remains unchanged or is updated by a separate, slower process.
        end
    end

endmodule
