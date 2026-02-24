module predictor (
    input clk,
    input rst,
    input signed [15:0] current_x_q [3:0], // Current state Q5.11 from KalmanCore
    input signed [15:0] u_q_history [15:0][1:0], // History of control inputs Q5.11 (max 16 entries)
    input [3:0] delay_ticks, // Number of ticks to predict forward (0-15)
    output reg signed [15:0] predicted_x_q [3:0] // Predicted state Q5.11
);

    // Parameters from coeffs.json (Kalman section) - A_q214, B_q214 are Q2.14
    // These would typically be loaded from memory or ROM in a real FPGA design
    reg signed [15:0] A_q214 [3:0][3:0];
    reg signed [15:0] B_q214 [3:0][1:0];

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
    end

    integer i, j, k;
    reg signed [31:0] acc_32bit;
    reg signed [15:0] x_temp [3:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 4; i = i + 1) begin
                predicted_x_q[i] <= 16'd0;
            end
        end else begin
            // Initialize x_temp with the current state
            for (i = 0; i < 4; i = i + 1) begin
                x_temp[i] = current_x_q[i];
            end

            // Iterate the prediction 'delay_ticks' times
            for (k = 0; k < delay_ticks; k = k + 1) begin
                // Perform one step of prediction: x_next = A*x_current + B*u_history[k]
                for (i = 0; i < 4; i = i + 1) begin
                    acc_32bit = 32'd0;
                    // A*x_current term (A_q214 (Q2.14) * x_temp (Q5.11) -> Q7.25)
                    for (j = 0; j < 4; j = j + 1) begin
                        acc_32bit = acc_32bit + ($signed(A_q214[i][j]) * $signed(x_temp[j]));
                    end
                    // B*u_history[k] term (B_q214 (Q2.14) * u_q_history[k] (Q5.11) -> Q7.25)
                    for (j = 0; j < 2; j = j + 1) begin
                        // u_q_history is indexed from 0 to max_delay-1, where 0 is the oldest
                        // We need to access u(t-i), so u_q_history[delay_ticks - 1 - k]
                        acc_32bit = acc_32bit + ($signed(B_q214[i][j]) * $signed(u_q_history[delay_ticks - 1 - k][j]));
                    end
                    // Scale Q7.25 to Q5.11: shift by 14 bits (25 - 11 = 14)
                    // Rounding: (acc + (1 << 13)) >> 14
                    x_temp[i] = (acc_32bit + (1 << 13)) >>> 14;
                    // Saturate to Q5.11 range (16-bit signed)
                    x_temp[i] = (x_temp[i] > 16'h7FFF) ? 16'h7FFF : (x_temp[i] < 16'h8000) ? 16'h8000 : x_temp[i];
                end
            end

            // Assign the final predicted state
            for (i = 0; i < 4; i = i + 1) begin
                predicted_x_q[i] <= x_temp[i];
            end
        end
    end

endmodule
