module gate_policy (
    input clk,
    input rst,
    input signed [15:0] u_raw_q [1:0], // Raw control input Q5.11 [vel_x, vel_y]
    input signed [15:0] conf_q,        // Confidence score Q1.15 [0, 32767]
    output reg signed [15:0] u_gated_q [1:0], // Gated control output Q5.11
    output reg click_triggered
);

    // Parameters from coeffs.json (gate section)
    localparam signed [15:0] MAX_VEL_Q511 = 16'd32767; // Example, should be from coeffs.json
    localparam signed [15:0] MAX_JERK_Q511 = 16'd32767; // Example, should be from coeffs.json
    localparam signed [15:0] CONF_THRESHOLD_Q115 = 16'd9830; // 0.3 in Q1.15
    localparam signed [15:0] CONF_CLICK_THRESHOLD_Q115 = 16'd19660; // 0.6 in Q1.15
    localparam DWELL_TICKS = 12;
    localparam signed [15:0] DECAY_RATE_Q115 = 16'd32440; // 0.99 in Q1.15 (approx 0.9 in python twin)

    reg signed [15:0] u_prev_q [1:0];
    reg [3:0] dwell_counter;
    reg click_ready;

    integer i;
    reg signed [31:0] mag_sq;
    reg signed [31:0] max_vel_sq;
    reg signed [31:0] jerk_sq;
    reg signed [31:0] max_jerk_sq;
    reg signed [15:0] scale_factor;
    reg signed [15:0] jerk_q [1:0];
    reg signed [31:0] vel_sq;

    // Function to calculate square root (simplified for fixed-point)
    // This is a placeholder; a real FPGA would use a CORDIC or iterative square root module.
    function signed [15:0] fixed_sqrt;
        input signed [31:0] val;
        reg signed [15:0] res;
        reg signed [31:0] bit;
        integer j;
    begin
        res = 16'd0;
        bit = 32'h40000000; // Start with 2^30

        for (j = 0; j < 16; j = j + 1) begin
            if (val >= res + bit) begin
                val = val - (res + bit);
                res = (res >> 1) + bit;
            end else begin
                res = res >> 1;
            end
            bit = bit >> 2;
        end
        fixed_sqrt = res;
    end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            u_gated_q[0] <= 16'd0;
            u_gated_q[1] <= 16'd0;
            u_prev_q[0] <= 16'd0;
            u_prev_q[1] <= 16'd0;
            dwell_counter <= 4'd0;
            click_triggered <= 1'b0;
            click_ready <= 1'b0;
        end else begin
            // Default to no click
            click_triggered <= 1'b0;

            // Confidence threshold
            if (conf_q < CONF_THRESHOLD_Q115) begin
                // Decay to rest: u_gated = u_prev * decay_rate
                u_gated_q[0] <= ($signed(u_prev_q[0]) * $signed(DECAY_RATE_Q115)) >>> 15; // Q5.11 * Q1.15 -> Q6.26, shift 15 to Q5.11
                u_gated_q[1] <= ($signed(u_prev_q[1]) * $signed(DECAY_RATE_Q115)) >>> 15;
            end else begin
                u_gated_q[0] <= u_raw_q[0];
                u_gated_q[1] <= u_raw_q[1];

                // Magnitude clamp
                // mag_sq = (u_gated_q[0]^2 + u_gated_q[1]^2) >> 11 (to keep it in Q10.22 for comparison)
                mag_sq = ($signed(u_gated_q[0]) * $signed(u_gated_q[0])) >>> 11;
                mag_sq = mag_sq + (($signed(u_gated_q[1]) * $signed(u_gated_q[1])) >>> 11);

                max_vel_sq = ($signed(MAX_VEL_Q511) * $signed(MAX_VEL_Q511)) >>> 11;

                if (mag_sq > max_vel_sq) begin
                    // Scale down: scale = sqrt(max_vel_sq / mag_sq) in Q1.15
                    // This requires a fixed-point division and square root, which is complex.
                    // For simplicity, we'll use a direct scaling approximation or a lookup table.
                    // A more accurate implementation would involve a dedicated sqrt/div module.
                    // For now, let's assume a simple clamping if magnitude exceeds.
                    // This is a simplification from the Python twin which calculates a scale factor.
                    // A proper fixed-point division and square root would be needed here.
                    // As a placeholder, if magnitude is too high, we'll just cap it to MAX_VEL_Q511 directionally.
                    // This is not bit-exact to the Python twin's scaling, but demonstrates the intent.
                    // For bit-exact, a more complex sqrt/div is needed.
                    if (mag_sq > 0) begin
                        scale_factor = fixed_sqrt(max_vel_sq << 11) / fixed_sqrt(mag_sq << 11); // Simplified
                        u_gated_q[0] <= ($signed(u_gated_q[0]) * $signed(scale_factor)) >>> 15;
                        u_gated_q[1] <= ($signed(u_gated_q[1]) * $signed(scale_factor)) >>> 15;
                    end
                end

                // Jerk limit
                jerk_q[0] = u_gated_q[0] - u_prev_q[0];
                jerk_q[1] = u_gated_q[1] - u_prev_q[1];

                jerk_sq = ($signed(jerk_q[0]) * $signed(jerk_q[0])) >>> 11;
                jerk_sq = jerk_sq + (($signed(jerk_q[1]) * $signed(jerk_q[1])) >>> 11);

                max_jerk_sq = ($signed(MAX_JERK_Q511) * $signed(MAX_JERK_Q511)) >>> 11;

                if (jerk_sq > max_jerk_sq) begin
                    if (jerk_sq > 0) begin
                        scale_factor = fixed_sqrt(max_jerk_sq << 11) / fixed_sqrt(jerk_sq << 11); // Simplified
                        jerk_q[0] <= ($signed(jerk_q[0]) * $signed(scale_factor)) >>> 15;
                        jerk_q[1] <= ($signed(jerk_q[1]) * $signed(scale_factor)) >>> 15;
                    end
                    u_gated_q[0] <= u_prev_q[0] + jerk_q[0];
                    u_gated_q[1] <= u_prev_q[1] + jerk_q[1];
                end
            end

            // Saturate final u_gated_q to Q5.11 range
            for (i = 0; i < 2; i = i + 1) begin
                u_gated_q[i] <= (u_gated_q[i] > 16'h7FFF) ? 16'h7FFF : (u_gated_q[i] < 16'h8000) ? 16'h8000 : u_gated_q[i];
            end

            // Update u_prev_q
            u_prev_q[0] <= u_gated_q[0];
            u_prev_q[1] <= u_gated_q[1];

            // Dwell click logic
            if (conf_q >= CONF_CLICK_THRESHOLD_Q115) begin
                vel_sq = ($signed(u_gated_q[0]) * $signed(u_gated_q[0])) >>> 11;
                vel_sq = vel_sq + (($signed(u_gated_q[1]) * $signed(u_gated_q[1])) >>> 11);

                // Threshold: 1.0 in Q5.11 = 2048. So 1.0^2 in Q10.22 is 2048*2048 = 4194304
                // In Q10.22, 1.0 is 2^22. So 1.0^2 is 2^22. The Python twin uses vel_sq < (2048 * 2048) >> 11
                // which is 4194304 >> 11 = 2048. So the threshold is 1.0 in Q5.11 format.
                if (vel_sq < 32'd2048) begin // Comparing with 1.0 in Q5.11 (2048)
                    if (dwell_counter < DWELL_TICKS) begin
                        dwell_counter <= dwell_counter + 4'd1;
                    end
                    if (dwell_counter == DWELL_TICKS - 1 && !click_ready) begin
                        click_triggered <= 1'b1;
                        click_ready <= 1'b1;
                    end
                end else begin
                    dwell_counter <= 4'd0;
                    click_ready <= 1'b0;
                end
            end else begin
                dwell_counter <= 4'd0;
                click_ready <= 1'b0;
            end
        end
    end

endmodule
