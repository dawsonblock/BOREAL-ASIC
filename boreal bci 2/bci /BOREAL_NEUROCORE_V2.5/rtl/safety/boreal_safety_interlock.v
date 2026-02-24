/**
 * Boreal Safety Interlock - 6 Layer Protection
 */

module boreal_safety_interlock #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter MAX_DUTY_PERCENT = 10,
    parameter WINDOW_SECONDS = 10
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        bite_switch_n,
    input  wire        emergency_halt_n,
    input  wire [15:0] heart_rate_variability,
    input  wire [15:0] inference_error,
    input  wire        ecg_r_wave_detected,
    input  wire        vns_active,
    input  wire        saturation_detected,
    output reg         system_enable,
    output reg         vns_enable,
    output reg  [2:0]  safety_state,
    output reg         ad_guard_triggered,
    output reg         duty_limit_active
);

    localparam STATE_SAFE = 3'b000;
    localparam STATE_BITE = 3'b001;
    localparam STATE_EMERGENCY = 3'b010;
    localparam STATE_AD_GUARD = 3'b011;
    localparam STATE_SATURATION = 3'b110;

    localparam MAX_CYCLES_ON = (CLK_FREQ_HZ * WINDOW_SECONDS * MAX_DUTY_PERCENT) / 100;
    reg [31:0] duty_counter;
    wire duty_limit_reached = (duty_counter >= MAX_CYCLES_ON);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            duty_counter <= 32'd0;
        else if (vns_active && duty_counter < MAX_CYCLES_ON)
            duty_counter <= duty_counter + 1'b1;
        else if (!vns_active && duty_counter > 0)
            duty_counter <= duty_counter - 1'b1;
    end

    reg [15:0] hrv_prev;
    reg [15:0] hrv_delta;
    wire ad_condition = (inference_error > 16'd1000) && (hrv_delta > 16'd50) && (heart_rate_variability < hrv_prev);

    always @(posedge clk) begin
        if (!rst_n) begin
            hrv_prev <= 16'd0;
            hrv_delta <= 16'd0;
        end else begin
            hrv_delta <= (heart_rate_variability > hrv_prev) ? (heart_rate_variability - hrv_prev) : (hrv_prev - heart_rate_variability);
            hrv_prev <= heart_rate_variability;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            system_enable <= 1'b0;
            vns_enable <= 1'b0;
            safety_state <= STATE_SAFE;
            ad_guard_triggered <= 1'b0;
            duty_limit_active <= 1'b0;
        end else begin
            if (!bite_switch_n) begin
                safety_state <= STATE_BITE;
                system_enable <= 1'b0;
                vns_enable <= 1'b0;
            end else if (!emergency_halt_n) begin
                safety_state <= STATE_EMERGENCY;
                system_enable <= 1'b0;
                vns_enable <= 1'b0;
            end else if (saturation_detected) begin
                safety_state <= STATE_SATURATION;
                system_enable <= 1'b0;
                vns_enable <= 1'b0;
            end else if (ad_condition) begin
                safety_state <= STATE_AD_GUARD;
                system_enable <= 1'b1;
                vns_enable <= 1'b0;
                ad_guard_triggered <= 1'b1;
            end else if (duty_limit_reached) begin
                safety_state <= STATE_SAFE;
                vns_enable <= 1'b0;
                duty_limit_active <= 1'b1;
            end else begin
                safety_state <= STATE_SAFE;
                system_enable <= 1'b1;
                vns_enable <= 1'b1;
                ad_guard_triggered <= 1'b0;
                duty_limit_active <= 1'b0;
            end
        end
    end
endmodule
