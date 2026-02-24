/**
 * Boreal Predictive Core v2.5 - Temporal Predictive Coding
 */

module boreal_predictive_core #(
    parameter WIDTH = 16,
    parameter LEAD_DEFAULT = 8'd40
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              en,
    input  wire signed [WIDTH-1:0] mu_pos_in,
    input  wire                    mu_valid,
    input  wire        [7:0]       lead_factor,
    input  wire                    calibrate_en,
    output reg  signed [WIDTH-1:0] pred_pos_out,
    output reg  signed [WIDTH-1:0] pred_vel_out,
    output reg                     pred_valid,
    output reg  [7:0]              current_lead,
    output reg                     calibration_done
);

    reg signed [WIDTH-1:0] mu_pos_prev;
    reg signed [WIDTH-1:0] mu_vel;
    reg        [15:0]      sample_counter;
    reg        [31:0]      latency_accum;

    wire signed [WIDTH-1:0] vel_raw = mu_pos_in - mu_pos_prev;
    wire signed [WIDTH-1:0] vel_smooth = (mu_vel - (mu_vel >>> 3)) + (vel_raw >>> 3);

    wire signed [31:0] pred_full = mu_pos_in + ((mu_vel * $signed({1'b0, current_lead})) >>> 4);
    wire overflow_pos = (pred_full > 32'sh00007FFF);
    wire overflow_neg = (pred_full < 32'shFFFF8000);
    wire signed [WIDTH-1:0] pred_saturated = overflow_pos ? 16'sh7FFF : (overflow_neg ? 16'sh8000 : pred_full[15:0]);

    localparam CALIB_SAMPLES = 16'd1000;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mu_pos_prev <= 16'sd0;
            mu_vel <= 16'sd0;
            pred_pos_out <= 16'sd0;
            pred_vel_out <= 16'sd0;
            pred_valid <= 1'b0;
            current_lead <= LEAD_DEFAULT;
            sample_counter <= 16'd0;
            latency_accum <= 32'd0;
            calibration_done <= 1'b0;
        end else if (en) begin
            if (mu_valid) begin
                mu_vel <= vel_smooth;
                mu_pos_prev <= mu_pos_in;
                pred_pos_out <= pred_saturated;
                pred_vel_out <= mu_vel;
                pred_valid <= 1'b1;

                if (calibrate_en && !calibration_done) begin
                    sample_counter <= sample_counter + 1'b1;
                    if ((mu_vel > 100 || mu_vel < -100))
                        latency_accum <= latency_accum + $unsigned(mu_vel < 0 ? -mu_vel : mu_vel);
                    if (sample_counter >= CALIB_SAMPLES) begin
                        if (latency_accum > 32'd50000 && current_lead < 255)
                            current_lead <= current_lead + 8'd5;
                        else if (latency_accum < 32'd10000 && current_lead > 8'd10)
                            current_lead <= current_lead - 8'd5;
                        calibration_done <= 1'b1;
                        sample_counter <= 16'd0;
                        latency_accum <= 32'd0;
                    end
                end else if (!calibrate_en) begin
                    current_lead <= lead_factor;
                    calibration_done <= 1'b0;
                end
            end else begin
                pred_valid <= 1'b0;
            end
        end else begin
            pred_valid <= 1'b0;
        end
    end
endmodule
