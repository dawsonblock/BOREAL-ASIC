/**
 * Boreal Apex Core v2.5 - Active Inference Engine
 * Implements Free Energy Principle gradient descent in hardware
 */

module boreal_apex_core #(
    parameter ADDR_WIDTH = 10,
    parameter CHANNELS = 8,
    parameter ALPHA_DC_NUM = 32702,
    parameter ETA_SHIFT = 4,
    parameter LAMBDA_SHIFT = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        emergency_halt_n,
    input  wire signed [23:0] raw_adc_in,
    input  wire        [2:0]  adc_channel_sel,
    input  wire               adc_data_ready,
    output reg  [ADDR_WIDTH-1:0] lut_addr,
    input  wire [31:0]           lut_data,
    output reg  signed [15:0] mu_out,
    output reg  signed [15:0] epsilon_out,
    output reg                valid_out,
    output reg                saturation_flag
);

    reg signed [15:0] mu [0:CHANNELS-1];
    reg signed [15:0] epsilon [0:CHANNELS-1];
    localparam signed MAX_VAL = 16'sh7FFF;
    localparam signed MIN_VAL = 16'sh8000;

    wire signed [15:0] sigmoid_val = lut_data[15:0];
    wire signed [15:0] deriv_val = lut_data[31:16];
    wire signed [15:0] current_mu = mu[adc_channel_sel];

    wire signed [31:0] grad_product = epsilon[adc_channel_sel] * deriv_val;
    wire signed [15:0] decay_term = current_mu >>> LAMBDA_SHIFT;
    wire signed [31:0] update_raw = grad_product - (decay_term <<< 15);
    wire signed [15:0] update_scaled = update_raw[20:5];
    wire signed [31:0] mu_sum = current_mu + update_scaled;

    wire overflow_pos = (mu_sum > MAX_VAL);
    wire overflow_neg = (mu_sum < MIN_VAL);
    wire signed [15:0] mu_new = overflow_pos ? MAX_VAL : (overflow_neg ? MIN_VAL : mu_sum[15:0]);

    wire signed [31:0] eps_calc = raw_adc_in[23:8] - sigmoid_val;
    wire signed [15:0] eps_new = (eps_calc > MAX_VAL) ? MAX_VAL : (eps_calc < MIN_VAL) ? MIN_VAL : eps_calc[15:0];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < CHANNELS; i = i + 1) begin
                mu[i] <= 16'sd0;
                epsilon[i] <= 16'sd0;
            end
            mu_out <= 16'sd0;
            epsilon_out <= 16'sd0;
            valid_out <= 1'b0;
            saturation_flag <= 1'b0;
            lut_addr <= 10'd512;
        end else if (!emergency_halt_n) begin
            for (i = 0; i < CHANNELS; i = i + 1) begin
                mu[i] <= 16'sd0;
            end
            valid_out <= 1'b0;
            saturation_flag <= 1'b1;
        end else if (adc_data_ready) begin
            mu[adc_channel_sel] <= mu_new;
            epsilon[adc_channel_sel] <= eps_new;
            lut_addr <= mu_new[15:6] + 10'd512;
            mu_out <= mu_new;
            epsilon_out <= eps_new;
            valid_out <= 1'b1;
            saturation_flag <= overflow_pos || overflow_neg;
        end else begin
            valid_out <= 1'b0;
            saturation_flag <= 1'b0;
        end
    end
endmodule
