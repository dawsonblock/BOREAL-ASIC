/**
 * Boreal DC Blocker - IIR High-Pass Filter with Saturation
 */

module boreal_dc_blocker #(
    parameter DATA_WIDTH = 24,
    parameter ALPHA_NUM = 32702,
    parameter OUT_WIDTH = 16
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      en,
    input  wire signed [DATA_WIDTH-1:0] x_in,
    input  wire        [2:0]         channel_sel,
    output reg  signed [OUT_WIDTH-1:0]  y_out,
    output reg                         valid,
    output reg                         saturation_flag
);

    reg signed [DATA_WIDTH-1:0] x_prev [0:7];
    reg signed [DATA_WIDTH+15:0] y_acc [0:7];

    wire signed [DATA_WIDTH-1:0] x_curr = x_in;
    wire signed [DATA_WIDTH-1:0] x_z1 = x_prev[channel_sel];
    wire signed [DATA_WIDTH+15:0] y_z1 = y_acc[channel_sel];

    wire signed [DATA_WIDTH+15:0] diff = ($signed(x_curr) - $signed(x_z1)) <<< 15;
    wire signed [DATA_WIDTH+15:0] feedback = (y_z1 * ALPHA_NUM) >>> 15;
    wire signed [DATA_WIDTH+15:0] y_full = diff + feedback;

    wire overflow_pos = (y_full > (2**(DATA_WIDTH+15-1) - 1));
    wire overflow_neg = (y_full < -(2**(DATA_WIDTH+15-1)));

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                x_prev[i] <= {DATA_WIDTH{1'b0}};
                y_acc[i] <= {(DATA_WIDTH+16){1'b0}};
            end
            y_out <= {OUT_WIDTH{1'b0}};
            valid <= 1'b0;
            saturation_flag <= 1'b0;
        end else if (en) begin
            x_prev[channel_sel] <= x_curr;
            if (overflow_pos)
                y_acc[channel_sel] <= (2**(DATA_WIDTH+15-1) - 1);
            else if (overflow_neg)
                y_acc[channel_sel] <= -(2**(DATA_WIDTH+15-1));
            else
                y_acc[channel_sel] <= y_full;

            wire signed [DATA_WIDTH-1:0] y_rounded = y_full[DATA_WIDTH+14:15] + y_full[14];
            if (y_rounded > 32767)
                y_out <= 16'sd32767;
            else if (y_rounded < -32768)
                y_out <= -16'sd32768;
            else
                y_out <= y_rounded[15:0];

            valid <= 1'b1;
            saturation_flag <= overflow_pos || overflow_neg;
        end else begin
            valid <= 1'b0;
            saturation_flag <= 1'b0;
        end
    end
endmodule
