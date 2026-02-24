/**
 * Boreal CORDIC - True iterative implementation
 */

module boreal_cordic #(
    parameter WIDTH = 16,
    parameter ITERATIONS = 16
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,
    input  wire signed [WIDTH-1:0] x_in,
    input  wire signed [WIDTH-1:0] y_in,
    output reg  signed [WIDTH-1:0] angle_out,
    output reg  signed [WIDTH-1:0] mag_out,
    output reg               valid
);

    localparam [15:0] CORDIC_ANGLES [0:15] = '{
        16'sd11520, 16'sd6800, 16'sd3593, 16'sd1824,
        16'sd916, 16'sd458, 16'sd229, 16'sd115,
        16'sd57, 16'sd29, 16'sd14, 16'sd7,
        16'sd4, 16'sd2, 16'sd1, 16'sd0
    };
    localparam GAIN_COMP = 16'sd39797;

    localparam IDLE = 2'b00, ROTATE = 2'b01, SCALE = 2'b10, DONE = 2'b11;
    reg [1:0] state;
    reg [4:0] iter;
    reg signed [WIDTH+1:0] x, y, z;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            iter <= 5'd0;
            valid <= 1'b0;
            angle_out <= 16'sd0;
            mag_out <= 16'sd0;
        end else begin
            case (state)
                IDLE: begin
                    valid <= 1'b0;
                    if (start) begin
                        if (x_in >= 0 && y_in >= 0) begin x <= x_in; y <= y_in; z <= 16'sd0; end
                        else if (x_in < 0 && y_in >= 0) begin x <= -x_in; y <= y_in; z <= 16'sd11520; end
                        else if (x_in < 0 && y_in < 0) begin x <= -x_in; y <= -y_in; z <= -16'sd11520; end
                        else begin x <= x_in; y <= -y_in; z <= 16'sd23040; end
                        iter <= 5'd0;
                        state <= ROTATE;
                    end
                end
                ROTATE: begin
                    if (y > 0) begin
                        x <= x + (y >>> iter);
                        y <= y - (x >>> iter);
                        z <= z + CORDIC_ANGLES[iter];
                    end else begin
                        x <= x - (y >>> iter);
                        y <= y + (x >>> iter);
                        z <= z - CORDIC_ANGLES[iter];
                    end
                    if (iter == ITERATIONS-1)
                        state <= SCALE;
                    else
                        iter <= iter + 1'b1;
                end
                SCALE: begin
                    mag_out <= (x * GAIN_COMP) >>> 16;
                    angle_out <= z;
                    state <= DONE;
                end
                DONE: begin
                    valid <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
