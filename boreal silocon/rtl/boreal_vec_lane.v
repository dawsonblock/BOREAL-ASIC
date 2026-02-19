// ============================================================================
// Boreal SoC - Vector Lane (single SIMD lane)
// ============================================================================
// Supports INT8 multiply-accumulate, scale, and clamp operations.
// Each lane processes one element per cycle.
// ============================================================================

module boreal_vec_lane (
    input  wire        clk,
    input  wire        rst_n,

    // --- Control ---
    input  wire        en,
    input  wire [ 2:0] op,       // 0=NOP, 1=MAC, 2=SCALE, 3=CLAMP, 4=LOAD_ACC, 5=ZERO_ACC
    input  wire [ 7:0] a,        // operand A (int8)
    input  wire [ 7:0] b,        // operand B (int8)
    input  wire [15:0] scale,    // requantisation scale (fixed-point)
    input  wire [15:0] zero_pt,  // requantisation zero point
    input  wire [31:0] clamp_min,
    input  wire [31:0] clamp_max,

    // --- Output ---
    output reg  [31:0] acc,      // 32-bit accumulator
    output reg         done
);

    localparam OP_NOP      = 3'd0;
    localparam OP_MAC      = 3'd1;
    localparam OP_SCALE    = 3'd2;
    localparam OP_CLAMP    = 3'd3;
    localparam OP_LOAD_ACC = 3'd4;
    localparam OP_ZERO_ACC = 3'd5;

    // Signed extension of int8 operands
    wire signed [15:0] a_ext = {{8{a[7]}}, a};
    wire signed [15:0] b_ext = {{8{b[7]}}, b};
    wire signed [31:0] product = a_ext * b_ext;

    wire signed [31:0] acc_signed = acc;

    // Scaled result: (acc * scale) >> 16 + zero_point
    wire signed [63:0] scaled_full = acc_signed * $signed({1'b0, scale});
    wire signed [31:0] scaled_val  = scaled_full[47:16] + $signed({1'b0, zero_pt});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc  <= 32'd0;
            done <= 1'b0;
        end else if (en) begin
            done <= 1'b1;
            case (op)
                OP_NOP:      ;  // no-op
                OP_MAC:      acc <= acc + product;
                OP_SCALE:    acc <= scaled_val;
                OP_CLAMP: begin
                    if (acc_signed < $signed(clamp_min))
                        acc <= clamp_min;
                    else if (acc_signed > $signed(clamp_max))
                        acc <= clamp_max;
                end
                OP_LOAD_ACC: acc <= {24'b0, a};
                OP_ZERO_ACC: acc <= 32'd0;
                default:     ;
            endcase
        end else begin
            done <= 1'b0;
        end
    end

endmodule
