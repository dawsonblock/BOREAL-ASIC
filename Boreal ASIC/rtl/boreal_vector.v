// ============================================================================
// Boreal SoC - Vector Engine (4-lane SIMD)
// ============================================================================
// MMIO-controlled vector processing unit with 4 parallel INT8 MAC lanes.
// Reads operands from SRAM, executes MAC / scale / clamp, writes results
// back to SRAM.  Supports matrix multiply (M x K) * (K x N).
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module boreal_vector #(
    parameter NUM_LANES = 4
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- MMIO slave interface ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- SRAM read port ---
    output reg         sram_rd_req,
    output reg  [31:0] sram_rd_addr,
    input  wire [31:0] sram_rd_data,
    input  wire        sram_rd_ack,

    // --- SRAM write port ---
    output reg         sram_wr_req,
    output reg  [31:0] sram_wr_addr,
    output reg  [31:0] sram_wr_data,
    input  wire        sram_wr_ack
);

    // -----------------------------------------------------------------------
    // MMIO registers
    // -----------------------------------------------------------------------
    localparam OFF_CMD    = 8'h00;
    localparam OFF_SRC    = 8'h04;
    localparam OFF_DST    = 8'h08;
    localparam OFF_LEN    = 8'h0C;
    localparam OFF_SCALE  = 8'h10;
    localparam OFF_ZERO   = 8'h14;
    localparam OFF_M      = 8'h18;
    localparam OFF_N      = 8'h1C;
    localparam OFF_K      = 8'h20;
    localparam OFF_STATUS = 8'h24;

    wire [7:0] reg_off = addr[7:0];

    reg [31:0] r_cmd, r_src, r_dst, r_len;
    reg [31:0] r_scale, r_zero;
    reg [31:0] r_m, r_n, r_k;
    reg        busy, done_flag, error_flag;

    // MMIO write
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_cmd   <= 32'h0;
            r_src   <= 32'h0;
            r_dst   <= 32'h0;
            r_len   <= 32'h0;
            r_scale <= 32'h0;
            r_zero  <= 32'h0;
            r_m     <= 32'h0;
            r_n     <= 32'h0;
            r_k     <= 32'h0;
        end else if (sel && wr) begin
            case (reg_off)
                OFF_CMD:   r_cmd   <= wdata;
                OFF_SRC:   r_src   <= wdata;
                OFF_DST:   r_dst   <= wdata;
                OFF_LEN:   r_len   <= wdata;
                OFF_SCALE: r_scale <= wdata;
                OFF_ZERO:  r_zero  <= wdata;
                OFF_M:     r_m     <= wdata;
                OFF_N:     r_n     <= wdata;
                OFF_K:     r_k     <= wdata;
                default: ;
            endcase
        end
    end

    // MMIO read
    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr) begin
            case (reg_off)
                OFF_CMD:    rdata = r_cmd;
                OFF_SRC:    rdata = r_src;
                OFF_DST:    rdata = r_dst;
                OFF_LEN:    rdata = r_len;
                OFF_SCALE:  rdata = r_scale;
                OFF_ZERO:   rdata = r_zero;
                OFF_M:      rdata = r_m;
                OFF_N:      rdata = r_n;
                OFF_K:      rdata = r_k;
                OFF_STATUS: rdata = {29'b0, error_flag, done_flag, busy};
                default:    rdata = 32'h0;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Vector lanes
    // -----------------------------------------------------------------------
    reg         lane_en;
    reg  [ 2:0] lane_op;
    reg  [ 7:0] lane_a [0:NUM_LANES-1];
    reg  [ 7:0] lane_b [0:NUM_LANES-1];
    wire [31:0] lane_acc [0:NUM_LANES-1];
    wire        lane_done [0:NUM_LANES-1];

    genvar g;
    generate
        for (g = 0; g < NUM_LANES; g = g + 1) begin : LANES
            boreal_vec_lane u_lane (
                .clk       (clk),
                .rst_n     (rst_n),
                .en        (lane_en),
                .op        (lane_op),
                .a         (lane_a[g]),
                .b         (lane_b[g]),
                .scale     (r_scale[15:0]),
                .zero_pt   (r_zero[15:0]),
                .clamp_min (32'hFFFF_FF80),  // -128
                .clamp_max (32'h0000_007F),  //  127
                .acc       (lane_acc[g]),
                .done      (lane_done[g])
            );
        end
    endgenerate

    // -----------------------------------------------------------------------
    // Engine FSM – simple element-wise MAC over r_len words
    // -----------------------------------------------------------------------
    localparam VS_IDLE     = 3'd0;
    localparam VS_ZERO     = 3'd1;
    localparam VS_RD_A     = 3'd2;
    localparam VS_RD_B     = 3'd3;
    localparam VS_MAC      = 3'd4;
    localparam VS_SCALE    = 3'd5;
    localparam VS_WRITE    = 3'd6;
    localparam VS_DONE     = 3'd7;

    reg [2:0]  vs_state;
    reg [31:0] elem_idx;
    reg [31:0] rd_a_data, rd_b_data;

    wire start_cmd = sel && wr && (reg_off == OFF_CMD) && wdata[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_state    <= VS_IDLE;
            busy        <= 1'b0;
            done_flag   <= 1'b0;
            error_flag  <= 1'b0;
            lane_en     <= 1'b0;
            lane_op     <= 3'd0;
            elem_idx    <= 32'd0;
            sram_rd_req <= 1'b0;
            sram_rd_addr<= 32'd0;
            sram_wr_req <= 1'b0;
            sram_wr_addr<= 32'd0;
            sram_wr_data<= 32'd0;
            rd_a_data   <= 32'd0;
            rd_b_data   <= 32'd0;
            lane_a[0] <= 8'd0; lane_a[1] <= 8'd0; lane_a[2] <= 8'd0; lane_a[3] <= 8'd0;
            lane_b[0] <= 8'd0; lane_b[1] <= 8'd0; lane_b[2] <= 8'd0; lane_b[3] <= 8'd0;
        end else begin
            case (vs_state)
                VS_IDLE: begin
                    lane_en     <= 1'b0;
                    sram_rd_req <= 1'b0;
                    sram_wr_req <= 1'b0;
                    if (start_cmd) begin
                        busy       <= 1'b1;
                        done_flag  <= 1'b0;
                        error_flag <= 1'b0;
                        elem_idx   <= 32'd0;
                        vs_state   <= VS_ZERO;
                    end
                end

                // Zero accumulators
                VS_ZERO: begin
                    lane_en  <= 1'b1;
                    lane_op  <= 3'd5;  // ZERO_ACC
                    vs_state <= VS_RD_A;
                end

                // Read source A word (4 packed int8 elements)
                VS_RD_A: begin
                    lane_en <= 1'b0;
                    if (elem_idx >= r_len) begin
                        vs_state <= VS_SCALE;
                    end else begin
                        sram_rd_req  <= 1'b1;
                        sram_rd_addr <= r_src + {elem_idx[29:0], 2'b00};
                        if (sram_rd_ack) begin
                            rd_a_data   <= sram_rd_data;
                            sram_rd_req <= 1'b0;
                            vs_state    <= VS_RD_B;
                        end
                    end
                end

                // Read source B word
                VS_RD_B: begin
                    sram_rd_req  <= 1'b1;
                    sram_rd_addr <= r_dst + {elem_idx[29:0], 2'b00};
                    if (sram_rd_ack) begin
                        rd_b_data   <= sram_rd_data;
                        sram_rd_req <= 1'b0;
                        vs_state    <= VS_MAC;
                    end
                end

                // MAC: each lane gets one byte pair
                VS_MAC: begin
                    lane_a[0] <= rd_a_data[ 7: 0];
                    lane_a[1] <= rd_a_data[15: 8];
                    lane_a[2] <= rd_a_data[23:16];
                    lane_a[3] <= rd_a_data[31:24];
                    lane_b[0] <= rd_b_data[ 7: 0];
                    lane_b[1] <= rd_b_data[15: 8];
                    lane_b[2] <= rd_b_data[23:16];
                    lane_b[3] <= rd_b_data[31:24];
                    lane_en   <= 1'b1;
                    lane_op   <= 3'd1;  // MAC
                    elem_idx  <= elem_idx + 1;
                    vs_state  <= VS_RD_A;
                end

                // Scale + requantise
                VS_SCALE: begin
                    lane_en  <= 1'b1;
                    lane_op  <= 3'd2;  // SCALE
                    vs_state <= VS_WRITE;
                end

                // Write packed result to destination
                VS_WRITE: begin
                    lane_en      <= 1'b0;
                    sram_wr_req  <= 1'b1;
                    sram_wr_addr <= r_dst;
                    sram_wr_data <= {lane_acc[3][7:0], lane_acc[2][7:0],
                                     lane_acc[1][7:0], lane_acc[0][7:0]};
                    if (sram_wr_ack) begin
                        sram_wr_req <= 1'b0;
                        vs_state    <= VS_DONE;
                    end
                end

                VS_DONE: begin
                    busy      <= 1'b0;
                    done_flag <= 1'b1;
                    vs_state  <= VS_IDLE;
                end

                default: vs_state <= VS_IDLE;
            endcase
        end
    end

endmodule
