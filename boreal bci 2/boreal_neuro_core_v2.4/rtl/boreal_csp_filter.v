/*
 * Boreal BCI 14-Channel Common Spatial Pattern (CSP) Filter
 * 
 * Purpose:
 * Projects 14 channels of raw spatial EEG data into an N-dimensional 
 * discriminative subspace (default N=4 components).
 * 
 * Pipeline:
 * Stage 1: Register Inputs
 * Stage 2: Parallel DSP Multipliers (Q1.15 Fixed Point)
 * Stage 3: Accumulator Tree
 * Stage 4: Register Outputs
 */

module boreal_csp_filter #(
    parameter NUM_CHANNELS = 14,
    parameter NUM_COMPONENTS = 4
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        data_valid_in,

    // 14 Channels of 16-bit Input Data
    input  wire signed [15:0] ch_in_0,
    input  wire signed [15:0] ch_in_1,
    input  wire signed [15:0] ch_in_2,
    input  wire signed [15:0] ch_in_3,
    input  wire signed [15:0] ch_in_4,
    input  wire signed [15:0] ch_in_5,
    input  wire signed [15:0] ch_in_6,
    input  wire signed [15:0] ch_in_7,
    input  wire signed [15:0] ch_in_8,
    input  wire signed [15:0] ch_in_9,
    input  wire signed [15:0] ch_in_10,
    input  wire signed [15:0] ch_in_11,
    input  wire signed [15:0] ch_in_12,
    input  wire signed [15:0] ch_in_13,

    // Projection Weights Matrix: W[NUM_COMPONENTS][NUM_CHANNELS]
    // In production, this can be mapped to block RAM or AXI registers
    // using continuous assignments or memory initialization.
    // For this module we assume they are provided as stable inputs contextually.
    
    // N discriminative output components
    output reg signed [15:0] comp_out_0,
    output reg signed [15:0] comp_out_1,
    output reg signed [15:0] comp_out_2,
    output reg signed [15:0] comp_out_3,
    output reg               data_valid_out
);

    // Hardcoded Example Weights (Q1.15 Format)
    // These would normally be loaded dynamically via SPI/AXI after Python training
    // Format: W[component][channel]
    // 0x7FFF = ~0.999, 0x0000 = 0.0, 0x8000 = -1.0
    wire signed [15:0] W_0 [0:13];
    wire signed [15:0] W_1 [0:13];
    wire signed [15:0] W_2 [0:13];
    wire signed [15:0] W_3 [0:13];

    // Dummy Weights for Compilation (Identity/Combinatorial approximations)
    assign W_0[0] = 16'h7FFF; assign W_0[1] = 16'h4000; assign W_0[2] = 16'h2000; assign W_0[3] = 16'h1000; assign W_0[4] = 16'h0000; assign W_0[5] = 16'h0000; assign W_0[6] = 16'h0000; assign W_0[7] = 16'h0000; assign W_0[8] = 16'h0000; assign W_0[9] = 16'h0000; assign W_0[10] = 16'h0000; assign W_0[11] = 16'h0000; assign W_0[12] = 16'h0000; assign W_0[13] = 16'h8000;
    assign W_1[0] = 16'h0000; assign W_1[1] = 16'h0000; assign W_1[2] = 16'h0000; assign W_1[3] = 16'h0000; assign W_1[4] = 16'h7FFF; assign W_1[5] = 16'h4000; assign W_1[6] = 16'h2000; assign W_1[7] = 16'h0000; assign W_1[8] = 16'h0000; assign W_1[9] = 16'h0000; assign W_1[10] = 16'h0000; assign W_1[11] = 16'h0000; assign W_1[12] = 16'h8000; assign W_1[13] = 16'h8000;
    assign W_2[0] = 16'h0000; assign W_2[1] = 16'h8000; assign W_2[2] = 16'h0000; assign W_2[3] = 16'h0000; assign W_2[4] = 16'h0000; assign W_2[5] = 16'h0000; assign W_2[6] = 16'h0000; assign W_2[7] = 16'h7FFF; assign W_2[8] = 16'h4000; assign W_2[9] = 16'h2000; assign W_2[10] = 16'h0000; assign W_2[11] = 16'h0000; assign W_2[12] = 16'h0000; assign W_2[13] = 16'h0000;
    assign W_3[0] = 16'h8000; assign W_3[1] = 16'h8000; assign W_3[2] = 16'h8000; assign W_3[3] = 16'h0000; assign W_3[4] = 16'h0000; assign W_3[5] = 16'h0000; assign W_3[6] = 16'h0000; assign W_3[7] = 16'h0000; assign W_3[8] = 16'h0000; assign W_3[9] = 16'h0000; assign W_3[10] = 16'h7FFF; assign W_3[11] = 16'h4000; assign W_3[12] = 16'h2000; assign W_3[13] = 16'h1000;

    // ------------------------------------------------------------------
    // STAGE 1: Input Registering (Isolate from cross-module timing paths)
    // ------------------------------------------------------------------
    reg signed [15:0] reg_ch_in [0:13];
    reg               reg_valid_stg1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_valid_stg1 <= 0;
            for (integer i=0; i<14; i=i+1) reg_ch_in[i] <= 0;
        end else begin
            reg_valid_stg1 <= data_valid_in;
            if (data_valid_in) begin
                reg_ch_in[0]  <= ch_in_0;
                reg_ch_in[1]  <= ch_in_1;
                reg_ch_in[2]  <= ch_in_2;
                reg_ch_in[3]  <= ch_in_3;
                reg_ch_in[4]  <= ch_in_4;
                reg_ch_in[5]  <= ch_in_5;
                reg_ch_in[6]  <= ch_in_6;
                reg_ch_in[7]  <= ch_in_7;
                reg_ch_in[8]  <= ch_in_8;
                reg_ch_in[9]  <= ch_in_9;
                reg_ch_in[10] <= ch_in_10;
                reg_ch_in[11] <= ch_in_11;
                reg_ch_in[12] <= ch_in_12;
                reg_ch_in[13] <= ch_in_13;
            end
        end
    end

    // ------------------------------------------------------------------
    // STAGE 2: Multipliers 
    // Q1.15 * Q1.15 = Q2.30 -> Shift back to Q1.15
    // ------------------------------------------------------------------
    reg signed [31:0] mult_0 [0:13];
    reg signed [31:0] mult_1 [0:13];
    reg signed [31:0] mult_2 [0:13];
    reg signed [31:0] mult_3 [0:13];
    reg               reg_valid_stg2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_valid_stg2 <= 0;
            for (integer i=0; i<14; i=i+1) begin
                mult_0[i] <= 0; mult_1[i] <= 0; mult_2[i] <= 0; mult_3[i] <= 0;
            end
        end else begin
            reg_valid_stg2 <= reg_valid_stg1;
            if (reg_valid_stg1) begin
                for (integer i=0; i<14; i=i+1) begin
                    mult_0[i] <= reg_ch_in[i] * W_0[i];
                    mult_1[i] <= reg_ch_in[i] * W_1[i];
                    mult_2[i] <= reg_ch_in[i] * W_2[i];
                    mult_3[i] <= reg_ch_in[i] * W_3[i];
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // STAGE 3 & 4: Accumulation & Output Normalization
    // ------------------------------------------------------------------
    reg signed [31:0] acc_0, acc_1, acc_2, acc_3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_valid_out <= 0;
            comp_out_0 <= 0;
            comp_out_1 <= 0;
            comp_out_2 <= 0;
            comp_out_3 <= 0;
        end else begin
            data_valid_out <= reg_valid_stg2;
            if (reg_valid_stg2) begin
                
                // Note: In an unrolled accumulator loop for latency priority
                acc_0 = mult_0[0] + mult_0[1] + mult_0[2] + mult_0[3] + mult_0[4] + mult_0[5] + mult_0[6] + 
                        mult_0[7] + mult_0[8] + mult_0[9] + mult_0[10] + mult_0[11] + mult_0[12] + mult_0[13];
                        
                acc_1 = mult_1[0] + mult_1[1] + mult_1[2] + mult_1[3] + mult_1[4] + mult_1[5] + mult_1[6] + 
                        mult_1[7] + mult_1[8] + mult_1[9] + mult_1[10] + mult_1[11] + mult_1[12] + mult_1[13];
                        
                acc_2 = mult_2[0] + mult_2[1] + mult_2[2] + mult_2[3] + mult_2[4] + mult_2[5] + mult_2[6] + 
                        mult_2[7] + mult_2[8] + mult_2[9] + mult_2[10] + mult_2[11] + mult_2[12] + mult_2[13];
                        
                acc_3 = mult_3[0] + mult_3[1] + mult_3[2] + mult_3[3] + mult_3[4] + mult_3[5] + mult_3[6] + 
                        mult_3[7] + mult_3[8] + mult_3[9] + mult_3[10] + mult_3[11] + mult_3[12] + mult_3[13];

                // Shift from Q2.30 accumulator back down to Q1.15
                // And simple saturation check
                
                // Comp 0
                if ((acc_0 >>> 15) > 32767) comp_out_0 <= 32767;
                else if ((acc_0 >>> 15) < -32768) comp_out_0 <= -32768;
                else comp_out_0 <= (acc_0 >>> 15);
                
                // Comp 1
                if ((acc_1 >>> 15) > 32767) comp_out_1 <= 32767;
                else if ((acc_1 >>> 15) < -32768) comp_out_1 <= -32768;
                else comp_out_1 <= (acc_1 >>> 15);

                // Comp 2
                if ((acc_2 >>> 15) > 32767) comp_out_2 <= 32767;
                else if ((acc_2 >>> 15) < -32768) comp_out_2 <= -32768;
                else comp_out_2 <= (acc_2 >>> 15);

                // Comp 3
                if ((acc_3 >>> 15) > 32767) comp_out_3 <= 32767;
                else if ((acc_3 >>> 15) < -32768) comp_out_3 <= -32768;
                else comp_out_3 <= (acc_3 >>> 15);

            end
        end
    end

endmodule
