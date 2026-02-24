/*
 * Boreal BCI 2D Cursor Controller
 * Core Module for 14-Channel Neural Decoding to 2D Manifold Space
 * Features:
 * 1. UART Data Ingestion (Sync Bytes 0xAA/0x55)
 * 2. Hardware 14-to-2 Manifold Mapping (Active Inference principle)
 * 3. Saturation Arithmetic for Cursor Bounds Guardrails
 * 4. Temporal Predictive Coding (Look-ahead velocity projection)
 * 5. Common Spatial Pattern (CSP) Filter Projection
 * 6. Kalman Alpha-Beta State Estimation
 */

module boreal_cursor_controller (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    input  wire        emergency_halt_n, // Physical Bite-Switch Override // Global Enable

    input  wire [7:0]  rx_byte_in,
    input  wire        rx_valid_in,

    // LMS Adaptive Feedback (External Error Source)
    input  wire signed [15:0] lms_error_x,
    input  wire signed [15:0] lms_error_y,

    // Processed 2D Cursor Output
    output reg signed [15:0] cursor_x,
    output reg signed [15:0] cursor_y,
    output reg               cursor_valid,

    // Discrete Symbolic Mapping Output
    output wire [3:0]        symbol_trigger,
    output wire              symbol_valid
);

    // ------------------------------------------------------------------
    // 1. UART RECEIVER & PARSER
    // Standard 115200 Baud (Assuming 100MHz System Clock)
    // ------------------------------------------------------------------
    wire [7:0] rx_byte = rx_byte_in;
    wire       rx_valid = rx_valid_in;
    reg [4:0]  byte_idx;
    reg [7:0]  packet_buffer [0:29]; // 30-byte packet buffer 
    reg [15:0] ch_data [0:13];       // 14 Channels of 16-bit EEG data
    reg        inference_trigger;

    // Simple placeholder instantiation representing a UART RX block
    // (Assuming standard instantiation from the Boreal suite if rx_valid_in was not directly driven)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_idx <= 0;
            inference_trigger <= 0;
            for (integer i=0; i<14; i=i+1) ch_data[i] <= 0;
        end else if (rx_valid) begin
            // 0xAA is Top Sync Byte
            if (rx_byte == 8'hAA && (byte_idx == 0 || byte_idx == 30)) begin
                packet_buffer[0] <= rx_byte;
                byte_idx <= 1;
            end else if (byte_idx > 0 && byte_idx < 30) begin
                packet_buffer[byte_idx] <= rx_byte;
                byte_idx <= byte_idx + 1;
                
                // 0x55 is End Sync Byte
                if (byte_idx == 29 && rx_byte == 8'h55) begin
                    // Packet Complete. Assemble 14 Channels.
                    for (integer i=0; i<14; i=i+1) begin
                        ch_data[i] <= {packet_buffer[1 + i*2], packet_buffer[2 + i*2]};
                    end
                    inference_trigger <= 1;
                    byte_idx <= 30;
                end
            end
        end else begin
            inference_trigger <= 0; // Strobe to 0
        end
    end

    // ------------------------------------------------------------------
    // 2. COMMON SPATIAL PATTERN (CSP) FILTER
    // 14 Channels -> 4 Independent Components
    // ------------------------------------------------------------------
    wire signed [15:0] csp_comp_0;
    wire signed [15:0] csp_comp_1;
    wire signed [15:0] csp_comp_2;
    wire signed [15:0] csp_comp_3;
    wire csp_valid;

    boreal_csp_filter #(
        .NUM_CHANNELS(14),
        .NUM_COMPONENTS(4)
    ) csp_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_valid_in(inference_trigger),
        .ch_in_0(ch_data[0]),
        .ch_in_1(ch_data[1]),
        .ch_in_2(ch_data[2]),
        .ch_in_3(ch_data[3]),
        .ch_in_4(ch_data[4]),
        .ch_in_5(ch_data[5]),
        .ch_in_6(ch_data[6]),
        .ch_in_7(ch_data[7]),
        .ch_in_8(ch_data[8]),
        .ch_in_9(ch_data[9]),
        .ch_in_10(ch_data[10]),
        .ch_in_11(ch_data[11]),
        .ch_in_12(ch_data[12]),
        .ch_in_13(ch_data[13]),
        .comp_out_0(csp_comp_0),
        .comp_out_1(csp_comp_1),
        .comp_out_2(csp_comp_2),
        .comp_out_3(csp_comp_3),
        .data_valid_out(csp_valid)
    );

    // ------------------------------------------------------------------
    // 3. KALMAN STATE ESTIMATION
    // Remove individual component jitter preserving kinetic layout
    // ------------------------------------------------------------------
    wire signed [15:0] kf_pos_0, kf_vel_0;
    wire signed [15:0] kf_pos_1, kf_vel_1;
    wire signed [15:0] kf_pos_2, kf_vel_2;
    wire signed [15:0] kf_pos_3, kf_vel_3;
    wire kf_valid_0;

    // Use established BCI smoothing characteristics: Alpha=0.2, Beta=0.01
    localparam signed [15:0] KALMAN_ALPHA = 16'h1999;
    localparam signed [15:0] KALMAN_BETA  = 16'h0147;

    boreal_kalman_filter kf_inst_0 (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .measurement_in(csp_comp_0), .valid_in(csp_valid),
        .alpha_gain(KALMAN_ALPHA), .beta_gain(KALMAN_BETA),
        .position_out(kf_pos_0), .velocity_out(kf_vel_0), .valid_out(kf_valid_0)
    );

    boreal_kalman_filter kf_inst_1 (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .measurement_in(csp_comp_1), .valid_in(csp_valid),
        .alpha_gain(KALMAN_ALPHA), .beta_gain(KALMAN_BETA),
        .position_out(kf_pos_1), .velocity_out(kf_vel_1), .valid_out()
    );

    boreal_kalman_filter kf_inst_2 (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .measurement_in(csp_comp_2), .valid_in(csp_valid),
        .alpha_gain(KALMAN_ALPHA), .beta_gain(KALMAN_BETA),
        .position_out(kf_pos_2), .velocity_out(kf_vel_2), .valid_out()
    );

    boreal_kalman_filter kf_inst_3 (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .measurement_in(csp_comp_3), .valid_in(csp_valid),
        .alpha_gain(KALMAN_ALPHA), .beta_gain(KALMAN_BETA),
        .position_out(kf_pos_3), .velocity_out(kf_vel_3), .valid_out()
    );

    wire chain_valid = kf_valid_0; // Fully synchronized pipeline

    // ------------------------------------------------------------------
    // 4. LEAST MEAN SQUARES (LMS) ADAPTIVE DECODER
    // Real-time Learning module replacing static CSP spatial tracking
    // ------------------------------------------------------------------
    wire signed [15:0] lms_out_x;
    wire signed [15:0] lms_out_y;
    wire lms_valid_x, lms_valid_y;

    boreal_lms_decoder #(
        .NUM_COMPONENTS(4),
        .MU_SHIFT(4) 
    ) lms_x (
        .clk(clk),
        .rst_n(rst_n),
        .enable(1'b1),
        .valid_in(chain_valid),
        .comp_in_0(kf_pos_0),
        .comp_in_1(kf_pos_1),
        .comp_in_2(kf_pos_2),
        .comp_in_3(kf_pos_3),
        .error_in(lms_error_x),
        .state_out(lms_out_x),
        .valid_out(lms_valid_x)
    );

    boreal_lms_decoder #(
        .NUM_COMPONENTS(4),
        .MU_SHIFT(4)
    ) lms_y (
        .clk(clk),
        .rst_n(rst_n),
        .enable(1'b1),
        .valid_in(chain_valid),
        .comp_in_0(kf_pos_0),
        .comp_in_1(kf_pos_1),
        .comp_in_2(kf_pos_2),
        .comp_in_3(kf_pos_3),
        .error_in(lms_error_y),
        .state_out(lms_out_y),
        .valid_out(lms_valid_y)
    );

    // ------------------------------------------------------------------
    // 4. ACTIVE INFERENCE ENGINE & TEMPORAL CODING
    // ------------------------------------------------------------------
    reg signed [15:0] mu_x, mu_y;            
    reg signed [31:0] sum_x, sum_y;          

    localparam signed MAX_CURSOR = 16'h7FFF; // 32767
    localparam signed MIN_CURSOR = 16'h8000; // -32768
    localparam signed K_LEAD = 4; // Velocity multiplier 

    reg signed [31:0] next_mu_x;
    reg signed [31:0] next_mu_y;
    reg signed [15:0] velocity_x;
    reg signed [15:0] velocity_y;
    reg signed [31:0] predicted_x;
    reg signed [31:0] predicted_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mu_x <= 0;
            mu_y <= 0;
            cursor_valid <= 0;
            cursor_x <= 0;
            cursor_y <= 0;
        end else if (!emergency_halt_n) begin
            // Physical Override - Bite Switch zeros out cursor instantly
            mu_x <= 0;
            mu_y <= 0;
            cursor_x <= 0;
            cursor_y <= 0;
            cursor_valid <= 0;
        end else if (lms_valid_x && lms_valid_y) begin
            // Active Inference computes the gradient vector based on the 
            // dynamically learned state output directly from the LMS filter, 
            // replacing historical static differencing blocks.
            sum_x = lms_out_x;
            sum_y = lms_out_y;

            // Hardware Gradient Descent: mu = mu + eta*(Prediction Error)
            // eta is implemented as a fixed-point bit-shift (>> 5)
            // lambda (decay prior) is implemented as mu >> 4
            next_mu_x = mu_x + (sum_x >>> 5) - (mu_x >>> 4);
            next_mu_y = mu_y + (sum_y >>> 5) - (mu_y >>> 4);

            // Saturation Arithmetic to prevent catastrophic overflow artifacting
            if (next_mu_x > MAX_CURSOR) mu_x <= MAX_CURSOR[15:0];
            else if (next_mu_x < MIN_CURSOR) mu_x <= MIN_CURSOR[15:0];
            else mu_x <= next_mu_x[15:0];

            if (next_mu_y > MAX_CURSOR) mu_y <= MAX_CURSOR[15:0];
            else if (next_mu_y < MIN_CURSOR) mu_y <= MIN_CURSOR[15:0];
            else mu_y <= next_mu_y[15:0];

            // ------------------------------------------------------------------
            // TEMPORAL PREDICTIVE CODING
            // ------------------------------------------------------------------
            velocity_x = next_mu_x[15:0] - mu_x;
            velocity_y = next_mu_y[15:0] - mu_y;

            predicted_x = next_mu_x + (velocity_x * K_LEAD);
            predicted_y = next_mu_y + (velocity_y * K_LEAD);

            // Final Saturation Check before Outputting Forward-Projected Cursor
            if (predicted_x > MAX_CURSOR) cursor_x <= MAX_CURSOR[15:0];
            else if (predicted_x < MIN_CURSOR) cursor_x <= MIN_CURSOR[15:0];
            else cursor_x <= predicted_x[15:0];

            if (predicted_y > MAX_CURSOR) cursor_y <= MAX_CURSOR[15:0];
            else if (predicted_y < MIN_CURSOR) cursor_y <= MIN_CURSOR[15:0];
            else cursor_y <= predicted_y[15:0];

            cursor_valid <= 1;
        end else begin
            cursor_valid <= 0;
        end
    end

    // ------------------------------------------------------------------
    // 6. SYMBOLIC INTENT MAPPER 
    // Translates 2D analog cursor into RFSN-Compatible Tokens
    // ------------------------------------------------------------------
    boreal_symbolic_mapper #(
        .THRESHOLD_POS(16'd10000),
        .THRESHOLD_NEG(-16'd10000),
        .DWELL_TIME_CYCLES(10),
        .COOLDOWN_CYCLES(50)
    ) symbolic_mapper_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(1'b1),
        .valid_in(cursor_valid),
        .cursor_x(cursor_x),
        .cursor_y(cursor_y),
        .symbol_trigger(symbol_trigger),
        .valid_out(symbol_valid)
    );

endmodule
