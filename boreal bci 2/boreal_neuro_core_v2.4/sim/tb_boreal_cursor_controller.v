`timescale 1ns / 1ps

module tb_boreal_cursor_controller;

    // Inputs
    reg clk;
    reg rst_n;
    reg uart_rx; // Unused in this direct test
    reg emergency_halt_n;
    reg [7:0] rx_byte_in;
    reg       rx_valid_in;
    reg signed [15:0] lms_error_x;
    reg signed [15:0] lms_error_y;

    // Outputs
    wire signed [15:0] cursor_x;
    wire signed [15:0] cursor_y;
    wire cursor_valid;
    wire [3:0] symbol_trigger;
    wire symbol_valid;

    // Instantiate the Unit Under Test (UUT)
    boreal_cursor_controller uut (
        .clk(clk), 
        .rst_n(rst_n), 
        .uart_rx(uart_rx), 
        .emergency_halt_n(emergency_halt_n), 
        .rx_byte_in(rx_byte_in),
        .rx_valid_in(rx_valid_in),
        .lms_error_x(lms_error_x),
        .lms_error_y(lms_error_y),
        .cursor_x(cursor_x), 
        .cursor_y(cursor_y), 
        .cursor_valid(cursor_valid),
        .symbol_trigger(symbol_trigger),
        .symbol_valid(symbol_valid)
    );

    // Clock generation (100MHz)
    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (symbol_valid && symbol_trigger != 0) begin
            $display("[PULSE CAUGHT] Symbol Trigger Fired: %b at time %0t. Final Cursor: (%d, %d)", symbol_trigger, $time, cursor_x, cursor_y);
        end
    end

    // Task to simulate receiving a valid 30-byte payload from the UART
    task simulate_uart_packet;
        input [15:0] ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13;
        begin
            @(posedge clk);
            rx_valid_in = 1;
            
            // Header
            rx_byte_in = 8'hAA; @(posedge clk);
            
            // CH 0
            rx_byte_in = ch0[15:8]; @(posedge clk);
            rx_byte_in = ch0[7:0]; @(posedge clk);
            // CH 1
            rx_byte_in = ch1[15:8]; @(posedge clk);
            rx_byte_in = ch1[7:0]; @(posedge clk);
            // CH 2
            rx_byte_in = ch2[15:8]; @(posedge clk);
            rx_byte_in = ch2[7:0]; @(posedge clk);
            // CH 3
            rx_byte_in = ch3[15:8]; @(posedge clk);
            rx_byte_in = ch3[7:0]; @(posedge clk);
            // CH 4
            rx_byte_in = ch4[15:8]; @(posedge clk);
            rx_byte_in = ch4[7:0]; @(posedge clk);
            // CH 5
            rx_byte_in = ch5[15:8]; @(posedge clk);
            rx_byte_in = ch5[7:0]; @(posedge clk);
            // CH 6
            rx_byte_in = ch6[15:8]; @(posedge clk);
            rx_byte_in = ch6[7:0]; @(posedge clk);
            // CH 7
            rx_byte_in = ch7[15:8]; @(posedge clk);
            rx_byte_in = ch7[7:0]; @(posedge clk);
            // CH 8
            rx_byte_in = ch8[15:8]; @(posedge clk);
            rx_byte_in = ch8[7:0]; @(posedge clk);
            // CH 9
            rx_byte_in = ch9[15:8]; @(posedge clk);
            rx_byte_in = ch9[7:0]; @(posedge clk);
            // CH 10
            rx_byte_in = ch10[15:8]; @(posedge clk);
            rx_byte_in = ch10[7:0]; @(posedge clk);
            // CH 11
            rx_byte_in = ch11[15:8]; @(posedge clk);
            rx_byte_in = ch11[7:0]; @(posedge clk);
            // CH 12
            rx_byte_in = ch12[15:8]; @(posedge clk);
            rx_byte_in = ch12[7:0]; @(posedge clk);
            // CH 13
            rx_byte_in = ch13[15:8]; @(posedge clk);
            rx_byte_in = ch13[7:0]; @(posedge clk);

            // Footer
            rx_byte_in = 8'h55; @(posedge clk);
            
            rx_valid_in = 0;
            
            // Wait for full pipeline computation
            #150;
        end
    endtask

    integer i;

    initial begin
        // Initialize Inputs
        clk = 0;
        rst_n = 0;
        uart_rx = 1;
        emergency_halt_n = 1;
        rx_valid_in = 0;
        rx_byte_in = 0;
        lms_error_x = 0;
        lms_error_y = 0;
        
        $dumpfile("sim/boreal_cursor_controller.vcd");
        $dumpvars(0, tb_boreal_cursor_controller);

        // Reset the system
        #100;
        rst_n = 1;
        #100;

        $display("=== STARTING FULL END-TO-END PIPELINE SIMULATION ===");

        // Goal: Move Cursor X right until it triggers
        // We simulate a strong signal on CH0 (which maps positively in CSP Component 0)
        // We apply a constant positive LMS Error to train the cursor to move right
        lms_error_x = 30000;
        lms_error_y = 0;
        
        $display("--- Test 1: Staring Right (Training Iterations) ---");
        for (i = 0; i < 400; i = i + 1) begin
            simulate_uart_packet(15000, 15000, 15000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            if (i % 50 == 0) $display("Packet %0d -> Cursor X: %d, Cursor Y: %d", i, cursor_x, cursor_y);
        end

        // By Packet 400, the cursor should have crossed 10000 and dwelled enough to trigger RIGHT (0001).

        $display("--- Test 2: Staring Up ---");
        lms_error_x = 0;
        lms_error_y = 30000; // Positive Y Error
        for (i = 0; i < 400; i = i + 1) begin
            simulate_uart_packet(0, 0, 0, 0, 0, 15000, 15000, 15000, 0, 0, 0, 8000, 8000, 0);
            if (i % 50 == 0) $display("Packet %0d -> Cursor X: %d, Cursor Y: %d", i+400, cursor_x, cursor_y);
        end

        $display("--- Test 3: Hardware Kill Switch Override ---");
        #50;
        emergency_halt_n = 0;
        #20;
        $display("Bite Switch Triggered -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);
        emergency_halt_n = 1;
        #20;

        $display("=== SIMULATION COMPLETE ===");
        $finish;
    end
      
endmodule
