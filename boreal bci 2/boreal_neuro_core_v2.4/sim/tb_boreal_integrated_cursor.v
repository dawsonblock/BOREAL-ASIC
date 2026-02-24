`timescale 1ns / 1ps

module tb_boreal_integrated_cursor;

    // Inputs
    reg clk;
    reg rst_n;
    reg uart_rx; // Unused 
    reg emergency_halt_n;
    reg [7:0] rx_byte_in;
    reg       rx_valid_in;

    // Outputs
    wire signed [15:0] cursor_x;
    wire signed [15:0] cursor_y;
    wire cursor_valid;

    // Instantiate UUT
    boreal_cursor_controller uut (
        .clk(clk), 
        .rst_n(rst_n), 
        .uart_rx(uart_rx), 
        .emergency_halt_n(emergency_halt_n), 
        .rx_byte_in(rx_byte_in),
        .rx_valid_in(rx_valid_in),
        .cursor_x(cursor_x), 
        .cursor_y(cursor_y), 
        .cursor_valid(cursor_valid)
    );

    // 100MHz Clock
    always #5 clk = ~clk;

    // Simulate 30-byte UART injection
    task simulate_uart_packet;
        input [15:0] ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13;
        begin
            @(posedge clk);
            rx_valid_in = 1;
            
            rx_byte_in = 8'hAA; @(posedge clk);
            rx_byte_in = ch0[15:8]; @(posedge clk); rx_byte_in = ch0[7:0]; @(posedge clk);
            rx_byte_in = ch1[15:8]; @(posedge clk); rx_byte_in = ch1[7:0]; @(posedge clk);
            rx_byte_in = ch2[15:8]; @(posedge clk); rx_byte_in = ch2[7:0]; @(posedge clk);
            rx_byte_in = ch3[15:8]; @(posedge clk); rx_byte_in = ch3[7:0]; @(posedge clk);
            rx_byte_in = ch4[15:8]; @(posedge clk); rx_byte_in = ch4[7:0]; @(posedge clk);
            rx_byte_in = ch5[15:8]; @(posedge clk); rx_byte_in = ch5[7:0]; @(posedge clk);
            rx_byte_in = ch6[15:8]; @(posedge clk); rx_byte_in = ch6[7:0]; @(posedge clk);
            rx_byte_in = ch7[15:8]; @(posedge clk); rx_byte_in = ch7[7:0]; @(posedge clk);
            rx_byte_in = ch8[15:8]; @(posedge clk); rx_byte_in = ch8[7:0]; @(posedge clk);
            rx_byte_in = ch9[15:8]; @(posedge clk); rx_byte_in = ch9[7:0]; @(posedge clk);
            rx_byte_in = ch10[15:8]; @(posedge clk); rx_byte_in = ch10[7:0]; @(posedge clk);
            rx_byte_in = ch11[15:8]; @(posedge clk); rx_byte_in = ch11[7:0]; @(posedge clk);
            rx_byte_in = ch12[15:8]; @(posedge clk); rx_byte_in = ch12[7:0]; @(posedge clk);
            rx_byte_in = ch13[15:8]; @(posedge clk); rx_byte_in = ch13[7:0]; @(posedge clk);
            rx_byte_in = 8'h55; @(posedge clk);
            
            rx_valid_in = 0;
            
            // Wait for pipeline deep enough constraints (CSP + Kalman + Active Inference)
            #200; 
        end
    endtask

    initial begin
        clk = 0;
        rst_n = 0;
        uart_rx = 1;
        emergency_halt_n = 1;
        rx_valid_in = 0;
        rx_byte_in = 0;

        $dumpfile("sim/boreal_integrated.vcd");
        $dumpvars(0, tb_boreal_integrated_cursor);

        #100;
        rst_n = 1;
        #100;

        $display("=== STARTING INTEGRATED BOREAL ARCHITECTURE SIMULATION ===");

        // Test 1: Injection triggering CSP Comp 0 (X positive gradient mapped)
        // With default dummy weights: W_0 weights ch0, ch1, ch2, ch3 positively.
        // Kalman filter will initially smooth this, so it takes a few packets to ramp up.
        $display("--- Test 1: Positive X Iterations ---");
        // We pulse multiple packets to see the Kalman state vector ramp up the value
        simulate_uart_packet(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        $display("Packet 1 -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);
        simulate_uart_packet(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        $display("Packet 2 -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);
        simulate_uart_packet(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        $display("Packet 3 -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);
        simulate_uart_packet(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        $display("Packet 4 -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);

        // Test 2: Suddenly switch to Y negative gradient (CSP Comp 3 weights)
        $display("--- Test 2: Switch to Y Negative Iterations ---");
        simulate_uart_packet(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000);
        $display("Packet 5 -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);
        simulate_uart_packet(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1000, 1000, 1000, 1000);
        $display("Packet 6 -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);

        // Test 3: Emergency Stop Test
        $display("--- Test 3: Hardware Override ---");
        #50;
        emergency_halt_n = 0;
        #20;
        $display("Triggered -> Cursor X: %d, Cursor Y: %d", cursor_x, cursor_y);
        emergency_halt_n = 1;

        $display("=== SIMULATION COMPLETE ===");
        $finish;
    end
      
endmodule
