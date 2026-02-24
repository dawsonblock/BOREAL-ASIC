`timescale 1ns/1ps
module tb_boreal_apex_core;
    parameter CLK_PERIOD = 10;
    reg clk, rst_n, emergency_halt_n, adc_data_ready;
    reg [23:0] raw_adc_in;
    reg [31:0] lut_data;
    wire [9:0] lut_addr;
    wire signed [15:0] mu_out, epsilon_out;
    wire valid_out, saturation_flag;

    boreal_apex_core uut (
        .clk(clk), .rst_n(rst_n), .emergency_halt_n(emergency_halt_n),
        .raw_adc_in(raw_adc_in), .adc_channel_sel(3'd0), .adc_data_ready(adc_data_ready),
        .lut_addr(lut_addr), .lut_data(lut_data),
        .mu_out(mu_out), .epsilon_out(epsilon_out),
        .valid_out(valid_out), .saturation_flag(saturation_flag)
    );

    initial begin clk = 0; forever #(CLK_PERIOD/2) clk = ~clk; end
    initial begin
        $display("Starting Boreal Apex Core Test");
        rst_n = 0; emergency_halt_n = 1; raw_adc_in = 24'd0; adc_data_ready = 0; lut_data = {16'h2000, 16'h4000};
        #(CLK_PERIOD * 10); rst_n = 1; #(CLK_PERIOD * 10);

        $display("Test 1: Basic operation");
        raw_adc_in = 24'd1000000; adc_data_ready = 1; #(CLK_PERIOD); adc_data_ready = 0; #(CLK_PERIOD * 5);
        $display("Mu=%d Epsilon=%d", mu_out, epsilon_out);

        $display("Test 2: Emergency halt");
        emergency_halt_n = 0; #(CLK_PERIOD * 5);
        $display("After halt: Mu=%d (should be 0)", mu_out);

        #(CLK_PERIOD * 100);
        $display("Tests complete");
        $finish;
    end
    initial begin $dumpfile("boreal_apex_core.vcd"); $dumpvars(0, tb_boreal_apex_core); end
endmodule
