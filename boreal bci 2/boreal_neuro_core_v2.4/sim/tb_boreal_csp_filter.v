`timescale 1ns / 1ps

module tb_boreal_csp_filter;

    // Inputs
    reg clk;
    reg rst_n;
    reg data_valid_in;
    reg signed [15:0] ch_in [0:13];

    // Outputs
    wire signed [15:0] comp_out_0;
    wire signed [15:0] comp_out_1;
    wire signed [15:0] comp_out_2;
    wire signed [15:0] comp_out_3;
    wire data_valid_out;

    // Instantiate UUT
    boreal_csp_filter uut (
        .clk(clk),
        .rst_n(rst_n),
        .data_valid_in(data_valid_in),
        .ch_in_0(ch_in[0]),
        .ch_in_1(ch_in[1]),
        .ch_in_2(ch_in[2]),
        .ch_in_3(ch_in[3]),
        .ch_in_4(ch_in[4]),
        .ch_in_5(ch_in[5]),
        .ch_in_6(ch_in[6]),
        .ch_in_7(ch_in[7]),
        .ch_in_8(ch_in[8]),
        .ch_in_9(ch_in[9]),
        .ch_in_10(ch_in[10]),
        .ch_in_11(ch_in[11]),
        .ch_in_12(ch_in[12]),
        .ch_in_13(ch_in[13]),
        .comp_out_0(comp_out_0),
        .comp_out_1(comp_out_1),
        .comp_out_2(comp_out_2),
        .comp_out_3(comp_out_3),
        .data_valid_out(data_valid_out)
    );

    // Clock gen
    always #5 clk = ~clk;

    // Task to apply stimulus
    task apply_snapshot;
        input signed [15:0] in0, in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13;
        begin
            @(posedge clk);
            data_valid_in = 1;
            ch_in[0] = in0; ch_in[1] = in1; ch_in[2] = in2; ch_in[3] = in3;
            ch_in[4] = in4; ch_in[5] = in5; ch_in[6] = in6; ch_in[7] = in7;
            ch_in[8] = in8; ch_in[9] = in9; ch_in[10] = in10; ch_in[11] = in11;
            ch_in[12] = in12; ch_in[13] = in13;
            @(posedge clk);
            data_valid_in = 0;
            // Wait for pipeline depth (3 cycles minimum: register, mult, accum/shift)
            #30; 
        end
    endtask

    initial begin
        // Init
        clk = 0;
        rst_n = 0;
        data_valid_in = 0;
        for(integer i=0; i<14; i=i+1) ch_in[i] = 0;

        $dumpfile("sim/boreal_csp.vcd");
        $dumpvars(0, tb_boreal_csp_filter);

        // Reset
        #100;
        rst_n = 1;
        #10;

        $display("=== BOREAL CSP FILTER SIMULATION ===");
        
        // ---------------------------------------------------------
        // Test 1: Zeros
        // Expected: All 0s
        // ---------------------------------------------------------
        apply_snapshot(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        $display("T1 (Zeros)   -> Comp0: %d, Comp1: %d, Comp2: %d, Comp3: %d", comp_out_0, comp_out_1, comp_out_2, comp_out_3);

        // ---------------------------------------------------------
        // Test 2: Target Comp 0 (Weights: 0: ~1.0, 1: 0.5, 2: 0.25, 3: 0.125, 13: -1.0)
        // Input: 0=1000, 1=1000, 2=1000, 3=1000, 13=-1000
        // Expected Comp0: 1000*(1 + .5 + .25 + .125 + 1) ~= 2875
        // Expected Others: ~0 or slight noise depending on weight matrix overlaps
        // ---------------------------------------------------------
        apply_snapshot(1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1000);
        $display("T2 (Comp 0)  -> Comp0: %d, Comp1: %d, Comp2: %d, Comp3: %d", comp_out_0, comp_out_1, comp_out_2, comp_out_3);

        // ---------------------------------------------------------
        // Test 3: Target Comp 1 (Weights: 4: ~1.0, 5: 0.5, 6: 0.25, 12: -1.0, 13: -1.0)
        // Input: 4=2000, 5=2000, 6=2000, 12=-1000, 13=-1000
        // Expected Comp1: 2000*(1 + .5 + .25) -1000*(-1) -1000*(-1) = 3500 + 1000 + 1000 = 5500
        // ---------------------------------------------------------
        apply_snapshot(0, 0, 0, 0, 2000, 2000, 2000, 0, 0, 0, 0, 0, -1000, -1000);
        $display("T3 (Comp 1)  -> Comp0: %d, Comp1: %d, Comp2: %d, Comp3: %d", comp_out_0, comp_out_1, comp_out_2, comp_out_3);

        // ---------------------------------------------------------
        // Test 4: Maximum Negative Input (Edge case saturation test against Comp 3)
        // Weight Comp 3: 0:-1.0, 1:-1.0, 2:-1.0, 10:1.0, 11:0.5, 12:0.25, 13:0.125
        // Input everything to -32768
        // Expected: Saturation clamp at -32768 or 32767
        // ---------------------------------------------------------
        apply_snapshot(-32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768, -32768);
        $display("T4 (Max Neg) -> Comp0: %d, Comp1: %d, Comp2: %d, Comp3: %d", comp_out_0, comp_out_1, comp_out_2, comp_out_3);

        $display("=== SIMULATION COMPLETE ===");
        $finish;
    end

endmodule
