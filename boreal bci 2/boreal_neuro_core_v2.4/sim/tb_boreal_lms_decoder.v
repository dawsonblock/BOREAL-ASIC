`timescale 1ns / 1ps

module tb_boreal_lms_decoder;

    // Inputs
    reg clk;
    reg rst_n;
    reg enable;
    reg valid_in;
    reg signed [15:0] comp_in_0;
    reg signed [15:0] comp_in_1;
    reg signed [15:0] comp_in_2;
    reg signed [15:0] comp_in_3;
    reg signed [15:0] error_in;

    // Outputs
    wire signed [15:0] state_out;
    wire valid_out;

    // Instantiate UUT
    // Using MU_SHIFT = 4 for faster convergence during short simulation
    // mu = 1/16 = 0.0625
    boreal_lms_decoder #(
        .NUM_COMPONENTS(4),
        .MU_SHIFT(4) 
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .valid_in(valid_in),
        .comp_in_0(comp_in_0),
        .comp_in_1(comp_in_1),
        .comp_in_2(comp_in_2),
        .comp_in_3(comp_in_3),
        .error_in(error_in),
        .state_out(state_out),
        .valid_out(valid_out)
    );

    // Clock generator (100MHz)
    always #5 clk = ~clk;

    // Helper task to apply an inference cycle
    task apply_inference;
        input signed [15:0] c0, c1, c2, c3;
        input signed [15:0] err;
        begin
            @(posedge clk);
            valid_in = 1;
            comp_in_0 = c0;
            comp_in_1 = c1;
            comp_in_2 = c2;
            comp_in_3 = c3;
            error_in  = err;
            
            @(posedge clk);
            valid_in = 0;
            
            #50; // Allow pipelined state and weight registers to update
        end
    endtask

    integer i;

    initial begin
        // Reset Inputs
        clk = 0;
        rst_n = 0;
        enable = 0;
        valid_in = 0;
        comp_in_0 = 0;
        comp_in_1 = 0;
        comp_in_2 = 0;
        comp_in_3 = 0;
        error_in = 0;

        $dumpfile("sim/boreal_lms.vcd");
        $dumpvars(0, tb_boreal_lms_decoder);

        // Wait for global reset
        #100;
        rst_n = 1;
        #20;
        enable = 1;

        $display("=== BOREAL LMS DECODER SIMULATION ===");

        // ---------------------------------------------------------
        // Test 1: Positive Error Feedback
        // Objective: Inject a constant positive signal and a positive error
        // Expected: The weights should grow positively, pushing state_out higher over time 
        // ---------------------------------------------------------
        $display("--- Test 1: Positive Adaptation ---");
        
        $display("Initial State (W=0)  -> Out: %d | (Expecting ~ 0)", state_out);
        
        // Loop 10 times to watch the weights adapt
        for (i = 0; i < 15; i = i + 1) begin
            apply_inference(10000, 10000, 10000, 10000, 5000);
            $display("Adapt %0d -> Out: %d", i, state_out);
        end


        // ---------------------------------------------------------
        // Test 2: Negative Error Feedback (Reversal)
        // Objective: Reverse the error signal to negative
        // Expected: The weights should begin shrinking, driving state_out down
        // ---------------------------------------------------------
        $display("--- Test 2: Negative Adaptation Reversal ---");
        
        // Loop 10 times with negative error
        for (i = 0; i < 15; i = i + 1) begin
            apply_inference(10000, 10000, 10000, 10000, -5000);
            $display("Adapt %0d -> Out: %d", i, state_out);
        end

        // ---------------------------------------------------------
        // Test 3: Convergence (Zero Error)
        // Objective: Set error to zero.
        // Expected: The weights lock in place and state_out remains stable
        // ---------------------------------------------------------
        $display("--- Test 3: Stable Convergence ---");
        apply_inference(10000, 10000, 10000, 10000, 0);
        $display("Stable 1 -> Out: %d", state_out);
        apply_inference(10000, 10000, 10000, 10000, 0);
        $display("Stable 2 -> Out: %d", state_out);
        apply_inference(1000, 1000, 1000, 1000, 0);
        $display("Stable 3 -> Out: %d", state_out);


        $display("=== SIMULATION COMPLETE ===");
        $finish;
    end

endmodule
