`timescale 1ns / 1ps

module tb_boreal_kalman_filter;

    // Inputs
    reg clk;
    reg rst_n;
    reg enable;
    reg signed [15:0] measurement_in;
    reg valid_in;
    reg signed [15:0] alpha_gain;
    reg signed [15:0] beta_gain;

    // Outputs
    wire signed [15:0] position_out;
    wire signed [15:0] velocity_out;
    wire valid_out;

    // Instantiate UUT
    boreal_kalman_filter uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .measurement_in(measurement_in),
        .valid_in(valid_in),
        .alpha_gain(alpha_gain),
        .beta_gain(beta_gain),
        .position_out(position_out),
        .velocity_out(velocity_out),
        .valid_out(valid_out)
    );

    // Clock gen (100MHz)
    always #5 clk = ~clk;

    // Helper task to apply a measurement
    task apply_measurement;
        input signed [15:0] z;
        begin
            @(posedge clk);
            valid_in = 1;
            measurement_in = z;
            @(posedge clk);
            if (!enable) begin // Emulate module behavior
               valid_in = 0;
            end
            @(posedge clk);
            valid_in = 0;
            #20; // Allow state to update
        end
    endtask

    initial begin
        // Initialize
        clk = 0;
        rst_n = 0;
        enable = 0;
        measurement_in = 0;
        valid_in = 0;
        
        // Typical BCI parameters (Alpha=0.2, Beta=0.01)
        // Alpha = 0.2 * 32768 = 6553.6 -> 16'h1999
        alpha_gain = 16'h1999;
        // Beta = 0.01 * 32768 = 327.68 -> 16'h0147
        beta_gain = 16'h0147;

        $dumpfile("sim/boreal_kalman.vcd");
        $dumpvars(0, tb_boreal_kalman_filter);

        #100;
        rst_n = 1;
        #20;
        enable = 1;

        $display("=== BOREAL KALMAN FILTER SIMULATION ===");
        
        // ---------------------------------------------------------
        // Test 1: Step Response (Jumps from 0 to 1000)
        // Expected: Position asymptotically approaches 1000 without overshoot 
        // ---------------------------------------------------------
        $display("--- Test 1: Step Response (Z=1000) ---");
        apply_measurement(1000);
        $display("T=1 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1000);
        $display("T=2 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1000);
        $display("T=3 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1000);
        $display("T=4 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1000);
        $display("T=5 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);
        
        // ---------------------------------------------------------
        // Test 2: High-Frequency Noise Rejection 
        // ---------------------------------------------------------
        $display("--- Test 2: High-Frequency Noise Spike ---");
        // Simulate a massive artifact (e.g., eye blink or muscle twitch)
        apply_measurement(15000); 
        $display("T=6 | Meas: 15000 | Pos: %d | Vel: %d", position_out, velocity_out);
        
        // Return to baseline immediately
        apply_measurement(1000); 
        $display("T=7 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1000);
        $display("T=8 | Meas: 1000 | Pos: %d | Vel: %d", position_out, velocity_out);

        // ---------------------------------------------------------
        // Test 3: Tracking a moving target (Velocity estimation)
        // ---------------------------------------------------------
        $display("--- Test 3: Ramp Tracking ---");
        apply_measurement(1100); $display("T=9  | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1200); $display("T=10 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1300); $display("T=11 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1400); $display("T=12 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1500); $display("T=13 | Pos: %d | Vel: %d", position_out, velocity_out);
        apply_measurement(1600); $display("T=14 | Pos: %d | Vel: %d", position_out, velocity_out);

        $display("=== SIMULATION COMPLETE ===");
        $finish;
    end

endmodule
