`timescale 1ns / 1ps

module tb_boreal_symbolic_mapper;

    // Inputs
    reg clk;
    reg rst_n;
    reg enable;
    reg valid_in;
    reg signed [15:0] cursor_x;
    reg signed [15:0] cursor_y;

    // Outputs
    wire [3:0] symbol_trigger;
    wire valid_out;

    // Instantiate UUT
    // Using shorter parameters for simulation speed
    // DWELL: 3 cycles, COOLDOWN: 5 cycles
    boreal_symbolic_mapper #(
        .THRESHOLD_POS(16'd10000),
        .THRESHOLD_NEG(-16'd10000),
        .DWELL_TIME_CYCLES(3),
        .COOLDOWN_CYCLES(5)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .valid_in(valid_in),
        .cursor_x(cursor_x),
        .cursor_y(cursor_y),
        .symbol_trigger(symbol_trigger),
        .valid_out(valid_out)
    );

    // 100MHz clock
    always #5 clk = ~clk;

    // Monitor for Output Triggers
    always @(posedge clk) begin
        if (valid_out) begin
            $display("[PULSE CAUGHT] Symbol Trigger Fired: %b at time %0t", symbol_trigger, $time);
        end
    end

    task apply_cursor;
        input signed [15:0] x;
        input signed [15:0] y;
        begin
            @(posedge clk);
            valid_in = 1;
            cursor_x = x;
            cursor_y = y;
            @(posedge clk);
            valid_in = 0;
            #10;
        end
    endtask

    initial begin
        clk = 0;
        rst_n = 0;
        enable = 0;
        valid_in = 0;
        cursor_x = 0;
        cursor_y = 0;

        $dumpfile("sim/boreal_mapper.vcd");
        $dumpvars(0, tb_boreal_symbolic_mapper);

        #100;
        rst_n = 1;
        #20;
        enable = 1;

        $display("=== BOREAL SYMBOLIC MAPPER SIMULATION ===");

        // -------------------------------------------------------------
        // Test 1: Transient Spike Rejection (False Positive Prevention)
        // Expected: Spikes over the boundary that last < 3 packets should do nothing
        // -------------------------------------------------------------
        $display("--- Test 1: Transient Spike Rejection ---");
        apply_cursor(15000, 0); // Dwell 1
        apply_cursor(15000, 0); // Dwell 2
        apply_cursor(0, 0);     // Slip off boundary. Reset.
        apply_cursor(15000, 0); // Dwell 1
        $display("Test 1 Finished. Should have 0 pulses so far.");

        // -------------------------------------------------------------
        // Test 2: Successful Intent Integration
        // Expected: Holding the boundary for >= 3 packets should fire a single trigger
        // -------------------------------------------------------------
        $display("--- Test 2: Successful Symbol Trigger ---");
        apply_cursor(15000, 0); // Dwell 2
        apply_cursor(15000, 0); // Dwell 3 -> Should PULSE!
        apply_cursor(15000, 0); // Cooldown 1
        $display("Test 2 Finished. Should have caught RIGHT trigger.");

        // -------------------------------------------------------------
        // Test 3: Refractory Cooldown Period
        // Expected: Even while holding the boundary, further triggers are suppressed
        // -------------------------------------------------------------
        $display("--- Test 3: Refractory Cooldown ---");
        apply_cursor(15000, 0); // Cooldown 2
        apply_cursor(15000, 0); // Cooldown 3
        apply_cursor(15000, 0); // Cooldown 4
        apply_cursor(15000, 0); // Cooldown 5 (Cooldown Ends)
        apply_cursor(15000, 0); // Dwell 1 again
        $display("Test 3 Finished. No extra pulses should have fired.");

        // -------------------------------------------------------------
        // Test 4: Re-engage after Cooldown (Different Direction)
        // -------------------------------------------------------------
        $display("--- Test 4: Trigger UP ---");
        apply_cursor(0, 15000); // Dwell 1
        apply_cursor(0, 15000); // Dwell 2
        apply_cursor(0, 15000); // Dwell 3 -> Should PULSE!
        $display("Test 4 Finished. Should have caught UP trigger.");

        $display("=== SIMULATION COMPLETE ===");
        $finish;
    end

endmodule
