`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// tb_boreal_phase_a.v  (Phase-A deterministic replay testbench)
// - Programs Gate policy via MMIO
// - Writes AI mailbox input r0
// - Starts VM
// - Reads ledger entries and prints them
// - Runs twice with reset in between; compares first entry for replay match
// ============================================================================
`include "boreal_pkg.vh"

module tb_boreal_phase_a;
    reg clk = 0;
    always #5 clk = ~clk;

    reg rst_n = 0;

    reg        cpu_req_valid;
    reg        cpu_req_we;
    reg [31:0] cpu_req_addr;
    reg [31:0] cpu_req_wdata;
    reg [3:0]  cpu_req_wstrb;
    wire       cpu_resp_valid;
    wire [31:0] cpu_resp_rdata;
    wire       cpu_resp_err;

    boreal_soc_top_fpga dut (
        .clk(clk), .rst_n(rst_n),
        .cpu_req_valid(cpu_req_valid),
        .cpu_req_we(cpu_req_we),
        .cpu_req_addr(cpu_req_addr),
        .cpu_req_wdata(cpu_req_wdata),
        .cpu_req_wstrb(cpu_req_wstrb),
        .cpu_resp_valid(cpu_resp_valid),
        .cpu_resp_rdata(cpu_resp_rdata),
        .cpu_resp_err(cpu_resp_err)
    );

    task mmio_write(input [31:0] addr, input [31:0] data);
        begin
            @(negedge clk);
            cpu_req_valid <= 1'b1;
            cpu_req_we    <= 1'b1;
            cpu_req_addr  <= addr;
            cpu_req_wdata <= data;
            cpu_req_wstrb <= 4'hF;
            @(negedge clk);
            cpu_req_valid <= 1'b0;
            cpu_req_we    <= 1'b0;
            cpu_req_addr  <= 32'h0;
            cpu_req_wdata <= 32'h0;
            cpu_req_wstrb <= 4'h0;
            @(posedge cpu_resp_valid);
            @(negedge clk);
            if (cpu_resp_err) $display("  MMIO WRITE ERR @%h", addr);
        end
    endtask

    task mmio_read(input [31:0] addr, output [31:0] data);
        begin
            @(negedge clk);
            cpu_req_valid <= 1'b1;
            cpu_req_we    <= 1'b0;
            cpu_req_addr  <= addr;
            cpu_req_wdata <= 32'h0;
            cpu_req_wstrb <= 4'h0;
            @(negedge clk);
            cpu_req_valid <= 1'b0;
            cpu_req_addr  <= 32'h0;
            @(posedge cpu_resp_valid);
            @(negedge clk);
            data = cpu_resp_rdata;
            if (cpu_resp_err) $display("  MMIO READ ERR @%h", addr);
        end
    endtask

    integer run;
    reg [31:0] tmp;
    integer test_pass, test_fail;

    // Store first run ledger words for comparison (first entry, 8 words)
    reg [31:0] gold [0:7];
    reg [31:0] now  [0:7];
    integer k;

    initial begin
        $dumpfile("tb_boreal_phase_a.vcd");
        $dumpvars(0, tb_boreal_phase_a);

        test_pass = 0;
        test_fail = 0;

        cpu_req_valid = 0;
        cpu_req_we    = 0;
        cpu_req_addr  = 0;
        cpu_req_wdata = 0;
        cpu_req_wstrb = 0;

        for (run = 0; run < 2; run = run + 1) begin
            // ---------------------------------------------------------------
            // Reset
            // ---------------------------------------------------------------
            rst_n = 0;
            repeat (5) @(negedge clk);
            rst_n = 1;
            repeat (5) @(negedge clk);

            $display("\n========== RUN %0d ==========", run);

            // ---------------------------------------------------------------
            // TEST 1: PRIV region blocked (both runs for determinism)
            // ---------------------------------------------------------------
            $display("\n[TEST 1] PRIV region access blocked");
            @(negedge clk);
            cpu_req_valid <= 1'b1;
            cpu_req_we    <= 1'b0;
            cpu_req_addr  <= 32'h2000_0000;
            cpu_req_wstrb <= 4'h0;
            @(negedge clk);
            cpu_req_valid <= 1'b0;
            cpu_req_addr  <= 32'h0;
            @(posedge cpu_resp_valid);
            @(negedge clk);
            if (run == 0) begin
                if (cpu_resp_err) begin
                    $display("  PASS: PRIV access returned error");
                    test_pass = test_pass + 1;
                end else begin
                    $display("  FAIL: PRIV access did NOT return error");
                    test_fail = test_fail + 1;
                end
            end

            // ---------------------------------------------------------------
            // TEST 2: SRAM write/read (both runs for determinism)
            // ---------------------------------------------------------------
            $display("\n[TEST 2] SRAM write/read");
            mmio_write(32'h0000_1000, 32'hDEAD_BEEF);
            mmio_read(32'h0000_1000, tmp);
            if (run == 0) begin
                if (tmp == 32'hDEAD_BEEF) begin
                    $display("  PASS: SRAM = %h", tmp);
                    test_pass = test_pass + 1;
                end else begin
                    $display("  FAIL: SRAM = %h (expected DEAD_BEEF)", tmp);
                    test_fail = test_fail + 1;
                end
            end

            // ---------------------------------------------------------------
            // Program Gate policy
            // ---------------------------------------------------------------
            $display("\n  Programming Gate policy...");
            // allow target bit 0x10 -> target[5:0] = 16 => allow_mask bit 16
            mmio_write(32'h1004_0000, 32'h0001_0000);   // ALLOW0
            mmio_write(32'h1004_0004, 32'h0000_0000);   // ALLOW1
            mmio_write(32'h1004_0008, 32'd10);           // RATE_LIMIT
            mmio_write(32'h1004_000C, 32'd100);          // RATE_WINDOW
            mmio_write(32'h1004_0010, 32'hA5A5_0001);   // POLICY_HASH
            // Clamp class 0: min=0, max=1000
            mmio_write(32'h1004_0020, 32'd0);            // CLAMP_MIN0
            mmio_write(32'h1004_0024, 32'd1000);         // CLAMP_MAX0

            // ---------------------------------------------------------------
            // TEST 3: Gate policy readback (both runs for determinism)
            // ---------------------------------------------------------------
            $display("\n[TEST 3] Gate policy readback");
            mmio_read(32'h1004_0010, tmp);
            if (run == 0) begin
                if (tmp == 32'hA5A5_0001) begin
                    $display("  PASS: POLICY_HASH = %h", tmp);
                    test_pass = test_pass + 1;
                end else begin
                    $display("  FAIL: POLICY_HASH = %h (expected A5A5_0001)", tmp);
                    test_fail = test_fail + 1;
                end
            end

            // ---------------------------------------------------------------
            // Write AI mailbox word0 = 150 -> VM clamps to 100, emits
            // ---------------------------------------------------------------
            mmio_write(32'h1002_0000, 32'd150);

            // Wait for mailbox read sequencer to latch AI words
            repeat (20) @(negedge clk);

            // Start VM
            mmio_write(32'h1003_0000, 32'd1);

            // Wait for VM to execute and Gate to respond
            repeat (60) @(negedge clk);

            // ---------------------------------------------------------------
            // Read ledger entry 0
            // ---------------------------------------------------------------
            // Set read address = 0
            mmio_write(32'h1005_0008, 32'd0);
            // Allow ledger read to settle
            repeat (5) @(negedge clk);

            // Read 8 words of the entry (at offsets 0x03..0x0A)
            mmio_read(32'h1005_000C, now[0]);
            mmio_read(32'h1005_0010, now[1]);
            mmio_read(32'h1005_0014, now[2]);
            mmio_read(32'h1005_0018, now[3]);
            mmio_read(32'h1005_001C, now[4]);
            mmio_read(32'h1005_0020, now[5]);
            mmio_read(32'h1005_0024, now[6]);
            mmio_read(32'h1005_0028, now[7]);

            $display("\n  RUN %0d LED0 words:", run);
            for (k = 0; k < 8; k = k + 1)
                $display("    w%0d = %h", k, now[k]);

            if (run == 0) begin
                // Store gold values
                for (k = 0; k < 8; k = k + 1) gold[k] = now[k];

                // TEST 4: Ledger entry non-zero (committed)
                $display("\n[TEST 4] Ledger entry committed");
                if (now[0] != 32'h0 || now[1] != 32'h0) begin
                    $display("  PASS: Ledger entry non-zero");
                    test_pass = test_pass + 1;
                end else begin
                    $display("  FAIL: Ledger entry all zeros");
                    test_fail = test_fail + 1;
                end

                // TEST 5: Ledger idx advanced
                $display("\n[TEST 5] Ledger index advanced");
                mmio_read(32'h1005_0000, tmp);
                if (tmp > 32'd0) begin
                    $display("  PASS: Ledger IDX = %0d", tmp);
                    test_pass = test_pass + 1;
                end else begin
                    $display("  FAIL: Ledger IDX = 0");
                    test_fail = test_fail + 1;
                end
            end else begin
                // ---------------------------------------------------------------
                // TEST 6: Deterministic replay match
                // ---------------------------------------------------------------
                $display("\n[TEST 6] Deterministic replay match");
                begin : replay_check
                    reg replay_ok;
                    replay_ok = 1'b1;
                    for (k = 0; k < 8; k = k + 1) begin
                        if (now[k] !== gold[k]) begin
                            $display("  MISMATCH at word %0d: gold=%h now=%h", k, gold[k], now[k]);
                            replay_ok = 1'b0;
                        end
                    end
                    if (replay_ok) begin
                        $display("  PASS: REPLAY MATCH");
                        test_pass = test_pass + 1;
                    end else begin
                        $display("  FAIL: REPLAY MISMATCH");
                        test_fail = test_fail + 1;
                    end
                end
            end
        end

        // ---------------------------------------------------------------
        // Summary
        // ---------------------------------------------------------------
        $display("\n=============================================");
        $display("Phase-A TB Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("=============================================");
        if (test_fail > 0) $display("\n*** SOME TESTS FAILED ***");
        else               $display("\nAll tests passed.");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
`default_nettype wire
