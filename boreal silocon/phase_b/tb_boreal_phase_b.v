`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// tb_boreal_phase_b.v  (Phase-B integration testbench)
// Tests: PRIV block, SRAM, DMA MMIO, VEC compute, Gate+VM via mailbox,
//        Ledger commit, deterministic replay
// ============================================================================
`include "boreal_pkg.vh"

module tb_boreal_phase_b;
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

    boreal_soc_top_fpga_phaseb dut (
        .clk(clk), .rst_n(rst_n),
        .cpu_req_valid(cpu_req_valid), .cpu_req_we(cpu_req_we),
        .cpu_req_addr(cpu_req_addr), .cpu_req_wdata(cpu_req_wdata),
        .cpu_req_wstrb(cpu_req_wstrb),
        .cpu_resp_valid(cpu_resp_valid), .cpu_resp_rdata(cpu_resp_rdata),
        .cpu_resp_err(cpu_resp_err)
    );

    task mmio_write(input [31:0] addr, input [31:0] data);
        begin
            @(negedge clk);
            cpu_req_valid <= 1'b1; cpu_req_we <= 1'b1;
            cpu_req_addr <= addr; cpu_req_wdata <= data; cpu_req_wstrb <= 4'hF;
            @(negedge clk);
            cpu_req_valid <= 1'b0; cpu_req_we <= 1'b0;
            cpu_req_addr <= 0; cpu_req_wdata <= 0; cpu_req_wstrb <= 0;
            @(posedge cpu_resp_valid);
            @(negedge clk);
        end
    endtask

    task mmio_read(input [31:0] addr, output [31:0] data);
        begin
            @(negedge clk);
            cpu_req_valid <= 1'b1; cpu_req_we <= 1'b0;
            cpu_req_addr <= addr; cpu_req_wdata <= 0; cpu_req_wstrb <= 0;
            @(negedge clk);
            cpu_req_valid <= 1'b0; cpu_req_addr <= 0;
            @(posedge cpu_resp_valid);
            @(negedge clk);
            data = cpu_resp_rdata;
        end
    endtask

    integer test_pass, test_fail;
    reg [31:0] tmp;

    // Replay storage
    integer run;
    reg [31:0] gold [0:7];
    reg [31:0] now  [0:7];
    integer k;

    initial begin
        $dumpfile("tb_boreal_phase_b.vcd");
        $dumpvars(0, tb_boreal_phase_b);

        test_pass = 0; test_fail = 0;
        cpu_req_valid = 0; cpu_req_we = 0;
        cpu_req_addr = 0; cpu_req_wdata = 0; cpu_req_wstrb = 0;

        for (run = 0; run < 2; run = run + 1) begin
            // Reset
            rst_n = 0;
            repeat (5) @(negedge clk);
            rst_n = 1;
            repeat (5) @(negedge clk);

            $display("\n========== PHASE-B RUN %0d ==========", run);

            // -----------------------------------------------------------
            // TEST 1: PRIV region blocked
            // -----------------------------------------------------------
            $display("\n[TEST 1] PRIV region blocked");
            @(negedge clk);
            cpu_req_valid <= 1; cpu_req_we <= 0;
            cpu_req_addr <= 32'h2000_0000; cpu_req_wstrb <= 0;
            @(negedge clk);
            cpu_req_valid <= 0; cpu_req_addr <= 0;
            @(posedge cpu_resp_valid);
            @(negedge clk);
            if (run == 0) begin
                if (cpu_resp_err) begin $display("  PASS"); test_pass = test_pass+1; end
                else              begin $display("  FAIL: no error"); test_fail = test_fail+1; end
            end

            // -----------------------------------------------------------
            // TEST 2: SRAM write/read
            // -----------------------------------------------------------
            $display("\n[TEST 2] SRAM write/read");
            mmio_write(32'h0000_1000, 32'hCAFE_BABE);
            mmio_read(32'h0000_1000, tmp);
            if (run == 0) begin
                if (tmp == 32'hCAFE_BABE) begin $display("  PASS: %h", tmp); test_pass = test_pass+1; end
                else begin $display("  FAIL: %h", tmp); test_fail = test_fail+1; end
            end

            // -----------------------------------------------------------
            // TEST 3: DMA MMIO readback
            // -----------------------------------------------------------
            $display("\n[TEST 3] DMA MMIO readback");
            mmio_write(32'h1000_0000, 32'h0000_2000); // HEAD
            mmio_read(32'h1000_0000, tmp);
            if (run == 0) begin
                if (tmp == 32'h0000_2000) begin $display("  PASS: HEAD=%h", tmp); test_pass = test_pass+1; end
                else begin $display("  FAIL: HEAD=%h", tmp); test_fail = test_fail+1; end
            end

            // -----------------------------------------------------------
            // TEST 4: VEC engine compute
            // -----------------------------------------------------------
            $display("\n[TEST 4] VEC engine compute");
            // Write A[0..1] at scratchpad offset 0x100 (word 0x100)
            mmio_write(32'h1001_0000 + (32'h100 << 2), 32'h01020304); // A word0: bytes 4,3,2,1
            mmio_write(32'h1001_0000 + (32'h101 << 2), 32'h01010101); // A word1
            // Write B[0..1] at scratchpad offset 0x300
            mmio_write(32'h1001_0000 + (32'h300 << 2), 32'h01020304); // B word0
            mmio_write(32'h1001_0000 + (32'h301 << 2), 32'h01010101); // B word1
            // Set LEN=8 (1 step of 8 elements)
            mmio_write(32'h1001_0004, 32'd8);
            // Start
            mmio_write(32'h1001_0000, 32'd1);
            repeat (20) @(negedge clk);
            // Read STATUS
            mmio_read(32'h1001_0000 + (32'h009 << 2), tmp);
            if (run == 0) begin
                if (tmp == 32'h2) begin $display("  PASS: STATUS=done"); test_pass = test_pass+1; end
                else begin $display("  FAIL: STATUS=%h", tmp); test_fail = test_fail+1; end
            end
            // Read OUT[0] at offset 0x500
            mmio_read(32'h1001_0000 + (32'h500 << 2), tmp);
            if (run == 0) begin
                // A byte0=4, B byte0=4 => 4*4=16
                if (tmp == 32'd16) begin $display("  PASS: OUT[0]=%0d", tmp); test_pass = test_pass+1; end
                else begin $display("  FAIL: OUT[0]=%0d (exp 16)", tmp); test_fail = test_fail+1; end
            end

            // -----------------------------------------------------------
            // Program Gate policy
            // -----------------------------------------------------------
            $display("\n  Programming Gate policy...");
            mmio_write(32'h1004_0000, 32'h0001_0000); // ALLOW0 bit16
            mmio_write(32'h1004_0004, 32'h0000_0000); // ALLOW1
            mmio_write(32'h1004_0008, 32'd10);         // RATE_LIMIT
            mmio_write(32'h1004_000C, 32'd100);        // RATE_WINDOW
            mmio_write(32'h1004_0010, 32'hA5A5_0001); // POLICY_HASH
            mmio_write(32'h1004_0020, 32'd0);           // CLAMP_MIN0
            mmio_write(32'h1004_0024, 32'd1000);       // CLAMP_MAX0

            // -----------------------------------------------------------
            // TEST 5: Gate policy readback
            // -----------------------------------------------------------
            $display("\n[TEST 5] Gate policy readback");
            mmio_read(32'h1004_0010, tmp);
            if (run == 0) begin
                if (tmp == 32'hA5A5_0001) begin $display("  PASS: %h", tmp); test_pass = test_pass+1; end
                else begin $display("  FAIL: %h", tmp); test_fail = test_fail+1; end
            end

            // -----------------------------------------------------------
            // Write AI mailbox word0 = 150, start VM
            // -----------------------------------------------------------
            mmio_write(32'h1002_0000, 32'd150);
            repeat (10) @(negedge clk);
            mmio_write(32'h1003_0000, 32'd1); // start VM

            // Wait for VM + Gate + response
            repeat (80) @(negedge clk);

            // -----------------------------------------------------------
            // Read ledger entry 0
            // -----------------------------------------------------------
            mmio_write(32'h1005_0008, 32'd0);
            repeat (5) @(negedge clk);

            mmio_read(32'h1005_000C, now[0]);
            mmio_read(32'h1005_0010, now[1]);
            mmio_read(32'h1005_0014, now[2]);
            mmio_read(32'h1005_0018, now[3]);
            mmio_read(32'h1005_001C, now[4]);
            mmio_read(32'h1005_0020, now[5]);
            mmio_read(32'h1005_0024, now[6]);
            mmio_read(32'h1005_0028, now[7]);

            $display("\n  RUN %0d LED0:", run);
            for (k = 0; k < 8; k = k + 1)
                $display("    w%0d = %h", k, now[k]);

            if (run == 0) begin
                for (k = 0; k < 8; k = k + 1) gold[k] = now[k];

                // TEST 6: Ledger non-zero
                $display("\n[TEST 6] Ledger entry committed");
                if (now[0] != 0 || now[1] != 0) begin
                    $display("  PASS"); test_pass = test_pass+1;
                end else begin
                    $display("  FAIL: all zeros"); test_fail = test_fail+1;
                end

                // TEST 7: Ledger idx advanced
                $display("\n[TEST 7] Ledger index advanced");
                mmio_read(32'h1005_0000, tmp);
                if (tmp > 0) begin $display("  PASS: IDX=%0d", tmp); test_pass = test_pass+1; end
                else         begin $display("  FAIL: IDX=0"); test_fail = test_fail+1; end
            end else begin
                // TEST 8: Deterministic replay
                $display("\n[TEST 8] Deterministic replay match");
                begin : replay_chk
                    reg ok;
                    ok = 1;
                    for (k = 0; k < 8; k = k + 1) begin
                        if (now[k] !== gold[k]) begin
                            $display("  MISMATCH w%0d: gold=%h now=%h", k, gold[k], now[k]);
                            ok = 0;
                        end
                    end
                    if (ok) begin $display("  PASS: REPLAY MATCH"); test_pass = test_pass+1; end
                    else    begin $display("  FAIL: REPLAY MISMATCH"); test_fail = test_fail+1; end
                end
            end
        end

        $display("\n=============================================");
        $display("Phase-B TB Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("=============================================");
        if (test_fail > 0) $display("\n*** SOME TESTS FAILED ***");
        else               $display("\nAll tests passed.");
        $finish;
    end

    initial begin
        #1000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
`default_nettype wire
