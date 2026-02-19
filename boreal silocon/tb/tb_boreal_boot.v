// ============================================================================
// Boreal SoC - Boot ROM Testbench
// ============================================================================
// Tests:
//   1. Boot completes with fuse_expected_hash = 0 (dev mode)
//   2. Boot produces a non-zero hash from ROM contents
//   3. Boot fails when fuse_expected_hash is a wrong value
//   4. ROM read interface returns correct data
//   5. Boot FSM stays in DONE state after completion
//   6. Reset re-triggers boot sequence
// ============================================================================
`timescale 1ns / 1ps

module tb_boreal_boot;

    reg         clk;
    reg         rst_n;

    // Boot ROM bus interface
    reg         rom_sel;
    reg  [31:0] rom_addr;
    wire [31:0] rom_rdata;
    wire        rom_ack;

    // Boot control
    wire        boot_done;
    wire        boot_pass;
    wire [31:0] boot_hash_out;

    // SHA-256 stub interface
    wire        sha_start;
    wire        sha_update;
    wire [31:0] sha_data;
    wire [31:0] sha_hash;
    wire        sha_ready;

    // Signature verify interface
    wire        sig_start;
    wire [31:0] sig_hash_in;
    wire        sig_pass;
    wire        sig_ready;

    // Fuse registers
    reg  [31:0] fuse_expected_hash;
    reg  [31:0] fuse_min_version;
    reg  [31:0] fuse_image_version;

    // -----------------------------------------------------------------------
    // DUT instances
    // -----------------------------------------------------------------------
    boreal_bootrom u_bootrom (
        .clk          (clk),
        .rst_n        (rst_n),
        .sel          (rom_sel),
        .addr         (rom_addr),
        .rdata        (rom_rdata),
        .ack          (rom_ack),
        .boot_done    (boot_done),
        .boot_pass    (boot_pass),
        .boot_hash_out(boot_hash_out),
        .sha_start    (sha_start),
        .sha_update   (sha_update),
        .sha_data     (sha_data),
        .sha_hash     (sha_hash),
        .sha_ready    (sha_ready),
        .sig_start    (sig_start),
        .sig_hash_in  (sig_hash_in),
        .sig_pass     (sig_pass),
        .sig_ready    (sig_ready)
    );

    boreal_sha256_stub u_sha (
        .clk    (clk),
        .rst_n  (rst_n),
        .start   (sha_start),
        .update  (sha_update),
        .data_in (sha_data),
        .hash_out(sha_hash),
        .ready   (sha_ready)
    );

    boreal_sigverify_stub u_sig (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (sig_start),
        .hash_in       (sig_hash_in),
        .expected_hash (fuse_expected_hash),
        .min_version   (fuse_min_version),
        .image_version (fuse_image_version),
        .pass          (sig_pass),
        .ready         (sig_ready)
    );

    // -----------------------------------------------------------------------
    // Clock generation – 10 ns period
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    integer test_pass, test_fail;
    reg [31:0] saved_hash;

    initial begin
        $dumpfile("tb_boreal_boot.vcd");
        $dumpvars(0, tb_boreal_boot);

        test_pass = 0;
        test_fail = 0;

        rom_sel  = 0;
        rom_addr = 0;

        // ==================================================================
        // TEST 1: Dev-mode boot (fuse_expected_hash = 0) should pass
        // ==================================================================
        $display("\n[TEST 1] Dev-mode boot (expected_hash=0)");
        fuse_expected_hash = 32'h0;
        fuse_min_version   = 32'h0;
        fuse_image_version = 32'h1;
        rst_n = 0;
        #100;
        rst_n = 1;

        wait(boot_done);
        if (boot_pass) begin
            $display("  PASS: Boot passed in dev mode");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Boot did not pass in dev mode");
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 2: Hash output should be non-zero
        // ==================================================================
        $display("\n[TEST 2] Hash output non-zero");
        saved_hash = boot_hash_out;
        if (boot_hash_out != 32'h0) begin
            $display("  PASS: boot_hash_out = %08h (non-zero)", boot_hash_out);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: boot_hash_out = 0 (expected non-zero)");
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 3: Boot with wrong expected hash should fail
        // ==================================================================
        $display("\n[TEST 3] Boot with wrong expected_hash");
        fuse_expected_hash = 32'hDEAD_BEEF;
        fuse_min_version   = 32'h0;
        fuse_image_version = 32'h1;
        rst_n = 0;
        #100;
        rst_n = 1;

        wait(boot_done);
        if (!boot_pass) begin
            $display("  PASS: Boot correctly rejected with wrong hash");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Boot passed with wrong expected hash");
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 4: Boot with correct hash should pass
        // ==================================================================
        $display("\n[TEST 4] Boot with correct expected_hash");
        fuse_expected_hash = saved_hash;
        fuse_min_version   = 32'h0;
        fuse_image_version = 32'h1;
        rst_n = 0;
        #100;
        rst_n = 1;

        wait(boot_done);
        if (boot_pass) begin
            $display("  PASS: Boot passed with correct hash (%08h)", saved_hash);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Boot failed with correct hash (%08h != %08h)",
                     boot_hash_out, saved_hash);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 5: ROM read interface
        // ==================================================================
        $display("\n[TEST 5] ROM read interface");
        #20;
        @(posedge clk);
        rom_sel  <= 1'b1;
        rom_addr <= 32'h0000_0000;
        @(posedge clk);
        @(posedge clk);
        rom_sel <= 1'b0;
        // ROM data for address 0 is initialized to 0 (empty ROM)
        if (rom_ack) begin
            $display("  PASS: ROM ack received, data = %08h", rom_rdata);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: ROM ack not received");
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 6: FSM stays in DONE state
        // ==================================================================
        $display("\n[TEST 6] FSM stays in DONE after boot");
        #200;
        if (boot_done && boot_pass) begin
            $display("  PASS: boot_done and boot_pass stable");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: boot signals unstable (done=%b, pass=%b)",
                     boot_done, boot_pass);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // Summary
        // ==================================================================
        #100;
        $display("\n========================================");
        $display(" Boot TB Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("========================================\n");

        if (test_fail > 0) $display("*** SOME TESTS FAILED ***");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #500_000;
        $display("[TIMEOUT] Boot testbench exceeded 500us");
        $finish;
    end

endmodule
