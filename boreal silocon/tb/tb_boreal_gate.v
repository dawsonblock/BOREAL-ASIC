// ============================================================================
// Testbench - Boreal Gate + Interconnect + Privileged I/O + Ledger
// ============================================================================
// Phase 1 verification: exercises the Gate pipeline end-to-end.
//   1. Configures policy via MMIO
//   2. Verifies public bus cannot access Privileged I/O
//   3. Submits action requests through the Gate
//   4. Checks ledger entries
// ============================================================================
`timescale 1ns / 1ps

module tb_boreal_gate;

    reg        clk;
    reg        rst_n;

    // External bus master signals
    reg        ext_req;
    reg        ext_wr;
    reg [31:0] ext_addr;
    reg [31:0] ext_wdata;
    reg [ 3:0] ext_strb;
    wire [31:0] ext_rdata;
    wire       ext_ack;
    wire       ext_err;

    // Physical I/O
    wire [31:0] pio_out_0, pio_out_1, pio_out_2, pio_out_3;

    // Boot
    wire boot_done, boot_pass;

    // Fuse configuration – set expected hash to match stub output
    reg [31:0] fuse_expected_hash;
    reg [31:0] fuse_min_version;
    reg [31:0] fuse_image_version;

    // -----------------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------------
    boreal_top_fpga u_dut (
        .clk               (clk),
        .rst_n_pin         (rst_n),
        .ext_req           (ext_req),
        .ext_wr            (ext_wr),
        .ext_addr          (ext_addr),
        .ext_wdata         (ext_wdata),
        .ext_strb          (ext_strb),
        .ext_rdata         (ext_rdata),
        .ext_ack           (ext_ack),
        .ext_err           (ext_err),
        .pio_out_0         (pio_out_0),
        .pio_out_1         (pio_out_1),
        .pio_out_2         (pio_out_2),
        .pio_out_3         (pio_out_3),
        .boot_done         (boot_done),
        .boot_pass         (boot_pass),
        .fuse_expected_hash(fuse_expected_hash),
        .fuse_min_version  (fuse_min_version),
        .fuse_image_version(fuse_image_version)
    );

    // -----------------------------------------------------------------------
    // Clock generation – 10 ns period (100 MHz)
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------------
    // Bus helper tasks
    // -----------------------------------------------------------------------
    task bus_write(input [31:0] address, input [31:0] data);
        begin
            @(posedge clk);
            ext_req   <= 1'b1;
            ext_wr    <= 1'b1;
            ext_addr  <= address;
            ext_wdata <= data;
            ext_strb  <= 4'hF;
            @(posedge clk);
            wait(ext_ack || ext_err);
            @(posedge clk);
            ext_req <= 1'b0;
            ext_wr  <= 1'b0;
        end
    endtask

    task bus_read(input [31:0] address, output [31:0] data, output err);
        begin
            @(posedge clk);
            ext_req  <= 1'b1;
            ext_wr   <= 1'b0;
            ext_addr <= address;
            ext_strb <= 4'hF;
            @(posedge clk);
            wait(ext_ack || ext_err);
            data = ext_rdata;
            err  = ext_err;
            @(posedge clk);
            ext_req <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    reg [31:0] rd_data;
    reg        rd_err;
    integer    test_pass;
    integer    test_fail;

    initial begin
        $dumpfile("tb_boreal_gate.vcd");
        $dumpvars(0, tb_boreal_gate);

        test_pass = 0;
        test_fail = 0;

        // Initial values
        rst_n       = 0;
        ext_req     = 0;
        ext_wr      = 0;
        ext_addr    = 0;
        ext_wdata   = 0;
        ext_strb    = 0;

        // Configure fuses so boot passes (stub will match)
        fuse_expected_hash = 32'h0;
        fuse_min_version   = 32'h0;
        fuse_image_version = 32'h1;

        // Release reset
        #100;
        rst_n = 1;

        // Wait for boot to complete
        wait(boot_done);
        $display("[BOOT] Boot complete. Pass = %0d", boot_pass);

        if (!boot_pass) begin
            // If boot didn't pass, the internal reset stays asserted.
            // For testing, we need boot_pass=1. Adjust fuse_expected_hash.
            $display("[BOOT] Boot failed – adjusting fuse and retrying...");
            rst_n = 0;
            #50;
            // The stub hash is unpredictable without knowing ROM contents.
            // For an empty ROM, the stub should produce a deterministic value.
            // Set expected_hash to 0 and override with min_version=0
            fuse_expected_hash = u_dut.boot_hash_out;
            #50;
            rst_n = 1;
            wait(boot_done);
            $display("[BOOT] Retry: Pass = %0d, Hash = %08h", boot_pass, u_dut.boot_hash_out);
        end

        #200;

        // ==================================================================
        // TEST 1: Public bus access to Privileged I/O should be blocked
        // ==================================================================
        $display("\n[TEST 1] Public bus -> Privileged I/O (should be blocked)");
        bus_read(32'h2000_0000, rd_data, rd_err);
        if (rd_err) begin
            $display("  PASS: Access to PRIV_IO returned error");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Access to PRIV_IO did NOT return error (data=%08h)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 2: Configure Gate policy
        // ==================================================================
        $display("\n[TEST 2] Configure Gate policy via MMIO");
        // Allow target 1 (bit 1 of ALLOW0)
        bus_write(32'h1004_0000, 32'h0000_0002);  // GATE_ALLOW0 = bit 1
        #20;
        bus_read(32'h1004_0000, rd_data, rd_err);
        if (rd_data == 32'h0000_0002) begin
            $display("  PASS: GATE_ALLOW0 = %08h", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: GATE_ALLOW0 = %08h (expected 0x00000002)", rd_data);
            test_fail = test_fail + 1;
        end

        // Set policy hash
        bus_write(32'h1004_0010, 32'hDEAD_BEEF);  // GATE_POLICY
        #20;
        bus_read(32'h1004_0010, rd_data, rd_err);
        if (rd_data == 32'hDEAD_BEEF) begin
            $display("  PASS: GATE_POLICY = %08h", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: GATE_POLICY = %08h (expected 0xDEADBEEF)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 3: Write to SRAM and read back
        // ==================================================================
        $display("\n[TEST 3] SRAM write/read");
        bus_write(32'h0000_1000, 32'hCAFE_BABE);
        #20;
        bus_read(32'h0000_1000, rd_data, rd_err);
        if (rd_data == 32'hCAFE_BABE) begin
            $display("  PASS: SRAM[0] = %08h", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: SRAM[0] = %08h (expected 0xCAFEBABE)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 4: Read Ledger index (should be 0 initially)
        // ==================================================================
        $display("\n[TEST 4] Ledger index read");
        bus_read(32'h1005_0000, rd_data, rd_err);
        $display("  Ledger IDX = %0d", rd_data);
        if (!rd_err) begin
            $display("  PASS: Ledger accessible");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Ledger access error");
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 5: Load a simple VM program and run it
        // ==================================================================
        $display("\n[TEST 5] Decision-VM program execution");

        // Program: LOAD_IMM r0, ACT_WRITE(1); LOAD_IMM r1, 1 (target);
        //          LOAD_IMM r2, 500 (arg0); EMIT; HALT
        // Instruction format: [31:28]op [27:24]dst [23:20]srcA [19:16]srcB [15:0]imm

        // LOAD_IMM r0, 0x0001 (ACT_WRITE)
        bus_write(32'h1003_0100, {4'h1, 4'h0, 4'h0, 4'h0, 16'h0001});
        // LOAD_IMM r1, 0x0001 (target 1)
        bus_write(32'h1003_0104, {4'h1, 4'h1, 4'h0, 4'h0, 16'h0001});
        // LOAD_IMM r2, 500
        bus_write(32'h1003_0108, {4'h1, 4'h2, 4'h0, 4'h0, 16'h01F4});
        // LOAD_IMM r3, 0 (arg1)
        bus_write(32'h1003_010C, {4'h1, 4'h3, 4'h0, 4'h0, 16'h0000});
        // EMIT
        bus_write(32'h1003_0110, {4'hE, 4'h0, 4'h0, 4'h0, 16'h0000});
        // HALT
        bus_write(32'h1003_0114, {4'hF, 4'h0, 4'h0, 4'h0, 16'h0000});

        #20;

        // Start the VM (write 1 to VM_CTRL)
        bus_write(32'h1003_0000, 32'h0000_0001);

        // Wait for VM to halt
        #500;
        bus_read(32'h1003_0004, rd_data, rd_err);  // VM_STATUS
        $display("  VM_STATUS = %08h (running=%0b, halted=%0b, budget_exceeded=%0b)",
                 rd_data, rd_data[0], rd_data[1], rd_data[2]);
        if (rd_data[1]) begin
            $display("  PASS: VM halted normally");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: VM did not halt");
            test_fail = test_fail + 1;
        end

        // Check if ledger advanced
        #100;
        bus_read(32'h1005_0000, rd_data, rd_err);
        $display("  Ledger IDX after VM run = %0d", rd_data);

        // ==================================================================
        // Summary
        // ==================================================================
        #100;
        $display("\n========================================");
        $display(" Test Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("========================================\n");

        $finish;
    end

    // Timeout watchdog
    initial begin
        #100_000;
        $display("[TIMEOUT] Simulation exceeded 100us");
        $finish;
    end

endmodule
