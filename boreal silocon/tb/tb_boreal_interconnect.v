// ============================================================================
// Boreal SoC - Interconnect Testbench
// ============================================================================
// Tests:
//   1. Public bus address decode to each slave region
//   2. Gate bus has priority over public bus
//   3. Public bus blocked from Privileged I/O (security)
//   4. Gate bus CAN access Privileged I/O
//   5. Unmapped address returns error
//   6. Simultaneous requests – gate wins
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module tb_boreal_interconnect;

    reg         clk;
    reg         rst_n;

    // Public master
    reg         pub_req, pub_wr;
    reg  [31:0] pub_addr, pub_wdata;
    reg  [ 3:0] pub_strb;
    wire [31:0] pub_rdata;
    wire        pub_ack, pub_err;

    // Gate master
    reg         gate_req, gate_wr;
    reg  [31:0] gate_addr, gate_wdata;
    reg  [ 3:0] gate_strb;
    wire [31:0] gate_rdata;
    wire        gate_ack, gate_err;

    // Slave wires (directly stub responses)
    wire        rom_sel;    wire [31:0] rom_addr;
    wire        sram_sel;   wire        sram_wr;   wire [31:0] sram_addr, sram_wdata; wire [3:0] sram_strb;
    wire        dma_sel;    wire        dma_wr;    wire [31:0] dma_addr, dma_wdata;
    wire        vec_sel;    wire        vec_wr;    wire [31:0] vec_addr, vec_wdata;
    wire        aimbox_sel; wire        aimbox_wr; wire [31:0] aimbox_addr, aimbox_wdata;
    wire        dvm_sel;    wire        dvm_wr;    wire [31:0] dvm_addr, dvm_wdata;
    wire        gatereg_sel;wire        gatereg_wr;wire [31:0] gatereg_addr, gatereg_wdata;
    wire        ledger_sel; wire        ledger_wr; wire [31:0] ledger_addr, ledger_wdata;
    wire        priv_sel;   wire        priv_wr;   wire [31:0] priv_addr, priv_wdata;

    // Stub slave responses: ack = sel, rdata = unique pattern per slave
    wire [31:0] rom_rdata    = rom_sel    ? 32'hAA00_0001 : 32'h0;
    wire        rom_ack      = rom_sel;
    wire [31:0] sram_rdata   = sram_sel   ? 32'hBB00_0002 : 32'h0;
    wire        sram_ack     = sram_sel;
    wire [31:0] dma_rdata    = dma_sel    ? 32'hCC00_0003 : 32'h0;
    wire        dma_ack      = dma_sel;
    wire [31:0] vec_rdata    = vec_sel    ? 32'hDD00_0004 : 32'h0;
    wire        vec_ack      = vec_sel;
    wire [31:0] aimbox_rdata = aimbox_sel ? 32'hEE00_0005 : 32'h0;
    wire        aimbox_ack   = aimbox_sel;
    wire [31:0] dvm_rdata    = dvm_sel    ? 32'hFF00_0006 : 32'h0;
    wire        dvm_ack      = dvm_sel;
    wire [31:0] gatereg_rdata= gatereg_sel? 32'hAB00_0007 : 32'h0;
    wire        gatereg_ack  = gatereg_sel;
    wire [31:0] ledger_rdata = ledger_sel ? 32'hCD00_0008 : 32'h0;
    wire        ledger_ack   = ledger_sel;
    wire [31:0] priv_rdata   = priv_sel   ? 32'hEF00_0009 : 32'h0;
    wire        priv_ack     = priv_sel;

    // -----------------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------------
    boreal_interconnect u_ic (
        .clk(clk), .rst_n(rst_n),
        .pub_req(pub_req), .pub_wr(pub_wr), .pub_addr(pub_addr),
        .pub_wdata(pub_wdata), .pub_strb(pub_strb),
        .pub_rdata(pub_rdata), .pub_ack(pub_ack), .pub_err(pub_err),
        .gate_req(gate_req), .gate_wr(gate_wr), .gate_addr(gate_addr),
        .gate_wdata(gate_wdata), .gate_strb(gate_strb),
        .gate_rdata(gate_rdata), .gate_ack(gate_ack), .gate_err(gate_err),
        .rom_sel(rom_sel), .rom_addr(rom_addr),
        .rom_rdata(rom_rdata), .rom_ack(rom_ack),
        .sram_sel(sram_sel), .sram_wr(sram_wr), .sram_addr(sram_addr),
        .sram_wdata(sram_wdata), .sram_strb(sram_strb),
        .sram_rdata(sram_rdata), .sram_ack(sram_ack),
        .dma_sel(dma_sel), .dma_wr(dma_wr), .dma_addr(dma_addr),
        .dma_wdata(dma_wdata), .dma_rdata(dma_rdata), .dma_ack(dma_ack),
        .vec_sel(vec_sel), .vec_wr(vec_wr), .vec_addr(vec_addr),
        .vec_wdata(vec_wdata), .vec_rdata(vec_rdata), .vec_ack(vec_ack),
        .aimbox_sel(aimbox_sel), .aimbox_wr(aimbox_wr), .aimbox_addr(aimbox_addr),
        .aimbox_wdata(aimbox_wdata), .aimbox_rdata(aimbox_rdata), .aimbox_ack(aimbox_ack),
        .dvm_sel(dvm_sel), .dvm_wr(dvm_wr), .dvm_addr(dvm_addr),
        .dvm_wdata(dvm_wdata), .dvm_rdata(dvm_rdata), .dvm_ack(dvm_ack),
        .gatereg_sel(gatereg_sel), .gatereg_wr(gatereg_wr), .gatereg_addr(gatereg_addr),
        .gatereg_wdata(gatereg_wdata), .gatereg_rdata(gatereg_rdata), .gatereg_ack(gatereg_ack),
        .ledger_sel(ledger_sel), .ledger_wr(ledger_wr), .ledger_addr(ledger_addr),
        .ledger_wdata(ledger_wdata), .ledger_rdata(ledger_rdata), .ledger_ack(ledger_ack),
        .priv_sel(priv_sel), .priv_wr(priv_wr), .priv_addr(priv_addr),
        .priv_wdata(priv_wdata), .priv_rdata(priv_rdata), .priv_ack(priv_ack)
    );

    // -----------------------------------------------------------------------
    // Clock
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    integer test_pass, test_fail;

    initial begin
        $dumpfile("tb_boreal_interconnect.vcd");
        $dumpvars(0, tb_boreal_interconnect);

        test_pass = 0;
        test_fail = 0;

        pub_req = 0; pub_wr = 0; pub_addr = 0; pub_wdata = 0; pub_strb = 4'hF;
        gate_req = 0; gate_wr = 0; gate_addr = 0; gate_wdata = 0; gate_strb = 4'hF;
        rst_n = 0;
        #100;
        rst_n = 1;
        #20;

        // ==================================================================
        // TEST 1: Public bus -> ROM region
        // ==================================================================
        $display("\n[TEST 1] Public bus -> ROM (0x0000_0000)");
        pub_req  <= 1'b1; pub_wr <= 1'b0; pub_addr <= 32'h0000_0000;
        #10;
        if (rom_sel && pub_ack && !pub_err) begin
            $display("  PASS: ROM selected, ack received");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: rom_sel=%b, ack=%b, err=%b", rom_sel, pub_ack, pub_err);
            test_fail = test_fail + 1;
        end
        pub_req <= 1'b0;
        #10;

        // ==================================================================
        // TEST 2: Public bus -> SRAM region
        // ==================================================================
        $display("\n[TEST 2] Public bus -> SRAM (0x0000_1000)");
        pub_req  <= 1'b1; pub_wr <= 1'b0; pub_addr <= 32'h0000_1000;
        #10;
        if (sram_sel && pub_ack && !pub_err) begin
            $display("  PASS: SRAM selected");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: sram_sel=%b, ack=%b, err=%b", sram_sel, pub_ack, pub_err);
            test_fail = test_fail + 1;
        end
        pub_req <= 1'b0;
        #10;

        // ==================================================================
        // TEST 3: Public bus -> DMA region
        // ==================================================================
        $display("\n[TEST 3] Public bus -> DMA (0x1000_0000)");
        pub_req  <= 1'b1; pub_wr <= 1'b0; pub_addr <= 32'h1000_0000;
        #10;
        if (dma_sel && pub_ack && !pub_err) begin
            $display("  PASS: DMA selected");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: dma_sel=%b, ack=%b, err=%b", dma_sel, pub_ack, pub_err);
            test_fail = test_fail + 1;
        end
        pub_req <= 1'b0;
        #10;

        // ==================================================================
        // TEST 4: Public bus -> Privileged I/O BLOCKED
        // ==================================================================
        $display("\n[TEST 4] Public bus -> Priv I/O BLOCKED (0x2000_0000)");
        pub_req  <= 1'b1; pub_wr <= 1'b0; pub_addr <= 32'h2000_0000;
        #10;
        if (pub_err && !priv_sel) begin
            $display("  PASS: Access blocked, error returned, priv_sel=%b", priv_sel);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: pub_err=%b, priv_sel=%b", pub_err, priv_sel);
            test_fail = test_fail + 1;
        end
        pub_req <= 1'b0;
        #10;

        // ==================================================================
        // TEST 5: Gate bus -> Privileged I/O ALLOWED
        // ==================================================================
        $display("\n[TEST 5] Gate bus -> Priv I/O ALLOWED (0x2000_0000)");
        gate_req  <= 1'b1; gate_wr <= 1'b0; gate_addr <= 32'h2000_0000;
        #10;
        if (priv_sel && gate_ack && !gate_err) begin
            $display("  PASS: Gate can access Priv I/O");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: priv_sel=%b, gate_ack=%b, gate_err=%b",
                     priv_sel, gate_ack, gate_err);
            test_fail = test_fail + 1;
        end
        gate_req <= 1'b0;
        #10;

        // ==================================================================
        // TEST 6: Unmapped address returns error
        // ==================================================================
        $display("\n[TEST 6] Unmapped address (0x3000_0000)");
        pub_req  <= 1'b1; pub_wr <= 1'b0; pub_addr <= 32'h3000_0000;
        #10;
        if (pub_err) begin
            $display("  PASS: Unmapped address returns error");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: No error for unmapped address");
            test_fail = test_fail + 1;
        end
        pub_req <= 1'b0;
        #10;

        // ==================================================================
        // TEST 7: Gate priority – both request, gate wins
        // ==================================================================
        $display("\n[TEST 7] Gate priority over public bus");
        pub_req   <= 1'b1; pub_wr  <= 1'b0; pub_addr  <= 32'h0000_1000;
        gate_req  <= 1'b1; gate_wr <= 1'b0; gate_addr <= 32'h1000_0000;
        #10;
        // Gate should win: DMA selected, not SRAM
        if (dma_sel && !sram_sel && gate_ack) begin
            $display("  PASS: Gate wins arbitration (DMA selected)");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: dma_sel=%b, sram_sel=%b, gate_ack=%b",
                     dma_sel, sram_sel, gate_ack);
            test_fail = test_fail + 1;
        end
        pub_req  <= 1'b0;
        gate_req <= 1'b0;
        #10;

        // ==================================================================
        // Summary
        // ==================================================================
        #100;
        $display("\n========================================");
        $display(" Interconnect TB Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("========================================\n");

        if (test_fail > 0) $display("*** SOME TESTS FAILED ***");
        $finish;
    end

    // Timeout
    initial begin
        #50_000;
        $display("[TIMEOUT] Interconnect testbench exceeded 50us");
        $finish;
    end

endmodule
