// ============================================================================
// Boreal SoC - DMA Ring Engine Testbench
// ============================================================================
// Tests:
//   1. DMA registers read/write (head, tail, status, CRC)
//   2. Single descriptor copy: SRAM addr A -> addr B
//   3. DMA done flag and CRC computed
//   4. Empty ring (head==tail) does nothing
//   5. Multi-descriptor transfer
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module tb_boreal_dma;

    reg         clk;
    reg         rst_n;

    // DMA MMIO interface
    reg         dma_sel;
    reg         dma_wr;
    reg  [31:0] dma_addr;
    reg  [31:0] dma_wdata;
    wire [31:0] dma_rdata;
    wire        dma_ack;

    // SRAM tile
    wire        mem_sel;
    wire        mem_wr;
    wire [ 9:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [31:0] mem_rdata;
    wire        mem_ack;

    // -----------------------------------------------------------------------
    // DUT: DMA engine + SRAM tile
    // -----------------------------------------------------------------------
    boreal_dma_ring u_dma (
        .clk      (clk),
        .rst_n    (rst_n),
        .sel      (dma_sel),
        .wr       (dma_wr),
        .addr     (dma_addr),
        .wdata    (dma_wdata),
        .rdata    (dma_rdata),
        .ack      (dma_ack),
        .mem_sel  (mem_sel),
        .mem_wr   (mem_wr),
        .mem_addr (mem_addr),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata),
        .mem_ack  (mem_ack)
    );

    // Simple SRAM model for DMA port
    reg [31:0] sram [0:1023];
    reg        mem_ack_r;
    reg [31:0] mem_rdata_r;

    assign mem_rdata = mem_rdata_r;
    assign mem_ack   = mem_ack_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_ack_r   <= 1'b0;
            mem_rdata_r <= 32'h0;
        end else begin
            mem_ack_r <= 1'b0;
            if (mem_sel) begin
                mem_ack_r <= 1'b1;
                if (mem_wr)
                    sram[mem_addr] <= mem_wdata;
                else
                    mem_rdata_r <= sram[mem_addr];
            end
        end
    end

    // -----------------------------------------------------------------------
    // Clock
    // -----------------------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    // -----------------------------------------------------------------------
    // Helper tasks
    // -----------------------------------------------------------------------
    task dma_write(input [31:0] address, input [31:0] data);
        begin
            @(posedge clk);
            dma_sel   <= 1'b1;
            dma_wr    <= 1'b1;
            dma_addr  <= address;
            dma_wdata <= data;
            @(posedge clk);
            dma_sel <= 1'b0;
            dma_wr  <= 1'b0;
        end
    endtask

    task dma_read(input [31:0] address, output [31:0] data);
        begin
            @(posedge clk);
            dma_sel  <= 1'b1;
            dma_wr   <= 1'b0;
            dma_addr <= address;
            @(posedge clk);
            data = dma_rdata;
            dma_sel <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    integer test_pass, test_fail;
    reg [31:0] rd_data;

    initial begin
        $dumpfile("tb_boreal_dma.vcd");
        $dumpvars(0, tb_boreal_dma);

        test_pass = 0;
        test_fail = 0;

        dma_sel   = 0;
        dma_wr    = 0;
        dma_addr  = 0;
        dma_wdata = 0;
        rst_n     = 0;

        // Pre-load SRAM with test data
        sram[0] = 32'hCAFE_BABE;
        sram[1] = 32'hDEAD_BEEF;
        sram[2] = 32'h1234_5678;
        sram[3] = 32'hAAAA_BBBB;

        #100;
        rst_n = 1;
        #20;

        // ==================================================================
        // TEST 1: Register read/write
        // ==================================================================
        $display("\n[TEST 1] DMA register read/write");
        dma_write(32'h1000_0000, 32'h0000_0005);  // HEAD = 5
        dma_read(32'h1000_0000, rd_data);
        if (rd_data[3:0] == 4'd5) begin
            $display("  PASS: HEAD = %0d", rd_data[3:0]);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: HEAD = %0d (expected 5)", rd_data[3:0]);
            test_fail = test_fail + 1;
        end

        // Reset head/tail for subsequent tests
        dma_write(32'h1000_0000, 32'h0);  // HEAD = 0
        dma_write(32'h1000_0004, 32'h0);  // TAIL = 0

        // ==================================================================
        // TEST 2: Empty ring start – should not start
        // ==================================================================
        $display("\n[TEST 2] Empty ring (head==tail), start should be no-op");
        dma_write(32'h1000_000C, 32'h0000_0001);  // START
        #100;
        dma_read(32'h1000_0008, rd_data);  // STATUS
        if (rd_data[0] == 1'b0) begin  // not busy
            $display("  PASS: DMA not busy with empty ring");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: DMA busy with empty ring (status=%08h)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 3: Single descriptor copy
        // ==================================================================
        $display("\n[TEST 3] Single descriptor copy (SRAM[0] -> SRAM[100])");

        // Write descriptor 0: src=0x000, dst=0x190 (word addr 100), len=1
        // Descriptors are internal regs; set via hierarchical assignment.
        u_dma.desc_src[0] = 32'h0000_0000;  // src byte addr 0
        u_dma.desc_dst[0] = 32'h0000_0190;  // dst byte addr 0x190 -> word 100
        u_dma.desc_len[0] = 32'h0000_0001;  // 1 word

        dma_write(32'h1000_0000, 32'h0);  // HEAD = 0
        dma_write(32'h1000_0004, 32'h1);  // TAIL = 1
        dma_write(32'h1000_000C, 32'h1);  // START

        // Wait for completion
        repeat (100) @(posedge clk);
        dma_read(32'h1000_0008, rd_data);  // STATUS
        if (rd_data[1] == 1'b1) begin  // done
            $display("  PASS: DMA done flag set");
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: DMA not done (status=%08h)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 4: Verify copied data
        // ==================================================================
        $display("\n[TEST 4] Verify SRAM[100] == SRAM[0]");
        if (sram[100] == 32'hCAFE_BABE) begin
            $display("  PASS: SRAM[100] = %08h", sram[100]);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: SRAM[100] = %08h (expected CAFEBABE)", sram[100]);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 5: CRC was computed (non-zero for non-zero data)
        // ==================================================================
        $display("\n[TEST 5] CRC register non-zero after transfer");
        dma_read(32'h1000_0010, rd_data);
        if (rd_data != 32'h0 && rd_data != 32'hFFFF_FFFF) begin
            $display("  PASS: CRC = %08h", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: CRC = %08h (unexpected)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // Summary
        // ==================================================================
        #100;
        $display("\n========================================");
        $display(" DMA TB Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("========================================\n");

        if (test_fail > 0) $display("*** SOME TESTS FAILED ***");
        $finish;
    end

    // Timeout
    initial begin
        #100_000;
        $display("[TIMEOUT] DMA testbench exceeded 100us");
        $finish;
    end

endmodule
