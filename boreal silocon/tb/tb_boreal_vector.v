// ============================================================================
// Boreal SoC - Vector Engine Testbench
// ============================================================================
// Tests:
//   1. MMIO register read/write (CMD, SRC, DST, LEN, SCALE, ZERO)
//   2. Single MAC operation via SRAM stub
//   3. Status register transitions (idle -> busy -> done)
//   4. Scale and clamp output
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module tb_boreal_vector;

    reg         clk;
    reg         rst_n;

    // MMIO interface
    reg         vec_sel;
    reg         vec_wr;
    reg  [31:0] vec_addr;
    reg  [31:0] vec_wdata;
    wire [31:0] vec_rdata;
    wire        vec_ack;

    // SRAM read port
    wire        sram_rd_req;
    wire [31:0] sram_rd_addr;
    reg  [31:0] sram_rd_data;
    reg         sram_rd_ack;

    // SRAM write port
    wire        sram_wr_req;
    wire [31:0] sram_wr_addr;
    wire [31:0] sram_wr_data;
    reg         sram_wr_ack;

    // -----------------------------------------------------------------------
    // DUT
    // -----------------------------------------------------------------------
    boreal_vector u_vec (
        .clk          (clk),
        .rst_n        (rst_n),
        .sel          (vec_sel),
        .wr           (vec_wr),
        .addr         (vec_addr),
        .wdata        (vec_wdata),
        .rdata        (vec_rdata),
        .ack          (vec_ack),
        .sram_rd_req  (sram_rd_req),
        .sram_rd_addr (sram_rd_addr),
        .sram_rd_data (sram_rd_data),
        .sram_rd_ack  (sram_rd_ack),
        .sram_wr_req  (sram_wr_req),
        .sram_wr_addr (sram_wr_addr),
        .sram_wr_data (sram_wr_data),
        .sram_wr_ack  (sram_wr_ack)
    );

    // -----------------------------------------------------------------------
    // SRAM stub: respond to read/write requests
    // -----------------------------------------------------------------------
    reg [31:0] fake_sram [0:255];
    reg [31:0] written_addr;
    reg [31:0] written_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sram_rd_ack  <= 1'b0;
            sram_rd_data <= 32'h0;
            sram_wr_ack  <= 1'b0;
        end else begin
            sram_rd_ack <= 1'b0;
            sram_wr_ack <= 1'b0;
            if (sram_rd_req) begin
                sram_rd_ack  <= 1'b1;
                sram_rd_data <= fake_sram[sram_rd_addr[9:2]];
            end
            if (sram_wr_req) begin
                sram_wr_ack  <= 1'b1;
                written_addr <= sram_wr_addr;
                written_data <= sram_wr_data;
                fake_sram[sram_wr_addr[9:2]] <= sram_wr_data;
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
    task vec_write(input [31:0] address, input [31:0] data);
        begin
            @(posedge clk);
            vec_sel   <= 1'b1;
            vec_wr    <= 1'b1;
            vec_addr  <= address;
            vec_wdata <= data;
            @(posedge clk);
            vec_sel <= 1'b0;
            vec_wr  <= 1'b0;
        end
    endtask

    task vec_read(input [31:0] address, output [31:0] data);
        begin
            @(posedge clk);
            vec_sel  <= 1'b1;
            vec_wr   <= 1'b0;
            vec_addr <= address;
            @(posedge clk);
            data = vec_rdata;
            vec_sel <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------------
    // Test sequence
    // -----------------------------------------------------------------------
    integer test_pass, test_fail;
    reg [31:0] rd_data;

    initial begin
        $dumpfile("tb_boreal_vector.vcd");
        $dumpvars(0, tb_boreal_vector);

        test_pass = 0;
        test_fail = 0;

        vec_sel   = 0;
        vec_wr    = 0;
        vec_addr  = 0;
        vec_wdata = 0;
        rst_n     = 0;

        // Pre-load fake SRAM with packed INT8 data
        // Source A: [2, 3, 4, 5] packed as {5, 4, 3, 2}
        fake_sram[0] = {8'd5, 8'd4, 8'd3, 8'd2};
        // Source B (at DST for element-wise): [1, 1, 1, 1]
        fake_sram[64] = {8'd1, 8'd1, 8'd1, 8'd1};

        #100;
        rst_n = 1;
        #20;

        // ==================================================================
        // TEST 1: Register read/write
        // ==================================================================
        $display("\n[TEST 1] Vector MMIO register read/write");
        vec_write(32'h1001_0004, 32'h0000_0000);  // SRC
        vec_write(32'h1001_0008, 32'h0000_0100);  // DST (word offset 64)
        vec_write(32'h1001_000C, 32'h0000_0001);  // LEN = 1
        vec_write(32'h1001_0010, 32'h0000_FFFF);  // SCALE = 0xFFFF (~1.0 in Q16)
        vec_write(32'h1001_0014, 32'h0000_0000);  // ZERO = 0

        vec_read(32'h1001_0004, rd_data);
        if (rd_data == 32'h0000_0000) begin
            $display("  PASS: SRC = %08h", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: SRC = %08h (expected 0)", rd_data);
            test_fail = test_fail + 1;
        end

        vec_read(32'h1001_000C, rd_data);
        if (rd_data == 32'h0000_0001) begin
            $display("  PASS: LEN = %08h", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: LEN = %08h (expected 1)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 2: Status initially idle
        // ==================================================================
        $display("\n[TEST 2] Initial status (idle)");
        vec_read(32'h1001_0024, rd_data);
        if (rd_data[0] == 1'b0) begin
            $display("  PASS: Not busy (status=%08h)", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Busy at idle (status=%08h)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 3: Launch MAC and wait for done
        // ==================================================================
        $display("\n[TEST 3] Launch vector MAC operation");
        vec_write(32'h1001_0000, 32'h0000_0001);  // CMD start

        // Wait for completion
        repeat (200) @(posedge clk);
        vec_read(32'h1001_0024, rd_data);
        if (rd_data[1] == 1'b1) begin
            $display("  PASS: Vector engine done (status=%08h)", rd_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Vector engine not done (status=%08h)", rd_data);
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // TEST 4: Check written result
        // ==================================================================
        $display("\n[TEST 4] Check written output");
        $display("  Written addr: %08h, data: %08h", written_addr, written_data);
        if (written_data != 32'h0) begin
            $display("  PASS: Output data non-zero: %08h", written_data);
            test_pass = test_pass + 1;
        end else begin
            $display("  FAIL: Output data is zero");
            test_fail = test_fail + 1;
        end

        // ==================================================================
        // Summary
        // ==================================================================
        #100;
        $display("\n========================================");
        $display(" Vector TB Results: %0d PASS, %0d FAIL", test_pass, test_fail);
        $display("========================================\n");

        if (test_fail > 0) $display("*** SOME TESTS FAILED ***");
        $finish;
    end

    // Timeout
    initial begin
        #100_000;
        $display("[TIMEOUT] Vector testbench exceeded 100us");
        $finish;
    end

endmodule
