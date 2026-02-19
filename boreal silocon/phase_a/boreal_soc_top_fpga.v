`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_soc_top_fpga.v  (Phase-A top)
// - Instantiates: SRAM, mailboxes, VM, Gate, Ledger, Interconnect
// - Provides CPU master port for tb / bring-up
// ============================================================================
`include "boreal_pkg.vh"

module boreal_soc_top_fpga (
    input  wire        clk,
    input  wire        rst_n,

    // CPU public bus master
    input  wire        cpu_req_valid,
    input  wire        cpu_req_we,
    input  wire [31:0] cpu_req_addr,
    input  wire [31:0] cpu_req_wdata,
    input  wire [3:0]  cpu_req_wstrb,
    output wire        cpu_resp_valid,
    output wire [31:0] cpu_resp_rdata,
    output wire        cpu_resp_err
);
    // Cycle counter
    reg [31:0] cycle_ctr;
    always @(posedge clk) begin
        if (!rst_n) cycle_ctr <= 32'h0;
        else        cycle_ctr <= cycle_ctr + 1;
    end

    // -----------------------------------------------------------------
    // SRAM
    // -----------------------------------------------------------------
    wire        sram_req_valid, sram_req_we;
    wire [31:0] sram_req_addr, sram_req_wdata;
    wire [3:0]  sram_req_wstrb;
    wire        sram_resp_valid;
    wire [31:0] sram_resp_rdata;
    wire        sram_resp_err;

    boreal_sram_tile #(.WORDS(1024), .INIT_HEX("")) u_sram0 (
        .clk(clk), .rst_n(rst_n),
        .req_valid(sram_req_valid),
        .req_we(sram_req_we),
        .req_addr(sram_req_addr),
        .req_wdata(sram_req_wdata),
        .req_wstrb(sram_req_wstrb),
        .resp_valid(sram_resp_valid),
        .resp_rdata(sram_resp_rdata),
        .resp_err(sram_resp_err)
    );

    // -----------------------------------------------------------------
    // Mailbox
    // -----------------------------------------------------------------
    wire        mbox_we;
    wire [31:0] mbox_addr_word, mbox_wdata;
    wire [3:0]  mbox_wstrb;
    wire [31:0] mbox_rdata;

    reg  [7:0]  a_ridx;
    wire [31:0] a_rdata;

    boreal_mailbox_dp #(.WORDS(256)) u_mbox (
        .clk(clk),
        .cpu_we(mbox_we),
        .cpu_addr(mbox_addr_word),
        .cpu_wdata(mbox_wdata),
        .cpu_wstrb(mbox_wstrb),
        .cpu_rdata(mbox_rdata),
        .a_ridx(a_ridx),
        .a_rdata(a_rdata),
        .b_we(1'b0),
        .b_widx(8'h00),
        .b_wdata(32'h0)
    );

    // Build mb_ai from internal reads using a small sequencer
    reg [127:0] mb_ai;
    reg [2:0]   ai_rd_state;

    always @(posedge clk) begin
        if (!rst_n) begin
            a_ridx      <= 8'h00;
            ai_rd_state <= 3'd0;
            mb_ai       <= 128'h0;
        end else begin
            case (ai_rd_state)
                3'd0: begin a_ridx <= 8'd0; ai_rd_state <= 3'd1; end
                3'd1: begin mb_ai[ 31:  0] <= a_rdata; a_ridx <= 8'd1; ai_rd_state <= 3'd2; end
                3'd2: begin mb_ai[ 63: 32] <= a_rdata; a_ridx <= 8'd2; ai_rd_state <= 3'd3; end
                3'd3: begin mb_ai[ 95: 64] <= a_rdata; a_ridx <= 8'd3; ai_rd_state <= 3'd4; end
                3'd4: begin mb_ai[127: 96] <= a_rdata; ai_rd_state <= 3'd0; end
                default: ai_rd_state <= 3'd0;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // VM MMIO: start control bit at vm_addr 0
    // -----------------------------------------------------------------
    reg         vm_start;
    wire        vm_we;
    wire [7:0]  vm_addr;
    wire [31:0] vm_wdata;
    wire [3:0]  vm_wstrb;
    reg  [31:0] vm_rdata;

    always @(posedge clk) begin
        if (!rst_n) begin
            vm_start <= 1'b0;
            vm_rdata <= 32'h0;
        end else begin
            if (vm_we && vm_addr == 8'h00) begin
                if (vm_wstrb[0]) vm_start <= vm_wdata[0];
            end
            if (vm_addr == 8'h00) vm_rdata <= {31'h0, vm_start};
            else                  vm_rdata <= 32'h0;
        end
    end

    // -----------------------------------------------------------------
    // Gate
    // -----------------------------------------------------------------
    wire        gate_we;
    wire [7:0]  gate_addr;
    wire [31:0] gate_wdata;
    wire [3:0]  gate_wstrb;
    wire [31:0] gate_rdata;

    wire         act_valid;
    wire [511:0] act_data;
    wire         gate_resp_valid;
    wire [159:0] gate_resp_data;

    wire         io_we;
    wire [31:0]  io_addr, io_wdata;

    wire         led_wr;
    wire [255:0] led_event;

    boreal_gate u_gate (
        .clk(clk), .rst_n(rst_n),
        .req_valid(act_valid),
        .req_data(act_data),
        .resp_valid(gate_resp_valid),
        .resp_data(gate_resp_data),
        .io_we(io_we),
        .io_addr(io_addr),
        .io_wdata(io_wdata),
        .led_wr(led_wr),
        .led_event(led_event),
        .cycle_ctr(cycle_ctr),
        .mmio_we(gate_we),
        .mmio_addr(gate_addr),
        .mmio_wdata(gate_wdata),
        .mmio_wstrb(gate_wstrb),
        .mmio_rdata(gate_rdata)
    );

    // -----------------------------------------------------------------
    // Decision-VM
    // -----------------------------------------------------------------
    boreal_decision_vm u_vm (
        .clk(clk), .rst_n(rst_n),
        .mb_ai(mb_ai),
        .gate_resp_valid(gate_resp_valid),
        .gate_resp_data(gate_resp_data),
        .act_valid(act_valid),
        .act_data(act_data),
        .start(vm_start),
        .done()
    );

    // -----------------------------------------------------------------
    // Ledger
    // -----------------------------------------------------------------
    wire        led_rd_req;
    wire [9:0]  led_rd_addr;
    wire [255:0] led_rd_data;
    wire [31:0]  led_idx;

    boreal_ledger #(.DEPTH(1024)) u_ledger (
        .clk(clk), .rst_n(rst_n),
        .wr(led_wr),
        .event_in(led_event),
        .cycle_in(cycle_ctr),
        .rd_req(led_rd_req),
        .rd_addr(led_rd_addr),
        .rd_data(led_rd_data),
        .idx(led_idx)
    );

    // -----------------------------------------------------------------
    // Interconnect
    // -----------------------------------------------------------------
    wire        dma_req_valid  = 1'b0;
    wire        dma_req_we     = 1'b0;
    wire [31:0] dma_req_addr   = 32'h0;
    wire [31:0] dma_req_wdata  = 32'h0;
    wire [3:0]  dma_req_wstrb  = 4'h0;
    wire        dma_resp_valid;
    wire [31:0] dma_resp_rdata;
    wire        dma_resp_err;

    boreal_interconnect u_ic (
        .clk(clk), .rst_n(rst_n),

        .cpu_req_valid(cpu_req_valid),
        .cpu_req_we(cpu_req_we),
        .cpu_req_addr(cpu_req_addr),
        .cpu_req_wdata(cpu_req_wdata),
        .cpu_req_wstrb(cpu_req_wstrb),
        .cpu_resp_valid(cpu_resp_valid),
        .cpu_resp_rdata(cpu_resp_rdata),
        .cpu_resp_err(cpu_resp_err),

        .dma_req_valid(dma_req_valid),
        .dma_req_we(dma_req_we),
        .dma_req_addr(dma_req_addr),
        .dma_req_wdata(dma_req_wdata),
        .dma_req_wstrb(dma_req_wstrb),
        .dma_resp_valid(dma_resp_valid),
        .dma_resp_rdata(dma_resp_rdata),
        .dma_resp_err(dma_resp_err),

        .sram_req_valid(sram_req_valid),
        .sram_req_we(sram_req_we),
        .sram_req_addr(sram_req_addr),
        .sram_req_wdata(sram_req_wdata),
        .sram_req_wstrb(sram_req_wstrb),
        .sram_resp_valid(sram_resp_valid),
        .sram_resp_rdata(sram_resp_rdata),
        .sram_resp_err(sram_resp_err),

        .mbox_we(mbox_we),
        .mbox_addr_word(mbox_addr_word),
        .mbox_wdata(mbox_wdata),
        .mbox_wstrb(mbox_wstrb),
        .mbox_rdata(mbox_rdata),

        .vm_we(vm_we),
        .vm_addr(vm_addr),
        .vm_wdata(vm_wdata),
        .vm_wstrb(vm_wstrb),
        .vm_rdata(vm_rdata),

        .gate_we(gate_we),
        .gate_addr(gate_addr),
        .gate_wdata(gate_wdata),
        .gate_wstrb(gate_wstrb),
        .gate_rdata(gate_rdata),

        .led_rd_req(led_rd_req),
        .led_rd_addr(led_rd_addr),
        .led_rd_data(led_rd_data),
        .led_idx(led_idx)
    );
endmodule
`default_nettype wire
