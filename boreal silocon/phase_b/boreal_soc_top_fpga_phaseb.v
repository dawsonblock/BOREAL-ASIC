`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_soc_top_fpga_phaseb.v  (Phase-B top)
// Instantiates: SRAM, DMA, VEC, MBOX_BLOCK, VM, GATE_ATOMIC, LEDGER, IC
// ============================================================================
`include "boreal_pkg.vh"

module boreal_soc_top_fpga_phaseb (
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

    boreal_sram_tile #(.WORDS(1024)) u_sram0 (
        .clk(clk), .rst_n(rst_n),
        .req_valid(sram_req_valid), .req_we(sram_req_we),
        .req_addr(sram_req_addr), .req_wdata(sram_req_wdata),
        .req_wstrb(sram_req_wstrb),
        .resp_valid(sram_resp_valid), .resp_rdata(sram_resp_rdata),
        .resp_err(sram_resp_err)
    );

    // -----------------------------------------------------------------
    // DMA
    // -----------------------------------------------------------------
    wire        dma_mmio_we;
    wire [7:0]  dma_mmio_addr;
    wire [31:0] dma_mmio_wdata;
    wire [3:0]  dma_mmio_wstrb;
    wire [31:0] dma_mmio_rdata;

    wire        dma_m_req_valid, dma_m_req_we;
    wire [31:0] dma_m_req_addr, dma_m_req_wdata;
    wire [3:0]  dma_m_req_wstrb;
    wire        dma_m_resp_valid;
    wire [31:0] dma_m_resp_rdata;
    wire        dma_m_resp_err;

    boreal_dma u_dma (
        .clk(clk), .rst_n(rst_n),
        .mmio_we(dma_mmio_we), .mmio_addr(dma_mmio_addr),
        .mmio_wdata(dma_mmio_wdata), .mmio_wstrb(dma_mmio_wstrb),
        .mmio_rdata(dma_mmio_rdata),
        .m_req_valid(dma_m_req_valid), .m_req_we(dma_m_req_we),
        .m_req_addr(dma_m_req_addr), .m_req_wdata(dma_m_req_wdata),
        .m_req_wstrb(dma_m_req_wstrb),
        .m_resp_valid(dma_m_resp_valid), .m_resp_rdata(dma_m_resp_rdata),
        .m_resp_err(dma_m_resp_err)
    );

    // -----------------------------------------------------------------
    // Vector Engine
    // -----------------------------------------------------------------
    wire        vec_mmio_we;
    wire [10:0] vec_mmio_addr;
    wire [31:0] vec_mmio_wdata;
    wire [3:0]  vec_mmio_wstrb;
    wire [31:0] vec_mmio_rdata;

    boreal_vector_engine #(.SP_WORDS(2048)) u_vec (
        .clk(clk), .rst_n(rst_n),
        .mmio_we(vec_mmio_we), .mmio_addr(vec_mmio_addr),
        .mmio_wdata(vec_mmio_wdata), .mmio_wstrb(vec_mmio_wstrb),
        .mmio_rdata(vec_mmio_rdata)
    );

    // -----------------------------------------------------------------
    // Mailbox Block
    // -----------------------------------------------------------------
    wire        mbox_mmio_we;
    wire [7:0]  mbox_mmio_addr_word;
    wire [31:0] mbox_mmio_wdata;
    wire [3:0]  mbox_mmio_wstrb;
    wire [31:0] mbox_mmio_rdata;

    wire [127:0] mb_ai;

    // VM -> request mailbox
    wire        vm_req_we;
    wire [3:0]  vm_req_widx;
    wire [31:0] vm_req_wdata;
    wire        vm_req_valid_set;
    wire        vm_resp_ack;

    // Gate <-> request mailbox
    wire        gate_req_valid;
    wire [31:0] gate_req_w0,  gate_req_w1,  gate_req_w2,  gate_req_w3;
    wire [31:0] gate_req_w4,  gate_req_w5,  gate_req_w6,  gate_req_w7;
    wire [31:0] gate_req_w8,  gate_req_w9,  gate_req_w10, gate_req_w11;
    wire [31:0] gate_req_w12, gate_req_w13, gate_req_w14, gate_req_w15;
    wire        gate_req_consume;

    // Gate -> response mailbox
    wire        gate_resp_we;
    wire [2:0]  gate_resp_widx;
    wire [31:0] gate_resp_wdata_w;
    wire        gate_resp_valid_set;

    // Response -> VM
    wire        gate_resp_valid;
    wire [31:0] resp_w0, resp_w1, resp_w2, resp_w3, resp_w4;

    boreal_mbox_block u_mbox (
        .clk(clk), .rst_n(rst_n),
        .mmio_we(mbox_mmio_we), .mmio_addr_word(mbox_mmio_addr_word),
        .mmio_wdata(mbox_mmio_wdata), .mmio_wstrb(mbox_mmio_wstrb),
        .mmio_rdata(mbox_mmio_rdata),
        .mb_ai(mb_ai),
        .vm_req_we(vm_req_we), .vm_req_widx(vm_req_widx),
        .vm_req_wdata(vm_req_wdata), .vm_req_valid_set(vm_req_valid_set),
        .vm_resp_ack(vm_resp_ack),
        .gate_req_valid(gate_req_valid),
        .gate_req_words_0(gate_req_w0),   .gate_req_words_1(gate_req_w1),
        .gate_req_words_2(gate_req_w2),   .gate_req_words_3(gate_req_w3),
        .gate_req_words_4(gate_req_w4),   .gate_req_words_5(gate_req_w5),
        .gate_req_words_6(gate_req_w6),   .gate_req_words_7(gate_req_w7),
        .gate_req_words_8(gate_req_w8),   .gate_req_words_9(gate_req_w9),
        .gate_req_words_10(gate_req_w10), .gate_req_words_11(gate_req_w11),
        .gate_req_words_12(gate_req_w12), .gate_req_words_13(gate_req_w13),
        .gate_req_words_14(gate_req_w14), .gate_req_words_15(gate_req_w15),
        .gate_req_consume(gate_req_consume),
        .gate_resp_we(gate_resp_we), .gate_resp_widx(gate_resp_widx),
        .gate_resp_wdata(gate_resp_wdata_w),
        .gate_resp_valid_set(gate_resp_valid_set),
        .gate_resp_valid(gate_resp_valid),
        .gate_resp_words_0(resp_w0), .gate_resp_words_1(resp_w1),
        .gate_resp_words_2(resp_w2), .gate_resp_words_3(resp_w3),
        .gate_resp_words_4(resp_w4)
    );

    // -----------------------------------------------------------------
    // VM MMIO: start control
    // -----------------------------------------------------------------
    reg         vm_start;
    wire        vm_mmio_we;
    wire [7:0]  vm_mmio_addr;
    wire [31:0] vm_mmio_wdata;
    wire [3:0]  vm_mmio_wstrb;
    reg  [31:0] vm_mmio_rdata;

    always @(posedge clk) begin
        if (!rst_n) begin
            vm_start    <= 1'b0;
            vm_mmio_rdata <= 32'h0;
        end else begin
            if (vm_mmio_we && vm_mmio_addr == 8'h00)
                vm_start <= vm_mmio_wdata[0];
            vm_mmio_rdata <= (vm_mmio_addr == 8'h00) ? {31'h0, vm_start} : 32'h0;
        end
    end

    // -----------------------------------------------------------------
    // Decision-VM (Phase-B)
    // -----------------------------------------------------------------
    boreal_decision_vm_phaseb u_vm (
        .clk(clk), .rst_n(rst_n),
        .mb_ai(mb_ai),
        .vm_req_we(vm_req_we), .vm_req_widx(vm_req_widx),
        .vm_req_wdata(vm_req_wdata), .vm_req_valid_set(vm_req_valid_set),
        .vm_resp_valid(gate_resp_valid),
        .vm_resp_w0(resp_w0), .vm_resp_w1(resp_w1),
        .vm_resp_w2(resp_w2), .vm_resp_w3(resp_w3), .vm_resp_w4(resp_w4),
        .vm_resp_ack(vm_resp_ack),
        .start(vm_start),
        .done()
    );

    // -----------------------------------------------------------------
    // Gate Atomic (Phase-B)
    // -----------------------------------------------------------------
    wire        gate_mmio_we;
    wire [7:0]  gate_mmio_addr;
    wire [31:0] gate_mmio_wdata;
    wire [3:0]  gate_mmio_wstrb;
    wire [31:0] gate_mmio_rdata;

    wire        led_wr;
    wire [255:0] led_event;

    boreal_gate_atomic u_gate (
        .clk(clk), .rst_n(rst_n), .cycle_ctr(cycle_ctr),
        .mb_req_valid(gate_req_valid),
        .mb_req_w0(gate_req_w0), .mb_req_w1(gate_req_w1),
        .mb_req_w2(gate_req_w2), .mb_req_w3(gate_req_w3),
        .mb_req_w4(gate_req_w4), .mb_req_w5(gate_req_w5),
        .mb_req_w6(gate_req_w6), .mb_req_w7(gate_req_w7),
        .mb_req_consume(gate_req_consume),
        .mb_resp_we(gate_resp_we), .mb_resp_widx(gate_resp_widx),
        .mb_resp_wdata(gate_resp_wdata_w),
        .mb_resp_valid_set(gate_resp_valid_set),
        .io_we(), .io_addr(), .io_wdata(),
        .led_wr(led_wr), .led_event(led_event),
        .mmio_we(gate_mmio_we), .mmio_addr(gate_mmio_addr),
        .mmio_wdata(gate_mmio_wdata), .mmio_wstrb(gate_mmio_wstrb),
        .mmio_rdata(gate_mmio_rdata)
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
        .wr(led_wr), .event_in(led_event), .cycle_in(cycle_ctr),
        .rd_req(led_rd_req), .rd_addr(led_rd_addr),
        .rd_data(led_rd_data), .idx(led_idx)
    );

    // -----------------------------------------------------------------
    // Interconnect (Phase-B)
    // -----------------------------------------------------------------
    boreal_interconnect_phaseb u_ic (
        .clk(clk), .rst_n(rst_n),
        .cpu_req_valid(cpu_req_valid), .cpu_req_we(cpu_req_we),
        .cpu_req_addr(cpu_req_addr), .cpu_req_wdata(cpu_req_wdata),
        .cpu_req_wstrb(cpu_req_wstrb),
        .cpu_resp_valid(cpu_resp_valid), .cpu_resp_rdata(cpu_resp_rdata),
        .cpu_resp_err(cpu_resp_err),

        .dma_req_valid(dma_m_req_valid), .dma_req_we(dma_m_req_we),
        .dma_req_addr(dma_m_req_addr), .dma_req_wdata(dma_m_req_wdata),
        .dma_req_wstrb(dma_m_req_wstrb),
        .dma_resp_valid(dma_m_resp_valid), .dma_resp_rdata(dma_m_resp_rdata),
        .dma_resp_err(dma_m_resp_err),

        .sram_req_valid(sram_req_valid), .sram_req_we(sram_req_we),
        .sram_req_addr(sram_req_addr), .sram_req_wdata(sram_req_wdata),
        .sram_req_wstrb(sram_req_wstrb),
        .sram_resp_valid(sram_resp_valid), .sram_resp_rdata(sram_resp_rdata),
        .sram_resp_err(sram_resp_err),

        .dma_mmio_we(dma_mmio_we), .dma_mmio_addr(dma_mmio_addr),
        .dma_mmio_wdata(dma_mmio_wdata), .dma_mmio_wstrb(dma_mmio_wstrb),
        .dma_mmio_rdata(dma_mmio_rdata),

        .vec_mmio_we(vec_mmio_we), .vec_mmio_addr(vec_mmio_addr),
        .vec_mmio_wdata(vec_mmio_wdata), .vec_mmio_wstrb(vec_mmio_wstrb),
        .vec_mmio_rdata(vec_mmio_rdata),

        .mbox_mmio_we(mbox_mmio_we), .mbox_mmio_addr_word(mbox_mmio_addr_word),
        .mbox_mmio_wdata(mbox_mmio_wdata), .mbox_mmio_wstrb(mbox_mmio_wstrb),
        .mbox_mmio_rdata(mbox_mmio_rdata),

        .vm_mmio_we(vm_mmio_we), .vm_mmio_addr(vm_mmio_addr),
        .vm_mmio_wdata(vm_mmio_wdata), .vm_mmio_wstrb(vm_mmio_wstrb),
        .vm_mmio_rdata(vm_mmio_rdata),

        .gate_mmio_we(gate_mmio_we), .gate_mmio_addr(gate_mmio_addr),
        .gate_mmio_wdata(gate_mmio_wdata), .gate_mmio_wstrb(gate_mmio_wstrb),
        .gate_mmio_rdata(gate_mmio_rdata),

        .led_rd_req(led_rd_req), .led_rd_addr(led_rd_addr),
        .led_rd_data(led_rd_data), .led_idx(led_idx)
    );
endmodule
`default_nettype wire
