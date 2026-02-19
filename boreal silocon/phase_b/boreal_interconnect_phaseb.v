`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_interconnect_phaseb.v (Phase-B)
// - Two masters: CPU + DMA (CPU priority)
// - Slaves: SRAM0, DMA MMIO, VEC MMIO, MBOX, VM, GATE, LEDGER
// - PRIV hard blocked for ALL public masters
// - 2-cycle fixed latency pipeline
// ============================================================================
`include "boreal_pkg.vh"

module boreal_interconnect_phaseb (
    input  wire        clk,
    input  wire        rst_n,

    // CPU master
    input  wire        cpu_req_valid,
    input  wire        cpu_req_we,
    input  wire [31:0] cpu_req_addr,
    input  wire [31:0] cpu_req_wdata,
    input  wire [3:0]  cpu_req_wstrb,
    output reg         cpu_resp_valid,
    output reg  [31:0] cpu_resp_rdata,
    output reg         cpu_resp_err,

    // DMA master
    input  wire        dma_req_valid,
    input  wire        dma_req_we,
    input  wire [31:0] dma_req_addr,
    input  wire [31:0] dma_req_wdata,
    input  wire [3:0]  dma_req_wstrb,
    output reg         dma_resp_valid,
    output reg  [31:0] dma_resp_rdata,
    output reg         dma_resp_err,

    // SRAM0
    output reg         sram_req_valid,
    output reg         sram_req_we,
    output reg  [31:0] sram_req_addr,
    output reg  [31:0] sram_req_wdata,
    output reg  [3:0]  sram_req_wstrb,
    input  wire        sram_resp_valid,
    input  wire [31:0] sram_resp_rdata,
    input  wire        sram_resp_err,

    // DMA MMIO
    output reg         dma_mmio_we,
    output reg  [7:0]  dma_mmio_addr,
    output reg  [31:0] dma_mmio_wdata,
    output reg  [3:0]  dma_mmio_wstrb,
    input  wire [31:0] dma_mmio_rdata,

    // VEC MMIO
    output reg         vec_mmio_we,
    output reg  [10:0] vec_mmio_addr,
    output reg  [31:0] vec_mmio_wdata,
    output reg  [3:0]  vec_mmio_wstrb,
    input  wire [31:0] vec_mmio_rdata,

    // MBOX MMIO
    output reg         mbox_mmio_we,
    output reg  [7:0]  mbox_mmio_addr_word,
    output reg  [31:0] mbox_mmio_wdata,
    output reg  [3:0]  mbox_mmio_wstrb,
    input  wire [31:0] mbox_mmio_rdata,

    // VM MMIO
    output reg         vm_mmio_we,
    output reg  [7:0]  vm_mmio_addr,
    output reg  [31:0] vm_mmio_wdata,
    output reg  [3:0]  vm_mmio_wstrb,
    input  wire [31:0] vm_mmio_rdata,

    // Gate MMIO
    output reg         gate_mmio_we,
    output reg  [7:0]  gate_mmio_addr,
    output reg  [31:0] gate_mmio_wdata,
    output reg  [3:0]  gate_mmio_wstrb,
    input  wire [31:0] gate_mmio_rdata,

    // Ledger MMIO
    output reg         led_rd_req,
    output reg  [9:0]  led_rd_addr,
    input  wire [255:0] led_rd_data,
    input  wire [31:0]  led_idx
);
    // Select master (CPU priority)
    wire use_cpu = cpu_req_valid;
    wire m_req_valid = use_cpu ? cpu_req_valid : dma_req_valid;
    wire m_req_we    = use_cpu ? cpu_req_we    : dma_req_we;
    wire [31:0] m_req_addr  = use_cpu ? cpu_req_addr  : dma_req_addr;
    wire [31:0] m_req_wdata = use_cpu ? cpu_req_wdata : dma_req_wdata;
    wire [3:0]  m_req_wstrb = use_cpu ? cpu_req_wstrb : dma_req_wstrb;

    // Address decode
    wire is_priv  = (m_req_addr[31:28] == `REGN_PRIV2);
    wire is_sram0 = (m_req_addr[31:12] == 20'h00001);
    wire is_dma   = (m_req_addr[31:12] == 20'h10000);
    wire is_vec   = (m_req_addr[31:16] == 16'h1001);  // 64KB VEC region
    wire is_mbox  = (m_req_addr[31:12] == 20'h10020);
    wire is_vm    = (m_req_addr[31:12] == 20'h10030);
    wire is_gate  = (m_req_addr[31:12] == 20'h10040);
    wire is_led   = (m_req_addr[31:12] == 20'h10050);

    // 2-stage pipeline
    reg        last_valid;
    reg        resp_fire;
    reg        last_use_cpu;
    reg [31:0] last_addr;
    reg        last_is_priv, last_is_sram0, last_is_dma, last_is_vec;
    reg        last_is_mbox, last_is_vm, last_is_gate, last_is_led;
    reg [7:0]  led_woff;

    always @(posedge clk) begin
        if (!rst_n) begin
            cpu_resp_valid<=0; cpu_resp_rdata<=0; cpu_resp_err<=0;
            dma_resp_valid<=0; dma_resp_rdata<=0; dma_resp_err<=0;
            sram_req_valid<=0;
            dma_mmio_we<=0; vec_mmio_we<=0; mbox_mmio_we<=0;
            vm_mmio_we<=0; gate_mmio_we<=0; led_rd_req<=0;
            last_valid<=0; resp_fire<=0; last_use_cpu<=1; last_addr<=0;
            last_is_priv<=0; last_is_sram0<=0; last_is_dma<=0; last_is_vec<=0;
            last_is_mbox<=0; last_is_vm<=0; last_is_gate<=0; last_is_led<=0;
            led_woff<=0;
        end else begin
            cpu_resp_valid<=0; dma_resp_valid<=0;
            sram_req_valid<=0;
            dma_mmio_we<=0; vec_mmio_we<=0; mbox_mmio_we<=0;
            vm_mmio_we<=0; gate_mmio_we<=0; led_rd_req<=0;

            resp_fire  <= last_valid;
            last_valid <= 1'b0;

            // Launch request
            if (m_req_valid) begin
                last_valid   <= 1'b1;
                last_use_cpu <= use_cpu;
                last_addr    <= m_req_addr;
                last_is_priv <= is_priv;
                last_is_sram0<= is_sram0;
                last_is_dma  <= is_dma;
                last_is_vec  <= is_vec;
                last_is_mbox <= is_mbox;
                last_is_vm   <= is_vm;
                last_is_gate <= is_gate;
                last_is_led  <= is_led;

                if (is_sram0) begin
                    sram_req_valid <= 1; sram_req_we <= m_req_we;
                    sram_req_addr <= m_req_addr - `BASE_SRAM0;
                    sram_req_wdata <= m_req_wdata; sram_req_wstrb <= m_req_wstrb;
                end else if (is_dma) begin
                    dma_mmio_we <= m_req_we;
                    dma_mmio_addr <= (m_req_addr - `BASE_MMIO_DMA) >> 2;
                    dma_mmio_wdata <= m_req_wdata; dma_mmio_wstrb <= m_req_wstrb;
                end else if (is_vec) begin
                    vec_mmio_we <= m_req_we;
                    vec_mmio_addr <= (m_req_addr - `BASE_MMIO_VEC) >> 2;
                    vec_mmio_wdata <= m_req_wdata; vec_mmio_wstrb <= m_req_wstrb;
                end else if (is_mbox) begin
                    mbox_mmio_we <= m_req_we;
                    mbox_mmio_addr_word <= (m_req_addr - `BASE_MMIO_MBOX) >> 2;
                    mbox_mmio_wdata <= m_req_wdata; mbox_mmio_wstrb <= m_req_wstrb;
                end else if (is_vm) begin
                    vm_mmio_we <= m_req_we;
                    vm_mmio_addr <= (m_req_addr - `BASE_MMIO_VM) >> 2;
                    vm_mmio_wdata <= m_req_wdata; vm_mmio_wstrb <= m_req_wstrb;
                end else if (is_gate) begin
                    gate_mmio_we <= m_req_we;
                    gate_mmio_addr <= (m_req_addr - `BASE_MMIO_GATE) >> 2;
                    gate_mmio_wdata <= m_req_wdata; gate_mmio_wstrb <= m_req_wstrb;
                end else if (is_led) begin
                    if (m_req_we && (((m_req_addr - `BASE_MMIO_LED) >> 2) == 8'h02)) begin
                        led_rd_req <= 1; led_rd_addr <= m_req_wdata[9:0];
                    end
                end
            end

            // Generate response (2-cycle pipeline)
            if (resp_fire) begin
                if (last_is_priv) begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=0; cpu_resp_err<=1; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=0; dma_resp_err<=1; end
                end else if (last_is_sram0) begin
                    if (last_use_cpu) begin cpu_resp_valid<=sram_resp_valid; cpu_resp_rdata<=sram_resp_rdata; cpu_resp_err<=sram_resp_err; end
                    else              begin dma_resp_valid<=sram_resp_valid; dma_resp_rdata<=sram_resp_rdata; dma_resp_err<=sram_resp_err; end
                end else if (last_is_dma) begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=dma_mmio_rdata; cpu_resp_err<=0; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=dma_mmio_rdata; dma_resp_err<=0; end
                end else if (last_is_vec) begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=vec_mmio_rdata; cpu_resp_err<=0; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=vec_mmio_rdata; dma_resp_err<=0; end
                end else if (last_is_mbox) begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=mbox_mmio_rdata; cpu_resp_err<=0; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=mbox_mmio_rdata; dma_resp_err<=0; end
                end else if (last_is_vm) begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=vm_mmio_rdata; cpu_resp_err<=0; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=vm_mmio_rdata; dma_resp_err<=0; end
                end else if (last_is_gate) begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=gate_mmio_rdata; cpu_resp_err<=0; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=gate_mmio_rdata; dma_resp_err<=0; end
                end else if (last_is_led) begin
                    led_woff = (last_addr - 32'h1005_0000) >> 2;
                    if (last_use_cpu) begin
                        cpu_resp_valid<=1; cpu_resp_err<=0;
                        case (led_woff)
                            8'h00: cpu_resp_rdata <= led_idx;
                            8'h01: cpu_resp_rdata <= 32'd1024;
                            8'h03: cpu_resp_rdata <= led_rd_data[ 31:  0];
                            8'h04: cpu_resp_rdata <= led_rd_data[ 63: 32];
                            8'h05: cpu_resp_rdata <= led_rd_data[ 95: 64];
                            8'h06: cpu_resp_rdata <= led_rd_data[127: 96];
                            8'h07: cpu_resp_rdata <= led_rd_data[159:128];
                            8'h08: cpu_resp_rdata <= led_rd_data[191:160];
                            8'h09: cpu_resp_rdata <= led_rd_data[223:192];
                            8'h0A: cpu_resp_rdata <= led_rd_data[255:224];
                            default: cpu_resp_rdata <= 0;
                        endcase
                    end else begin
                        dma_resp_valid<=1; dma_resp_err<=0; dma_resp_rdata<=0;
                    end
                end else begin
                    if (last_use_cpu) begin cpu_resp_valid<=1; cpu_resp_rdata<=0; cpu_resp_err<=1; end
                    else              begin dma_resp_valid<=1; dma_resp_rdata<=0; dma_resp_err<=1; end
                end
            end
        end
    end
endmodule
`default_nettype wire
