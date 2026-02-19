`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_interconnect.v  (Phase-A interconnect)
// - Two masters: CPU + DMA (DMA unused in Phase-A but stubbed)
// - Fixed priority: CPU over DMA
// - PRIV @ 0x2000_0000+ : always resp_err=1
// - 1-cycle fixed latency for all responses
// ============================================================================
`include "boreal_pkg.vh"

module boreal_interconnect (
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

    // DMA master (stub)
    input  wire        dma_req_valid,
    input  wire        dma_req_we,
    input  wire [31:0] dma_req_addr,
    input  wire [31:0] dma_req_wdata,
    input  wire [3:0]  dma_req_wstrb,
    output reg         dma_resp_valid,
    output reg  [31:0] dma_resp_rdata,
    output reg         dma_resp_err,

    // SRAM0 slave
    output reg         sram_req_valid,
    output reg         sram_req_we,
    output reg  [31:0] sram_req_addr,
    output reg  [31:0] sram_req_wdata,
    output reg  [3:0]  sram_req_wstrb,
    input  wire        sram_resp_valid,
    input  wire [31:0] sram_resp_rdata,
    input  wire        sram_resp_err,

    // MBOX MMIO
    output reg         mbox_we,
    output reg  [31:0] mbox_addr_word,
    output reg  [31:0] mbox_wdata,
    output reg  [3:0]  mbox_wstrb,
    input  wire [31:0] mbox_rdata,

    // VM MMIO
    output reg         vm_we,
    output reg  [7:0]  vm_addr,
    output reg  [31:0] vm_wdata,
    output reg  [3:0]  vm_wstrb,
    input  wire [31:0] vm_rdata,

    // Gate MMIO
    output reg         gate_we,
    output reg  [7:0]  gate_addr,
    output reg  [31:0] gate_wdata,
    output reg  [3:0]  gate_wstrb,
    input  wire [31:0] gate_rdata,

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

    // Address decode (page = addr[31:12])
    wire [3:0] top = m_req_addr[31:28];
    wire is_priv  = (top == `REGN_PRIV2);
    wire is_sram0 = (m_req_addr[31:12] == 20'h00001);  // 0x0000_1xxx
    wire is_mbox  = (m_req_addr[31:12] == 20'h10020);  // 0x1002_0xxx
    wire is_vm    = (m_req_addr[31:12] == 20'h10030);  // 0x1003_0xxx
    wire is_gate  = (m_req_addr[31:12] == 20'h10040);  // 0x1004_0xxx
    wire is_led   = (m_req_addr[31:12] == 20'h10050);  // 0x1005_0xxx

    // 2-stage pipeline: last_valid (req latched) -> resp_fire (slave output ready)
    reg        last_valid;
    reg        resp_fire;
    reg        last_use_cpu;
    reg [31:0] last_addr;
    reg        last_we;
    reg        last_is_priv, last_is_sram0, last_is_mbox, last_is_vm, last_is_gate, last_is_led;

    // Ledger word offset helper (module-scope for iverilog compat)
    reg [7:0] led_woff;

    always @(posedge clk) begin
        if (!rst_n) begin
            cpu_resp_valid <= 1'b0;
            cpu_resp_rdata <= 32'h0;
            cpu_resp_err   <= 1'b0;
            dma_resp_valid <= 1'b0;
            dma_resp_rdata <= 32'h0;
            dma_resp_err   <= 1'b0;

            sram_req_valid <= 1'b0;
            mbox_we    <= 1'b0;
            vm_we      <= 1'b0;
            gate_we    <= 1'b0;
            led_rd_req <= 1'b0;

            last_valid   <= 1'b0;
            resp_fire    <= 1'b0;
            last_use_cpu <= 1'b1;
            last_addr    <= 32'h0;
            last_we      <= 1'b0;
            last_is_priv <= 1'b0;
            last_is_sram0<= 1'b0;
            last_is_mbox <= 1'b0;
            last_is_vm   <= 1'b0;
            last_is_gate <= 1'b0;
            last_is_led  <= 1'b0;
            led_woff     <= 8'h0;
        end else begin
            // defaults: no response, clear slave enables
            cpu_resp_valid <= 1'b0;
            dma_resp_valid <= 1'b0;
            sram_req_valid <= 1'b0;
            mbox_we    <= 1'b0;
            vm_we      <= 1'b0;
            gate_we    <= 1'b0;
            led_rd_req <= 1'b0;

            // Pipeline advance: resp_fire = delayed last_valid
            resp_fire  <= last_valid;
            last_valid <= 1'b0;

            // ---------------------------------------------------------------
            // Launch request (captured into pipeline regs)
            // ---------------------------------------------------------------
            if (m_req_valid) begin
                last_valid   <= 1'b1;
                last_use_cpu <= use_cpu;
                last_addr    <= m_req_addr;
                last_we      <= m_req_we;

                last_is_priv  <= is_priv;
                last_is_sram0 <= is_sram0;
                last_is_mbox  <= is_mbox;
                last_is_vm    <= is_vm;
                last_is_gate  <= is_gate;
                last_is_led   <= is_led;

                if (is_sram0) begin
                    sram_req_valid <= 1'b1;
                    sram_req_we    <= m_req_we;
                    sram_req_addr  <= m_req_addr - `BASE_SRAM0;
                    sram_req_wdata <= m_req_wdata;
                    sram_req_wstrb <= m_req_wstrb;
                end else if (is_mbox) begin
                    mbox_we        <= m_req_we;
                    mbox_addr_word <= (m_req_addr - `BASE_MMIO_MBOX) >> 2;
                    mbox_wdata     <= m_req_wdata;
                    mbox_wstrb     <= m_req_wstrb;
                end else if (is_vm) begin
                    vm_we    <= m_req_we;
                    vm_addr  <= (m_req_addr - `BASE_MMIO_VM) >> 2;
                    vm_wdata <= m_req_wdata;
                    vm_wstrb <= m_req_wstrb;
                end else if (is_gate) begin
                    gate_we    <= m_req_we;
                    gate_addr  <= (m_req_addr - `BASE_MMIO_GATE) >> 2;
                    gate_wdata <= m_req_wdata;
                    gate_wstrb <= m_req_wstrb;
                end else if (is_led) begin
                    if (m_req_we && (((m_req_addr - `BASE_MMIO_LED) >> 2) == 8'h02)) begin
                        led_rd_req  <= 1'b1;
                        led_rd_addr <= m_req_wdata[9:0];
                    end
                end
            end

            // ---------------------------------------------------------------
            // Generate response (2 cycles after request, slave outputs ready)
            // ---------------------------------------------------------------
            if (resp_fire) begin
                if (last_is_priv) begin
                    // PRIV always errors
                    if (last_use_cpu) begin
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_rdata <= 32'h0;
                        cpu_resp_err   <= 1'b1;
                    end else begin
                        dma_resp_valid <= 1'b1;
                        dma_resp_rdata <= 32'h0;
                        dma_resp_err   <= 1'b1;
                    end
                end else if (last_is_sram0) begin
                    if (last_use_cpu) begin
                        cpu_resp_valid <= sram_resp_valid;
                        cpu_resp_rdata <= sram_resp_rdata;
                        cpu_resp_err   <= sram_resp_err;
                    end else begin
                        dma_resp_valid <= sram_resp_valid;
                        dma_resp_rdata <= sram_resp_rdata;
                        dma_resp_err   <= sram_resp_err;
                    end
                end else if (last_is_mbox) begin
                    if (last_use_cpu) begin
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_rdata <= mbox_rdata;
                        cpu_resp_err   <= 1'b0;
                    end else begin
                        dma_resp_valid <= 1'b1;
                        dma_resp_rdata <= mbox_rdata;
                        dma_resp_err   <= 1'b0;
                    end
                end else if (last_is_vm) begin
                    if (last_use_cpu) begin
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_rdata <= vm_rdata;
                        cpu_resp_err   <= 1'b0;
                    end else begin
                        dma_resp_valid <= 1'b1;
                        dma_resp_rdata <= vm_rdata;
                        dma_resp_err   <= 1'b0;
                    end
                end else if (last_is_gate) begin
                    if (last_use_cpu) begin
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_rdata <= gate_rdata;
                        cpu_resp_err   <= 1'b0;
                    end else begin
                        dma_resp_valid <= 1'b1;
                        dma_resp_rdata <= gate_rdata;
                        dma_resp_err   <= 1'b0;
                    end
                end else if (last_is_led) begin
                    led_woff = (last_addr - 32'h1005_0000) >> 2;
                    if (last_use_cpu) begin
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_err   <= 1'b0;
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
                            default: cpu_resp_rdata <= 32'h0;
                        endcase
                    end else begin
                        dma_resp_valid <= 1'b1;
                        dma_resp_err   <= 1'b0;
                        dma_resp_rdata <= 32'h0;
                    end
                end else begin
                    // unmapped -> error
                    if (last_use_cpu) begin
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_rdata <= 32'h0;
                        cpu_resp_err   <= 1'b1;
                    end else begin
                        dma_resp_valid <= 1'b1;
                        dma_resp_rdata <= 32'h0;
                        dma_resp_err   <= 1'b1;
                    end
                end
            end // last_valid
        end
    end
endmodule
`default_nettype wire
