// ============================================================================
// Boreal SoC - Bus Interconnect
// ============================================================================
// Simple 1-master-to-N-slave address-decoded crossbar.  Two master ports
// (public bus, gate bus) are arbitrated with fixed priority (gate wins).
// Slave select is purely combinational based on address bits.
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module boreal_interconnect (
    input  wire        clk,
    input  wire        rst_n,

    // --- Public master port (CPU / DMA / external) ---
    input  wire        pub_req,
    input  wire        pub_wr,
    input  wire [31:0] pub_addr,
    input  wire [31:0] pub_wdata,
    input  wire [ 3:0] pub_strb,
    output reg  [31:0] pub_rdata,
    output reg         pub_ack,
    output reg         pub_err,

    // --- Gate master port (privileged) ---
    input  wire        gate_req,
    input  wire        gate_wr,
    input  wire [31:0] gate_addr,
    input  wire [31:0] gate_wdata,
    input  wire [ 3:0] gate_strb,
    output reg  [31:0] gate_rdata,
    output reg         gate_ack,
    output reg         gate_err,

    // --- Slave ports ---
    // Boot ROM
    output wire        rom_sel,
    output wire [31:0] rom_addr,
    input  wire [31:0] rom_rdata,
    input  wire        rom_ack,

    // SRAM
    output wire        sram_sel,
    output wire        sram_wr,
    output wire [31:0] sram_addr,
    output wire [31:0] sram_wdata,
    output wire [ 3:0] sram_strb,
    input  wire [31:0] sram_rdata,
    input  wire        sram_ack,

    // DMA
    output wire        dma_sel,
    output wire        dma_wr,
    output wire [31:0] dma_addr,
    output wire [31:0] dma_wdata,
    input  wire [31:0] dma_rdata,
    input  wire        dma_ack,

    // Vector Engine
    output wire        vec_sel,
    output wire        vec_wr,
    output wire [31:0] vec_addr,
    output wire [31:0] vec_wdata,
    input  wire [31:0] vec_rdata,
    input  wire        vec_ack,

    // AI Mailbox
    output wire        aimbox_sel,
    output wire        aimbox_wr,
    output wire [31:0] aimbox_addr,
    output wire [31:0] aimbox_wdata,
    input  wire [31:0] aimbox_rdata,
    input  wire        aimbox_ack,

    // Decision-VM
    output wire        dvm_sel,
    output wire        dvm_wr,
    output wire [31:0] dvm_addr,
    output wire [31:0] dvm_wdata,
    input  wire [31:0] dvm_rdata,
    input  wire        dvm_ack,

    // Gate registers
    output wire        gatereg_sel,
    output wire        gatereg_wr,
    output wire [31:0] gatereg_addr,
    output wire [31:0] gatereg_wdata,
    input  wire [31:0] gatereg_rdata,
    input  wire        gatereg_ack,

    // Ledger
    output wire        ledger_sel,
    output wire        ledger_wr,
    output wire [31:0] ledger_addr,
    output wire [31:0] ledger_wdata,
    input  wire [31:0] ledger_rdata,
    input  wire        ledger_ack,

    // Privileged I/O
    output wire        priv_sel,
    output wire        priv_wr,
    output wire [31:0] priv_addr,
    output wire [31:0] priv_wdata,
    input  wire [31:0] priv_rdata,
    input  wire        priv_ack
);

    // -----------------------------------------------------------------------
    // Arbitration – gate always wins when both request simultaneously
    // -----------------------------------------------------------------------
    wire        arb_req;
    wire        arb_wr;
    wire [31:0] arb_addr;
    wire [31:0] arb_wdata;
    wire [ 3:0] arb_strb;
    wire        arb_is_gate;

    assign arb_is_gate = gate_req;
    assign arb_req     = gate_req | pub_req;
    assign arb_wr      = arb_is_gate ? gate_wr    : pub_wr;
    assign arb_addr    = arb_is_gate ? gate_addr   : pub_addr;
    assign arb_wdata   = arb_is_gate ? gate_wdata  : pub_wdata;
    assign arb_strb    = arb_is_gate ? gate_strb   : pub_strb;

    // -----------------------------------------------------------------------
    // Address decode – generate slave selects
    // -----------------------------------------------------------------------
    wire sel_rom    = arb_req && (arb_addr[31:12] == 20'h0000_0);  // 0x0000_0xxx
    wire sel_sram   = arb_req && (arb_addr[31:12] == 20'h0000_1);  // 0x0000_1xxx
    wire sel_dma    = arb_req && (arb_addr[31:16] == 16'h1000);     // 0x1000_xxxx
    wire sel_vec    = arb_req && (arb_addr[31:16] == 16'h1001);     // 0x1001_xxxx
    wire sel_aimbox = arb_req && (arb_addr[31:16] == 16'h1002);     // 0x1002_xxxx
    wire sel_dvm    = arb_req && (arb_addr[31:16] == 16'h1003);     // 0x1003_xxxx
    wire sel_gate   = arb_req && (arb_addr[31:16] == 16'h1004);     // 0x1004_xxxx
    wire sel_ledger = arb_req && (arb_addr[31:16] == 16'h1005);     // 0x1005_xxxx
    wire sel_priv   = arb_req && (arb_addr[31:28] == 4'h2);         // 0x2xxx_xxxx

    // -----------------------------------------------------------------------
    // Security: public bus CANNOT access privileged I/O region
    // -----------------------------------------------------------------------
    wire pub_priv_violation = pub_req && !arb_is_gate && sel_priv;

    // -----------------------------------------------------------------------
    // Drive slave ports
    // -----------------------------------------------------------------------
    assign rom_sel     = sel_rom;
    assign rom_addr    = arb_addr;

    assign sram_sel    = sel_sram;
    assign sram_wr     = arb_wr;
    assign sram_addr   = arb_addr;
    assign sram_wdata  = arb_wdata;
    assign sram_strb   = arb_strb;

    assign dma_sel     = sel_dma;
    assign dma_wr      = arb_wr;
    assign dma_addr    = arb_addr;
    assign dma_wdata   = arb_wdata;

    assign vec_sel     = sel_vec;
    assign vec_wr      = arb_wr;
    assign vec_addr    = arb_addr;
    assign vec_wdata   = arb_wdata;

    assign aimbox_sel  = sel_aimbox;
    assign aimbox_wr   = arb_wr;
    assign aimbox_addr = arb_addr;
    assign aimbox_wdata= arb_wdata;

    assign dvm_sel     = sel_dvm;
    assign dvm_wr      = arb_wr;
    assign dvm_addr    = arb_addr;
    assign dvm_wdata   = arb_wdata;

    assign gatereg_sel = sel_gate;
    assign gatereg_wr  = arb_wr;
    assign gatereg_addr= arb_addr;
    assign gatereg_wdata=arb_wdata;

    assign ledger_sel  = sel_ledger;
    assign ledger_wr   = arb_wr;
    assign ledger_addr = arb_addr;
    assign ledger_wdata= arb_wdata;

    assign priv_sel    = sel_priv && arb_is_gate;  // only gate can access
    assign priv_wr     = arb_wr;
    assign priv_addr   = arb_addr;
    assign priv_wdata  = arb_wdata;

    // -----------------------------------------------------------------------
    // Read-data mux & ack routing
    // -----------------------------------------------------------------------
    reg [31:0] mux_rdata;
    reg        mux_ack;
    reg        mux_err;

    always @(*) begin
        mux_rdata = 32'h0;
        mux_ack   = 1'b0;
        mux_err   = 1'b0;

        if (pub_priv_violation) begin
            mux_err = 1'b1;
            mux_ack = 1'b1;
        end else if (sel_rom)    begin mux_rdata = rom_rdata;    mux_ack = rom_ack;    end
        else if (sel_sram)       begin mux_rdata = sram_rdata;   mux_ack = sram_ack;   end
        else if (sel_dma)        begin mux_rdata = dma_rdata;    mux_ack = dma_ack;    end
        else if (sel_vec)        begin mux_rdata = vec_rdata;    mux_ack = vec_ack;    end
        else if (sel_aimbox)     begin mux_rdata = aimbox_rdata; mux_ack = aimbox_ack; end
        else if (sel_dvm)        begin mux_rdata = dvm_rdata;    mux_ack = dvm_ack;    end
        else if (sel_gate)       begin mux_rdata = gatereg_rdata;mux_ack = gatereg_ack;end
        else if (sel_ledger)     begin mux_rdata = ledger_rdata; mux_ack = ledger_ack; end
        else if (sel_priv)       begin mux_rdata = priv_rdata;   mux_ack = priv_ack;   end
        else if (arb_req)        begin mux_err   = 1'b1;         mux_ack = 1'b1;       end
    end

    // Route responses back to the requesting master
    always @(*) begin
        pub_rdata  = 32'h0;
        pub_ack    = 1'b0;
        pub_err    = 1'b0;
        gate_rdata = 32'h0;
        gate_ack   = 1'b0;
        gate_err   = 1'b0;

        if (arb_is_gate) begin
            gate_rdata = mux_rdata;
            gate_ack   = mux_ack;
            gate_err   = mux_err;
        end else begin
            pub_rdata  = mux_rdata;
            pub_ack    = mux_ack;
            pub_err    = mux_err;
        end
    end

endmodule
