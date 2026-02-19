// ============================================================================
// Formal wrapper for boreal_interconnect
// ============================================================================
`include "boreal_pkg.v"

module fv_interconnect_top (
    input wire        clk,
    input wire        rst_n,
    input wire        pub_req,
    input wire        pub_wr,
    input wire [31:0] pub_addr,
    input wire [31:0] pub_wdata,
    input wire [ 3:0] pub_strb,
    input wire        gate_req,
    input wire        gate_wr,
    input wire [31:0] gate_addr,
    input wire [31:0] gate_wdata,
    input wire [ 3:0] gate_strb,
    input wire [31:0] rom_rdata,
    input wire        rom_ack,
    input wire [31:0] sram_rdata,
    input wire        sram_ack,
    input wire [31:0] dma_rdata,
    input wire        dma_ack,
    input wire [31:0] vec_rdata,
    input wire        vec_ack,
    input wire [31:0] aimbox_rdata,
    input wire        aimbox_ack,
    input wire [31:0] dvm_rdata,
    input wire        dvm_ack,
    input wire [31:0] gatereg_rdata,
    input wire        gatereg_ack,
    input wire [31:0] ledger_rdata,
    input wire        ledger_ack,
    input wire [31:0] priv_rdata,
    input wire        priv_ack
);

    // Reset assumption: rst_n low at init, then high
    reg init_done = 0;
    always @(posedge clk) begin
        if (!init_done) begin
            assume (!rst_n);
            init_done <= 1;
        end else begin
            assume (rst_n);
        end
    end

    wire [31:0] pub_rdata_o;
    wire        pub_ack_o, pub_err_o;
    wire [31:0] gate_rdata_o;
    wire        gate_ack_o, gate_err_o;

    wire rom_sel_o;
    wire [31:0] rom_addr_o;
    wire sram_sel_o, sram_wr_o;
    wire [31:0] sram_addr_o, sram_wdata_o;
    wire [3:0]  sram_strb_o;
    wire dma_sel_o, dma_wr_o, vec_sel_o, vec_wr_o;
    wire [31:0] dma_addr_o, dma_wdata_o, vec_addr_o, vec_wdata_o;
    wire aimbox_sel_o, aimbox_wr_o, dvm_sel_o, dvm_wr_o;
    wire [31:0] aimbox_addr_o, aimbox_wdata_o, dvm_addr_o, dvm_wdata_o;
    wire gatereg_sel_o, gatereg_wr_o, ledger_sel_o, ledger_wr_o;
    wire [31:0] gatereg_addr_o, gatereg_wdata_o, ledger_addr_o, ledger_wdata_o;
    wire priv_sel_o, priv_wr_o;
    wire [31:0] priv_addr_o, priv_wdata_o;

    boreal_interconnect u_dut (
        .clk(clk), .rst_n(rst_n),
        .pub_req(pub_req), .pub_wr(pub_wr), .pub_addr(pub_addr),
        .pub_wdata(pub_wdata), .pub_strb(pub_strb),
        .pub_rdata(pub_rdata_o), .pub_ack(pub_ack_o), .pub_err(pub_err_o),
        .gate_req(gate_req), .gate_wr(gate_wr), .gate_addr(gate_addr),
        .gate_wdata(gate_wdata), .gate_strb(gate_strb),
        .gate_rdata(gate_rdata_o), .gate_ack(gate_ack_o), .gate_err(gate_err_o),
        .rom_sel(rom_sel_o), .rom_addr(rom_addr_o),
        .rom_rdata(rom_rdata), .rom_ack(rom_ack),
        .sram_sel(sram_sel_o), .sram_wr(sram_wr_o), .sram_addr(sram_addr_o),
        .sram_wdata(sram_wdata_o), .sram_strb(sram_strb_o),
        .sram_rdata(sram_rdata), .sram_ack(sram_ack),
        .dma_sel(dma_sel_o), .dma_wr(dma_wr_o), .dma_addr(dma_addr_o),
        .dma_wdata(dma_wdata_o), .dma_rdata(dma_rdata), .dma_ack(dma_ack),
        .vec_sel(vec_sel_o), .vec_wr(vec_wr_o), .vec_addr(vec_addr_o),
        .vec_wdata(vec_wdata_o), .vec_rdata(vec_rdata), .vec_ack(vec_ack),
        .aimbox_sel(aimbox_sel_o), .aimbox_wr(aimbox_wr_o), .aimbox_addr(aimbox_addr_o),
        .aimbox_wdata(aimbox_wdata_o), .aimbox_rdata(aimbox_rdata), .aimbox_ack(aimbox_ack),
        .dvm_sel(dvm_sel_o), .dvm_wr(dvm_wr_o), .dvm_addr(dvm_addr_o),
        .dvm_wdata(dvm_wdata_o), .dvm_rdata(dvm_rdata), .dvm_ack(dvm_ack),
        .gatereg_sel(gatereg_sel_o), .gatereg_wr(gatereg_wr_o), .gatereg_addr(gatereg_addr_o),
        .gatereg_wdata(gatereg_wdata_o), .gatereg_rdata(gatereg_rdata), .gatereg_ack(gatereg_ack),
        .ledger_sel(ledger_sel_o), .ledger_wr(ledger_wr_o), .ledger_addr(ledger_addr_o),
        .ledger_wdata(ledger_wdata_o), .ledger_rdata(ledger_rdata), .ledger_ack(ledger_ack),
        .priv_sel(priv_sel_o), .priv_wr(priv_wr_o), .priv_addr(priv_addr_o),
        .priv_wdata(priv_wdata_o), .priv_rdata(priv_rdata), .priv_ack(priv_ack)
    );

    // Re-derive internal combinational signals (Yosys can't do hierarchical refs)
    wire w_arb_is_gate = gate_req;
    wire w_arb_req     = gate_req | pub_req;
    wire [31:0] w_arb_addr = w_arb_is_gate ? gate_addr : pub_addr;

    wire w_sel_rom    = w_arb_req && (w_arb_addr[31:12] == 20'h0000_0);
    wire w_sel_sram   = w_arb_req && (w_arb_addr[31:12] == 20'h0000_1);
    wire w_sel_dma    = w_arb_req && (w_arb_addr[31:16] == 16'h1000);
    wire w_sel_vec    = w_arb_req && (w_arb_addr[31:16] == 16'h1001);
    wire w_sel_aimbox = w_arb_req && (w_arb_addr[31:16] == 16'h1002);
    wire w_sel_dvm    = w_arb_req && (w_arb_addr[31:16] == 16'h1003);
    wire w_sel_gate   = w_arb_req && (w_arb_addr[31:16] == 16'h1004);
    wire w_sel_ledger = w_arb_req && (w_arb_addr[31:16] == 16'h1005);
    wire w_sel_priv   = w_arb_req && (w_arb_addr[31:28] == 4'h2);
    wire w_pub_priv_v = pub_req && !w_arb_is_gate && w_sel_priv;

    fv_interconnect_props u_props (
        .clk(clk), .rst_n(rst_n),
        .pub_req(pub_req), .pub_addr(pub_addr),
        .pub_ack(pub_ack_o), .pub_err(pub_err_o),
        .gate_req(gate_req), .gate_addr(gate_addr), .gate_ack(gate_ack_o),
        .arb_is_gate(w_arb_is_gate),
        .arb_req(w_arb_req),
        .arb_addr(w_arb_addr),
        .sel_priv(w_sel_priv),
        .priv_sel(priv_sel_o),
        .pub_priv_violation(w_pub_priv_v),
        .sel_rom(w_sel_rom),
        .sel_sram(w_sel_sram),
        .sel_dma(w_sel_dma),
        .sel_vec(w_sel_vec),
        .sel_aimbox(w_sel_aimbox),
        .sel_dvm(w_sel_dvm),
        .sel_gate(w_sel_gate),
        .sel_ledger(w_sel_ledger)
    );

endmodule
