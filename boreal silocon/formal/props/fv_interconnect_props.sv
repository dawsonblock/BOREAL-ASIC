// ============================================================================
// Formal properties for boreal_interconnect
// ============================================================================
`include "boreal_pkg.v"

module fv_interconnect_props (
    input wire        clk,
    input wire        rst_n,
    input wire        pub_req,
    input wire [31:0] pub_addr,
    input wire        pub_ack,
    input wire        pub_err,
    input wire        gate_req,
    input wire [31:0] gate_addr,
    input wire        gate_ack,
    input wire        arb_is_gate,
    input wire        arb_req,
    input wire [31:0] arb_addr,
    input wire        sel_priv,
    input wire        priv_sel,
    input wire        pub_priv_violation,
    input wire        sel_rom,
    input wire        sel_sram,
    input wire        sel_dma,
    input wire        sel_vec,
    input wire        sel_aimbox,
    input wire        sel_dvm,
    input wire        sel_gate,
    input wire        sel_ledger
);

    // P1: Public master accessing PRIV → error + priv_sel=0
    wire p1_cond = rst_n && pub_req && !gate_req && (pub_addr[31:28] == 4'h2);
    always @(*) if (p1_cond) assert (pub_err && !priv_sel);

    // P2: priv_sel → gate is active master
    always @(*) if (rst_n && priv_sel) assert (arb_is_gate);

    // P3: No request → no slave selected
    wire no_sel = !sel_rom && !sel_sram && !sel_dma && !sel_vec &&
                  !sel_aimbox && !sel_dvm && !sel_gate && !sel_ledger && !sel_priv;
    always @(*) if (rst_n && !arb_req) assert (no_sel);

    // P5: Gate always wins arbitration
    always @(*) if (rst_n && gate_req) assert (arb_is_gate);

endmodule
