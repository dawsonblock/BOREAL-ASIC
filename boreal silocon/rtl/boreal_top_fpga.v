// ============================================================================
// Boreal SoC - Top-Level FPGA Wrapper
// ============================================================================
// Instantiates all modules and wires the interconnect, gate pipeline,
// compute elements, decision pipeline, and secure boot subsystem.
// ============================================================================
`include "boreal_pkg.v"

module boreal_top_fpga (
    input  wire        clk,
    input  wire        rst_n_pin,

    // --- External bus master (debug / host) ---
    input  wire        ext_req,
    input  wire        ext_wr,
    input  wire [31:0] ext_addr,
    input  wire [31:0] ext_wdata,
    input  wire [ 3:0] ext_strb,
    output wire [31:0] ext_rdata,
    output wire        ext_ack,
    output wire        ext_err,

    // --- Physical I/O outputs ---
    output wire [31:0] pio_out_0,
    output wire [31:0] pio_out_1,
    output wire [31:0] pio_out_2,
    output wire [31:0] pio_out_3,

    // --- Boot status ---
    output wire        boot_done,
    output wire        boot_pass,

    // --- Signature verify configuration (typically from fuses) ---
    input  wire [31:0] fuse_expected_hash,
    input  wire [31:0] fuse_min_version,
    input  wire [31:0] fuse_image_version
);

    // -----------------------------------------------------------------------
    // Reset gating – hold system in reset until boot passes
    // -----------------------------------------------------------------------
    wire rst_n_internal;
    wire boot_done_w, boot_pass_w;

    assign rst_n_internal = rst_n_pin & boot_pass_w;
    assign boot_done      = boot_done_w;
    assign boot_pass      = boot_pass_w;

    // Boot ROM runs on raw pin reset (always active)
    wire rst_n_boot = rst_n_pin;

    // -----------------------------------------------------------------------
    // Interconnect wires
    // -----------------------------------------------------------------------
    // ROM slave
    wire        rom_sel;
    wire [31:0] rom_addr, rom_rdata;
    wire        rom_ack;

    // SRAM slave
    wire        sram_sel, sram_wr, sram_ack;
    wire [31:0] sram_addr, sram_wdata, sram_rdata;
    wire [ 3:0] sram_strb;

    // DMA slave
    wire        dma_sel, dma_wr, dma_ack;
    wire [31:0] dma_addr, dma_wdata, dma_rdata;

    // Vector engine slave
    wire        vec_sel, vec_wr, vec_ack;
    wire [31:0] vec_addr, vec_wdata, vec_rdata;

    // AI mailbox slave
    wire        aimbox_sel, aimbox_wr, aimbox_ack;
    wire [31:0] aimbox_addr, aimbox_wdata, aimbox_rdata;

    // Decision-VM slave
    wire        dvm_sel, dvm_wr, dvm_ack;
    wire [31:0] dvm_addr, dvm_wdata, dvm_rdata;

    // Gate registers slave
    wire        gatereg_sel, gatereg_wr, gatereg_ack;
    wire [31:0] gatereg_addr, gatereg_wdata, gatereg_rdata;

    // Ledger slave
    wire        ledger_sel, ledger_wr, ledger_ack;
    wire [31:0] ledger_addr, ledger_wdata, ledger_rdata;

    // Privileged I/O slave
    wire        priv_sel, priv_wr, priv_ack;
    wire [31:0] priv_addr, priv_wdata, priv_rdata;

    // Gate master port
    wire        gate_bus_req, gate_bus_wr, gate_bus_ack, gate_bus_err;
    wire [31:0] gate_bus_addr, gate_bus_wdata, gate_bus_rdata;
    wire [ 3:0] gate_bus_strb;

    // -----------------------------------------------------------------------
    // Gate internal wiring
    // -----------------------------------------------------------------------
    wire [31:0] policy_allow0, policy_allow1;
    wire [31:0] policy_rate_limit, policy_rate_window;
    wire [31:0] policy_hash, policy_override;
    wire [31:0] nonce_counter;

    // Gate -> Ledger
    wire        gate_ledger_wr_en;
    wire [255:0] gate_ledger_wr_data;
    wire [31:0] ledger_idx;

    // Decision-VM -> Gate action request
    wire        act_valid, act_ready;
    wire [31:0] act_opcode, act_target, act_arg0, act_arg1;
    wire [31:0] act_context_hash, act_policy_hash, act_bounds, act_nonce;

    // Gate -> response
    wire        resp_valid;
    wire [31:0] resp_committed, resp_reason, resp_applied0, resp_applied1, resp_ledger_idx;

    // AI Mailbox -> Decision-VM
    wire [ 3:0] mb_rd_idx;
    wire        mb_rd_slot;
    wire [31:0] mb_rd_data;
    wire        mb_slot0_valid, mb_slot1_valid;
    wire        mb_slot0_ack, mb_slot1_ack;

    // DMA -> SRAM
    wire        dma_mem_sel, dma_mem_wr, dma_mem_ack;
    wire [ 9:0] dma_mem_addr;
    wire [31:0] dma_mem_wdata, dma_mem_rdata;

    // SHA / Sig wires
    wire        sha_start, sha_update, sha_ready;
    wire [31:0] sha_data, sha_hash;
    wire        sig_start, sig_pass, sig_ready;
    wire [31:0] sig_hash_in;
    wire [31:0] boot_hash_out;

    // -----------------------------------------------------------------------
    // Interconnect
    // -----------------------------------------------------------------------
    boreal_interconnect u_xbar (
        .clk          (clk),
        .rst_n        (rst_n_internal),

        // Public master (external debug/host)
        .pub_req      (ext_req),
        .pub_wr       (ext_wr),
        .pub_addr     (ext_addr),
        .pub_wdata    (ext_wdata),
        .pub_strb     (ext_strb),
        .pub_rdata    (ext_rdata),
        .pub_ack      (ext_ack),
        .pub_err      (ext_err),

        // Gate master
        .gate_req     (gate_bus_req),
        .gate_wr      (gate_bus_wr),
        .gate_addr    (gate_bus_addr),
        .gate_wdata   (gate_bus_wdata),
        .gate_strb    (gate_bus_strb),
        .gate_rdata   (gate_bus_rdata),
        .gate_ack     (gate_bus_ack),
        .gate_err     (gate_bus_err),

        // Slaves
        .rom_sel      (rom_sel),
        .rom_addr     (rom_addr),
        .rom_rdata    (rom_rdata),
        .rom_ack      (rom_ack),

        .sram_sel     (sram_sel),
        .sram_wr      (sram_wr),
        .sram_addr    (sram_addr),
        .sram_wdata   (sram_wdata),
        .sram_strb    (sram_strb),
        .sram_rdata   (sram_rdata),
        .sram_ack     (sram_ack),

        .dma_sel      (dma_sel),
        .dma_wr       (dma_wr),
        .dma_addr     (dma_addr),
        .dma_wdata    (dma_wdata),
        .dma_rdata    (dma_rdata),
        .dma_ack      (dma_ack),

        .vec_sel      (vec_sel),
        .vec_wr       (vec_wr),
        .vec_addr     (vec_addr),
        .vec_wdata    (vec_wdata),
        .vec_rdata    (vec_rdata),
        .vec_ack      (vec_ack),

        .aimbox_sel   (aimbox_sel),
        .aimbox_wr    (aimbox_wr),
        .aimbox_addr  (aimbox_addr),
        .aimbox_wdata (aimbox_wdata),
        .aimbox_rdata (aimbox_rdata),
        .aimbox_ack   (aimbox_ack),

        .dvm_sel      (dvm_sel),
        .dvm_wr       (dvm_wr),
        .dvm_addr     (dvm_addr),
        .dvm_wdata    (dvm_wdata),
        .dvm_rdata    (dvm_rdata),
        .dvm_ack      (dvm_ack),

        .gatereg_sel  (gatereg_sel),
        .gatereg_wr   (gatereg_wr),
        .gatereg_addr (gatereg_addr),
        .gatereg_wdata(gatereg_wdata),
        .gatereg_rdata(gatereg_rdata),
        .gatereg_ack  (gatereg_ack),

        .ledger_sel   (ledger_sel),
        .ledger_wr    (ledger_wr),
        .ledger_addr  (ledger_addr),
        .ledger_wdata (ledger_wdata),
        .ledger_rdata (ledger_rdata),
        .ledger_ack   (ledger_ack),

        .priv_sel     (priv_sel),
        .priv_wr      (priv_wr),
        .priv_addr    (priv_addr),
        .priv_wdata   (priv_wdata),
        .priv_rdata   (priv_rdata),
        .priv_ack     (priv_ack)
    );

    // -----------------------------------------------------------------------
    // Boot ROM
    // -----------------------------------------------------------------------
    boreal_bootrom u_bootrom (
        .clk          (clk),
        .rst_n        (rst_n_boot),
        .sel          (rom_sel),
        .addr         (rom_addr),
        .rdata        (rom_rdata),
        .ack          (rom_ack),
        .boot_done    (boot_done_w),
        .boot_pass    (boot_pass_w),
        .boot_hash_out(boot_hash_out),
        .sha_start    (sha_start),
        .sha_update   (sha_update),
        .sha_data     (sha_data),
        .sha_hash     (sha_hash),
        .sha_ready    (sha_ready),
        .sig_start    (sig_start),
        .sig_hash_in  (sig_hash_in),
        .sig_pass     (sig_pass),
        .sig_ready    (sig_ready)
    );

    // -----------------------------------------------------------------------
    // SHA-256 Stub
    // -----------------------------------------------------------------------
    boreal_sha256_stub u_sha256 (
        .clk      (clk),
        .rst_n    (rst_n_boot),
        .start    (sha_start),
        .update   (sha_update),
        .data_in  (sha_data),
        .hash_out (sha_hash),
        .ready    (sha_ready)
    );

    // -----------------------------------------------------------------------
    // Signature Verification Stub
    // -----------------------------------------------------------------------
    boreal_sigverify_stub u_sigverify (
        .clk           (clk),
        .rst_n         (rst_n_boot),
        .start         (sig_start),
        .hash_in       (sig_hash_in),
        .pass          (sig_pass),
        .ready         (sig_ready),
        .expected_hash (fuse_expected_hash),
        .min_version   (fuse_min_version),
        .image_version (fuse_image_version)
    );

    // -----------------------------------------------------------------------
    // SRAM Tile
    // -----------------------------------------------------------------------
    boreal_sram_tile_bram u_sram (
        .clk       (clk),
        .rst_n     (rst_n_internal),
        .sel       (sram_sel),
        .wr        (sram_wr),
        .addr      (sram_addr),
        .wdata     (sram_wdata),
        .strb      (sram_strb),
        .rdata     (sram_rdata),
        .ack       (sram_ack),
        .dma_sel   (dma_mem_sel),
        .dma_wr    (dma_mem_wr),
        .dma_addr  (dma_mem_addr),
        .dma_wdata (dma_mem_wdata),
        .dma_rdata (dma_mem_rdata),
        .dma_ack   (dma_mem_ack)
    );

    // -----------------------------------------------------------------------
    // DMA Ring
    // -----------------------------------------------------------------------
    boreal_dma_ring u_dma (
        .clk       (clk),
        .rst_n     (rst_n_internal),
        .sel       (dma_sel),
        .wr        (dma_wr),
        .addr      (dma_addr),
        .wdata     (dma_wdata),
        .rdata     (dma_rdata),
        .ack       (dma_ack),
        .mem_sel   (dma_mem_sel),
        .mem_wr    (dma_mem_wr),
        .mem_addr  (dma_mem_addr),
        .mem_wdata (dma_mem_wdata),
        .mem_rdata (dma_mem_rdata),
        .mem_ack   (dma_mem_ack)
    );

    // -----------------------------------------------------------------------
    // Vector Engine
    // -----------------------------------------------------------------------
    // Vector engine SRAM access goes through bus (simplified for first build)
    wire        vec_sram_rd_req, vec_sram_rd_ack;
    wire [31:0] vec_sram_rd_addr, vec_sram_rd_data;
    wire        vec_sram_wr_req, vec_sram_wr_ack;
    wire [31:0] vec_sram_wr_addr, vec_sram_wr_data;

    boreal_vector u_vector (
        .clk          (clk),
        .rst_n        (rst_n_internal),
        .sel          (vec_sel),
        .wr           (vec_wr),
        .addr         (vec_addr),
        .wdata        (vec_wdata),
        .rdata        (vec_rdata),
        .ack          (vec_ack),
        .sram_rd_req  (vec_sram_rd_req),
        .sram_rd_addr (vec_sram_rd_addr),
        .sram_rd_data (vec_sram_rd_data),
        .sram_rd_ack  (vec_sram_rd_ack),
        .sram_wr_req  (vec_sram_wr_req),
        .sram_wr_addr (vec_sram_wr_addr),
        .sram_wr_data (vec_sram_wr_data),
        .sram_wr_ack  (vec_sram_wr_ack)
    );

    // Tie off vector SRAM ports (to be connected via a second SRAM port or arbiter)
    assign vec_sram_rd_data = 32'h0;
    assign vec_sram_rd_ack  = vec_sram_rd_req;
    assign vec_sram_wr_ack  = vec_sram_wr_req;

    // -----------------------------------------------------------------------
    // AI Mailbox
    // -----------------------------------------------------------------------
    boreal_ai_mailbox u_aimbox (
        .clk           (clk),
        .rst_n         (rst_n_internal),
        .sel           (aimbox_sel),
        .wr            (aimbox_wr),
        .addr          (aimbox_addr),
        .wdata         (aimbox_wdata),
        .rdata         (aimbox_rdata),
        .ack           (aimbox_ack),
        .vm_rd_idx     (mb_rd_idx),
        .vm_rd_slot    (mb_rd_slot),
        .vm_rd_data    (mb_rd_data),
        .vm_slot0_valid(mb_slot0_valid),
        .vm_slot1_valid(mb_slot1_valid),
        .vm_slot0_ack  (mb_slot0_ack),
        .vm_slot1_ack  (mb_slot1_ack)
    );

    // -----------------------------------------------------------------------
    // Decision-VM
    // -----------------------------------------------------------------------
    boreal_decision_vm u_dvm (
        .clk             (clk),
        .rst_n           (rst_n_internal),
        .sel             (dvm_sel),
        .wr              (dvm_wr),
        .addr            (dvm_addr),
        .wdata           (dvm_wdata),
        .rdata           (dvm_rdata),
        .ack             (dvm_ack),
        .mb_rd_idx       (mb_rd_idx),
        .mb_rd_slot      (mb_rd_slot),
        .mb_rd_data      (mb_rd_data),
        .mb_slot0_valid  (mb_slot0_valid),
        .mb_slot1_valid  (mb_slot1_valid),
        .mb_slot0_ack    (mb_slot0_ack),
        .mb_slot1_ack    (mb_slot1_ack),
        .act_valid       (act_valid),
        .act_opcode      (act_opcode),
        .act_target      (act_target),
        .act_arg0        (act_arg0),
        .act_arg1        (act_arg1),
        .act_context_hash(act_context_hash),
        .act_policy_hash (act_policy_hash),
        .act_bounds      (act_bounds),
        .act_nonce       (act_nonce),
        .act_ready       (act_ready)
    );

    // -----------------------------------------------------------------------
    // Gate Policy Registers
    // -----------------------------------------------------------------------
    boreal_gate_policy u_gate_policy (
        .clk          (clk),
        .rst_n        (rst_n_internal),
        .sel          (gatereg_sel),
        .wr           (gatereg_wr),
        .addr         (gatereg_addr),
        .wdata        (gatereg_wdata),
        .rdata        (gatereg_rdata),
        .ack          (gatereg_ack),
        .allow0       (policy_allow0),
        .allow1       (policy_allow1),
        .rate_limit   (policy_rate_limit),
        .rate_window  (policy_rate_window),
        .policy_hash  (policy_hash),
        .override_reg (policy_override),
        .nonce_val    (nonce_counter)
    );

    // -----------------------------------------------------------------------
    // Central Gate
    // -----------------------------------------------------------------------
    boreal_gate u_gate (
        .clk             (clk),
        .rst_n           (rst_n_internal),

        // Action request from Decision-VM
        .act_valid       (act_valid),
        .act_opcode      (act_opcode),
        .act_target      (act_target),
        .act_arg0        (act_arg0),
        .act_arg1        (act_arg1),
        .act_context_hash(act_context_hash),
        .act_policy_hash (act_policy_hash),
        .act_bounds      (act_bounds),
        .act_nonce       (act_nonce),
        .act_ready       (act_ready),

        // Response
        .resp_valid      (resp_valid),
        .resp_committed  (resp_committed),
        .resp_reason     (resp_reason),
        .resp_applied0   (resp_applied0),
        .resp_applied1   (resp_applied1),
        .resp_ledger_idx (resp_ledger_idx),

        // Policy
        .allow0          (policy_allow0),
        .allow1          (policy_allow1),
        .rate_limit      (policy_rate_limit),
        .rate_window     (policy_rate_window),
        .policy_hash     (policy_hash),
        .override_reg    (policy_override),

        // Nonce
        .nonce_counter   (nonce_counter),

        // Privileged I/O master
        .priv_req        (gate_bus_req),
        .priv_wr         (gate_bus_wr),
        .priv_addr       (gate_bus_addr),
        .priv_wdata      (gate_bus_wdata),
        .priv_ack        (priv_ack),

        // Ledger
        .ledger_wr_en    (gate_ledger_wr_en),
        .ledger_wr_data  (gate_ledger_wr_data),
        .ledger_idx      (ledger_idx)
    );

    // Gate bus strobe (always full-word for gate writes)
    assign gate_bus_strb = 4'hF;

    // -----------------------------------------------------------------------
    // Ledger
    // -----------------------------------------------------------------------
    boreal_ledger u_ledger (
        .clk     (clk),
        .rst_n   (rst_n_internal),
        .wr_en   (gate_ledger_wr_en),
        .wr_data (gate_ledger_wr_data),
        .idx     (ledger_idx),
        .sel     (ledger_sel),
        .wr      (ledger_wr),
        .addr    (ledger_addr),
        .wdata   (ledger_wdata),
        .rdata   (ledger_rdata),
        .ack     (ledger_ack)
    );

    // -----------------------------------------------------------------------
    // Privileged I/O
    // -----------------------------------------------------------------------
    boreal_priv_io u_priv_io (
        .clk       (clk),
        .rst_n     (rst_n_internal),
        .sel       (priv_sel),
        .wr        (priv_wr),
        .addr      (priv_addr),
        .wdata     (priv_wdata),
        .rdata     (priv_rdata),
        .ack       (priv_ack),
        .pio_out_0 (pio_out_0),
        .pio_out_1 (pio_out_1),
        .pio_out_2 (pio_out_2),
        .pio_out_3 (pio_out_3)
    );

endmodule
