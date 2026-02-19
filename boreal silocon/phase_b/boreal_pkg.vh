// ============================================================================
// boreal_pkg.vh — Phase-B constants (superset of Phase-A)
// ============================================================================
`ifndef BOREAL_PKG_VH
`define BOREAL_PKG_VH

// Regions (top nibble decode)
`define REGN_BOOT   4'h0
`define REGN_MMIO1  4'h1
`define REGN_PRIV2  4'h2

// Base addresses
`define BASE_BOOTROM     32'h0000_0000
`define BASE_SRAM0       32'h0000_1000

`define BASE_MMIO_DMA    32'h1000_0000
`define BASE_MMIO_VEC    32'h1001_0000
`define BASE_MMIO_MBOX   32'h1002_0000
`define BASE_MMIO_VM     32'h1003_0000
`define BASE_MMIO_GATE   32'h1004_0000
`define BASE_MMIO_LED    32'h1005_0000

`define BASE_PRIV_IO     32'h2000_0000

// Mailbox word-addressed offsets (within 0x1002_0000)
`define MBOX_AI_W0       8'h00
`define MBOX_AI_W1       8'h01
`define MBOX_AI_W2       8'h02
`define MBOX_AI_W3       8'h03

`define MBOX_REQ_VALID   8'h40
`define MBOX_REQ_W0      8'h44
`define MBOX_RESP_VALID  8'h60
`define MBOX_RESP_W0     8'h64
`define MBOX_RESP_ACK    8'h69

// Gate reason codes
`define GATE_OK             32'd0
`define GATE_CLAMPED        32'd1
`define GATE_RATE_LIMITED   32'd2
`define GATE_POLICY_DENIED  32'd3
`define GATE_INVALID        32'd4
`define GATE_NONCE_ERR      32'd5
`define GATE_POLICY_HASHERR 32'd6

// DMA status bits
`define DMA_BUSY   0
`define DMA_DONE   1
`define DMA_ERR    2

`endif
