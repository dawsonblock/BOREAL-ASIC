// ============================================================================
// Boreal SoC - Global Parameters Package
// ============================================================================
// Shared address map constants, opcodes, and reason codes for all modules.
// ============================================================================

// --- Address Map ---
`define ADDR_BOOTROM_BASE   32'h0000_0000
`define ADDR_SRAM_BASE      32'h0000_1000
`define ADDR_DMA_BASE       32'h1000_0000
`define ADDR_VEC_BASE       32'h1001_0000
`define ADDR_AIMBOX_BASE    32'h1002_0000
`define ADDR_DVM_BASE       32'h1003_0000
`define ADDR_GATE_BASE      32'h1004_0000
`define ADDR_LEDGER_BASE    32'h1005_0000
`define ADDR_PRIV_IO_BASE   32'h2000_0000

// --- Action Opcodes ---
`define ACT_NOP             32'h0000_0000
`define ACT_WRITE           32'h0000_0001
`define ACT_NET_TX          32'h0000_0002
`define ACT_FLASH           32'h0000_0003
`define ACT_CONFIG          32'h0000_0004

// --- Reason Codes ---
`define REASON_OK           32'h0000_0000
`define REASON_CLAMPED      32'h0000_0001
`define REASON_RATE_LIMITED 32'h0000_0002
`define REASON_POLICY_DENIED 32'h0000_0003
`define REASON_INVALID      32'h0000_0004
`define REASON_NONCE_ERROR  32'h0000_0005

// --- Bus Parameters ---
`define DATA_W  32
`define ADDR_W  32
`define STRB_W  4
