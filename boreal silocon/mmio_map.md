# Boreal SoC MMIO Register Map

## Overview

This document provides a complete reference for all MMIO registers in the Boreal SoC. All registers are 32 bits wide and aligned to 4-byte boundaries.

## Address Map Summary

| Base Address | Module | Description |
|-------------|--------|-------------|
| 0x0000_0000 | Boot ROM | Read-only boot code |
| 0x0000_1000 | SRAM Tile 0 | Main memory (4KB) |
| 0x1000_0000 | DMA | DMA control registers |
| 0x1001_0000 | Vector Engine | SIMD compute control |
| 0x1002_0000 | AI Mailbox | AI inference I/O |
| 0x1003_0000 | Decision-VM | VM control and status |
| 0x1004_0000 | Gate | Policy and commit control |
| 0x1005_0000 | Ledger | Event log access |
| 0x2000_0000 | Privileged I/O | **Gate only - no public access** |

---

## DMA Registers (0x1000_0000)

| Offset | Name | R/W | Reset | Description |
|--------|------|-----|-------|-------------|
| 0x00 | DMA_HEAD | R/W | 0x0 | Descriptor ring head pointer |
| 0x04 | DMA_TAIL | R/W | 0x0 | Descriptor ring tail pointer |
| 0x08 | DMA_STATUS | R | 0x0 | Status register |
| 0x0C | DMA_START | W | - | Write 1 to start transfer |
| 0x10 | DMA_CRC | R | 0x0 | CRC-32 of last transfer |

### DMA_STATUS Field Definition

| Bit | Name | Description |
|-----|------|-------------|
| 0 | busy | Transfer in progress |
| 1 | done | Transfer completed |
| 2 | error | Transfer error occurred |
| 31:3 | reserved | Reserved |

---

## Vector Engine Registers (0x1001_0000)

| Offset | Name | R/W | Reset | Description |
|--------|------|-----|-------|-------------|
| 0x00 | VEC_CMD | R/W | 0x0 | Command register |
| 0x04 | VEC_SRC | R/W | 0x0 | Source address |
| 0x08 | VEC_DST | R/W | 0x0 | Destination address |
| 0x0C | VEC_LEN | R/W | 0x0 | Operation length |
| 0x10 | VEC_SCALE | R/W | 0x0 | Requantization scale |
| 0x14 | VEC_ZERO | R/W | 0x0 | Requantization zero point |
| 0x18 | VEC_M | R/W | 0x0 | Matrix dimension M |
| 0x1C | VEC_N | R/W | 0x0 | Matrix dimension N |
| 0x20 | VEC_K | R/W | 0x0 | Matrix dimension K |
| 0x24 | VEC_STATUS | R | 0x0 | Status register |

### VEC_CMD Field Definition

| Bit | Name | Description |
|-----|------|-------------|
| 0 | start | Write 1 to start operation |
| 1 | irq_en | Enable interrupt on completion |

### VEC_STATUS Field Definition

| Bit | Name | Description |
|-----|------|-------------|
| 0 | busy | Operation in progress |
| 1 | done | Operation completed |
| 2 | error | Operation error |

---

## Gate Registers (0x1004_0000)

| Offset | Name | R/W | Reset | Description |
|--------|------|-----|-------|-------------|
| 0x00 | GATE_ALLOW0 | R/W | 0x0 | Allow mask for targets 0-31 |
| 0x04 | GATE_ALLOW1 | R/W | 0x0 | Allow mask for targets 32-63 |
| 0x08 | GATE_RATE_LIM | R/W | 0xA | Maximum commits per window |
| 0x0C | GATE_RATE_WIN | R/W | 0x3E8 | Rate window in cycles |
| 0x10 | GATE_POLICY | R/W | 0x0 | Active policy hash |
| 0x14 | GATE_OVERRIDE | R/W | 0x0 | Debug override (fuse-lockable) |
| 0x18 | GATE_NONCE | R | - | Monotonic counter read |

### GATE_ALLOW0/1 Field Definition

Each bit enables the corresponding target ID:
- Bit 0 = target 0 allowed
- Bit 1 = target 1 allowed
- etc.

---

## Ledger Registers (0x1005_0000)

| Offset | Name | R/W | Reset | Description |
|--------|------|-----|-------|-------------|
| 0x00 | LEDGER_IDX | R | 0x0 | Current write index |
| 0x04 | LEDGER_DEPTH | R | 0x400 | Maximum entries (1024) |
| 0x08 | LEDGER_RD_ADDR | W | - | Read address select |
| 0x0C | LEDGER_RD_DATA0 | R | - | Entry data [31:0] |
| 0x10 | LEDGER_RD_DATA1 | R | - | Entry data [63:32] |
| 0x14 | LEDGER_RD_DATA2 | R | - | Entry data [95:64] |
| 0x18 | LEDGER_RD_DATA3 | R | - | Entry data [127:96] |

---

## Security Notes

1. **Privileged I/O (0x2000_0000)** is NOT accessible from the public bus
2. Only the Gate module can write to Privileged I/O
3. GATE_OVERRIDE should be locked via fuse in production
4. Boot ROM is immutable after fabrication
