# Boreal FPGA Build Steps

## Overview

This document outlines the recommended build order for bringing up the Boreal SoC on FPGA. Each step builds on the previous, allowing incremental verification.

---

## Phase 1: Core Infrastructure

### Step 1.1: Interconnect + Gate + Privileged I/O

**Objective:** Verify non-bypassable Gate architecture

**Actions:**
1. Synthesize `boreal_interconnect.v`, `boreal_gate.v`, `boreal_gate_policy.v`, `boreal_priv_io.v`
2. Create a testbench that attempts to access privileged addresses from both masters
3. Verify that public bus transactions to 0x2000_0000 return errors
4. Verify that Gate can successfully write to privileged I/O

**Success Criteria:**
- All public bus attempts to PRIV_IO region are blocked
- Gate writes to privileged I/O complete successfully
- Policy registers can be configured via MMIO

### Step 1.2: Ledger Integration

**Objective:** Verify event logging

**Actions:**
1. Add `boreal_ledger.v` to the design
2. Configure Gate to commit actions and verify ledger entries are written
3. Verify ledger index increments correctly
4. Test ledger read via MMIO

**Success Criteria:**
- Each Gate commit creates a ledger entry
- Index increments monotonically
- Entries can be read back via MMIO

---

## Phase 2: Compute Elements

### Step 2.1: SRAM and DMA

**Objective:** Verify memory subsystem

**Actions:**
1. Instantiate `boreal_sram_tile_bram.v` and `boreal_dma_ring.v`
2. Program DMA to copy data between SRAM locations
3. Verify CRC calculation
4. Test burst transfers

**Success Criteria:**
- DMA copies complete correctly
- CRC matches expected values
- Memory contents are correct after transfer

### Step 2.2: Vector Engine

**Objective:** Verify SIMD compute

**Actions:**
1. Add `boreal_vector.v` and `boreal_vec_lane.v`
2. Load test data into SRAM
3. Execute vector operations (MAC, scale, clamp)
4. Verify results

**Success Criteria:**
- MAC operations produce correct results
- Saturation/clamping works correctly
- Performance meets timing requirements

---

## Phase 3: Decision Pipeline

### Step 3.1: AI Mailbox

**Objective:** Verify inference I/O

**Actions:**
1. Add `boreal_ai_mailbox.v`
2. Write test data to mailbox from AI side
3. Read data from VM side
4. Verify valid flags

**Success Criteria:**
- Data written correctly visible to reader
- Valid flags work correctly

### Step 3.2: Decision-VM

**Objective:** Verify deterministic action generation

**Actions:**
1. Add `boreal_decision_vm.v`
2. Load test program (or use built-in simple program)
3. Provide AI mailbox inputs
4. Verify action requests generated

**Success Criteria:**
- VM produces expected action requests
- Cycle budget is enforced
- Same inputs produce same outputs

### Step 3.3: Full Pipeline Test

**Objective:** End-to-end verification

**Actions:**
1. Run complete pipeline: AI mailbox → VM → Gate → Ledger
2. Verify deterministic replay (run twice, compare ledgers)
3. Test policy enforcement (reject disallowed actions)
4. Test rate limiting

**Success Criteria:**
- Identical ledger contents on identical runs
- Policy correctly rejects unauthorized actions
- Rate limiting prevents rapid commits

---

## Phase 4: Secure Boot

### Step 4.1: Boot ROM and SHA

**Objective:** Verify measured boot

**Actions:**
1. Add `boreal_bootrom.v` and `boreal_sha256_stub.v`
2. Create test boot image with header
3. Execute boot sequence
4. Verify hash computation

**Success Criteria:**
- Boot ROM executes correctly
- SHA-256 produces expected hash
- Boot sequence proceeds or halts based on verification

### Step 4.2: Signature Verification

**Objective:** Verify authenticated boot

**Actions:**
1. Add `boreal_sigverify_stub.v` (or real implementation)
2. Sign test image with test key
3. Verify boot accepts valid signatures
4. Verify boot rejects invalid signatures

**Success Criteria:**
- Valid signatures pass verification
- Invalid signatures halt boot
- Version check prevents rollback

---

## Phase 5: Integration

### Step 5.1: Full SoC Synthesis

**Objective:** Complete FPGA implementation

**Actions:**
1. Synthesize `boreal_top_fpga.v` with all modules
2. Run static timing analysis
3. Verify resource utilization
4. Generate bitstream

**Success Criteria:**
- Timing closure achieved
- Resource utilization within budget
- Bitstream generates successfully

### Step 5.2: Hardware Bring-up

**Objective:** Validate on actual hardware

**Actions:**
1. Program FPGA with bitstream
2. Connect to debug interface
3. Run basic tests
4. Verify all major functions

**Success Criteria:**
- FPGA boots correctly
- All MMIO registers accessible
- Pipeline operates correctly

---

## Build Commands

### Vivado (Xilinx)

```bash
# Create project
vivado -mode batch -source create_project.tcl

# Synthesize
vivado -mode batch -source synthesize.tcl

# Implement
vivado -mode batch -source implement.tcl

# Generate bitstream
vivado -mode batch -source bitstream.tcl
```

### Quartus (Intel)

```bash
# Create project
quartus_sh --flow compile boreal_soc

# Synthesize
quartus_map boreal_soc

# Fit
quartus_fit boreal_soc

# Program
quartus_pgm -m jtag -o "p;boreal_soc.sof"
```

---

## Resource Budget

| Resource | Artix-7 (XC7A100T) | Cyclone V (5CGXFC7C7) |
|----------|-------------------|----------------------|
| LUTs | ~25,000 (target) | ~25,000 (target) |
| Registers | ~12,000 (target) | ~12,000 (target) |
| BRAM | 20 blocks | 20 blocks |
| DSP | 16 blocks | 16 blocks |

---

## Verification Checklist

- [ ] Gate blocks privileged I/O access from public bus
- [ ] Ledger entries are never modified after write
- [ ] Decision-VM terminates within cycle budget
- [ ] Same inputs produce identical ledgers (determinism)
- [ ] Rate limiting works correctly
- [ ] Policy allowlist enforced
- [ ] Boot ROM executes correctly
- [ ] Signature verification works
- [ ] All MMIO registers accessible
- [ ] Timing closure achieved
