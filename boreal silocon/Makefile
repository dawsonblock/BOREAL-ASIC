# ============================================================================
# Boreal SoC - Build & Simulation Makefile
# ============================================================================
# Targets:
#   make sim_gate    - Run Phase 1 testbench (Gate + Interconnect + Ledger)
#   make lint        - Lint all RTL with Verilator
#   make clean       - Remove generated files
# ============================================================================

IVERILOG  = iverilog
VVP       = vvp
VERILATOR = verilator

RTL_DIR   = rtl
TB_DIR    = tb
BUILD_DIR = build

RTL_SRCS  = \
    $(RTL_DIR)/boreal_pkg.v \
    $(RTL_DIR)/boreal_interconnect.v \
    $(RTL_DIR)/boreal_gate_policy.v \
    $(RTL_DIR)/boreal_gate.v \
    $(RTL_DIR)/boreal_priv_io.v \
    $(RTL_DIR)/boreal_ledger.v \
    $(RTL_DIR)/boreal_sram_tile_bram.v \
    $(RTL_DIR)/boreal_dma_ring.v \
    $(RTL_DIR)/boreal_vec_lane.v \
    $(RTL_DIR)/boreal_vector.v \
    $(RTL_DIR)/boreal_ai_mailbox.v \
    $(RTL_DIR)/boreal_decision_vm.v \
    $(RTL_DIR)/boreal_bootrom.v \
    $(RTL_DIR)/boreal_sha256_stub.v \
    $(RTL_DIR)/boreal_sigverify_stub.v \
    $(RTL_DIR)/boreal_top_fpga.v

# --------------------------------------------------------------------------
# Phase 1 testbench: Gate pipeline
# --------------------------------------------------------------------------
sim_gate: $(BUILD_DIR)/tb_boreal_gate.vvp
	$(VVP) $<

$(BUILD_DIR)/tb_boreal_gate.vvp: $(TB_DIR)/tb_boreal_gate.v $(RTL_SRCS) | $(BUILD_DIR)
	$(IVERILOG) -o $@ -I $(RTL_DIR) -g2012 $^

# --------------------------------------------------------------------------
# Lint with Verilator
# --------------------------------------------------------------------------
lint:
	$(VERILATOR) --lint-only -I$(RTL_DIR) --top-module boreal_top_fpga \
		$(filter-out $(RTL_DIR)/boreal_pkg.v, $(RTL_SRCS))

# --------------------------------------------------------------------------
# Utilities
# --------------------------------------------------------------------------
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR) *.vcd

.PHONY: sim_gate lint clean
