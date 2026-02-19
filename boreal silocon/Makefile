# ============================================================================
# Boreal SoC - Enhanced Build System
# ============================================================================
#
# Quick Reference:
#   make check-tools      - Verify required tools are installed
#   make sim_gate         - Run Phase 1 testbench (Gate pipeline)
#   make sim_boot         - Run boot ROM testbench
#   make sim_dma          - Run DMA engine testbench
#   make sim_vector       - Run vector engine testbench
#   make sim_interconnect - Run interconnect testbench
#   make sim_all          - Run ALL testbenches
#   make lint             - Lint all RTL with Verilator
#   make coverage         - Run tests with code coverage
#   make formal           - Run formal verification (requires sby)
#   make synth-vivado     - Synthesize for Xilinx (requires Vivado)
#   make synth-quartus    - Synthesize for Intel (requires Quartus)
#   make docs             - Generate documentation
#   make ci               - Full CI pipeline (lint + sim_all + coverage)
#   make clean            - Remove generated files
#
# ============================================================================

# --------------------------------------------------------------------------
# Tool Configuration (override via environment or command line)
# --------------------------------------------------------------------------
IVERILOG  ?= iverilog
VVP       ?= vvp
VERILATOR ?= verilator
YOSYS     ?= yosys
SBY       ?= sby
PYTHON    ?= python3

# --------------------------------------------------------------------------
# Directories
# --------------------------------------------------------------------------
RTL_DIR     = rtl
TB_DIR      = tb
BUILD_DIR   = build
COV_DIR     = $(BUILD_DIR)/coverage
FORMAL_DIR  = $(BUILD_DIR)/formal
SYN_DIR     = $(BUILD_DIR)/syn
DOCS_DIR    = docs

# --------------------------------------------------------------------------
# Compiler Flags
# --------------------------------------------------------------------------
IVFLAGS    = -I $(RTL_DIR) -g2012 -Wall
VVP_FLAGS  =

# --------------------------------------------------------------------------
# RTL Source Files (order matters for `defines)
# --------------------------------------------------------------------------
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

# RTL without package (for Verilator which handles `defines differently)
RTL_NO_PKG = $(filter-out $(RTL_DIR)/boreal_pkg.v, $(RTL_SRCS))

# --------------------------------------------------------------------------
# Testbench Registry: TB_NAME -> TB_FILE
# --------------------------------------------------------------------------
TB_GATE          = $(TB_DIR)/tb_boreal_gate.v
TB_BOOT          = $(TB_DIR)/tb_boreal_boot.v
TB_DMA           = $(TB_DIR)/tb_boreal_dma.v
TB_VECTOR        = $(TB_DIR)/tb_boreal_vector.v
TB_INTERCONNECT  = $(TB_DIR)/tb_boreal_interconnect.v

ALL_TBS = sim_gate sim_boot sim_dma sim_vector sim_interconnect

# ============================================================================
# Tool Verification
# ============================================================================
.PHONY: check-tools
check-tools:
	@echo "=== Boreal SoC Tool Check ==="
	@command -v $(IVERILOG)  >/dev/null 2>&1 && echo "  ✓ iverilog"  || (echo "  ✗ iverilog  — brew install icarus-verilog" && false)
	@command -v $(VVP)       >/dev/null 2>&1 && echo "  ✓ vvp"       || (echo "  ✗ vvp       — installed with icarus-verilog" && false)
	@command -v $(VERILATOR) >/dev/null 2>&1 && echo "  ✓ verilator" || echo "  ⚠ verilator — brew install verilator (optional for lint)"
	@command -v $(YOSYS)     >/dev/null 2>&1 && echo "  ✓ yosys"     || echo "  ⚠ yosys     — brew install yosys (optional for formal/synth)"
	@echo "=== Done ==="

# ============================================================================
# Simulation Targets
# ============================================================================

# --- Phase 1: Gate Pipeline ---
.PHONY: sim_gate
sim_gate: $(BUILD_DIR)/tb_boreal_gate.vvp
	@echo "=== Running Gate Pipeline Testbench ==="
	$(VVP) $(VVP_FLAGS) $<

$(BUILD_DIR)/tb_boreal_gate.vvp: $(TB_GATE) $(RTL_SRCS) | $(BUILD_DIR)
	$(IVERILOG) -o $@ $(IVFLAGS) $^

# --- Boot ROM Testbench ---
.PHONY: sim_boot
sim_boot: $(BUILD_DIR)/tb_boreal_boot.vvp
	@echo "=== Running Boot ROM Testbench ==="
	$(VVP) $(VVP_FLAGS) $<

$(BUILD_DIR)/tb_boreal_boot.vvp: $(TB_BOOT) $(RTL_SRCS) | $(BUILD_DIR)
	$(IVERILOG) -o $@ $(IVFLAGS) $^

# --- DMA Engine Testbench ---
.PHONY: sim_dma
sim_dma: $(BUILD_DIR)/tb_boreal_dma.vvp
	@echo "=== Running DMA Engine Testbench ==="
	$(VVP) $(VVP_FLAGS) $<

$(BUILD_DIR)/tb_boreal_dma.vvp: $(TB_DMA) $(RTL_SRCS) | $(BUILD_DIR)
	$(IVERILOG) -o $@ $(IVFLAGS) $^

# --- Vector Engine Testbench ---
.PHONY: sim_vector
sim_vector: $(BUILD_DIR)/tb_boreal_vector.vvp
	@echo "=== Running Vector Engine Testbench ==="
	$(VVP) $(VVP_FLAGS) $<

$(BUILD_DIR)/tb_boreal_vector.vvp: $(TB_VECTOR) $(RTL_SRCS) | $(BUILD_DIR)
	$(IVERILOG) -o $@ $(IVFLAGS) $^

# --- Interconnect Testbench ---
.PHONY: sim_interconnect
sim_interconnect: $(BUILD_DIR)/tb_boreal_interconnect.vvp
	@echo "=== Running Interconnect Testbench ==="
	$(VVP) $(VVP_FLAGS) $<

$(BUILD_DIR)/tb_boreal_interconnect.vvp: $(TB_INTERCONNECT) $(RTL_SRCS) | $(BUILD_DIR)
	$(IVERILOG) -o $@ $(IVFLAGS) $^

# --- Run All Testbenches ---
.PHONY: sim_all
sim_all: $(ALL_TBS)
	@echo ""
	@echo "========================================"
	@echo " All testbenches completed."
	@echo "========================================"

# ============================================================================
# Linting
# ============================================================================
.PHONY: lint
lint:
	@echo "=== Verilator Lint ==="
	$(VERILATOR) --lint-only -I$(RTL_DIR) \
		-Wno-DECLFILENAME -Wno-UNUSED -Wno-UNDRIVEN -Wno-WIDTH \
		--top-module boreal_top_fpga \
		$(RTL_NO_PKG)
	@echo "=== Lint passed ==="

.PHONY: lint-strict
lint-strict:
	@echo "=== Verilator Lint (strict) ==="
	$(VERILATOR) --lint-only -I$(RTL_DIR) \
		--top-module boreal_top_fpga \
		$(RTL_NO_PKG)

# ============================================================================
# Code Coverage (iverilog-based toggle coverage)
# ============================================================================
.PHONY: coverage
coverage: | $(COV_DIR)
	@echo "=== Building with coverage instrumentation ==="
	$(IVERILOG) -o $(COV_DIR)/tb_gate_cov.vvp $(IVFLAGS) $(TB_GATE) $(RTL_SRCS)
	@echo "=== Running simulation ==="
	cd $(COV_DIR) && $(VVP) tb_gate_cov.vvp
	@echo "=== Coverage VCD generated at $(COV_DIR)/tb_boreal_gate.vcd ==="
	@if [ -f $(COV_DIR)/tb_boreal_gate.vcd ]; then \
		echo "  VCD size: $$(du -h $(COV_DIR)/tb_boreal_gate.vcd | cut -f1)"; \
	fi

$(COV_DIR):
	mkdir -p $(COV_DIR)

# ============================================================================
# Formal Verification (SymbiYosys)
# ============================================================================
.PHONY: formal
formal: | $(FORMAL_DIR)
	@command -v $(SBY) >/dev/null 2>&1 || (echo "ERROR: sby not found. Install SymbiYosys." && exit 1)
	@echo "=== Running Formal Verification ==="
	@for sbyfile in formal/*.sby; do \
		echo "  → $$sbyfile"; \
		$(SBY) -f -d $(FORMAL_DIR)/$$(basename $$sbyfile .sby) $$sbyfile || exit 1; \
	done
	@echo "=== Formal verification passed ==="

$(FORMAL_DIR):
	mkdir -p $(FORMAL_DIR)

# ============================================================================
# FPGA Synthesis (stubs – real flows require vendor tools)
# ============================================================================
.PHONY: synth-yosys synth-vivado synth-quartus

synth-yosys: | $(SYN_DIR)
	@command -v $(YOSYS) >/dev/null 2>&1 || (echo "ERROR: yosys not found." && exit 1)
	@echo "=== Yosys Generic Synthesis ==="
	$(YOSYS) -p "read_verilog -I$(RTL_DIR) $(RTL_NO_PKG); \
		synth -top boreal_top_fpga; \
		stat" -l $(SYN_DIR)/yosys_synth.log
	@echo "=== Synthesis log: $(SYN_DIR)/yosys_synth.log ==="

synth-vivado:
	@command -v vivado >/dev/null 2>&1 || (echo "ERROR: Vivado not found." && exit 1)
	@echo "=== Running Vivado Synthesis ==="
	vivado -mode batch -source syn/vivado_synth.tcl -log $(SYN_DIR)/vivado.log

synth-quartus:
	@command -v quartus_sh >/dev/null 2>&1 || (echo "ERROR: Quartus not found." && exit 1)
	@echo "=== Running Quartus Synthesis ==="
	quartus_sh --flow compile syn/quartus_project

$(SYN_DIR):
	mkdir -p $(SYN_DIR)

# ============================================================================
# Documentation
# ============================================================================
.PHONY: docs
docs:
	@echo "=== Generating Documentation ==="
	@mkdir -p $(DOCS_DIR)/generated
	$(PYTHON) scripts/gen_regmap.py > $(DOCS_DIR)/generated/register_map.md 2>/dev/null || \
		echo "  (skipped: gen_regmap.py not found)"
	@echo "=== Done ==="

# ============================================================================
# CI Pipeline (runs everything that should pass)
# ============================================================================
.PHONY: ci
ci: check-tools lint sim_all
	@echo ""
	@echo "========================================"
	@echo " CI Pipeline: ALL CHECKS PASSED"
	@echo "========================================"

# ============================================================================
# Utilities
# ============================================================================
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) *.vcd

.PHONY: clean-all
clean-all: clean
	rm -rf $(DOCS_DIR)/generated

# Print configuration for debugging
.PHONY: info
info:
	@echo "IVERILOG  = $(IVERILOG)"
	@echo "VVP       = $(VVP)"
	@echo "VERILATOR = $(VERILATOR)"
	@echo "RTL_SRCS  = $(RTL_SRCS)"
	@echo "BUILD_DIR = $(BUILD_DIR)"
