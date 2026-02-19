#!/usr/bin/env bash
# ============================================================================
# Boreal SoC - Tool Installation & Verification Script
# ============================================================================
# Usage: ./scripts/setup_tools.sh [--install] [--check]
#   --check   : Verify all tools are installed (default)
#   --install : Attempt to install missing tools via Homebrew / pip
# ============================================================================
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

MODE="${1:---check}"
ERRORS=0
WARNINGS=0

# --------------------------------------------------------------------------
# Helper functions
# --------------------------------------------------------------------------
check_tool() {
    local name="$1"
    local min_version="${2:-}"
    local category="${3:-required}"

    if command -v "$name" &>/dev/null; then
        local ver
        ver=$("$name" --version 2>&1 | head -1 || echo "unknown")
        printf "${GREEN}  ✓${NC} %-20s %s\n" "$name" "$ver"
        return 0
    else
        if [ "$category" = "required" ]; then
            printf "${RED}  ✗${NC} %-20s NOT FOUND (required)\n" "$name"
            ERRORS=$((ERRORS + 1))
        else
            printf "${YELLOW}  ⚠${NC} %-20s NOT FOUND (optional)\n" "$name"
            WARNINGS=$((WARNINGS + 1))
        fi
        return 1
    fi
}

install_brew_pkg() {
    local pkg="$1"
    if command -v brew &>/dev/null; then
        echo "  → Installing $pkg via Homebrew..."
        brew install "$pkg"
    else
        echo "  → Homebrew not found. Please install $pkg manually."
        return 1
    fi
}

install_pip_pkg() {
    local pkg="$1"
    echo "  → Installing $pkg via pip..."
    python3 -m pip install --user "$pkg"
}

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
echo ""
echo "============================================================"
echo " Boreal SoC - Tool Verification"
echo "============================================================"
echo ""

# --- Simulation Tools ---
echo -e "${CYAN}Simulation Tools:${NC}"
if ! check_tool "iverilog" "" "required"; then
    [ "$MODE" = "--install" ] && install_brew_pkg "icarus-verilog"
fi
if ! check_tool "vvp" "" "required"; then
    echo "    (installed with icarus-verilog)"
fi
check_tool "gtkwave" "" "optional"
echo ""

# --- Linting ---
echo -e "${CYAN}Linting & Static Analysis:${NC}"
if ! check_tool "verilator" "" "required"; then
    [ "$MODE" = "--install" ] && install_brew_pkg "verilator"
fi
check_tool "svlint" "" "optional"
echo ""

# --- Formal Verification ---
echo -e "${CYAN}Formal Verification:${NC}"
check_tool "sby" "" "optional"
check_tool "yosys" "" "optional"
if ! check_tool "yosys" "" "optional" 2>/dev/null; then
    [ "$MODE" = "--install" ] && install_brew_pkg "yosys" 2>/dev/null || true
fi
echo ""

# --- FPGA Synthesis ---
echo -e "${CYAN}FPGA Synthesis:${NC}"
check_tool "vivado" "" "optional"
check_tool "quartus_sh" "" "optional"
echo ""

# --- Build Utilities ---
echo -e "${CYAN}Build Utilities:${NC}"
check_tool "make" "" "required"
check_tool "python3" "" "required"
check_tool "git" "" "required"
echo ""

# --- Python packages (optional) ---
echo -e "${CYAN}Python Packages:${NC}"
for pkg in fusesoc cocotb wavedrom; do
    if python3 -c "import $pkg" 2>/dev/null; then
        printf "${GREEN}  ✓${NC} %-20s installed\n" "$pkg"
    else
        printf "${YELLOW}  ⚠${NC} %-20s not installed (optional)\n" "$pkg"
        WARNINGS=$((WARNINGS + 1))
        [ "$MODE" = "--install" ] && install_pip_pkg "$pkg" 2>/dev/null || true
    fi
done
echo ""

# --- Summary ---
echo "============================================================"
if [ $ERRORS -eq 0 ]; then
    printf "${GREEN} All required tools present.${NC}"
    if [ $WARNINGS -gt 0 ]; then
        printf " ${YELLOW}(%d optional tools missing)${NC}" "$WARNINGS"
    fi
    echo ""
    echo "============================================================"
    exit 0
else
    printf "${RED} %d required tool(s) missing.${NC}\n" "$ERRORS"
    echo " Run: ./scripts/setup_tools.sh --install"
    echo "============================================================"
    exit 1
fi
