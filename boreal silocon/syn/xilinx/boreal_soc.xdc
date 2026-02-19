# ===========================================================================
# Boreal SoC — Xilinx Artix-7 Constraints (XDC)
# ===========================================================================

# Clock — 100 MHz
create_clock -period 10.000 -name sys_clk [get_ports clk]

# Reset
set_false_path -from [get_ports rst_n]

# I/O Standards (placeholder — update for your board)
set_property IOSTANDARD LVCMOS33 [get_ports *]

# Timing
set_input_delay  -clock sys_clk -max 3.0 [all_inputs]
set_input_delay  -clock sys_clk -min 0.5 [all_inputs]
set_output_delay -clock sys_clk -max 3.0 [all_outputs]
set_output_delay -clock sys_clk -min 0.5 [all_outputs]

# BRAM inference guidance
set_property RAM_STYLE BLOCK [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ BMEM.*}]

# Bitstream settings
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
