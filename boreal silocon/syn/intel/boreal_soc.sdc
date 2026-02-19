# ===========================================================================
# Boreal SoC — Intel Cyclone V Constraints (SDC)
# ===========================================================================

# Clock — 100 MHz
create_clock -name sys_clk -period 10.000 [get_ports clk]
set_clock_uncertainty -setup 0.200 [get_clocks sys_clk]
set_clock_uncertainty -hold  0.050 [get_clocks sys_clk]

# Reset — asynchronous, false path
set_false_path -from [get_ports rst_n]

# Conservative I/O delays
set_input_delay  -clock sys_clk -max 4.0 [all_inputs]
set_input_delay  -clock sys_clk -min 0.5 [all_inputs]
set_output_delay -clock sys_clk -max 4.0 [all_outputs]
set_output_delay -clock sys_clk -min 0.5 [all_outputs]
