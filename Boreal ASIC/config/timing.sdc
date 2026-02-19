# BOREAL ASIC Timing Constraints (SDC)
# SKY130 130nm process, 40MHz target frequency

# Clock definitions
create_clock -name clk -period 25.0 [get_ports clk]
set_clock_uncertainty 0.5 [get_clocks clk]

# Reset timing
set_false_path -from [get_ports reset_n] -to [all_registers]

# Clock domain crossings (if any)
# set_false_path -from [get_clocks clk] -to [get_clocks other_clock]

# Input delays (external interfaces)
set_input_delay -clock clk 2.0 [get_ports {uart_rx jtag_* gpio[*] sensor_*}]
set_input_delay -clock clk 1.0 [get_ports {reset_n}]

# Output delays (external interfaces)
set_output_delay -clock clk 2.0 [get_ports {uart_tx jtag_tdo gpio[*] gpio_oe[*] actuator_*}]
set_output_delay -clock clk 1.0 [get_ports {scan_out}]

# Drive strength and load capacitance
set_drive 0.001 [get_ports {uart_rx jtag_* gpio[*] sensor_* reset_n}]
set_load 0.01 [get_ports {uart_tx jtag_tdo gpio[*] gpio_oe[*] actuator_* scan_out}]

# Multi-cycle paths (for slow peripherals)
set_multicycle_path -setup 4 -from [get_pins */gpio_reg*/Q] -to [get_ports gpio[*]]
set_multicycle_path -hold 3 -from [get_pins */gpio_reg*/Q] -to [get_ports gpio[*]]

# False paths for DFT signals
set_false_path -through [get_pins */scan_en]
set_false_path -through [get_pins */scan_*]

# Maximum transition time
set_max_transition 1.0 [current_design]

# Maximum fanout
set_max_fanout 10 [current_design]

# Operating conditions
set_operating_conditions -min_library "sky130_fd_sc_hd__tt_025C_1v80" \
                        -min "sky130_fd_sc_hd__tt_025C_1v80" \
                        -max_library "sky130_fd_sc_hd__tt_025C_1v80" \
                        -max "sky130_fd_sc_hd__tt_025C_1v80"

# Wire load model
set_wire_load_model -name "WireAreaForZero" -library "sky130_fd_sc_hd__tt_025C_1v80"

# Area constraints (keep within die size)
set_max_area 200000
