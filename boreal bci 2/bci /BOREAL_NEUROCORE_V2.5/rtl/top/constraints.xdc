## Boreal Neuro-Core v2.5 Constraints
## Target: Xilinx Artix-7 XC7A35T

create_clock -period 10.000 -name sys_clk [get_ports clk_100m]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
