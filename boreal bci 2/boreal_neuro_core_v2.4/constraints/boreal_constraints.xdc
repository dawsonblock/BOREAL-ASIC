# Boreal Neuro-Core v2.4 Physical Constraints (Xilinx Artix-7 Example)
# Target Board: CMOD A7 or similar breadboard-compatible FPGA

# Timing Constraints
create_clock -period 10.000 -name sys_clk_pin -waveform {0.000 5.000} [get_ports clk_100m]

# ADS1299 SPI Interface
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { ads_sclk }];
set_property -dict { PACKAGE_PIN M19 IOSTANDARD LVCMOS33 } [get_ports { ads_miso }];
set_property -dict { PACKAGE_PIN P18 IOSTANDARD LVCMOS33 } [get_ports { ads_cs_n }];
set_property -dict { PACKAGE_PIN R18 IOSTANDARD LVCMOS33 } [get_ports { ads_drdy_n }];

# Hardware Override (Bite Switch)
set_property -dict { PACKAGE_PIN A18 IOSTANDARD LVCMOS33 } [get_ports { bite_switch_n }];

# PWM Outputs
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33 } [get_ports { pwm_motor_out }];
set_property -dict { PACKAGE_PIN K19 IOSTANDARD LVCMOS33 } [get_ports { stim_trigger_out }];

# Performance Path Constraints
set_max_delay -from [get_cells inference_engine/mu_reg*] -to [get_ports pwm_motor_out] 5.000
