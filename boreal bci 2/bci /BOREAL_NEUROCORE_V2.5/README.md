# Boreal Neuro-Core v2.5

Production-grade FPGA BCI with Active Inference.

## Quick Start

1. Generate LUT: `python3 sw/lut_generator.py -o rtl/memory/`
2. Simulate: `cd sim && iverilog -o test.vvp ../rtl/core/*.v tb_*.v && vvp test.vvp`
3. Build: Open `rtl/top/boreal_system_top.v` in Vivado

## Safety

- Hardware bite switch override
- 10% VNS duty cycle limit
- AD-Guard autonomic monitoring

## License

MIT License - Research use only. Not FDA approved.
