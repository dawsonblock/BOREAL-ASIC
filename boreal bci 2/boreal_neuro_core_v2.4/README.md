# Boreal Neuro-Core v2.4

The Boreal Neuro-Core v2.4 is a specialized FPGA architecture designed to close the loop between the human nervous system and high-speed robotics. It bypasses the latency of software-based neural processing by executing **Active Inference** directly in the silicon fabric.

## Architecture Highlights

- **Inference Latency**: < 500ns (Input to Internal State Update).
- **Core Principle**: Variational Free Energy Minimization (FEP).
- **Signal Integrity**: Integrated 24-bit DC-blocking high-pass filters and saturation arithmetic to prevent overflows.
- **Hardware Plasticity**: Hebbian Learning Module for real-time synaptic weight updates in BRAM.
- **Scaling**: Daisy-chain support for up to 32 EEG channels (4x ADS1299).

## Project Structure

```text
boreal_neuro_core_v2.4/
├── rtl/                # Verilog Source Files
│   ├── boreal_system_top.v     # Top-level wrapper
│   ├── boreal_apex_core.v      # Inference engine & filtering
│   ├── boreal_memory.v         # Dual-port BRAM for weights/LUT
│   ├── boreal_pll_tracker.v    # Adaptive phase tracking
│   ├── boreal_vns_control.v    # Vagus Nerve Stimulation controller
│   ├── boreal_peripherals.v    # Learning engine, PWM, and expansion
│   └── boreal_ads1299_spi.v    # ADS1299 SPI interface
├── scripts/            # Support Scripts
│   └── lut_gen.py              # Sigmoid and Derivative LUT generator
├── sim/                # Simulation & Verification
│   └── boreal_tb.v             # System testbench
├── constraints/        # FPGA Physical Constraints
│   └── boreal_constraints.xdc  # Xilinx Artix-7 constraints
├── data/               # Generated Data Files
│   └── boreal_lut.mem          # Memory initialization file
└── docs/               # Documentation
    ├── Project_Summary.md      # Overview of the project
    └── Mathematical_Framework.md # FEP and Active Inference details
```

## Getting Started

### 1. Initialize Memory
Run the Python script to generate the Sigmoid and Derivative LUT used by the inference engine:
```bash
cd scripts
python3 lut_gen.py
```
This will create `boreal_lut.mem` in the `data/` directory.

### 2. Synthesis & Implementation
Load all `.v` files from the `rtl/` directory and the `.xdc` file from the `constraints/` directory into your FPGA toolchain (e.g., Xilinx Vivado).

### 3. Deployment
- Connect the **ADS1299 SPI** pins as defined in the constraints.
- Ensure the **bite_switch_n** physical override is connected for safety.
- Monitor the **pwm_motor_out** and **stim_trigger_out** for control signals.

## Mathematical Foundation

The system operates by minimizing **Variational Free Energy (F)**, treating the brain-machine interface as an Active Inference problem. The goal is to minimize the "Surprise" between the internal model and external neural data.

For more details, see the documentation in the `docs/` directory.

## Safety Notice

**Ensure the emergency bite-switch is physically accessible during all testing phases.** The system includes hardware-level safety guards, but physical overrides are mandatory for human-in-the-loop operation.
