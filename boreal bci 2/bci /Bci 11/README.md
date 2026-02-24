# Boreal Neuro-Core v3.0

Deterministic FPGA-based neural control kernel with predictive decoding for real-time BCI applications.

## Overview

The Boreal Neuro-Core v3.0 is a complete real-time signal processing pipeline designed for low-latency, deterministic neural control. It implements:

- **Fixed-rate processing**: 4-8 ms control ticks with bounded jitter
- **Predictive delay compensation**: Forward prediction through measured transport delay
- **Safety gating**: Confidence-thresholded, clamped, and jerk-limited outputs
- **Bit-accurate software twin**: Python implementation matching FPGA behavior exactly
- **Complete training pipeline**: Ridge regression → fixed-point export → deployment

## System Architecture

```
ADC → Conditioning → Bandpower → Features → Decoder → Kalman → Predictor → Gate → Output
        (DC/Notch)    (EWMA)      (Log)      (Linear)  (State)   (Delay)   (Safety)
```

### Timing Budget

| Stage | Latency |
|-------|---------|
| ADC → Epoch | 0.1-0.5 ms |
| Filter chain | < 5 µs |
| Feature update | 32-128 ms (windowed) |
| Decoder + Kalman | < 10 µs |
| Prediction | < 5 µs |
| Control tick | 4-8 ms |
| **Perceived response** | **10-30 ms** (with prediction) |

## Repository Structure

```
boreal_neurocore_v3/
├── twin/                    # Software twin (bit-accurate)
│   ├── fixed_point.py       # Fixed-point arithmetic core
│   ├── filters.py           # IIR filters (DC blocker, notch, bandpass)
│   ├── bandpower.py         # EWMA bandpower features
│   ├── decoder.py           # Linear decoder + LMS adaptive
│   ├── kalman.py            # Kalman filter + delay predictor
│   ├── gate.py              # Safety gate policy
│   ├── timing_emulator.py   # Timing/jitter simulation
│   └── quantize_to_fpga.py  # Coefficient export pipeline
│
├── train/                   # Training pipeline
│   └── calibrate.py         # Phase 1-3 calibration
│
├── host/                    # Host-side tools
│   └── metrics_fitts.py     # Fitts' Law evaluation
│
├── tests/                   # Verification tests
│   └── test_twin.py         # Complete test suite
│
├── demo.py                  # Quick start demonstration
└── README.md
```

## Quick Start

### Installation

```bash
cd boreal_neurocore_v3
python3 -c "import twin; print('OK')"
```

### Run Tests

```bash
python3 tests/test_twin.py
```

### Run Demo

```bash
python3 demo.py
```

### Train and Export

```python
from train.calibrate import quick_calibrate

# Run full calibration with synthetic data
cal = quick_calibrate(
    data_path='./my_data',
    output_dir='./fpga_coeffs'
)

# Export for FPGA
result = cal.export_for_fpga('./fpga_coeffs')
print(f"Exported: {result['json_path']}")
```

## Fixed-Point Formats

| Signal | Format | Range | Precision |
|--------|--------|-------|-----------|
| Samples/Features | Q1.15 | [-1, 1) | 3e-5 |
| State/Control | Q5.11 | [-32, 32) | 4.9e-4 |
| Matrices (A,B,H) | Q2.14 | [-2, 2) | 6.1e-5 |
| Covariance (P,Q,R) | Q6.10 | [-64, 64) | 9.8e-4 |

## Software Twin Components

### 1. Fixed-Point Core (`fixed_point.py`)

```python
from twin import FixedPoint, FMT_SAMPLE, FMT_STATE

# Quantize float to fixed-point
fp = FixedPoint(FMT_SAMPLE)
x_q = fp.to_fixed([0.5, -0.25, 0.1])
x_back = fp.from_fixed(x_q)

# MAC with saturation
acc = fp.mac(a_q, b_q, accum=prev_acc)
```

### 2. Filters (`filters.py`)

```python
from twin import create_eeg_conditioning

# Create conditioning chain
chain = create_eeg_conditioning(fs=250.0, line_freq=60.0)

# Process samples
for sample in eeg_stream:
    y = chain.process_sample(sample)
```

### 3. Bandpower Features (`bandpower.py`)

```python
from twin import create_standard_bandpower

# Multi-channel bandpower extractor
extractor = create_standard_bandpower(n_channels=16, fs=250.0)

# Process frame
features = extractor.process_frame(band_data)  # Returns every 8ms
```

### 4. Decoder (`decoder.py`)

```python
from twin import LinearDecoder, DecoderConfig

# Train decoder
config = DecoderConfig(n_features=48, n_outputs=2)
decoder = LinearDecoder(config)
metrics = decoder.train(features, targets)

# Quantize for FPGA
decoder.quantize()
coeffs = decoder.export_coeffs()
```

### 5. Kalman + Predictor (`kalman.py`)

```python
from twin import PredictiveController

# Create controller with delay compensation
controller = PredictiveController(dt=0.008, max_delay_ms=60.0)

# Update delay estimate from measurements
controller.update_delay_estimate(measured_delay_ms)

# Process control cycle
x_pred = controller.process(u_decoded)
position = controller.get_output_position()
```

### 6. Safety Gate (`gate.py`)

```python
from twin import SafetyGate, GateConfig

# Configure gate policy
gate = SafetyGate(GateConfig(
    conf_threshold=0.3,
    max_velocity=20.0,
    max_jerk=100.0,
    dwell_ticks=12
))

# Apply gate
u_gated, click = gate.gate(u_raw, confidence=0.8)
```

### 7. Timing Emulator (`timing_emulator.py`)

```python
from twin import TimingEmulator, create_wireless_timing

# Emulate wireless transport
timing = TimingEmulator(create_wireless_timing())

# Send packets
timing.send_packet(data, timestamp)

# Process time step
outputs = timing.process(dt_ms=1.0)

# Get statistics
stats = timing.get_stats()
print(f"Mean jitter: {stats.mean_jitter:.2f} ms")
```

## Coefficient Export

The export pipeline generates FPGA-ready files:

```python
from twin import export_full_system

result = export_full_system(
    kalman, decoder,
    output_dir='./fpga',
    dc_alpha=0.995,
    max_vel=20.0,
    max_jerk=100.0,
    conf_threshold=0.3
)

# Generated files:
# - coeffs.json          # Complete coefficient set
# - mem/*.mem            # $readmemh files
# - params.v             # Verilog localparams
```

### JSON Structure

```json
{
  "version": "3.0",
  "timestamp": "2024-01-01T00:00:00",
  "coefficients": {
    "kalman": {
      "A_q214": [[...], ...],
      "B_q214": [[...], ...],
      "Q_q610": [[...], ...],
      "R_q610": [[...], ...]
    },
    "decoder": {
      "W_q511": [[...], ...],
      "b_q511": [...]
    },
    "filters": {
      "dc_alpha_q115": 32576,
      "notch_b_q115": [...]
    },
    "gate": {
      "max_vel_q511": 40960,
      "conf_threshold_q115": 9830
    }
  },
  "error_bounds": {
    "kalman": { "A": {...}, ... },
    "decoder": { "W": {...}, ... }
  }
}
```

## Training Pipeline

### Phase 1: Data Collection (5-10 min)

```python
from train.calibrate import DataCollector, CalibrationConfig

config = CalibrationConfig(
    fs=250.0,
    n_channels=16,
    rest_duration_s=60.0,
    task_duration_s=300.0
)

collector = DataCollector(config)
collector.collect_rest()
collector.collect_task(target_pattern='cursor_2d')
```

### Phase 2: Training (minutes)

```python
from train.calibrate import Calibrator

cal = Calibrator(config)
cal.run_phase1_collect()
metrics = cal.run_phase2_train(lambda_reg=0.01)

print(f"RMSE: {metrics['rmse']:.3f}, R²: {metrics['r2']:.3f}")
```

### Phase 3: Validation

```python
validation = cal.run_phase3_validate(n_trials=10)
print(f"Success rate: {validation['success_rate']:.2f}")
```

## Performance Validation

### Fitts' Law Metrics

```python
from host.metrics_fitts import FittsEvaluator, RadialFittsTask

# Generate task sequence
task_gen = RadialFittsTask(
    center=(512, 384),
    amplitudes=[128, 256, 512],
    widths=[32, 64, 128]
)
tasks = task_gen.generate_sequence(n_repetitions=3)

# Evaluate
evaluator = FittsEvaluator()
for task in tasks:
    evaluator.start_trial(task, timestamp)
    # ... run control ...
    evaluator.end_trial(success, timestamp)

# Get throughput
summary = evaluator.get_summary()
print(f"Throughput: {summary['throughput']['throughput']:.2f} bits/s")
```

### Timing Statistics

```python
from twin import TimingEmulator, create_default_timing

timing = TimingEmulator(create_default_timing())

# Run simulation
for i in range(1000):
    packet = {'seq': i, 'data': ...}
    timing.send_packet(packet, i * 8.0)
    outputs = timing.process(1.0)

# Analyze
stats = timing.get_stats()
print(f"Jitter: {stats.std_jitter:.2f} ms (std)")
print(f"Latency: {stats.mean_latency:.2f} ms (mean)")
```

## FPGA Integration

### Build Order

1. **Ingest + framing** - ADC interface, timestamping
2. **Filters** - DC blocker, notch, bandpass bank
3. **Bandpower** - EWMA power accumulator
4. **Feature packer** - Vector assembly
5. **Decoder** - DSP MAC for linear model
6. **Kalman** - Fixed-point state estimator
7. **Predictor** - Delay compensation
8. **Gate** - Clamp, jerk limit, confidence
9. **Host link** - USB/PCIe/UART
10. **Ledger** - Hash chain for audit

### Resource Estimate (Artix-7)

| Resource | Typical |
|----------|---------|
| DSP Slices | 60-90 |
| LUTs | 18k-35k |
| BRAM | 24-48 |
| Clock | 100-200 MHz |
| Power | ~2-3W |

## Biological Limits

| Metric | Typical Range |
|--------|---------------|
| Continuous DoF | 1-3 |
| Reaction time | 20-50 ms (SNR limited) |
| Throughput | 0.5-3 bits/s |
| Recalibration | Minutes (warm start) |

**Note**: Deterministic FPGA improves stability and feel, not raw neural bandwidth.

## Verification

### Bit-Exact Testing

```python
# Twin generates stimulus
twin_output = twin_process(stimulus)

# RTL simulation (using exported coefficients)
rtl_output = rtl_simulate(stimulus, coeffs)

# Compare within ±1 LSB
assert np.allclose(twin_output, rtl_output, atol=1)
```

### Test Suite

```bash
python3 tests/test_twin.py
```

Tests cover:
- Fixed-point arithmetic (quantization, saturation, MAC)
- Filter chain (DC blocker, notch, bandpass)
- Bandpower extraction (EWMA, log transform)
- Decoder training and quantization
- Kalman tracking and prediction
- Safety gate policies
- Timing emulation
- Coefficient export

## License

MIT License - See LICENSE file for details.

## References

- Fitts, P.M. (1954). The information capacity of the human motor system.
- Wolpaw, J.R. et al. (2002). Brain-computer interfaces for communication and control.
- Fixed-point DSP: Oppenheim & Schafer, Discrete-Time Signal Processing.

## Contact

For questions about implementation or integration, refer to the specification document or open an issue.
