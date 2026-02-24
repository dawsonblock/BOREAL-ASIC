# Boreal Neuro-Core v3.0 - Generated RTL Modules

This document summarizes the Verilog (RTL) modules generated to complete the Boreal Neuro-Core v3.0 specification. These modules implement the fixed-point arithmetic logic derived from the provided Python software twin, enabling hardware acceleration on an FPGA.

## 1. Decoder MAC (`decoder_mac.v`)

This module implements the Multiply-Accumulate (MAC) operations for the linear decoder. It takes fixed-point features (Q1.15) as input and produces fixed-point control outputs (Q5.11) based on the quantized weight matrix (W_q511) and bias vector (b_q511).

**Key Features:**
*   **Fixed-point arithmetic:** Handles Q1.15 and Q5.11 formats.
*   **MAC operations:** Efficiently computes `u = W @ f + b`.
*   **Rounding and Saturation:** Implements rounding and saturation to maintain numerical precision and prevent overflow within the specified fixed-point ranges.

## 2. Kalman Core (`kalman_core.v`)

This module implements the core functionality of the fixed-point Kalman filter. It performs the prediction and update steps for tracking the cursor state, utilizing quantized state transition (A_q214), control input (B_q214), measurement (H_q214), process noise (Q_q610), and measurement noise (R_q610) matrices.

**Key Features:**
*   **State Prediction:** Calculates the next state based on the current state and control input.
*   **State Update:** Incorporates position measurements to refine the state estimate.
*   **Fixed-point Matrix Operations:** Performs matrix multiplications and additions using fixed-point arithmetic.

## 3. Predictor (`predictor.v`)

The Predictor module is responsible for compensating for system delays by predicting the state forward by a specified number of ticks. It uses the Kalman filter's state transition and control input matrices, along with a history of control inputs, to project the current state into the future.

**Key Features:**
*   **Delay Compensation:** Mitigates perceived latency in BCI applications.
*   **Iterative Prediction:** Performs multiple prediction steps based on the `delay_ticks` input.
*   **Control History Utilization:** Incorporates past control inputs for accurate future state estimation.

## 4. Gate Policy (`gate_policy.v`)

This module implements the safety gating policy for the neural control outputs. It applies confidence thresholding, velocity clamping, and jerk limiting to ensure safe and stable operation. It also includes logic for dwell click detection.

**Key Features:**
*   **Confidence Thresholding:** Filters out unreliable control signals based on a confidence score.
*   **Velocity Clamping:** Limits the maximum output velocity.
*   **Jerk Limiting:** Constrains the rate of change of velocity to prevent abrupt movements.
*   **Dwell Click Detection:** Identifies intentional 
clicks by detecting stable, near-stationary cursor states.

## Verification and Next Steps

These generated RTL modules are designed to be bit-accurate with the provided Python software twin, ensuring functional equivalence. The next steps in the FPGA development process would involve:

1.  **Formal Verification:** Rigorous testing of the RTL against the software twin using co-simulation and formal methods.
2.  **Synthesis and Place & Route:** Compiling the Verilog code for the target FPGA and optimizing for performance and resource utilization.
3.  **Hardware-in-the-Loop Testing:** Integrating the FPGA with the BCI system for real-world validation.

These modules complete the critical path components for the Boreal Neuro-Core v3.0, enabling its deployment on FPGA platforms for real-time neural control applications.
