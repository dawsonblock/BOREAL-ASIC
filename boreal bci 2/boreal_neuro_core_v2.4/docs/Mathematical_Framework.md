# The Formal Methods of Boreal v2.5

## A Mathematical Framework for Neuro-Silicon Homeostasis

This document outlines the rigorous mathematical foundations of the Boreal Neuro-Core. The system operates by minimizing a single information-theoretic objective: **Variational Free Energy (F)**.

### 1. The Core Objective: Variational Free Energy

The Boreal Core treats the brain-machine interface as a problem of **Active Inference**. The goal is to minimize the "Surprise" (surprisal) between the internal model and external neural spikes.

The Variational Free Energy $F$ is defined as:

$$F = D_{KL}[q(\psi) || p(\psi | y)] - \ln p(y)$$

Where:
- $y$ is the observed neural data (EEG/EPOC X).
- $\psi$ is the "Hidden State" or the user's intended manifold.
- $q(\psi)$ is the FPGA’s internal "guess" (the recognition density).
- $D_{KL}$ is the Kullback-Leibler divergence (the "Inference Gap").

To implement this in silicon, we simplify $F$ into a quadratic **Prediction Error**:

$$\epsilon = y - \sigma(W \cdot \mu)$$

Where:
- $\mu$ is the internal state (Manifold Position).
- $W$ is the synaptic weight matrix stored in BRAM.
- $\sigma$ is the non-linear activation function (Sigmoid).

### 2. State Estimation: Gradient Descent in Silicon

To update the manifold position $\mu$ in real-time, the FPGA performs a gradient descent on the Free Energy manifold $F$:

$$\dot{\mu} = -\frac{\partial F}{\partial \mu}$$

The discretized update rule executed in the Verilog fabric every clock cycle is:

$$\mu_{t+1} = \mu_t + \eta \cdot (\epsilon \cdot \sigma' - \lambda \cdot \mu_t)$$

Where:
- $\eta$ is the learning rate (Inference Sensitivity).
- $\sigma'$ is the derivative of the activation function (stored in the ROM LUT).
- $\lambda$ is the "decay" or "prior" that prevents the manifold from drifting into instability.

### 3. Temporal Predictive Coding (Lag Cancellation)

To cancel the $\approx 30ms$ Bluetooth lag of the EPOC X, we model the temporal dynamics of the manifold as a second-order system:

$$\hat{\mu}_{t+k} = \mu_t + k \cdot \dot{\mu}_t$$

Where:
- $\dot{\mu}_t = \mu_t - \mu_{t-1}$ (The velocity vector).
- $k$ is the Lead Factor (The look-ahead constant).

By outputting $\hat{\mu}_{t+k}$ instead of $\mu_t$, the FPGA "front-runs" the biological intent, achieving zero-perceived-latency control.

### 4. Signal Conditioning: IIR DC-Blocker

To strip the 500mV DC offset from EEG electrodes without losing the $10\mu V$ neural signal, we use a single-pole **Infinite Impulse Response (IIR)** filter:

$$y[n] = x[n] - x[n-1] + \alpha \cdot y[n-1]$$

Where $\alpha \approx 0.995$. This acts as a high-pass filter that provides perfect isolation of the neural manifold from biological drift.

### 5. Stimulus Physics: Charge Balancing

To prevent tissue damage, the stimulation output must maintain **Electrochemical Neutrality**:

$$\int I(t) dt = 0$$

The Boreal Core ensures this by generating **Biphasic Square Waves**. Every positive pulse of current (+I) is followed by an identical negative pulse (-I), resulting in a net charge injection of zero coulombs.
