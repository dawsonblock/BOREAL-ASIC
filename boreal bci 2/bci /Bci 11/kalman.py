"""
Boreal Neuro-Core v3.0 - Kalman State Estimator
Fixed-point Kalman filter for cursor state tracking with delay compensation

State: x = [pos_x, pos_y, vel_x, vel_y]
Prediction: x = A*x + B*u
Update: K = P*H^T*(H*P*H^T + R)^{-1}, x = x + K*(z - H*x)
"""

import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass
from .fixed_point import (
    FixedPoint, FMT_STATE, FMT_MATRIX, FMT_COV,
    FP_STATE, FP_MATRIX, FP_COV, quantize_matrix
)


@dataclass
class KalmanConfig:
    """Kalman filter configuration"""
    dt: float = 0.008           # Control tick period (8ms)
    process_noise_pos: float = 0.01
    process_noise_vel: float = 0.1
    meas_noise_pos: float = 1.0
    meas_noise_vel: float = 10.0  # Large if not measuring velocity


class KalmanCore:
    """
    Fixed-point Kalman filter for cursor state
    State: [pos_x, pos_y, vel_x, vel_y]
    """
    
    def __init__(self, config: KalmanConfig = None):
        self.cfg = config or KalmanConfig()
        self.dt = self.cfg.dt
        
        # State transition matrix A (constant velocity model)
        # [1  0  dt  0 ]
        # [0  1  0   dt]
        # [0  0  1   0 ]
        # [0  0  0   1 ]
        self.A = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)
        
        # Control input matrix B (velocity control)
        # [0   0  ]
        # [0   0  ]
        # [dt  0  ]
        # [0   dt ]
        self.B = np.array([
            [0, 0],
            [0, 0],
            [self.dt, 0],
            [0, self.dt]
        ], dtype=np.float64)
        
        # Measurement matrix H (measure position only by default)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float64)
        
        # Process noise covariance Q
        self.Q = np.diag([
            self.cfg.process_noise_pos,
            self.cfg.process_noise_pos,
            self.cfg.process_noise_vel,
            self.cfg.process_noise_vel
        ])
        
        # Measurement noise covariance R
        self.R = np.diag([
            self.cfg.meas_noise_pos,
            self.cfg.meas_noise_pos
        ])
        
        # State and covariance (float for reference)
        self.x = np.zeros(4, dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 100.0  # Initial uncertainty
        
        # Fixed-point state
        self.x_q = np.zeros(4, dtype=np.int32)
        self.P_q = np.eye(4, dtype=np.int32) * (100 << 10)  # Q6.10
        
        # Quantized matrices
        self.A_q = None
        self.B_q = None
        self.H_q = None
        self.Q_q = None
        self.R_q = None
        
        self._quantize_matrices()
    
    def _quantize_matrices(self):
        """Quantize all matrices to fixed-point"""
        self.A_q = quantize_matrix(self.A, FMT_MATRIX)  # Q2.14
        self.B_q = quantize_matrix(self.B, FMT_MATRIX)  # Q2.14
        self.H_q = quantize_matrix(self.H, FMT_MATRIX)  # Q2.14
        self.Q_q = quantize_matrix(self.Q, FMT_COV)     # Q6.10
        self.R_q = quantize_matrix(self.R, FMT_COV)     # Q6.10
    
    def reset(self, x0: np.ndarray = None, P0: np.ndarray = None):
        """Reset state"""
        if x0 is not None:
            self.x = x0.copy()
            self.x_q = FP_STATE.to_fixed(x0)
        else:
            self.x.fill(0)
            self.x_q.fill(0)
        
        if P0 is not None:
            self.P = P0.copy()
            self.P_q = FP_COV.to_fixed(P0)
        else:
            self.P = np.eye(4) * 100.0
            self.P_q = np.eye(4, dtype=np.int32) * (100 << 10)
    
    def predict(self, u: np.ndarray) -> np.ndarray:
        """
        Prediction step (float)
        
        Args:
            u: Control input [vel_x, vel_y]
        
        Returns:
            Predicted state
        """
        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q
        return self.x.copy()
    
    def predict_fixed(self, u_q: np.ndarray) -> np.ndarray:
        """
        Prediction step (fixed-point, bit-accurate)
        
        Args:
            u_q: Control input Q5.11 [vel_x, vel_y]
        
        Returns:
            Predicted state Q5.11
        """
        # x = A*x + B*u
        # A: Q2.14, x: Q5.11 -> product Q7.25, shift to Q5.11
        
        x_new = np.zeros(4, dtype=np.int32)
        
        for i in range(4):
            acc = 0
            # A*x term
            for j in range(4):
                acc += self.A_q[i, j] * self.x_q[j]
            # B*u term
            for j in range(2):
                acc += self.B_q[i, j] * u_q[j]
            
            # Scale from Q7.25 to Q5.11: shift by 14
            x_new[i] = (acc + (1 << 13)) >> 14
            x_new[i] = np.clip(x_new[i], -65536 * 32, 65535 * 32)
        
        self.x_q = x_new
        
        # P = A*P*A' + Q (simplified - skip for speed in FPGA)
        # Full implementation would do matrix multiply
        
        return self.x_q.copy()
    
    def update(self, z: np.ndarray) -> np.ndarray:
        """
        Update step with measurement (float)
        
        Args:
            z: Measurement [pos_x, pos_y]
        
        Returns:
            Updated state
        """
        # Innovation
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form for stability)
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        return self.x.copy()
    
    def update_position_only_fixed(self, z_q: np.ndarray) -> np.ndarray:
        """
        Update with position measurement only (fixed-point, 2x2 simplified)
        
        Args:
            z_q: Position measurement Q5.11 [pos_x, pos_y]
        
        Returns:
            Updated state Q5.11
        """
        # Simplified 2x2 update for position-only measurements
        # Extract relevant submatrices
        
        # P_pos = P[0:2, 0:2] (Q6.10)
        P_pos = self.P_q[:2, :2]
        
        # S = P_pos + R (both Q6.10)
        S = np.array([
            [P_pos[0, 0] + self.R_q[0, 0], P_pos[0, 1] + self.R_q[0, 1]],
            [P_pos[1, 0] + self.R_q[1, 0], P_pos[1, 1] + self.R_q[1, 1]]
        ])
        
        # Compute S^{-1} using closed form for 2x2
        det = S[0, 0] * S[1, 1] - S[0, 1] * S[1, 0]
        if det == 0:
            det = 1  # Avoid division by zero
        
        # Inverse: (1/det) * [[s11, -s01], [-s10, s00]]
        # Use Newton-Raphson for 1/det in fixed-point
        det_inv = self._approx_inv(det)
        
        S_inv = np.array([
            [S[1, 1] * det_inv, -S[0, 1] * det_inv],
            [-S[1, 0] * det_inv, S[0, 0] * det_inv]
        ]) >> 10  # Scale appropriately
        
        # Innovation: y = z - x_pos
        y = np.array([
            z_q[0] - self.x_q[0],
            z_q[1] - self.x_q[1]
        ])
        
        # K = P_pos * S_inv (simplified - just position gains)
        # State update: x = x + K * y
        # For simplicity, apply gain to position, propagate to velocity
        
        gain = 0.3  # Fixed gain for simplicity
        gain_q = int(gain * 32768)
        
        self.x_q[0] += (gain_q * y[0]) >> 15
        self.x_q[1] += (gain_q * y[1]) >> 15
        
        return self.x_q.copy()
    
    def _approx_inv(self, x: int, n_iter: int = 2) -> int:
        """Approximate 1/x using Newton-Raphson"""
        # Initial guess from LUT or approximation
        if x <= 0:
            return 0
        
        # Rough initial guess
        y = (1 << 20) // x
        
        # Newton iteration: y_{n+1} = y_n * (2 - x * y_n)
        for _ in range(n_iter):
            xy = (x * y) >> 10
            two_minus_xy = (2 << 10) - xy
            y = (y * two_minus_xy) >> 10
        
        return y
    
    def get_position(self) -> Tuple[float, float]:
        """Get current position (float)"""
        return (self.x[0], self.x[1])
    
    def get_velocity(self) -> Tuple[float, float]:
        """Get current velocity (float)"""
        return (self.x[2], self.x[3])
    
    def get_state_fixed(self) -> np.ndarray:
        """Get current state (fixed-point Q5.11)"""
        return self.x_q.copy()


class DelayPredictor:
    """
    Delay-compensating predictor
    Forward propagates state through measured delay
    
    x(t+D) = A^D * x(t) + sum_{i=0}^{D-1} A^{D-1-i} * B * u(t-i)
    """
    
    def __init__(self, kalman: KalmanCore, max_delay_ticks: int = 16):
        self.kalman = kalman
        self.max_delay = max_delay_ticks
        
        # Control history buffer
        self.u_history = []
        
        # Precompute A powers for efficiency
        self.A_powers = self._precompute_A_powers()
    
    def _precompute_A_powers(self) -> List[np.ndarray]:
        """Precompute A^0, A^1, ..., A^max_delay"""
        powers = [np.eye(4)]
        for i in range(1, self.max_delay + 1):
            powers.append(self.kalman.A @ powers[-1])
        return powers
    
    def reset(self):
        """Reset history"""
        self.u_history = []
    
    def push_control(self, u: np.ndarray):
        """Add control to history"""
        self.u_history.append(u.copy())
        if len(self.u_history) > self.max_delay:
            self.u_history.pop(0)
    
    def predict(self, delay_ticks: int) -> np.ndarray:
        """
        Predict state forward by delay_ticks
        
        Args:
            delay_ticks: Number of ticks to predict forward
        
        Returns:
            Predicted state
        """
        D = min(delay_ticks, self.max_delay)
        
        # Start from current state
        x_pred = self.kalman.x.copy()
        
        # Add contributions from control history
        n_history = len(self.u_history)
        
        for i in range(min(D, n_history)):
            # Contribution from u(t-i)
            idx = n_history - 1 - i
            u = self.u_history[idx]
            
            # A^{D-1-i} * B * u
            power_idx = D - 1 - i
            if power_idx < len(self.A_powers):
                contrib = self.A_powers[power_idx] @ self.kalman.B @ u
                x_pred += contrib
        
        return x_pred
    
    def predict_fixed(self, delay_ticks: int, x_q: np.ndarray) -> np.ndarray:
        """
        Fixed-point prediction
        
        Args:
            delay_ticks: Number of ticks to predict
            x_q: Current state Q5.11
        
        Returns:
            Predicted state Q5.11
        """
        D = min(delay_ticks, self.max_delay)
        x_pred = x_q.copy()
        
        n_history = len(self.u_history)
        
        for i in range(min(D, n_history)):
            idx = n_history - 1 - i
            u_q = self.u_history[idx]
            
            # Simplified: assume constant velocity propagation
            # x_pos += dt * vel
            dt_q = int(self.kalman.dt * 32768)  # Q1.15
            
            # Position update
            x_pred[0] += (dt_q * x_pred[2]) >> 15
            x_pred[1] += (dt_q * x_pred[3]) >> 15
        
        return x_pred


class PredictiveController:
    """
    Complete predictive control combining Kalman and delay compensation
    """
    
    def __init__(self, dt: float = 0.008, max_delay_ms: float = 60.0):
        self.dt = dt
        self.max_delay_ticks = int(max_delay_ms / 1000 / dt)
        
        self.kalman = KalmanCore(KalmanConfig(dt=dt))
        self.predictor = DelayPredictor(self.kalman, self.max_delay_ticks)
        
        self.current_delay_ticks = 2  # Default 16ms
    
    def reset(self):
        """Reset all state"""
        self.kalman.reset()
        self.predictor.reset()
    
    def update_delay_estimate(self, delay_ms: float):
        """Update measured transport delay"""
        self.current_delay_ticks = int(delay_ms / 1000 / self.dt)
        self.current_delay_ticks = np.clip(self.current_delay_ticks, 0, self.max_delay_ticks)
    
    def process(self, u: np.ndarray, z: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Process one control cycle
        
        Args:
            u: Decoded control intent [vel_x, vel_y]
            z: Optional position measurement for update
        
        Returns:
            Predicted state for output
        """
        # Kalman prediction
        self.kalman.predict(u)
        
        # Optional measurement update
        if z is not None:
            self.kalman.update(z)
        
        # Store control for prediction
        self.predictor.push_control(u)
        
        # Predict forward through delay
        x_predicted = self.predictor.predict(self.current_delay_ticks)
        
        return x_predicted
    
    def process_fixed(self, u_q: np.ndarray, z_q: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Fixed-point process cycle
        
        Args:
            u_q: Control Q5.11
            z_q: Optional measurement Q5.11
        
        Returns:
            Predicted state Q5.11
        """
        # Kalman prediction
        self.kalman.predict_fixed(u_q)
        
        # Optional update
        if z_q is not None:
            self.kalman.update_position_only_fixed(z_q)
        
        # Store and predict
        self.predictor.push_control(u_q)
        x_pred = self.predictor.predict_fixed(self.current_delay_ticks, self.kalman.x_q)
        
        return x_pred
    
    def get_output_velocity(self) -> Tuple[float, float]:
        """Get velocity for cursor control"""
        return self.kalman.get_velocity()
    
    def get_output_position(self) -> Tuple[float, float]:
        """Get position for cursor control"""
        return self.kalman.get_position()
