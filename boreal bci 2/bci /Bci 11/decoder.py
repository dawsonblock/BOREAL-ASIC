"""
Boreal Neuro-Core v3.0 - Neural Decoder
Linear decoder with ridge regression training and fixed-point export

Decoder: u = W @ f + b
Training: W = (F^T F + lambda*I)^{-1} F^T U
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass
from .fixed_point import (
    FixedPoint, FMT_SAMPLE, FMT_STATE, FMT_MATRIX,
    FP_SAMPLE, FP_STATE, quantize_matrix, quantize_vector
)


@dataclass
class DecoderConfig:
    """Decoder configuration"""
    n_features: int = 48      # Number of input features
    n_outputs: int = 2        # Number of outputs (e.g., 2D velocity)
    lambda_reg: float = 0.01  # Ridge regularization


class LinearDecoder:
    """
    Linear decoder: u = W @ f + b
    Supports training with ridge regression and fixed-point export
    """
    
    def __init__(self, config: DecoderConfig = None):
        self.cfg = config or DecoderConfig()
        
        # Float weights (for training)
        self.W = np.zeros((self.cfg.n_outputs, self.cfg.n_features), dtype=np.float64)
        self.b = np.zeros(self.cfg.n_outputs, dtype=np.float64)
        
        # Fixed-point weights (for deployment)
        self.W_q = None
        self.b_q = None
        
        self.is_trained = False
    
    def train(self, features: np.ndarray, targets: np.ndarray) -> dict:
        """
        Train decoder using ridge regression
        
        Args:
            features: (n_samples, n_features) float array
            targets: (n_samples, n_outputs) float array
        
        Returns:
            Training metrics dict
        """
        n_samples, n_features = features.shape
        
        # Add bias term for convenience
        F_aug = np.hstack([features, np.ones((n_samples, 1))])
        
        # Ridge regression: (F^T F + lambda*I)^{-1} F^T U
        lambda_I = self.cfg.lambda_reg * np.eye(n_features + 1)
        lambda_I[-1, -1] = 0  # Don't regularize bias
        
        W_aug = np.linalg.solve(
            F_aug.T @ F_aug + lambda_I,
            F_aug.T @ targets
        )
        
        self.W = W_aug[:-1, :].T  # (n_outputs, n_features)
        self.b = W_aug[-1, :]     # (n_outputs,)
        
        # Compute training metrics
        predictions = self.decode(features)
        residuals = targets - predictions
        mse = np.mean(residuals**2)
        r2 = 1 - mse / np.var(targets)
        
        self.is_trained = True
        
        return {
            'mse': mse,
            'rmse': np.sqrt(mse),
            'r2': r2,
            'w_mean': np.mean(np.abs(self.W)),
            'w_max': np.max(np.abs(self.W))
        }
    
    def decode(self, features: np.ndarray) -> np.ndarray:
        """
        Decode features to control outputs (float)
        
        Args:
            features: (n_features,) or (n_samples, n_features)
        
        Returns:
            Control outputs
        """
        if features.ndim == 1:
            return self.W @ features + self.b
        else:
            return features @ self.W.T + self.b
    
    def decode_fixed(self, features_q: np.ndarray) -> np.ndarray:
        """
        Decode using fixed-point arithmetic (bit-accurate to FPGA)
        
        Args:
            features_q: (n_features,) Q1.15 features
        
        Returns:
            Control outputs Q5.11
        """
        if self.W_q is None or self.b_q is None:
            self.quantize()
        
        # MAC operation: W (Q5.11) @ f (Q1.15) -> Q6.26, then shift
        # Actually: W is Q5.11, f is Q1.15
        # Product: Q(5+1).(11+15) = Q6.26
        # Accumulate in 32-bit, then shift to Q5.11
        
        n_out = self.cfg.n_outputs
        result = np.zeros(n_out, dtype=np.int32)
        
        for i in range(n_out):
            acc = 0
            for j in range(self.cfg.n_features):
                acc += self.W_q[i, j] * features_q[j]
            
            # Scale: Q6.26 to Q5.11 requires shift by 15
            acc = (acc + (1 << 14)) >> 15  # Round then shift
            acc += self.b_q[i]
            result[i] = np.clip(acc, -32768 * 32, 32767 * 32)  # Q5.11 range
        
        return result.astype(np.int32)
    
    def quantize(self) -> dict:
        """
        Quantize weights to fixed-point
        
        Returns:
            Quantization error statistics
        """
        # W: map to Q5.11 (state format)
        self.W_q = quantize_matrix(self.W, FMT_STATE)
        
        # b: map to Q5.11
        self.b_q = quantize_vector(self.b, FMT_STATE)
        
        # Compute quantization error
        W_recon = FixedPoint(FMT_STATE).from_fixed(self.W_q)
        b_recon = FixedPoint(FMT_STATE).from_fixed(self.b_q)
        
        w_err = np.max(np.abs(self.W - W_recon))
        b_err = np.max(np.abs(self.b - b_recon))
        
        return {
            'W_max_error': w_err,
            'b_max_error': b_err,
            'W_q_range': (self.W_q.min(), self.W_q.max()),
            'b_q_range': (self.b_q.min(), self.b_q.max())
        }
    
    def export_coeffs(self) -> dict:
        """Export quantized coefficients for FPGA"""
        if self.W_q is None:
            self.quantize()
        
        return {
            'W_q511': self.W_q.tolist(),
            'b_q511': self.b_q.tolist(),
            'n_features': self.cfg.n_features,
            'n_outputs': self.cfg.n_outputs,
            'format_W': 'Q5.11',
            'format_b': 'Q5.11'
        }
    
    def save(self, path: str):
        """Save decoder to file"""
        np.savez(path,
                 W=self.W, b=self.b,
                 W_q=self.W_q, b_q=self.b_q,
                 config=self.cfg)
    
    @classmethod
    def load(cls, path: str) -> 'LinearDecoder':
        """Load decoder from file"""
        data = np.load(path, allow_pickle=True)
        config = data['config'].item()
        decoder = cls(config)
        decoder.W = data['W']
        decoder.b = data['b']
        if 'W_q' in data:
            decoder.W_q = data['W_q']
            decoder.b_q = data['b_q']
        decoder.is_trained = True
        return decoder


class LMSAdaptiveDecoder:
    """
    LMS adaptive decoder for online adaptation
    W_{k+1} = W_k + mu * e_k * f_k^T
    """
    
    def __init__(self, n_features: int, n_outputs: int, mu: float = 0.001):
        self.n_features = n_features
        self.n_outputs = n_outputs
        self.mu = mu
        
        # Weights
        self.W = np.zeros((n_outputs, n_features), dtype=np.float64)
        self.b = np.zeros(n_outputs, dtype=np.float64)
        
        # Fixed-point
        self.mu_shift = int(-np.log2(mu)) if mu > 0 else 10
        self.W_q = np.zeros((n_outputs, n_features), dtype=np.int32)
        self.b_q = np.zeros(n_outputs, dtype=np.int32)
    
    def decode(self, features: np.ndarray) -> np.ndarray:
        """Decode (float)"""
        return self.W @ features + self.b
    
    def decode_fixed(self, features_q: np.ndarray) -> np.ndarray:
        """Decode (fixed-point)"""
        result = np.zeros(self.n_outputs, dtype=np.int32)
        for i in range(self.n_outputs):
            acc = 0
            for j in range(self.n_features):
                acc += self.W_q[i, j] * features_q[j]
            acc = (acc + (1 << 14)) >> 15
            acc += self.b_q[i]
            result[i] = acc
        return result
    
    def update(self, features: np.ndarray, error: np.ndarray):
        """
        LMS update
        
        Args:
            features: (n_features,)
            error: (n_outputs,) prediction error
        """
        for i in range(self.n_outputs):
            self.W[i, :] += self.mu * error[i] * features
            self.b[i] += self.mu * error[i]
    
    def update_fixed(self, features_q: np.ndarray, error_q: np.ndarray,
                     enable: bool = True):
        """
        LMS update (fixed-point, bit-accurate)
        
        Args:
            features_q: (n_features,) Q1.15
            error_q: (n_outputs,) Q5.11
            enable: Update enable (gated by confidence)
        """
        if not enable:
            return
        
        for i in range(self.n_outputs):
            for j in range(self.n_features):
                # delta = mu * error * feature
                # error (Q5.11) * feature (Q1.15) = Q6.26
                # shift by mu_shift to get update magnitude
                prod = error_q[i] * features_q[j]
                delta = prod >> (self.mu_shift + 11)  # Scale appropriately
                
                # Update weight (accumulate in higher precision)
                self.W_q[i, j] += delta
                self.W_q[i, j] = np.clip(self.W_q[i, j], -2**20, 2**20 - 1)
            
            # Update bias
            self.b_q[i] += error_q[i] >> self.mu_shift
            self.b_q[i] = np.clip(self.b_q[i], -2**20, 2**20 - 1)
    
    def quantize_from_float(self):
        """Quantize current float weights"""
        self.W_q = quantize_matrix(self.W, FMT_STATE)
        self.b_q = quantize_vector(self.b, FMT_STATE)


def train_decoder_from_data(features: np.ndarray, targets: np.ndarray,
                            lambda_reg: float = 0.01) -> Tuple[LinearDecoder, dict]:
    """
    Train decoder from data
    
    Args:
        features: (n_samples, n_features)
        targets: (n_samples, n_outputs)
        lambda_reg: Regularization strength
    
    Returns:
        (decoder, metrics)
    """
    n_features = features.shape[1]
    n_outputs = targets.shape[1]
    
    config = DecoderConfig(
        n_features=n_features,
        n_outputs=n_outputs,
        lambda_reg=lambda_reg
    )
    
    decoder = LinearDecoder(config)
    metrics = decoder.train(features, targets)
    
    return decoder, metrics
