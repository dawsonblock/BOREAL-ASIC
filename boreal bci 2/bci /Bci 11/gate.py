"""
Boreal Neuro-Core v3.0 - Safety Gate Policy
Deterministic output gating with confidence thresholding, clamping, and jerk limiting

Gate rules:
- Confidence threshold: output zero if confidence < tau
- Magnitude clamp: limit max velocity
- Jerk limit: limit rate of change
- Dwell click: require stability for click events
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass
from .fixed_point import FixedPoint, FMT_STATE, FP_STATE


@dataclass
class GateConfig:
    """Gate policy configuration"""
    conf_threshold: float = 0.3       # Minimum confidence for output
    conf_click_threshold: float = 0.6  # Minimum confidence for click
    max_velocity: float = 20.0        # Max velocity (screen units/sec)
    max_jerk: float = 100.0           # Max jerk (screen units/sec^2)
    dwell_ticks: int = 12             # Ticks for dwell stability (~100ms)
    decay_rate: float = 0.9           # Velocity decay when gated


class SafetyGate:
    """
    Deterministic safety gate for neural control outputs
    """
    
    def __init__(self, config: GateConfig = None):
        self.cfg = config or GateConfig()
        
        # Previous output for jerk limiting
        self.u_prev = np.zeros(2, dtype=np.float64)
        self.u_prev_q = np.zeros(2, dtype=np.int32)
        
        # Dwell tracking
        self.dwell_counter = 0
        self.click_ready = False
        
        # Quantized limits
        self.max_vel_q = FP_STATE.to_fixed(self.cfg.max_velocity)
        self.max_jerk_q = FP_STATE.to_fixed(self.cfg.max_jerk)
    
    def reset(self):
        """Reset gate state"""
        self.u_prev.fill(0)
        self.u_prev_q.fill(0)
        self.dwell_counter = 0
        self.click_ready = False
    
    def gate(self, u: np.ndarray, confidence: float) -> Tuple[np.ndarray, bool]:
        """
        Apply gate policy (float version)
        
        Args:
            u: Raw control output [vel_x, vel_y]
            confidence: Confidence score [0, 1]
        
        Returns:
            (gated_output, click_triggered)
        """
        u = np.array(u)
        
        # Confidence threshold
        if confidence < self.cfg.conf_threshold:
            # Decay to rest
            u_gated = self.u_prev * self.cfg.decay_rate
        else:
            u_gated = u.copy()
            
            # Magnitude clamp
            mag = np.linalg.norm(u_gated)
            if mag > self.cfg.max_velocity:
                u_gated = u_gated * (self.cfg.max_velocity / mag)
            
            # Jerk limit
            jerk = u_gated - self.u_prev
            jerk_mag = np.linalg.norm(jerk)
            if jerk_mag > self.cfg.max_jerk:
                jerk = jerk * (self.cfg.max_jerk / jerk_mag)
                u_gated = self.u_prev + jerk
        
        # Update state
        self.u_prev = u_gated
        
        # Dwell click logic
        click = False
        if confidence >= self.cfg.conf_click_threshold:
            vel_mag = np.linalg.norm(u_gated)
            if vel_mag < 1.0:  # Near stationary
                self.dwell_counter += 1
                if self.dwell_counter >= self.cfg.dwell_ticks:
                    if not self.click_ready:
                        click = True
                        self.click_ready = True
            else:
                self.dwell_counter = 0
                self.click_ready = False
        else:
            self.dwell_counter = 0
            self.click_ready = False
        
        return u_gated, click
    
    def gate_fixed(self, u_q: np.ndarray, conf_q: int) -> Tuple[np.ndarray, bool]:
        """
        Apply gate policy (fixed-point, bit-accurate)
        
        Args:
            u_q: Raw control Q5.11 [vel_x, vel_y]
            conf_q: Confidence Q1.15 [0, 32767]
        
        Returns:
            (gated_output Q5.11, click_triggered)
        """
        u_q = np.array(u_q, dtype=np.int32)
        
        # Confidence threshold (Q1.15)
        conf_threshold_q = int(self.cfg.conf_threshold * 32767)
        conf_click_q = int(self.cfg.conf_click_threshold * 32767)
        
        if conf_q < conf_threshold_q:
            # Decay
            decay_q = FP_STATE.to_fixed(self.cfg.decay_rate)
            u_gated = np.zeros(2, dtype=np.int32)
            for i in range(2):
                u_gated[i] = (decay_q * self.u_prev_q[i]) >> 15
        else:
            u_gated = u_q.copy()
            
            # Magnitude clamp
            # |u|^2 = u0^2 + u1^2 (in Q10.22, need to scale)
            mag_sq = (u_gated[0] * u_gated[0] + u_gated[1] * u_gated[1]) >> 11
            max_vel_sq = (self.max_vel_q * self.max_vel_q) >> 11
            
            if mag_sq > max_vel_sq:
                # Scale down
                scale = int(np.sqrt(max_vel_sq / max(mag_sq, 1)) * 32768)
                for i in range(2):
                    u_gated[i] = (scale * u_gated[i]) >> 15
            
            # Jerk limit
            jerk = np.array([
                u_gated[0] - self.u_prev_q[0],
                u_gated[1] - self.u_prev_q[1]
            ])
            jerk_sq = (jerk[0] * jerk[0] + jerk[1] * jerk[1]) >> 11
            max_jerk_sq = (self.max_jerk_q * self.max_jerk_q) >> 11
            
            if jerk_sq > max_jerk_sq:
                scale = int(np.sqrt(max_jerk_sq / max(jerk_sq, 1)) * 32768)
                for i in range(2):
                    jerk[i] = (scale * jerk[i]) >> 15
                for i in range(2):
                    u_gated[i] = self.u_prev_q[i] + jerk[i]
        
        # Saturate
        for i in range(2):
            u_gated[i] = np.clip(u_gated[i], -65536 * 32, 65535 * 32)
        
        self.u_prev_q = u_gated.copy()
        
        # Dwell click
        click = False
        if conf_q >= conf_click_q:
            vel_sq = (u_gated[0] * u_gated[0] + u_gated[1] * u_gated[1]) >> 11
            # Threshold: 1.0 in Q5.11 = 2048
            if vel_sq < (2048 * 2048) >> 11:
                self.dwell_counter += 1
                if self.dwell_counter >= self.cfg.dwell_ticks:
                    if not self.click_ready:
                        click = True
                        self.click_ready = True
            else:
                self.dwell_counter = 0
                self.click_ready = False
        else:
            self.dwell_counter = 0
            self.click_ready = False
        
        return u_gated, click
    
    def compute_confidence(self, features: np.ndarray, 
                          innovation: float,
                          snr_proxy: float) -> float:
        """
        Compute confidence score from multiple signals
        
        Args:
            features: Feature vector
            innovation: Kalman innovation magnitude
            snr_proxy: Signal quality estimate
        
        Returns:
            Confidence [0, 1]
        """
        # Feature stability (variance of recent features)
        feat_var = np.var(features) if len(features) > 0 else 1.0
        
        # Combine signals (simple weighted sum)
        conf = 0.4 * np.clip(snr_proxy / 10.0, 0, 1)
        conf += 0.4 * np.clip(1.0 - innovation / 5.0, 0, 1)
        conf += 0.2 * np.clip(1.0 - feat_var, 0, 1)
        
        return np.clip(conf, 0, 1)


class ConfidenceEstimator:
    """
    Multi-signal confidence estimator
    """
    
    def __init__(self, n_features: int):
        self.n_features = n_features
        
        # Running statistics for feature stability
        self.feat_mean = np.zeros(n_features)
        self.feat_var = np.ones(n_features)
        self.alpha = 0.95
        
        # SNR tracking
        self.signal_power = 1.0
        self.noise_power = 0.1
    
    def reset(self):
        """Reset estimator"""
        self.feat_mean.fill(0)
        self.feat_var.fill(1)
        self.signal_power = 1.0
        self.noise_power = 0.1
    
    def update(self, features: np.ndarray, 
               kalman_innovation: float,
               artifact_detected: bool) -> float:
        """
        Update confidence estimate
        
        Args:
            features: Current feature vector
            kalman_innovation: Normalized innovation
            artifact_detected: Artifact flag
        
        Returns:
            Confidence [0, 1]
        """
        if artifact_detected:
            return 0.0
        
        # Update feature statistics
        self.feat_mean = self.alpha * self.feat_mean + (1 - self.alpha) * features
        diff = features - self.feat_mean
        self.feat_var = self.alpha * self.feat_var + (1 - self.alpha) * (diff ** 2)
        
        # Feature stability (inverse of variance)
        feat_stability = 1.0 / (1.0 + np.mean(self.feat_var))
        
        # Kalman consistency
        kalman_consistency = np.exp(-kalman_innovation / 2.0)
        
        # SNR estimate
        snr = self.signal_power / max(self.noise_power, 0.001)
        snr_score = np.clip(snr / 10.0, 0, 1)
        
        # Combined confidence
        conf = 0.3 * feat_stability + 0.4 * kalman_consistency + 0.3 * snr_score
        
        return np.clip(conf, 0, 1)
    
    def update_fixed(self, features_q: np.ndarray,
                     innov_q: int,
                     artifact_flag: bool) -> int:
        """
        Fixed-point confidence update
        
        Returns:
            Confidence Q1.15
        """
        if artifact_flag:
            return 0
        
        # Simplified: use innovation as primary signal
        # innov expected in Q5.11, threshold around 5.0 = 10240
        
        threshold_q = 10240  # 5.0 in Q5.11
        
        if innov_q > threshold_q:
            conf_q = 0
        else:
            # Linear falloff
            conf_q = ((threshold_q - innov_q) * 32767) // threshold_q
        
        return np.clip(conf_q, 0, 32767)
