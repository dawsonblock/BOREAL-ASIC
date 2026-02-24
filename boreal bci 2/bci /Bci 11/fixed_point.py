"""
Boreal Neuro-Core v3.0 - Fixed-Point Arithmetic Core
Bit-accurate fixed-point math matching FPGA implementation

Format conventions:
- Samples/features: Q1.15 (16-bit signed, range [-1, 1-2^-15])
- State/control: Q5.11 (16-bit signed, range [-32, 32-2^-11])
- Matrices A,B,H: Q2.14 (16-bit signed, range [-2, 2-2^-14])
- Covariance P,Q,R: Q6.10 (16-bit signed, range [-64, 64-2^-10])
- Accumulators: 32-bit signed with saturation
"""

import numpy as np
from typing import Union, Tuple
from dataclasses import dataclass


@dataclass
class FixedFormat:
    """Fixed-point format specification"""
    int_bits: int      # Number of integer bits (including sign)
    frac_bits: int     # Number of fractional bits
    
    @property
    def total_bits(self) -> int:
        return self.int_bits + self.frac_bits
    
    @property
    def scale(self) -> int:
        return 1 << self.frac_bits
    
    @property
    def min_val(self) -> float:
        return -(1 << (self.int_bits - 1))
    
    @property
    def max_val(self) -> float:
        return (1 << (self.int_bits - 1)) - (1 / self.scale)


# Standard formats used throughout
FMT_SAMPLE = FixedFormat(1, 15)    # Q1.15 for samples/features
FMT_STATE = FixedFormat(5, 11)     # Q5.11 for state/control
FMT_MATRIX = FixedFormat(2, 14)    # Q2.14 for A, B, H matrices
FMT_COV = FixedFormat(6, 10)       # Q6.10 for P, Q, R covariance


class FixedPoint:
    """Fixed-point arithmetic with saturation and convergent rounding"""
    
    def __init__(self, format_spec: FixedFormat = FMT_SAMPLE):
        self.fmt = format_spec
        self.scale = format_spec.scale
        self.min_int = -(1 << (format_spec.total_bits - 1))
        self.max_int = (1 << (format_bits := format_spec.total_bits - 1)) - 1
    
    def to_fixed(self, x: Union[float, np.ndarray]) -> Union[int, np.ndarray]:
        """Convert float to fixed-point integer with convergent rounding"""
        scaled = np.array(x) * self.scale
        # Convergent rounding: ties to even
        rounded = np.where(
            (scaled - np.floor(scaled)) == 0.5,
            np.where(np.floor(scaled) % 2 == 0, np.floor(scaled), np.ceil(scaled)),
            np.round(scaled)
        )
        return self._saturate(rounded.astype(np.int32))
    
    def from_fixed(self, x: Union[int, np.ndarray]) -> Union[float, np.ndarray]:
        """Convert fixed-point integer to float"""
        return np.array(x).astype(np.float64) / self.scale
    
    def _saturate(self, x: np.ndarray) -> np.ndarray:
        """Saturate to representable range"""
        return np.clip(x, self.min_int, self.max_int).astype(np.int32)
    
    def add(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Fixed-point addition with saturation"""
        result = a.astype(np.int32) + b.astype(np.int32)
        return self._saturate(result)
    
    def sub(self, a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Fixed-point subtraction with saturation"""
        result = a.astype(np.int32) - b.astype(np.int32)
        return self._saturate(result)
    
    def mul(self, a: np.ndarray, b: np.ndarray, 
            result_fmt: FixedFormat = None) -> np.ndarray:
        """
        Fixed-point multiplication with rounding to result format.
        Default keeps same format as self.
        """
        if result_fmt is None:
            result_fmt = self.fmt
        
        # Multiply in extended precision
        prod = a.astype(np.int32) * b.astype(np.int32)
        
        # Scale down: product is Q(2*int).(2*frac)
        shift = self.fmt.frac_bits - result_fmt.frac_bits
        
        # Convergent rounding
        if shift > 0:
            round_bit = 1 << (shift - 1)
            rounded = (prod + round_bit) >> shift
        else:
            rounded = prod << (-shift)
        
        # Saturate to result format
        result_fp = FixedPoint(result_fmt)
        return result_fp._saturate(rounded)
    
    def mac(self, a: np.ndarray, b: np.ndarray, 
            accum: np.ndarray = None) -> np.ndarray:
        """
        Multiply-accumulate with 32-bit accumulator.
        Returns full 32-bit result (caller must round/sat as needed).
        """
        prod = a.astype(np.int64) * b.astype(np.int64)
        if accum is not None:
            prod = prod + accum.astype(np.int64)
        # Saturate to 32-bit
        return np.clip(prod, -2147483648, 2147483647).astype(np.int32)


def quantize_matrix(mat: np.ndarray, fmt: FixedFormat) -> np.ndarray:
    """Quantize a matrix to fixed-point format"""
    fp = FixedPoint(fmt)
    return fp.to_fixed(mat)


def quantize_vector(vec: np.ndarray, fmt: FixedFormat) -> np.ndarray:
    """Quantize a vector to fixed-point format"""
    fp = FixedPoint(fmt)
    return fp.to_fixed(vec)


def check_overflow(x: np.ndarray, fmt: FixedFormat, name: str = "") -> bool:
    """Check if values exceed fixed-point range"""
    fp = FixedPoint(fmt)
    x_arr = np.array(x)
    min_v, max_v = x_arr.min(), x_arr.max()
    
    overflow = False
    if min_v < fp.min_int:
        print(f"OVERFLOW: {name} min {min_v} < {fp.min_int}")
        overflow = True
    if max_v > fp.max_int:
        print(f"OVERFLOW: {name} max {max_v} > {fp.max_int}")
        overflow = True
    
    return overflow


def compute_quantization_error(float_val: np.ndarray, 
                                fixed_val: np.ndarray,
                                fmt: FixedFormat) -> dict:
    """Compute quantization error statistics"""
    fp = FixedPoint(fmt)
    reconstructed = fp.from_fixed(fixed_val)
    error = float_val - reconstructed
    
    return {
        'max_abs_error': np.max(np.abs(error)),
        'rmse': np.sqrt(np.mean(error**2)),
        'max_relative_error': np.max(np.abs(error / (float_val + 1e-10))),
        'lsb': 1.0 / fmt.scale
    }


# Convenience instances
FP_SAMPLE = FixedPoint(FMT_SAMPLE)
FP_STATE = FixedPoint(FMT_STATE)
FP_MATRIX = FixedPoint(FMT_MATRIX)
FP_COV = FixedPoint(FMT_COV)
