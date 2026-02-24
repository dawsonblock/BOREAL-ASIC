#!/usr/bin/env python3
"""Boreal LUT Generator - Creates sigmoid tables for FPGA"""
import numpy as np
import argparse
import os

def generate_lut(entries=1024, x_range=8.0, output_dir="."):
    scale = 2**15
    step = (2 * x_range) / entries
    lut_data = []

    for i in range(entries):
        x = -x_range + (i + 0.5) * step
        sig = 1.0 / (1.0 + np.exp(-x))
        deriv = sig * (1.0 - sig)
        sig_fixed = int(np.clip(round(sig * (scale - 1)), 0, scale - 1))
        deriv_fixed = int(np.clip(round(deriv * (scale - 1)), 0, scale - 1))
        packed = (deriv_fixed << 16) | sig_fixed
        lut_data.append(packed)

    mem_path = os.path.join(output_dir, "boreal_lut.mem")
    with open(mem_path, "w") as f:
        f.write("// Boreal LUT v2.5\n")
        for packed in lut_data:
            f.write(f"{packed:08X}\n")

    print(f"Generated {mem_path} with {entries} entries")
    return lut_data

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--entries", type=int, default=1024)
    parser.add_argument("-r", "--range", type=float, default=8.0)
    parser.add_argument("-o", "--output", type=str, default=".")
    args = parser.parse_args()
    generate_lut(args.entries, args.range, args.output)
