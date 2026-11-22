"""Fixed-point quantization utilities and high-Q resonator bit-width analysis.

Demonstrates why high-Q Coriolis Vibratory Gyroscopes (DRG Q=1e5, HRG Q=1e6)
require energy-preserving (Strang-split exact rotation) discretization and
>= 40-bit state accumulators (matching Zynq UltraScale+ DSP48E2 48-bit accumulators).
"""

from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class QFormat:
    """Signed two's-complement fixed-point format Q(int_bits).(frac_bits)."""
    total_bits: int
    frac_bits: int

    @property
    def int_bits(self) -> int:
        return self.total_bits - self.frac_bits

    @property
    def scale(self) -> int:
        return 1 << self.frac_bits

    @property
    def min_int(self) -> int:
        return -(1 << (self.total_bits - 1))

    @property
    def max_int(self) -> int:
        return (1 << (self.total_bits - 1)) - 1

    def to_int(self, val: float) -> int:
        q = int(round(val * self.scale))
        return max(self.min_int, min(self.max_int, q))

    def to_float(self, q: int) -> float:
        return float(q) / float(self.scale)

    def mul_shift(self, a: int, b: int, b_frac: int) -> int:
        """Signed multiply with round-to-nearest and saturation to this format."""
        prod = a * b
        if b_frac > 0:
            prod = (prod + (1 << (b_frac - 1))) >> b_frac
        return max(self.min_int, min(self.max_int, prod))


def quantize_array(arr: np.ndarray, total_bits: int, frac_bits: int) -> np.ndarray:
    """Vectorized symmetric rounding + saturation to signed Q(total-frac).(frac)."""
    scale = float(1 << frac_bits)
    lo = -(1 << (total_bits - 1))
    hi = (1 << (total_bits - 1)) - 1
    q = np.clip(np.round(arr * scale), lo, hi)
    return q / scale


def strang_resonator_ringdown_fixed(
    f0_hz: float,
    q_factor: float,
    fs_hz: float,
    duration_s: float,
    state_bits: int,
    int_bits: int = 4,
    coeff_bits: int = 32,
) -> dict[str, float]:
    """Run a bit-accurate integer Strang-split 1-DOF resonator ringdown test.

    State update per step (zero numerical damping):
      1) Rotate (x, v) by theta = 2*pi*f0/fs using exact (cos_q, sin_q)
      2) Decay v by d = exp(-2*pi*f0 / (Q * fs))

    Returns measured effective Q, relative Q error, and final amplitude ratio.
    """
    state_fmt = QFormat(total_bits=state_bits, frac_bits=state_bits - int_bits)
    coeff_frac = coeff_bits - 2  # Q2.(coeff_bits-2), covers [-2, +2)
    coeff_fmt = QFormat(total_bits=coeff_bits, frac_bits=coeff_frac)

    w0 = 2.0 * math.pi * f0_hz
    dt = 1.0 / fs_hz
    theta = w0 * dt

    # Exact rotation & velocity damping factor per step
    # Note: damping only velocity by exp(-w0*dt/Q) gives amplitude envelope exp(-w0*t/(2Q))
    c_q = coeff_fmt.to_int(math.cos(theta))
    s_q = coeff_fmt.to_int(math.sin(theta))
    decay_exact = math.exp(-w0 * dt / q_factor)
    # Represent decay as (1 - alpha) so alpha has full resolution near 1.0!
    alpha_bits = min(coeff_bits, state_bits)
    alpha_frac = alpha_bits - 2
    alpha_q = int(round((1.0 - decay_exact) * (1 << alpha_frac)))

    x = state_fmt.to_int(1.0)
    v = 0
    n_steps = int(round(duration_s * fs_hz))
    c_half = 1 << (coeff_frac - 1)
    a_half = 1 << (alpha_frac - 1)
    lo, hi = state_fmt.min_int, state_fmt.max_int

    for _ in range(n_steps):
        # Exact 2x2 harmonic rotation
        x_new = (x * c_q + v * s_q + c_half) >> coeff_frac
        v_rot = (-x * s_q + v * c_q + c_half) >> coeff_frac
        # Exact high-Q damping: v_new = v_rot - alpha * v_rot
        dv = (v_rot * alpha_q + a_half) >> alpha_frac
        v_new = v_rot - dv
        x = lo if x_new < lo else (hi if x_new > hi else x_new)
        v = lo if v_new < lo else (hi if v_new > hi else v_new)

    amp_final = math.hypot(state_fmt.to_float(x), state_fmt.to_float(v))
    amp_ideal = math.exp(-w0 * duration_s / (2.0 * q_factor))

    if 0.0 < amp_final < 1.0:
        q_meas = -w0 * duration_s / (2.0 * math.log(amp_final))
        q_err_pct = 100.0 * (q_meas - q_factor) / q_factor
    else:
        q_meas = float("inf") if amp_final >= 1.0 else 0.0
        q_err_pct = float("inf")

    return {
        "state_bits": float(state_bits),
        "amp_final": amp_final,
        "amp_ideal": amp_ideal,
        "q_measured": q_meas,
        "q_err_pct": q_err_pct,
        "alpha_q": float(alpha_q),
    }


def run_bitwidth_study(
    f0_hz: float = 11_000.0,
    fs_hz: float = 1_000_000.0,
    duration_s: float = 0.05,
) -> None:
    """Print fixed-point accuracy table for DRG (Q=1e5) and HRG (Q=1e6)."""
    for label, q_val in [("DRG (Q=100k)", 1e5), ("HRG (Q=1M)", 1e6)]:
        print(f"\n=== Bit-Width Study: {label}, f0={f0_hz/1e3:.1f} kHz, fs={fs_hz/1e6:.2f} MHz, T={duration_s*1e3:.0f} ms ===")
        print(f"{'State Bits':>10} | {'alpha_q':>10} | {'Amp Final':>12} | {'Amp Ideal':>12} | {'Q Measured':>12} | {'Q Err (%)':>10}")
        print("-" * 80)
        for bits in (18, 24, 32, 40, 48):
            res = strang_resonator_ringdown_fixed(
                f0_hz=f0_hz,
                q_factor=q_val,
                fs_hz=fs_hz,
                duration_s=duration_s,
                state_bits=bits,
                coeff_bits=min(bits, 36),
            )
            print(
                f"{bits:>10d} | {int(res['alpha_q']):>10d} | "
                f"{res['amp_final']:>12.8f} | {res['amp_ideal']:>12.8f} | "
                f"{res['q_measured']:>12.1f} | {res['q_err_pct']:>10.3f}"
            )


if __name__ == "__main__":
    run_bitwidth_study()
