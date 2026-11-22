"""Pytest verification suite for Stage 1 DRG & HRG golden models and fixed-point scaling."""

import numpy as np
import pytest
from model.fixed_point import strang_resonator_ringdown_fixed
from model.mems_gyro_model import PRESETS, simulate_cvg_closed_loop, allan_deviation


@pytest.mark.parametrize(
    "q_val,min_bits",
    [
        (1e5, 40),  # DRG Q=100k at 11 kHz, 1 MHz tick -> < 0.5% Q error at 40/48 bits
        (1e6, 40),  # HRG Q=1M   at 11 kHz, 1 MHz tick -> < 0.5% Q error at 40/48 bits
    ],
)
def test_fixed_point_high_q_accuracy(q_val: float, min_bits: int):
    """Verify 40-bit and 48-bit Strang-split integrators preserve DRG/HRG Q accurately."""
    res48 = strang_resonator_ringdown_fixed(
        f0_hz=11_000.0,
        q_factor=q_val,
        fs_hz=1_000_000.0,
        duration_s=0.05,
        state_bits=48,
        coeff_bits=40,
    )
    assert abs(res48["q_err_pct"]) < 0.05, f"48-bit Q error too large: {res48}"

    res_min = strang_resonator_ringdown_fixed(
        f0_hz=11_000.0,
        q_factor=q_val,
        fs_hz=1_000_000.0,
        duration_s=0.05,
        state_bits=min_bits,
        coeff_bits=36,
    )
    assert abs(res_min["q_err_pct"]) < 0.5, f"{min_bits}-bit Q error too large: {res_min}"


@pytest.mark.parametrize("preset_key", ["drg_11khz_q100k", "hrg_micro_11khz_q1m", "hrg_space_4khz_q5m"])
def test_closed_loop_ftr_pll_agc_quad(preset_key: str):
    """Verify PLL lock under drift, AGC regulation, quad nulling, and FTR rate accuracy."""
    params = PRESETS[preset_key]
    target_rate = 100.0

    res = simulate_cvg_closed_loop(
        params=params,
        duration_s=0.35,
        fs_hz=200_000.0,
        omega_fn=lambda t: target_rate if t >= 0.15 else 0.0,
        df_drift_fn=lambda t: 10.0 * (t / 0.35),
        enable_noise=False,
    )

    m = (res.t_s >= 0.25) & (res.t_s <= 0.34)
    # 1. Drive PLL tracks thermal drift within < 0.2 Hz
    pll_err_hz = float(np.mean(np.abs(res.freq_pll_hz[m] - res.freq_true_hz[m])))
    assert pll_err_hz < 0.2, f"{preset_key}: PLL error {pll_err_hz:.3f} Hz"

    # 2. Drive AGC holds normalized amplitude within 0.5% of 1.0
    amp_err = float(np.mean(np.abs(res.drive_amp[m] - 1.0)))
    assert amp_err < 0.005, f"{preset_key}: AGC amplitude error {amp_err:.4f}"

    # 3. FTR tracks 100 deg/s step within 0.5% despite large quadrature coupling
    est_rate = float(np.mean(res.omega_est_dps[m]))
    assert abs(est_rate - target_rate) < 0.5, f"{preset_key}: FTR est={est_rate:.3f} dps"

    # 4. Quadrature nulling recovers fabrication quadrature coupling within 1%
    est_quad = float(np.mean(res.quad_cmd_dps[m]))
    assert abs(est_quad - params.quad_equiv_dps) < max(0.5, 0.02 * params.quad_equiv_dps), (
        f"{preset_key}: Quad est={est_quad:.3f} vs true={params.quad_equiv_dps:.3f}"
    )

    # 5. Sense displacement is actively nulled (|y| < 1e-3 of drive amplitude)
    assert float(np.mean(res.sense_amp[m])) < 1e-3


def test_allan_deviation_helper():
    rng = np.random.default_rng(0)
    # Pure white rate noise with sigma = 0.1 deg/s at 100 Hz -> ADEV(1s) ~ 0.01 * 3600 = 36 deg/hr
    samples = rng.normal(0.0, 0.1, size=20_000)
    taus, adevs = allan_deviation(samples, fs_hz=100.0)
    assert len(taus) > 10
    idx_1s = int(np.argmin(np.abs(taus - 1.0)))
    assert 25.0 < adevs[idx_1s] < 48.0
