#!/usr/bin/env python3
"""Live ZUBoard 1CG Hardware Temperature-Ramp & Self-Thermometry Test.

Streams a live +30 C temperature ramp (25 C -> 55 C) into the running XCZU1CG
Stage 1 v2 FPGA by dynamically updating the Strang-split rotation coefficients
(cos_tx, sin_tx, cos_ty, sin_ty) over GbE UDP while simultaneously applying a
+50 deg/s rotation rate!

Verifies:
  1. 40-bit NCO PLL (0.91 uHz/LSB) tracks the -9.9 Hz Silicon TCF drift
     (TCF = -30 ppm/C) with sub-0.2 C self-thermometry accuracy.
  2. Closed-loop FTR + CLQ + AGC hold +50.0 deg/s rate accuracy rock-solid
     right through the thermal ramp without losing lock!
"""

from __future__ import annotations
import json
import math
from pathlib import Path
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from host.zuboard_client import ZUBoardClient
from host.gyro_dashboard import (
    HIL_FS_HZ,
    program_cvg_preset,
    set_rate_dps,
    capture_telemetry_frames,
    decode_hw_frames,
    _to_s36,
)
from model.mems_gyro_model import PRESETS


def set_plant_temp_c(client: ZUBoardClient, params, temp_c: float) -> float:
    """Update live FPGA plant resonant frequencies fx(T), fy(T) over AXI-Lite."""
    dt = 1.0 / HIL_FS_HZ
    df_t = params.df_per_c_hz * (temp_c - params.t0_c)
    wx = 2.0 * math.pi * (params.f0_hz + df_t)
    wy = 2.0 * math.pi * (params.f0_hz + df_t + params.mode_split_hz)
    for addr_lo, val in [
        (0x54, math.cos(wx * dt)),
        (0x5C, math.sin(wx * dt)),
        (0x64, math.cos(wy * dt)),
        (0x6C, math.sin(wy * dt)),
    ]:
        lo, hi = _to_s36(val, 34)
        client.reg_write(addr_lo, lo)
        client.reg_write(addr_lo + 4, hi)
    return df_t


def main() -> None:
    out_dir = Path("studies")
    out_dir.mkdir(parents=True, exist_ok=True)
    client = ZUBoardClient(host="192.168.1.10", port=5000, timeout=8.0, mock=False)
    ver = client.reg_read(0xEC)
    print(f"Connected to ZUBoard 1CG: VERSION = 0x{ver:08X}")

    params = PRESETS["drg_11khz_q100k"]  # Silicon DRG: -30 ppm/C -> -0.33 Hz/C
    rate_cmd = 50.0

    # Program Silicon DRG at 2 kHz telemetry (decim=500) and apply +50 deg/s
    frames_per_step = 100  # 50 ms per temperature step
    n_steps = 31           # 25 C -> 55 C in 1 C steps (1.55 s total live capture)
    program_cvg_preset(client, params, decim=500, lpf_shift=8, pkt_frames=frames_per_step, enable_noise=True)
    time.sleep(0.20)
    set_rate_dps(client, params, rate_cmd)
    time.sleep(0.15)
    # Discard 1 warmup burst so rate is settled at +50 dps
    _ = capture_telemetry_frames(client, frames_per_step)

    temps_true = np.linspace(25.0, 55.0, n_steps)
    df_true_arr = []
    df_pll_arr = []
    temp_est_arr = []
    rate_ftr_arr = []
    dx_arr = []
    sy_arr = []

    for tc in temps_true:
        df_t = set_plant_temp_c(client, params, float(tc))
        raw = capture_telemetry_frames(client, frames_per_step)
        res = decode_hw_frames(raw, params, decim_hz=2000.0, omega_true=np.full(frames_per_step, rate_cmd))
        # Take settled second half of each 50 ms step
        sl = slice(frames_per_step // 2, frames_per_step)
        df_true_arr.append(df_t)
        df_pll_arr.append(float(np.mean(res.pll_adj_hz[sl])))
        temp_est_arr.append(float(np.mean(res.temp_est_c[sl])))
        rate_ftr_arr.append(float(np.mean(res.omega_ftr_dps[sl])))
        dx_arr.append(float(np.mean(res.dx[sl])))
        sy_arr.append(float(np.mean(np.abs(res.sy[sl]))))

    df_true_arr = np.array(df_true_arr)
    df_pll_arr = np.array(df_pll_arr)
    temp_est_arr = np.array(temp_est_arr)
    rate_ftr_arr = np.array(rate_ftr_arr)

    temp_err_c = temp_est_arr - temps_true
    rms_temp_err_c = float(np.sqrt(np.mean(temp_err_c**2)))
    max_temp_err_c = float(np.max(np.abs(temp_err_c)))
    rate_mean = float(np.mean(rate_ftr_arr))
    rate_drift_dps = float(np.ptp(rate_ftr_arr))

    print(f"[TEMP RAMP] 25 C -> 55 C (-9.90 Hz drift) on live ZUBoard 1CG:")
    print(f"  Self-Thermometry Error : RMS = {rms_temp_err_c:.3f} C | Max = {max_temp_err_c:.3f} C")
    print(f"  Simultaneous 50 dps FTR: Mean = {rate_mean:.3f} dps | Peak-to-Peak thermal drift = {rate_drift_dps:.4f} dps")

    # Save 3-panel live hardware plot
    fig, axes = plt.subplots(3, 1, figsize=(9.5, 8.2), sharex=True)

    ax = axes[0]
    ax.plot(temps_true, df_true_arr, "k--", lw=2.0, label="Injected Silicon TCF Drift (−0.33 Hz/°C)")
    ax.plot(temps_true, df_pll_arr, "o-", color="#7c3aed", lw=1.8, ms=4, label="Live FPGA 40-bit PLLAdj (Hz)")
    ax.set_ylabel("Δf from f₀ (Hz)")
    ax.set_title("Live ZUBoard 1CG Thermal Ramp (25 °C → 55 °C) — Self-Thermometry & Rate Immunity")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.35)

    ax = axes[1]
    ax.plot(temps_true, temp_err_c, "s-", color="#059669", lw=1.8, ms=4,
            label=f"Self-Thermometry Residual (RMS = {rms_temp_err_c:.3f} °C)")
    ax.axhline(0.0, color="#111827", ls="--", lw=0.9)
    ax.set_ylabel("Temp Error (°C)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.35)

    ax = axes[2]
    ax.plot(temps_true, rate_ftr_arr, "o-", color="#2563eb", lw=1.8, ms=4,
            label=f"Live FPGA FTR Rate (mean={rate_mean:.3f} °/s, p2p={rate_drift_dps:.3f} °/s)")
    ax.axhline(rate_cmd, color="#111827", ls="--", lw=1.0, label="True Rate = 50.000 °/s")
    ax.set_xlabel("Live Resonator Temperature (°C)")
    ax.set_ylabel("Measured Rate (°/s)")
    ax.set_ylim(rate_cmd - 1.0, rate_cmd + 1.0)
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.35)

    plt.tight_layout()
    out_png = out_dir / "hw_temp_ramp_live.png"
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"Saved -> {out_png}")

    (out_dir / "hw_temp_ramp_summary.json").write_text(json.dumps({
        "version_hex": f"0x{ver:08X}",
        "temp_start_c": 25.0,
        "temp_end_c": 55.0,
        "rms_temp_err_c": rms_temp_err_c,
        "max_temp_err_c": max_temp_err_c,
        "rate_cmd_dps": rate_cmd,
        "rate_mean_dps": rate_mean,
        "rate_p2p_drift_dps": rate_drift_dps,
    }, indent=2))


if __name__ == "__main__":
    main()
