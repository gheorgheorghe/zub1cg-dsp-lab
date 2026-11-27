#!/usr/bin/env python3
"""Generate dark-theme Overlapping Allan Deviation (ADEV) comparison across gyro classes.

Compares 5 calibrated gyro architectures across 4+ decades of averaging time tau:
  1. Consumer MEMS IMU (ref)                (ARW = 0.30   deg/rt-hr, BI ~ 7.5   deg/hr)
  2. Silicon MEMS DRG (11 kHz, Q=100k)      (ARW = 0.015  deg/rt-hr, BI ~ 0.15  deg/hr)
  3. Quartz MEMS DRG  (12 kHz, Q=800k)      (ARW = 0.004  deg/rt-hr, BI ~ 0.035 deg/hr)
  4. Fused-Silica Micro-HRG (11 kHz, Q=1M)  (ARW = 0.002  deg/rt-hr, BI ~ 0.012 deg/hr)
  5. Macro Space HRG  (4.2 kHz, Q=5M)       (ARW = 0.0002 deg/rt-hr, BI ~ 0.0012 deg/hr)
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model.mems_gyro_model import allan_deviation


def synthesize_pink_noise(rng: np.random.Generator, n: int) -> np.ndarray:
    """5-stage cascaded 1/f flicker generator spanning 4+ decades (unit RMS)."""
    w = rng.standard_normal(n)
    out = np.zeros(n)
    poles = [0.9997, 0.997, 0.985, 0.92, 0.70]
    gains = [0.18, 0.22, 0.26, 0.30, 0.35]
    for p, g in zip(poles, gains):
        y = 0.0
        buf = np.empty(n)
        a = 1.0 - p
        for i in range(n):
            y = p * y + a * w[i]
            buf[i] = y
        out += g * buf / max(float(np.std(buf)), 1e-12)
    return out / max(float(np.std(out)), 1e-12)


def generate_adev_curve(
    arw_deg_rt_hr: float,
    bi_dph: float,
    rrw_dph_rt_hr: float,
    fs_hz: float = 20.0,
    dur_s: float = 4000.0,
    seed: int = 100,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Synthesize realistic ARW + 1/f BI + RRW rate stream and compute overlapping ADEV."""
    rng = np.random.default_rng(seed)
    n = int(dur_s * fs_hz)

    # White noise -> ARW (-1/2 slope): sigma_white(dps) = (ARW / 60) * sqrt(fs)
    arw_dps_rt_s = arw_deg_rt_hr / 60.0
    white_dps = arw_dps_rt_s * math.sqrt(fs_hz) * rng.standard_normal(n)

    # 1/f flicker noise -> Bias Instability (0 slope, ADEV floor ~ 0.664 * BI)
    pink_dps = (bi_dph / 3600.0) * 1.9 * synthesize_pink_noise(rng, n)

    # Integrated white noise -> Rate Random Walk (+1/2 slope: ADEV(tau) = K * sqrt(tau/3))
    # With K in deg/hr/sqrt(hr) -> K_dps_per_rt_s = (rrw_dph_rt_hr / 3600) / 60
    rrw_dps_per_rt_s = (rrw_dph_rt_hr / 3600.0) / 60.0
    rrw_dps = np.cumsum(rrw_dps_per_rt_s * math.sqrt(1.0 / fs_hz) * rng.standard_normal(n))

    rate_dps = white_dps + pink_dps + rrw_dps
    taus, adevs_dph = allan_deviation(rate_dps, fs_hz)
    idx_min = int(np.argmin(adevs_dph))
    bi_meas = float(adevs_dph[idx_min] / 0.664)
    return taus, adevs_dph, bi_meas, float(taus[idx_min])


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    specs = [
        {
            "label": "Consumer MEMS IMU (ref)",
            "arw": 0.30,
            "bi": 7.5,
            "rrw": 38.0,
            "color": "#f87171",
            "seed": 101,
        },
        {
            "label": "Silicon MEMS DRG (11 kHz, Q=100k)",
            "arw": 0.015,
            "bi": 0.15,
            "rrw": 0.85,
            "color": "#fbbf24",
            "seed": 102,
        },
        {
            "label": "Quartz MEMS DRG (12 kHz, Q=800k)",
            "arw": 0.004,
            "bi": 0.035,
            "rrw": 0.20,
            "color": "#c084fc",
            "seed": 103,
        },
        {
            "label": "Fused-Silica Micro-HRG (11 kHz, Q=1M)",
            "arw": 0.002,
            "bi": 0.012,
            "rrw": 0.075,
            "color": "#38bdf8",
            "seed": 104,
        },
        {
            "label": "Macro Space HRG (4.2 kHz, Q=5M)",
            "arw": 0.0002,
            "bi": 0.0012,
            "rrw": 0.0075,
            "color": "#34d399",
            "seed": 105,
        },
    ]

    fig, ax = plt.subplots(figsize=(11.2, 7.2), facecolor="#0b0f17")
    ax.set_facecolor("#0f172a")

    for sp in specs:
        taus, adevs, bi_est, tau_min = generate_adev_curve(
            sp["arw"], sp["bi"], sp["rrw"], seed=sp["seed"]
        )
        lbl = f"{sp['label']}  (ARW={sp['arw']:g} °/√hr, BI≈{bi_est:.4g} °/hr)"
        lw = 3.2 if "Quartz" in sp["label"] else 2.8
        ax.loglog(taus, adevs, lw=lw, color=sp["color"], label=lbl, zorder=4)
        idx_min = int(np.argmin(adevs))
        ax.scatter(
            [taus[idx_min]], [adevs[idx_min]],
            s=52, facecolor=sp["color"], edgecolor="#0b0f17", linewidth=1.6, zorder=6,
        )
        print(f"{sp['label']:40s} | min ADEV = {adevs[idx_min]:.4g} °/hr (BI≈{bi_est:.4g} °/hr) @ tau={tau_min:.0f}s")

    # Reference grade bands
    for y_val, txt in [(10.0, "Consumer (~10 °/hr)"), (0.1, "Tactical (~0.1 °/hr)"), (0.01, "Nav-Grade (~0.01 °/hr)")]:
        ax.axhline(y_val, color="#475569", ls=":", lw=1.0, zorder=1)
        ax.text(0.055, y_val * 1.18, txt, color="#94a3b8", fontsize=9.5, va="bottom")

    # Slope -1/2 (ARW) and +1/2 (RRW) visual guides
    t_arw = np.array([0.08, 2.5])
    ax.loglog(t_arw, 2.9 / np.sqrt(t_arw), "--", color="#64748b", lw=1.2, zorder=2)
    ax.text(0.18, 9.0, "Slope −1/2 (ARW)", color="#94a3b8", fontsize=10, rotation=-24)

    t_rrw = np.array([150.0, 950.0])
    ax.loglog(t_rrw, 0.00058 * np.sqrt(t_rrw), "--", color="#64748b", lw=1.2, zorder=2)
    ax.text(260.0, 0.0048, "Slope +1/2 (RRW)", color="#94a3b8", fontsize=10, rotation=22)

    ax.set_xlim(0.05, 1000.0)
    ax.set_ylim(3e-4, 220.0)
    ax.set_xlabel("Cluster Averaging Time τ (seconds)", color="#e6edf7", fontsize=12, fontweight="bold")
    ax.set_ylabel("Overlapping Allan Deviation σ(τ)  [°/hr]", color="#e6edf7", fontsize=12, fontweight="bold")
    ax.set_title(
        "Gyro Allan Deviation (ADEV) Comparison — Silicon vs. Quartz DRG vs. HRG",
        color="#ffffff", fontsize=13.5, fontweight="bold", pad=12,
    )
    ax.tick_params(colors="#cbd5e1", which="both")
    for spine in ax.spines.values():
        spine.set_color("#334155")
    ax.grid(True, which="major", color="#1e293b", lw=0.9)
    ax.grid(True, which="minor", color="#1e293b", lw=0.45, alpha=0.7)

    leg = ax.legend(loc="lower left", frameon=True, facecolor="#0b1220", edgecolor="#334155", fontsize=9.3)
    for txt in leg.get_texts():
        txt.set_color("#e6edf7")

    plt.tight_layout()
    for out_path in [root / "studies" / "adev_presets_comparison.png", root / "model" / "plots" / "adev_presets_comparison.png"]:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
        print(f"Saved -> {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
