#!/usr/bin/env python3
"""Differential Electrode Pairs vs. Single-Ended Transduction Study (MEMS DRG/HRG).

Models exact nonlinear parallel-plate capacitive actuation & pickoff:
  d_+(x) = d0*(1 + delta) - x,    d_-(x) = d0*(1 - delta) + x
  C_+(x) = eps0*A / d_+(x),       C_-(x) = eps0*A / d_-(x)
  F_+(x,v) = +0.5*eps0*A*(Vp + v)^2 / d_+(x)^2
  F_-(x,v) = -0.5*eps0*A*(Vp - v)^2 / d_-(x)^2

Compares:
  1. Single-Ended (only '+' electrode)
  2. Real Mismatched Differential (delta = 2.0% DRIE gap mismatch)
  3. Ideal Differential Push-Pull (delta = 0%, matching our modal HIL plant)

Generates: studies/differential_electrode_pairs.png
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def norm_cap(u: np.ndarray, delta: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Return normalized C_+(u)/C0 and C_-(u)/C0 where u = x / d0."""
    c_p = 1.0 / (1.0 + delta - u)
    c_m = 1.0 / (1.0 - delta + u)
    return c_p, c_m


def norm_force(u: np.ndarray, v_ratio: np.ndarray | float, delta: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Return normalized F_+ and F_- (divided by 0.5*C0*Vp^2/d0), v_ratio = v_ac / Vp."""
    f_p = ((1.0 + v_ratio) ** 2) / ((1.0 + delta - u) ** 2)
    f_m = -((1.0 - v_ratio) ** 2) / ((1.0 - delta + u) ** 2)
    return f_p, f_m


def harmonic_dbc(sig: np.ndarray, n_cycles: int) -> dict[int, float]:
    """Compute exact harmonic magnitudes (dBc relative to fundamental f0)."""
    spec = np.abs(np.fft.rfft(sig)) / (len(sig) / 2.0)
    spec[0] /= 2.0  # DC
    fund = max(spec[n_cycles], 1e-18)
    out = {}
    for h in (0, 1, 2, 3, 4):
        idx = h * n_cycles
        val = max(spec[idx], 1e-18)
        out[h] = 20.0 * math.log10(val / fund)
    return out


def main() -> None:
    plt.rcParams.update({
        "figure.facecolor": "#0b0f17",
        "axes.facecolor": "#111827",
        "axes.edgecolor": "#334155",
        "axes.labelcolor": "#e2e8f0",
        "xtick.color": "#94a3b8",
        "ytick.color": "#94a3b8",
        "text.color": "#f1f5f9",
        "grid.color": "#1e293b",
        "grid.linestyle": "--",
        "grid.linewidth": 0.7,
        "font.size": 9.4,
    })

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.4), dpi=160)
    fig.suptitle(
        "Differential Push-Pull Electrode Pairs vs. Single-Ended Transduction (MEMS DRG / HRG)",
        fontsize=13.5, fontweight="bold", color="#f8fafc", y=0.975,
    )

    # -----------------------------------------------------------------------
    # Panel A: Nonlinear Pickoff Transfer Curve C(x) & Actuation F(v)
    # -----------------------------------------------------------------------
    ax = axes[0, 0]
    u = np.linspace(-0.35, 0.35, 401)
    c_p_ideal, c_m_ideal = norm_cap(u, delta=0.0)
    c_p_mis, c_m_mis = norm_cap(u, delta=0.02)

    # Scale each curve to unit slope at u=0 for apples-to-apples linearity comparison
    y_linear = u
    y_se = (c_p_ideal - 1.0)             # slope 1 at u=0: u + u^2 + u^3
    y_diff_mis = 0.5 * ((c_p_mis - c_m_mis) - (c_p_mis[200] - c_m_mis[200]))
    y_diff_ideal = 0.5 * (c_p_ideal - c_m_ideal)  # slope 1 at u=0: u + u^3 (ZERO u^2!)

    ax.plot(u * 100, y_linear, "--", color="#94a3b8", lw=1.4, label="Ideal Linear (Current Modal Sim)")
    ax.plot(u * 100, y_se, color="#f87171", lw=2.3, label="Single-Ended C+(x): Strong +x² Asymmetry")
    ax.plot(u * 100, y_diff_mis, color="#fbbf24", lw=2.0, ls="-.", label="Differential (δ = 2% gap mismatch)")
    ax.plot(u * 100, y_diff_ideal, color="#34d399", lw=2.5, label="Ideal Differential (C+ − C−)/2: Zero x²!")

    # Annotate asymmetry at +30% vs -30%
    err_se_pos = (y_se[-1] - y_linear[-1]) / y_linear[-1] * 100
    err_diff_pos = (y_diff_ideal[-1] - y_linear[-1]) / y_linear[-1] * 100
    ax.annotate(
        f"At +35% gap:\nSingle-Ended: +{err_se_pos:.0f}% error!\nDifferential: +{err_diff_pos:.0f}% (only x³)",
        xy=(35.0, float(y_se[-1])), xytext=(1.0, 0.36),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.4),
        fontsize=8.2, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.35", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("A. Normalized Pickoff Linearity: Single-Ended vs. Push-Pull", fontweight="bold", pad=8)
    ax.set_xlabel("Normalized Resonator Displacement x / d₀ (%)")
    ax.set_ylabel("Normalized Pickoff Output (slope = 1 at 0)")
    ax.set_xlim(-36, 36)
    ax.set_ylim(-0.45, 0.58)
    ax.grid(True)
    ax.legend(loc="lower right", fontsize=7.6, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel B: Harmonic Distortion Spectrum at x_max / d0 = 0.20 (20% gap)
    # -----------------------------------------------------------------------
    ax = axes[0, 1]
    n_pts = 4096
    n_cyc = 32
    ph = 2.0 * math.pi * n_cyc * np.arange(n_pts) / n_pts
    u_sin = 0.20 * np.sin(ph)

    cp0, cm0 = norm_cap(u_sin, delta=0.0)
    cp2, cm2 = norm_cap(u_sin, delta=0.02)
    h_se = harmonic_dbc(cp0 - 1.0, n_cyc)
    h_mis = harmonic_dbc(cp2 - cm2, n_cyc)
    h_id = harmonic_dbc(cp0 - cm0, n_cyc)

    labels = ["DC\n(Offset)", "f₀\n(Signal)", "2f₀\n(Even x²)", "3f₀\n(Odd x³)", "4f₀\n(Even x⁴)"]
    x_pos = np.arange(len(labels))
    w_bar = 0.25

    floor_db = -110.0
    h_se_bar = [max(h_se[k], floor_db) - floor_db for k in range(5)]
    h_mis_bar = [max(h_mis[k], floor_db) - floor_db for k in range(5)]
    h_id_bar = [max(h_id[k], floor_db) - floor_db for k in range(5)]

    ax.bar(x_pos - w_bar, h_se_bar, bottom=floor_db, width=w_bar, color="#f87171", edgecolor="#0b0f17", label="Single-Ended")
    ax.bar(x_pos, h_mis_bar, bottom=floor_db, width=w_bar, color="#fbbf24", edgecolor="#0b0f17", label="Diff. (δ = 2% mismatch)")
    ax.bar(x_pos + w_bar, h_id_bar, bottom=floor_db, width=w_bar, color="#34d399", edgecolor="#0b0f17", label="Ideal Diff. (δ = 0%)")

    # Annotate 2f0 suppression cleanly above the 2f0/3f0 bars without touching legend
    ax.annotate(
        f"2f₀ crushed!\nSE: {h_se[2]:.1f} dBc\nδ=2%: {h_mis[2]:.1f} dBc\nIdeal: < −110 dBc",
        xy=(2.0, h_mis[2]), xytext=(2.82, -26),
        arrowprops=dict(arrowstyle="->", color="#fbbf24", lw=1.3),
        fontsize=8.0, color="#fde68a",
        bbox=dict(boxstyle="round,pad=0.32", fc="#241c0d", ec="#fbbf24", lw=0.9),
    )

    ax.set_title("B. Pickoff Harmonics at x_max = 20% of Gap (Spectrum, dBc)", fontweight="bold", pad=8)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Harmonic Level (dBc rel. to f₀)")
    ax.set_ylim(-110, 20)
    ax.grid(True, axis="y")
    ax.legend(loc="upper center", ncol=3, fontsize=7.2, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel C: Common-Mode & Aggressor Rejection vs. Gap Mismatch delta (%)
    # -----------------------------------------------------------------------
    ax = axes[1, 0]
    delta_pct = np.logspace(-2, 1.0, 200)  # 0.01% to 10%
    delta = delta_pct / 100.0

    # 1. DC/Vp ripple force rejection: F_err / F_se_err = |(1+d)^-2 - (1-d)^-2| / 1 ~= 4*delta
    rej_vp = -20.0 * np.log10(np.maximum(np.abs((1 + delta) ** -2 - (1 - delta) ** -2) / 2.0, 1e-6))
    # 2. Capacitive feedthrough / n=0 common-mode rejection: |C+ - C-| / (C+ + C-) = delta
    rej_cm = -20.0 * np.log10(np.maximum(delta, 1e-6))
    # 3. 2f0 actuation v_d^2 rejection (force quadratic term): |(1+d)^-2 - (1-d)^-2| / ((1+d)^-2 + (1-d)^-2) ~= 2*delta
    rej_v2 = -20.0 * np.log10(np.maximum(2.0 * delta / (1.0 + delta ** 2), 1e-6))
    # 4. 4-quadrant (0/180 vs 90/270) n=1 linear shock rejection (~2*delta from opposing gap error)
    rej_n1 = -20.0 * np.log10(np.maximum(1.5 * delta, 1e-6))

    ax.semilogx(delta_pct, rej_cm, color="#34d399", lw=2.4, label="Common-Mode C_ft / n=0 Rejection (−20 log δ)")
    ax.semilogx(delta_pct, rej_n1, color="#38bdf8", lw=2.2, label="4-Quad n=1 Linear Shock Rejection")
    ax.semilogx(delta_pct, rej_v2, color="#fbbf24", lw=2.2, ls="--", label="Actuation v_d² (FTR Ω² error) Rejection")
    ax.semilogx(delta_pct, rej_vp, color="#c084fc", lw=2.0, ls=":", label="V_p Supply-Ripple Force Rejection")

    # Shade typical DRIE MEMS fabrication window (0.5% - 2.5%)
    ax.axvspan(0.5, 2.5, color="#38bdf8", alpha=0.14)
    ax.text(1.12, 11.5, "Typical DRIE\nMEMS Etch\nMismatch\n(0.5–2.5%)", ha="center",
            fontsize=7.8, fontweight="bold", color="#bae6fd",
            bbox=dict(boxstyle="round,pad=0.25", fc="#0c1c2e", ec="#38bdf8", lw=0.8))

    ax.set_title("C. Differential Rejection vs. Fabrication Gap Mismatch δ", fontweight="bold", pad=8)
    ax.set_xlabel("Fractional Gap Mismatch δ = Δd₀ / (2 d₀)  (%, log)")
    ax.set_ylabel("Suppression vs. Single-Ended (dB)")
    ax.set_xlim(0.01, 10.0)
    ax.set_ylim(5, 85)
    ax.grid(True, which="both")
    ax.legend(loc="upper right", fontsize=7.5, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel D: Closed-Loop FTR Scale-Factor & Feedthrough Error Across ±500 °/s
    # -----------------------------------------------------------------------
    ax = axes[1, 1]
    omega_dps = np.linspace(-500.0, 500.0, 401)
    # In FTR closed-loop, v_FTR / Vp scales linearly with rate (assume 500 deg/s -> v_FTR/Vp = 0.08)
    v_req = 0.08 * (omega_dps / 500.0)

    # Actual force delivered vs ideal linear command:
    # Single-ended: F_se(v) - F_se(0) = 2*v + v^2 -> relative scale error +0.5*v_req!
    # Plus common-mode capacitive feedthrough bias equivalent to 8 deg/s uncancelled in SE!
    ft_bias_se = 6.0  # deg/s equivalent feedthrough bias in single-ended
    # Single-ended rate error (deg/s): quadratic v^2 term + constant feedthrough bias
    err_se = (0.5 * v_req) * omega_dps + ft_bias_se
    # Mismatched differential (delta = 2%): v^2 attenuated by 2*delta (4%), feedthrough attenuated by delta (2%)
    err_mis = (0.5 * v_req * 0.04) * omega_dps + ft_bias_se * 0.02
    # Ideal differential (delta = 0%, current modal sim): 0 quadratic error, 0 CM feedthrough!
    err_id = np.zeros_like(omega_dps)

    ax.plot(omega_dps, err_se, color="#f87171", lw=2.4, label="Single-Ended: +6 °/s FT Bias + 20 °/s Ω² Bow!")
    ax.plot(omega_dps, err_mis, color="#fbbf24", lw=2.2, label="Diff. (δ = 2% mismatch): 0.12 °/s Bias, 0.8 °/s Ω²")
    ax.plot(omega_dps, err_id, color="#34d399", lw=2.5, ls="--", label="Ideal Diff. (Current Modal Sim): 0.00 °/s Error")

    ax.annotate(
        "v_FTR² term in Single-Ended\ncreates a huge parabolic Ω²\nscale-factor error!",
        xy=(420, float(err_se[-33])), xytext=(-450, 14.5),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.4),
        fontsize=8.2, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.35", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("D. Closed-Loop FTR Rate Error (±500 °/s + Feedthrough)", fontweight="bold", pad=8)
    ax.set_xlabel("True Input Spin Rate Ω (°/s)")
    ax.set_ylabel("FTR Rate Measurement Error (°/s)")
    ax.set_xlim(-500, 500)
    ax.set_ylim(-4, 30)
    ax.grid(True)
    ax.legend(loc="lower left", fontsize=7.5, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.945))
    out_path = Path(__file__).resolve().parent.parent / "studies" / "differential_electrode_pairs.png"
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)

    print(f"Saved: {out_path}")
    print(f"  2f0 @ x=20% gap: SE={h_se[2]:.1f} dBc | Diff(2%)={h_mis[2]:.1f} dBc | Ideal={h_id[2]:.1f} dBc")
    print(f"  3f0 @ x=20% gap: SE={h_se[3]:.1f} dBc | Diff(2%)={h_mis[3]:.1f} dBc | Ideal={h_id[3]:.1f} dBc")


if __name__ == "__main__":
    main()
