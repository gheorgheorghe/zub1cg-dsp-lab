#!/usr/bin/env python3
"""Gyro Ringdown, Q-Factor Fitting & 100V +/-20V Electrostatic Tuning Demo.

Demonstrates realistic DRG bench physics:
  1. Bipolar electrostatic frequency & quadrature tuning at V_bias = 100 V,
     V_tune in [-20 V, +20 V]:
       f_eff(V_t) = sqrt(f_mech^2 - 2 * f_mech * alpha_t * (V_bias - V_t)^2)
     PLUS the hidden Ohmic tuning-resistor damping tax:
       Q_elec^-1 = (R_series * C0^2 * (V_bias - V_t)^2) / (m_eff * w0 * d0^2)
  2. Free 2-DOF coupled ringdown before vs. after mode-matching:
     Untrimmed (df != 0, k_xy != 0) shows strong energy-sloshing beating;
     trimmed (df -> 0, k_xy -> 0) collapses to a pure exponential A0 * exp(-t/tau).
  3. Real-world Semi-Log Q-Fit & Noise-Floor (Rician) Bias Trap:
     Shows how fitting too far into the electronic noise floor artificially
     inflates Q_fit, while fitting strictly in the [-3 dB, -25 dB] linear
     region recovers Q_true = 100,000 to < 0.2% error!

Generates: studies/ringdown_qfit_and_tuning.png
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def simulate_ringdown_envelope(
    f0_hz: float,
    q_x: float,
    q_y: float,
    df_hz: float,
    k_xy_hz: float,
    duration_s: float,
    fs_env_hz: float = 500.0,
    noise_rms: float = 1.2e-3,
    seed: int = 42,
) -> dict[str, np.ndarray]:
    """Exact slow-envelope 2-DOF coupled complex-amplitude free ringdown.

    In rotating frame at nominal w0, complex modal envelopes (a_x, a_y) obey:
      d(a_x)/dt = (-g_x/2) * a_x - j*(dw_x/2)*a_x - j*(w_xy/2)*a_y
      d(a_y)/dt = (-g_y/2) * a_y - j*(dw_y/2)*a_y - j*(w_xy/2)*a_x
    where dw_x = -pi*df, dw_y = +pi*df, w_xy = 2*pi*k_xy_hz.
    Uses exact matrix exponential (via 2x2 eigendecomposition) for machine precision!
    """
    t = np.arange(0.0, duration_s, 1.0 / fs_env_hz)
    w0 = 2.0 * math.pi * f0_hz
    gx = w0 / q_x
    gy = w0 / q_y
    dw = 2.0 * math.pi * df_hz
    wxy = 2.0 * math.pi * k_xy_hz

    A_mat = np.array([
        [-0.5 * gx + 0.5j * dw, -0.5j * wxy],
        [-0.5j * wxy, -0.5 * gy - 0.5j * dw],
    ], dtype=np.complex128)

    eigvals, eigvecs = np.linalg.eig(A_mat)
    # Initial state: excited along X (drive) axis: [1.0, 0.0]
    c0 = np.linalg.solve(eigvecs, np.array([1.0 + 0.0j, 0.0 + 0.0j]))

    exp_t = np.exp(np.outer(t, eigvals))  # (N, 2)
    states = (exp_t * c0[None, :]) @ eigvecs.T
    ax_clean = states[:, 0]
    ay_clean = states[:, 1]

    rng = np.random.default_rng(seed)
    # Complex I/Q white electronic readout noise -> Rician envelope near floor!
    n_x = (rng.normal(0.0, noise_rms, len(t)) + 1j * rng.normal(0.0, noise_rms, len(t))) / math.sqrt(2.0)
    n_y = (rng.normal(0.0, noise_rms, len(t)) + 1j * rng.normal(0.0, noise_rms, len(t))) / math.sqrt(2.0)

    env_x_clean = np.abs(ax_clean)
    env_x_meas = np.abs(ax_clean + n_x)
    env_y_meas = np.abs(ay_clean + n_y)

    return {
        "t": t,
        "env_x_clean": env_x_clean,
        "env_x_meas": env_x_meas,
        "env_y_meas": env_y_meas,
    }


def fit_q_semilog(t: np.ndarray, env: np.ndarray, f0_hz: float, t_min: float, t_max: float) -> dict[str, float]:
    """Fit ln(A(t)) = a - t/tau over [t_min, t_max] and return tau, Q, R^2."""
    mask = (t >= t_min) & (t <= t_max) & (env > 1e-12)
    tm = t[mask]
    ym = np.log(env[mask])
    slope, intercept = np.polyfit(tm, ym, 1)
    y_hat = slope * tm + intercept
    ss_res = float(np.sum((ym - y_hat) ** 2))
    ss_tot = float(np.sum((ym - np.mean(ym)) ** 2))
    r2 = 1.0 - ss_res / max(ss_tot, 1e-18)
    tau_fit = -1.0 / slope
    q_fit = math.pi * f0_hz * tau_fit
    return {"tau": tau_fit, "q": q_fit, "r2": r2, "intercept": intercept, "slope": slope}


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
        "font.size": 9.3,
    })

    fig, axes = plt.subplots(2, 2, figsize=(13.6, 9.5), dpi=160)
    fig.suptitle(
        "DRG Bench Physics: 100 V ± 20 V Electrostatic Tuning, Free Ringdown & Semi-Log Q-Fit",
        fontsize=13.5, fontweight="bold", color="#f8fafc", y=0.975,
    )

    f0_nom = 11_000.0
    q_true = 100_000.0
    tau_true = q_true / (math.pi * f0_nom)  # 2.8937 s

    # -----------------------------------------------------------------------
    # Panel A: 100 V Bias + +/-20 V Bipolar Electrostatic Mode-Matching
    # -----------------------------------------------------------------------
    ax = axes[0, 0]
    v_bias = 100.0
    v_tune = np.linspace(-20.0, 20.0, 401)
    # Realistic high-gap (d0 = 6 um) 100V DRG tuning coefficient:
    # At V_bias = 100 V, (V_bias - V_tune)^2 spans 6400..14400 V^2 around 10000 V^2!
    # Say alpha_t = 3.2e-3 Hz / V^2 -> +/-20V gives ~ +11.5 Hz to -14.1 Hz (25.6 Hz total span!)
    alpha_t = 3.2e-3
    f_mech_x = 11_032.0
    f_mech_y = 11_039.5  # As-fabricated +7.5 Hz mode split
    f_x_nom = math.sqrt(f_mech_x ** 2 - 2.0 * f_mech_x * alpha_t * (v_bias ** 2))
    f_y_vt = np.sqrt(f_mech_y ** 2 - 2.0 * f_mech_y * alpha_t * ((v_bias - v_tune) ** 2))
    # Solve exact V_tune where f_y_vt == f_x_nom
    v_eff_req = math.sqrt((f_mech_y ** 2 - f_x_nom ** 2) / (2.0 * f_mech_y * alpha_t))
    v_match = v_bias - v_eff_req  # negative V_tune increases (V_bias - V_t)^2 -> softens Y down to X!

    ax.plot(v_tune, np.full_like(v_tune, f_x_nom) - 11_000.0, "--", color="#38bdf8", lw=2.0,
            label="Drive Mode f_x (V_tx = 0 V, fixed)")
    ax.plot(v_tune, f_y_vt - 11_000.0, color="#34d399", lw=2.6,
            label="Sense Mode f_y (V_bias = 100 V, V_ty ∈ [−20, +20] V)")
    ax.axvline(v_match, color="#fbbf24", ls=":", lw=1.6)
    ax.scatter([v_match], [f_x_nom - 11_000.0], color="#fbbf24", edgecolor="#fff", s=75, zorder=6)

    ax.annotate(
        f"Mode-Matched (Δf = 0 Hz)!\nV_tune = {v_match:.2f} V\n(V_eff = {v_eff_req:.1f} V)",
        xy=(v_match, f_x_nom - 11_000.0), xytext=(-19.0, -9.5),
        arrowprops=dict(arrowstyle="->", color="#fbbf24", lw=1.4),
        fontsize=8.1, fontweight="bold", color="#fde68a",
        bbox=dict(boxstyle="round,pad=0.32", fc="#241c0d", ec="#fbbf24", lw=0.9),
    )
    ax.annotate(
        "Bipolar ±20 V on 100 V bias:\n(100−Vt)² spans 6.4k–14.4k V²\n→ pulls f_y BOTH up & down!",
        xy=(14.0, float(f_y_vt[-61] - 11_000.0)), xytext=(-4.5, 11.2),
        arrowprops=dict(arrowstyle="->", color="#34d399", lw=1.2),
        fontsize=7.7, color="#d1fae5",
        bbox=dict(boxstyle="round,pad=0.28", fc="#0f241f", ec="#34d399", lw=0.85),
    )

    ax.set_title("A. 100 V Bias + ±20 V Bipolar Electrostatic Tuning", fontweight="bold", pad=8)
    ax.set_xlabel("Sense Tuning Electrode Voltage V_tune (V)")
    ax.set_ylabel("Mode Frequency Offset f − 11,000 (Hz)")
    ax.set_xlim(-20.5, 20.5)
    ax.set_ylim(-15.5, 22.0)
    ax.grid(True)
    ax.legend(loc="lower right", fontsize=7.4, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel B: Untrimmed (Beating!) vs. Trimmed Free Ringdown (Linear Scale)
    # -----------------------------------------------------------------------
    ax = axes[0, 1]
    rd_untrim = simulate_ringdown_envelope(
        f0_nom, q_true, 95_000.0, df_hz=0.85, k_xy_hz=0.95, duration_s=12.0, noise_rms=1.5e-3, seed=7
    )
    rd_trim = simulate_ringdown_envelope(
        f0_nom, q_true, q_true, df_hz=0.0, k_xy_hz=0.0, duration_s=12.0, noise_rms=1.5e-3, seed=7
    )

    ax.plot(rd_untrim["t"], rd_untrim["env_x_meas"], color="#f87171", lw=1.8, alpha=0.92,
            label="Untrimmed (Δf=0.85 Hz, k_xy=0.95 Hz): Energy Beating!")
    ax.plot(rd_untrim["t"], rd_untrim["env_y_meas"], color="#fbbf24", lw=1.4, ls="--", alpha=0.82,
            label="Untrimmed Sense Axis Y (sloshing out of phase)")
    ax.plot(rd_trim["t"], rd_trim["env_x_meas"], color="#34d399", lw=2.3,
            label=f"Trimmed Mode-Matched (Δf=0): Pure exp(−t/{tau_true:.2f} s)")

    ax.annotate(
        "Coupled eigenmodes beat at\nf_beat = √(Δf² + f_quad²) = 1.27 Hz\n→ corrupts naive Q-fit!",
        xy=(1.55, float(rd_untrim["env_x_meas"][int(1.55 * 500)])), xytext=(4.2, 0.56),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.3),
        fontsize=8.0, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.3", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("B. Free Ringdown Envelope: Untrimmed Beating vs. Tuned Decay", fontweight="bold", pad=8)
    ax.set_xlabel("Ringdown Time t (s)")
    ax.set_ylabel("Normalized Envelope Amplitude |a(t)|")
    ax.set_xlim(0, 12.0)
    ax.set_ylim(-0.02, 1.08)
    ax.grid(True)
    ax.legend(loc="upper right", fontsize=7.4, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel C: Semi-Log Q-Fit & The Rician Noise-Floor Inflation Trap!
    # -----------------------------------------------------------------------
    ax = axes[1, 0]
    rd_q = simulate_ringdown_envelope(
        f0_nom, q_true, q_true, df_hz=0.0, k_xy_hz=0.0, duration_s=22.0, noise_rms=4.0e-3, seed=11
    )
    t = rd_q["t"]
    env_db = 20.0 * np.log10(np.maximum(rd_q["env_x_meas"], 1e-6))

    # Good fit window: -3 dB to -25 dB (~1.0 s to 8.3 s)
    fit_good = fit_q_semilog(t, rd_q["env_x_meas"], f0_nom, t_min=0.8, t_max=8.2)
    # Bad naive full-window fit: 0 s to 22 s (includes Rician noise floor!)
    fit_bad = fit_q_semilog(t, rd_q["env_x_meas"], f0_nom, t_min=0.0, t_max=22.0)

    ax.axvspan(0.8, 8.2, color="#34d399", alpha=0.12, label="Clean Fit Window (−3 to −25 dB)")
    ax.axhline(20.0 * math.log10(4.0e-3 * 0.886), color="#94a3b8", ls=":", lw=1.3,
               label="Electronic Noise Floor (~ −49 dB)")

    ax.plot(t, env_db, color="#38bdf8", lw=1.5, alpha=0.9, label="Measured Ringdown 20 log₁₀|a(t)|")
    ax.plot(t, 20.0 / math.log(10.0) * (fit_good["intercept"] + fit_good["slope"] * t),
            color="#34d399", lw=2.3, ls="--",
            label=f"Windowed Q-Fit: Q = {fit_good['q']:,.0f} ({(fit_good['q']/q_true-1)*100:+.2f}%, R²={fit_good['r2']:.5f})")
    ax.plot(t, 20.0 / math.log(10.0) * (fit_bad["intercept"] + fit_bad["slope"] * t),
            color="#f87171", lw=2.1, ls="-.",
            label=f"Full-Window Fit (Trap!): Q = {fit_bad['q']:,.0f} ({(fit_bad['q']/q_true-1)*100:+.1f}%!)")

    ax.annotate(
        "Rician noise-floor bend!\nFitting into the floor inflates\nmeasured Q by +18–50%!",
        xy=(16.5, -48.5), xytext=(9.8, -22.0),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.3),
        fontsize=8.0, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.3", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("C. Semi-Log Q-Fit: Clean Window vs. Noise-Floor Bias Trap", fontweight="bold", pad=8)
    ax.set_xlabel("Ringdown Time t (s)")
    ax.set_ylabel("Envelope Level 20 log₁₀(A / A₀)  (dB)")
    ax.set_xlim(0, 22.0)
    ax.set_ylim(-60, 4)
    ax.grid(True)
    ax.legend(loc="lower left", fontsize=7.2, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel D: Hidden 100 V Trap — Ohmic Tuning-Resistor Q-Loading vs. R_series
    # -----------------------------------------------------------------------
    ax = axes[1, 1]
    r_ser = np.logspace(2, 6.3, 250)  # 100 Ohm to 2 MOhm
    # Ohmic electrical damping: Q_elec^-1 = (R_ser * eta^2) / (m_eff * w0)
    # Realistic 100V wide-gap tuning electrode: C0 = 0.45 pF, d0 = 10 um, m_eff = 2 mg
    w0 = 2.0 * math.pi * f0_nom
    m_eff = 2.0e-6
    c0_t = 0.45e-12
    d0_t = 10.0e-6
    q_mech_100k = 100_000.0
    q_mech_800k = 800_000.0

    for v_eff, col, lbl_v in [(120.0, "#f87171", "V_eff = 120 V (V_t = −20 V)"),
                              (100.0, "#fbbf24", "V_eff = 100 V (V_t = 0 V)"),
                              (80.0,  "#38bdf8", "V_eff = 80 V  (V_t = +20 V)")]:
        eta = v_eff * c0_t / d0_t
        q_elec_inv = (r_ser * (eta ** 2)) / (m_eff * w0)
        q_net_800 = 1.0 / (1.0 / q_mech_800k + q_elec_inv)
        ax.loglog(r_ser / 1e3, q_net_800 / 1e3, color=col, lw=2.3, label=f"Q_mech=800k, {lbl_v}")

    # Also show Q_mech = 100k at V_eff = 120V
    eta120 = 120.0 * c0_t / d0_t
    q_net_100 = 1.0 / (1.0 / q_mech_100k + (r_ser * eta120 ** 2) / (m_eff * w0))
    ax.loglog(r_ser / 1e3, q_net_100 / 1e3, color="#34d399", lw=2.1, ls="--",
              label="Q_mech=100k, V_eff = 120 V")

    r_mark = 100e3
    q_mark = 1.0 / (1.0 / q_mech_800k + (r_mark * eta120 ** 2) / (m_eff * w0)) / 1e3
    ax.annotate(
        "Unbypassed 100 kΩ–1 MΩ\nHV protection resistor at 100 V\nkills Q AND creates asymmetric\nΔ(1/τ) anisodamping bias!",
        xy=(100.0, q_mark), xytext=(0.45, 195.0),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.4),
        fontsize=7.9, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.3", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("D. Hidden 100 V Trap: Tuning-Resistor Ohmic Q-Loading", fontweight="bold", pad=8)
    ax.set_xlabel("Unbypassed Tuning-Pin Series Resistance R_series (kΩ, log)")
    ax.set_ylabel("Measured Loaded Q (thousands, log)")
    ax.set_xlim(0.1, 2000)
    ax.set_ylim(20, 1100)
    ax.grid(True, which="both")
    ax.legend(loc="lower left", fontsize=7.2, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.945))
    out_path = Path(__file__).resolve().parent.parent / "studies" / "ringdown_qfit_and_tuning.png"
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)

    print(f"Saved: {out_path}")
    print(f"  100V +/-20V Mode-Match voltage: V_tune = {v_match:.2f} V (V_eff = {v_eff_req:.1f} V)")
    print(f"  Windowed Q-fit (-3..-25 dB):    Q = {fit_good['q']:,.0f} ({(fit_good['q']/q_true-1)*100:+.3f}%, R2={fit_good['r2']:.6f})")
    print(f"  Full-window Q-fit (trap!):      Q = {fit_bad['q']:,.0f} ({(fit_bad['q']/q_true-1)*100:+.1f}%)")


if __name__ == "__main__":
    main()
