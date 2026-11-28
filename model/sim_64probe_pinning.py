#!/usr/bin/env python3
"""64-Probe Die ADC Simulation: 2D n=2 Cloverleaf Ring-Up & Fab-Asymmetry Mode Pinning.

Models a 2-DOF Coriolis Vibratory Gyroscope (DRG/HRG) with rotated fabrication
stiffness asymmetry (df_fab at principal angle theta_w) AND rotated damping
asymmetry / anisodamping (delta_Q at principal angle theta_tau), projected onto
N_PROBES = 64 perimeter die probes (phi_k = 2*pi*k/64, every 5.625 deg):

  r_k(t) = x(t) * cos(2*phi_k) + y(t) * sin(2*phi_k)

Demonstrates:
  1. 64-Probe Polar Cloverleaf Snapshots: ring-up from launch angle theta_0 = 5 deg
     wobbling elliptically and rotating/settling onto the fab pinning angle theta_pin = 28 deg!
  2. 64-Channel ADC Waterfall / Heatmap (Probe #0..63 vs. time): directly shows the
     4 bright cloverleaf antinodes marching across the 64 ADC channels and locking!
  3. Standing-Wave Pattern Angle theta(t) & Quadrature Ellipticity Q(t):
     Compares Untrimmed Fab-Pinned (locks to theta_pin = 28 deg!) vs.
     Electrostatically Trimmed / Mode-Matched (holds launch angle cleanly!).
  4. Rate Dead-Band / Adler Lock-In Curve (dtheta/dt vs. Omega_z):
     Shows how damping pinning traps the standing wave below Omega_lock = Delta(1/tau)/(2*Ag)
     until spin rate breaks lock (or mode-matching/damping trim removes pinning).

Generates: studies/cloverleaf_64probe_pinning.png
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


N_PROBES = 64
PHI_PROBES = np.linspace(0.0, 2.0 * math.pi, N_PROBES, endpoint=False)


def simulate_pinning_ringup(
    f0_hz: float = 11_000.0,
    q_mean: float = 100_000.0,
    dq_frac: float = 0.25,        # Fractional Q asymmetry (Q1 vs Q2) driving damping pinning!
    df_fab_hz: float = 0.45,      # Fabrication stiffness split (Hz)
    theta_pin_deg: float = 28.0,  # Principal fab pinning angle in physical deg (0..45 deg unique for n=2)
    theta_init_deg: float = 4.0,  # Initial launch angle of standing wave (physical deg)
    omega_z_dps: float = 0.0,
    ag: float = 0.40,
    trimmed: bool = False,
    duration_s: float = 10.0,
    fs_env_hz: float = 100.0,
) -> dict[str, np.ndarray]:
    """Simulate slow-envelope 2-DOF complex amplitudes (a_x, a_y) under energy-regulated ring-up.

    Uses Lynch energy regulation (keeps total modal energy E = |a_x|^2 + |a_y|^2 ringing up
    smoothly from 0.05 -> 1.0 with an isotropic sustaining gain) so the standing-wave pattern
    angle theta(t) is 100% FREE to rotate and pin to the natural low-loss / stiffness axis!
    """
    n = int(round(duration_s * fs_env_hz))
    dt = 1.0 / fs_env_hz
    t = np.arange(n) * dt

    w0 = 2.0 * math.pi * f0_hz
    gamma_0 = w0 / q_mean
    # Anisodamping rate Delta(1/tau) = gamma_0 * dq_frac (if trimmed, reduced 20x)
    d_gamma = gamma_0 * (dq_frac * (0.04 if trimmed else 1.0))
    df_eff = df_fab_hz * (0.02 if trimmed else 1.0)
    dw = 2.0 * math.pi * df_eff

    # Modal angle alpha = 2 * theta_physical (because cos(2*theta) -> 90 deg modal = 45 deg physical!)
    a_pin = math.radians(2.0 * theta_pin_deg)
    c2 = math.cos(2.0 * a_pin)
    s2 = math.sin(2.0 * a_pin)

    # Rotated 2x2 damping tensor G and stiffness-split tensor W in (a_x, a_y) basis:
    # Eigenvalues of G: gamma_0 - d_gamma/2 along a_pin (HIGH Q -> PINNING AXIS!),
    #                   gamma_0 + d_gamma/2 along a_pin + 90 deg (LOW Q -> decays faster!)
    g_xx = gamma_0 - 0.5 * d_gamma * c2
    g_yy = gamma_0 + 0.5 * d_gamma * c2
    g_xy = -0.5 * d_gamma * s2

    # Rotated 2x2 stiffness split matrix (eigenvalues -dw/2 along a_pin, +dw/2 orthogonal)
    w_xx = -0.5 * dw * c2
    w_yy = +0.5 * dw * c2
    w_xy = -0.5 * dw * s2

    cor = ag * math.radians(omega_z_dps)  # d(alpha)/dt = -2*Ag*Omega -> in complex envelope: +/- cor

    # Start small (ringing up from 0.08 amplitude) at physical angle theta_init_deg
    a0 = math.radians(2.0 * theta_init_deg)
    ax = complex(0.08 * math.cos(a0), 0.0)
    ay = complex(0.08 * math.sin(a0), 0.0)

    # Isotropic energy-sustaining loop gain (pushes E = |ax|^2 + |ay|^2 -> 1.0 without biasing angle!)
    k_e = 1.6  # ring-up time constant ~0.6 s

    def deriv(ux: complex, uy: complex) -> tuple[complex, complex]:
        e_now = (ux.real**2 + ux.imag**2) + (uy.real**2 + uy.imag**2)
        g_sustain = 0.5 * gamma_0 + k_e * (1.0 - e_now)
        # Note: isotropic sustain +g_sustain*(ux, uy) preserves rotational symmetry!
        dux = (g_sustain - 0.5 * g_xx) * ux - 0.5 * g_xy * uy - 1j * (w_xx * ux + w_xy * uy) + 2.0 * cor * uy
        duy = (g_sustain - 0.5 * g_yy) * uy - 0.5 * g_xy * ux - 1j * (w_xy * ux + w_yy * uy) - 2.0 * cor * ux
        return dux, duy

    ax_arr = np.zeros(n, dtype=np.complex128)
    ay_arr = np.zeros(n, dtype=np.complex128)
    theta_deg = np.zeros(n)
    amp_arr = np.zeros(n)
    quad_ratio = np.zeros(n)

    for i in range(n):
        ax_arr[i] = ax
        ay_arr[i] = ay

        # Lynch invariants from complex envelope (ax, ay):
        # E = |ax|^2 + |ay|^2, C = |ax|^2 - |ay|^2, S = 2*Re(ax * conj(ay)), Q = 2*Im(ax * conj(ay))
        e_val = abs(ax)**2 + abs(ay)**2
        c_val = abs(ax)**2 - abs(ay)**2
        s_val = 2.0 * (ax.real * ay.real + ax.imag * ay.imag)
        q_val = 2.0 * (ax.imag * ay.real - ax.real * ay.imag)

        # Physical standing-wave pattern angle theta = (1/4) * atan2(S, C) (mod 90 deg -> fold to [-45, +45] or [0, 90])
        ang_phys = math.degrees(0.25 * math.atan2(s_val, c_val))
        theta_deg[i] = ang_phys
        amp_arr[i] = math.sqrt(e_val)
        quad_ratio[i] = q_val / max(e_val, 1e-12)

        # RK4 step
        sub = 4
        h = dt / sub
        for _ in range(sub):
            k1x, k1y = deriv(ax, ay)
            k2x, k2y = deriv(ax + 0.5 * h * k1x, ay + 0.5 * h * k1y)
            k3x, k3y = deriv(ax + 0.5 * h * k2x, ay + 0.5 * h * k2y)
            k4x, k4y = deriv(ax + h * k3x, ay + h * k3y)
            ax += (h / 6.0) * (k1x + 2 * k2x + 2 * k3x + k4x)
            ay += (h / 6.0) * (k1y + 2 * k2y + 2 * k3y + k4y)

    # Project onto all 64 perimeter ADC probes!
    # Complex envelope on probe k at physical angle phi_k: a_k(t) = ax(t)*cos(2*phi_k) + ay(t)*sin(2*phi_k)
    c_p = np.cos(2.0 * PHI_PROBES)[None, :]  # (1, 64)
    s_p = np.sin(2.0 * PHI_PROBES)[None, :]  # (1, 64)
    probe_env = np.abs(ax_arr[:, None] * c_p + ay_arr[:, None] * s_p)  # (N, 64)

    return {
        "t": t,
        "theta_deg": theta_deg,
        "amp": amp_arr,
        "quad_ratio": quad_ratio,
        "probe_env": probe_env,
        "ax": ax_arr,
        "ay": ay_arr,
    }


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
        "font.size": 9.2,
    })

    theta_pin = 28.0
    theta_init = 4.0
    sim_pin = simulate_pinning_ringup(
        q_mean=45_000.0, dq_frac=0.35, df_fab_hz=0.055,
        theta_pin_deg=theta_pin, theta_init_deg=theta_init, trimmed=False, duration_s=10.0,
    )
    sim_trim = simulate_pinning_ringup(
        q_mean=45_000.0, dq_frac=0.35, df_fab_hz=0.055,
        theta_pin_deg=theta_pin, theta_init_deg=theta_init, trimmed=True, duration_s=10.0,
    )

    fig = plt.figure(figsize=(13.8, 9.6), dpi=160)
    fig.suptitle(
        "64-Probe Die ADC Simulation: 2D n=2 Cloverleaf Ring-Up & Fabrication Mode Pinning",
        fontsize=13.5, fontweight="bold", color="#f8fafc", y=0.975,
    )

    # -----------------------------------------------------------------------
    # Panel A (Polar): 64-Probe Cloverleaf Ring-Up -> Settling to Pinning Angle
    # -----------------------------------------------------------------------
    ax = fig.add_subplot(2, 2, 1, projection="polar")
    ax.set_facecolor("#111827")

    # Closed polar loop for smooth cloverleaf curve + 64 discrete probe dots
    phi_closed = np.append(PHI_PROBES, PHI_PROBES[0])
    snaps = [
        (0.35, "#94a3b8", ":",  1.6, "t = 0.35 s (Ring-Up @ 4°)"),
        (1.60, "#fbbf24", "--", 2.0, "t = 1.60 s (Rotating → 16°)"),
        (9.50, "#38bdf8", "-",  2.5, "t = 9.50 s (Pinned @ 28°!)"),
    ]
    for ts, col, ls, lw, lbl in snaps:
        idx = int(ts * 100)
        r = sim_pin["probe_env"][idx]
        r_closed = np.append(r, r[0])
        ax.plot(phi_closed, r_closed, color=col, ls=ls, lw=lw, label=lbl)
        if ts > 9.0:
            ax.scatter(PHI_PROBES, r, s=14, color="#7dd3fc", edgecolor="#0b0f17", linewidth=0.4, zorder=6,
                       label="64 Die ADC Probes (5.625°)")

    # Draw Fab Pinning Axis (28 deg, 118 deg, 208 deg, 298 deg)
    for k in range(4):
        ang = math.radians(theta_pin + k * 90.0)
        ax.plot([ang, ang], [0, 1.12], color="#34d399", ls="-.", lw=1.3, alpha=0.85)
    ax.text(math.radians(theta_pin + 6), 1.07, f"Pinning Axis\nθ_pin = {theta_pin:.0f}°",
            fontsize=8.0, fontweight="bold", color="#a7f3d0")

    ax.set_ylim(0, 1.18)
    ax.set_rticks([0.3, 0.6, 1.0])
    ax.tick_params(colors="#94a3b8", labelsize=8)
    ax.set_title("A. 64-Probe Polar Cloverleaf: Ring-Up & Pinning", fontweight="bold", pad=14)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.22, -0.14), fontsize=7.3,
              framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel B: 64-Channel ADC Waterfall Heatmap (Probes #0..63 vs. Time)
    # -----------------------------------------------------------------------
    ax = fig.add_subplot(2, 2, 2)
    im = ax.imshow(
        sim_pin["probe_env"].T,
        aspect="auto",
        origin="lower",
        extent=[0.0, 10.0, 0, N_PROBES - 1],
        cmap="inferno",
        vmin=0.0,
        vmax=1.05,
    )
    # Overlay the 4 pinning probe indices (theta_pin + 90k)/360 * 64
    for k in range(4):
        p_idx = ((theta_pin + k * 90.0) / 360.0) * N_PROBES
        ax.axhline(p_idx, color="#34d399", ls="--", lw=1.1, alpha=0.8)
    ax.text(6.6, ((theta_pin + 90.0) / 360.0) * N_PROBES + 1.5, "4 Cloverleaf Lobes Lock!",
            fontsize=8.0, fontweight="bold", color="#a7f3d0",
            bbox=dict(boxstyle="round,pad=0.22", fc="#0f241f", ec="#34d399", lw=0.8))

    cbar = fig.colorbar(im, ax=ax, pad=0.02)
    cbar.set_label("Probe Envelope |ADC_k| (norm)", fontsize=8.3)
    ax.set_title("B. 64-Channel Die ADC Waterfall (Lobes Marching to θ_pin)", fontweight="bold", pad=8)
    ax.set_xlabel("Ring-Up Time t (s)")
    ax.set_ylabel("Die ADC Probe Channel # (0..63 → 0°..360°)")
    ax.set_yticks([0, 16, 32, 48, 63])

    # -----------------------------------------------------------------------
    # Panel C: Standing-Wave Pattern Angle theta(t) — Pinned vs. Trimmed
    # -----------------------------------------------------------------------
    ax = fig.add_subplot(2, 2, 3)
    ax.axhline(theta_pin, color="#34d399", ls="--", lw=1.5, label=f"Fab Pinning Angle θ_pin = {theta_pin:.0f}°")
    ax.axhline(theta_init, color="#94a3b8", ls=":", lw=1.3, label=f"Launch Angle θ_0 = {theta_init:.0f}°")
    ax.plot(sim_pin["t"], sim_pin["theta_deg"], color="#f87171", lw=2.5,
            label="Untrimmed Fab Asymmetry: Rotates & Pins to 28°!")
    ax.plot(sim_trim["t"], sim_trim["theta_deg"], color="#38bdf8", lw=2.3,
            label="Trimmed (100V ±20V + Damping Comp): Holds 4°!")

    ax.annotate(
        "Lower-Q eigenmode decays faster\n→ standing wave rings up & pulls\ninto high-Q pinning axis θ_pin!",
        xy=(2.2, float(sim_pin["theta_deg"][220])), xytext=(4.4, 12.0),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.3),
        fontsize=8.0, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.3", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("C. Cloverleaf Pattern Angle θ(t): Fab Pinning vs. Trimmed", fontweight="bold", pad=8)
    ax.set_xlabel("Time t (s)")
    ax.set_ylabel("Standing-Wave Pattern Angle θ (deg)")
    ax.set_xlim(0, 10.0)
    ax.set_ylim(-2, 42)
    ax.grid(True)
    ax.legend(loc="lower right", fontsize=7.5, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    # -----------------------------------------------------------------------
    # Panel D: Rate Dead-Band / Adler Lock-In Curve (Precession vs. Omega_z)
    # -----------------------------------------------------------------------
    ax = fig.add_subplot(2, 2, 4)
    # Adler equation: d(theta)/dt = -Ag*Omega_z + 0.5*Delta(1/tau)*sin(4*(theta - theta_pin))
    # Average precession speed = 0 for |Omega_z| <= Omega_lock, else Ag * sqrt(Omega_z^2 - Omega_lock^2)
    omega_dps = np.linspace(0.0, 20.0, 400)
    ag = 0.40
    # Example untrimmed anisodamping lock threshold Omega_lock = Delta(1/tau)/(2*Ag)
    # For Q=60k, f0=11kHz, dq_frac=0.30 -> Delta(1/tau) = 0.173 rad/s -> Omega_lock = 12.4 deg/s (or 6.0 deg/s for dq=15%)
    for om_lock, col, lbl in [
        (0.0, "#34d399", "Ideal / Trimmed (Ω_lock = 0 °/s): Pure Linear Bryan Gain"),
        (3.5, "#38bdf8", "Modest Anisodamping (Ω_lock = 3.5 °/s dead-band)"),
        (8.0, "#f87171", "Strong Fab Pinning (Ω_lock = 8.0 °/s dead-band!)"),
    ]:
        avg_prec = np.where(omega_dps <= om_lock, 0.0, ag * np.sqrt(np.maximum(omega_dps**2 - om_lock**2, 0.0)))
        ax.plot(omega_dps, avg_prec, color=col, lw=2.4, label=lbl)

    ax.axvspan(0.0, 8.0, color="#ef4444", alpha=0.14)
    ax.annotate(
        "PINNED DEAD-BAND!\nBelow Ω_lock = Δ(1/τ)/(2A_g),\ncloverleaf stays stuck at θ_pin!",
        xy=(4.0, 0.05), xytext=(5.5, 4.9),
        arrowprops=dict(arrowstyle="->", color="#f87171", lw=1.3),
        fontsize=8.0, color="#fecaca",
        bbox=dict(boxstyle="round,pad=0.3", fc="#281218", ec="#f87171", lw=0.9),
    )

    ax.set_title("D. Whole-Angle Lock-In Dead-Band from Damping Pinning", fontweight="bold", pad=8)
    ax.set_xlabel("True Input Spin Rate Ω_z (°/s)")
    ax.set_ylabel("Mean Pattern Precession Rate |dθ/dt| (°/s)")
    ax.set_xlim(0, 20.0)
    ax.set_ylim(-0.2, 8.6)
    ax.grid(True)
    ax.legend(loc="upper left", fontsize=7.4, framealpha=0.92, facecolor="#0b0f17", edgecolor="#334155")

    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.945))
    out_path = Path(__file__).resolve().parent.parent / "studies" / "cloverleaf_64probe_pinning.png"
    fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)

    print(f"Saved: {out_path}")
    print(f"  Untrimmed angle: {sim_pin['theta_deg'][0]:.1f} deg -> {sim_pin['theta_deg'][-1]:.2f} deg (pin target {theta_pin:.1f} deg)")
    print(f"  Trimmed angle:   {sim_trim['theta_deg'][0]:.1f} deg -> {sim_trim['theta_deg'][-1]:.2f} deg")


if __name__ == "__main__":
    main()
