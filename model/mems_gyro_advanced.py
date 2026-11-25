#!/usr/bin/env python3
"""Advanced CVG / DRG / HRG Physics Suite (Python-First Golden Model).

Models three flagship real-world MEMS/HRG phenomena before any Stage 1.7 RTL:
  1. Electrostatic Spring Softening & DC Mode-Matching Tuning Curve:
       w_eff^2(V) = w_mech^2 - k_e * V^2
     Shows how a DC tuning bias V_tune on the sense electrodes collapses
     fabrication mode-split df = f_y - f_x -> 0 Hz (and approaches pull-in).

  2. Duffing Cubic Nonlinearity & AGC Overdrive (A-f Backbone Curve):
       x'' + (w0/Q)*x' + w0^2 * (x + beta_3 * x^3) = F*cos(w*t)
     Computes the bent Duffing shark-fin frequency response (softening
     electrostatic beta_3 < 0 vs. hardening mechanical beta_3 > 0), critical
     bifurcation amplitude A_crit ~ 1/sqrt(Q), and closed-loop AGC overdrive
     frequency pull (where AGC amplitude ripple leaks into PLL self-thermometry!).

  3. Whole-Angle (WA) HRG Mode (Lynch Energy/Quadrature Control):
     Simulates a 2-DOF Micro-HRG (Q=1M, Ag=0.28) in free-precession Whole-Angle
     mode vs. Force-to-Rebalance (FTR) mode under a violent 500 deg/s spin burst:
       - Energy loop regulates E = a^2 + q^2 -> E0
       - Quadrature loop nulls Q = 2*(x*vy - y*vx) -> 0
       - Standing wave pattern angle theta(t) precesses freely:
           d(theta)/dt = -Ag * Omega_z(t)
       - Unlimited rate range & pure geometric scale factor (Bryan's factor Ag)!
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model.mems_gyro_model import PRESETS, DEG2RAD, RAD2DEG


def simulate_electrostatic_softening() -> dict[str, np.ndarray]:
    """Model DC polarization V_p & sense tuning V_tune spring softening on Silicon DRG."""
    f0_mech = 11_050.0  # As-fabricated mechanical frequency (Hz)
    df_fab = 18.0       # As-fabricated mode split fy - fx = +18 Hz
    # Typical DRG electrostatic softening coefficient: df/d(V^2) ~ -0.18 Hz / V^2
    # (for ~2 um capacitive gap, ~10 pF electrode bank)
    alpha_vp = 0.18     # Hz / V^2 common bias softening
    alpha_vt = 0.12     # Hz / V^2 sense-axis differential tuning electrode

    vp = np.linspace(0.0, 22.0, 221)
    fx_vp = np.sqrt(np.maximum(f0_mech**2 - 2.0 * f0_mech * alpha_vp * vp**2, 0.0))
    fy_vp = np.sqrt(np.maximum((f0_mech + df_fab) ** 2 - 2.0 * f0_mech * alpha_vp * vp**2, 0.0))

    # Hold V_p = 15 V nominal, sweep sense tuning electrode V_tune = 0..16 V
    vp_nom = 15.0
    fx_nom = math.sqrt(f0_mech**2 - 2.0 * f0_mech * alpha_vp * vp_nom**2)
    vt = np.linspace(0.0, 16.0, 321)
    fy_vt = np.sqrt(np.maximum((fx_nom + df_fab) ** 2 - 2.0 * fx_nom * alpha_vt * vt**2, 0.0))
    df_vt = fy_vt - fx_nom
    v_match = math.sqrt(df_fab / alpha_vt)

    return {
        "vp": vp, "fx_vp": fx_vp, "fy_vp": fy_vp,
        "vt": vt, "fx_nom": np.full_like(vt, fx_nom), "fy_vt": fy_vt, "df_vt": df_vt,
        "v_match": np.array([v_match]),
    }


def simulate_duffing_backbone() -> dict[str, np.ndarray]:
    """Compute exact Duffing resonance shark-fin curves and A-f backbone pull."""
    f0 = 11_000.0
    w0 = 2.0 * math.pi * f0
    # Normalized electrostatic softening cubic term beta_3 = -3.5e-5 (at norm disp x=1)
    # Typical parallel-plate DRG softens with amplitude: w_res(A) = w0 * (1 + 3/8 * beta_3 * A^2)
    beta_3 = -3.5e-5
    q_eff = 20_000.0  # Effective Q for crisp visual shark-fin (real Q=100k bends 5x sharper!)

    amps = np.linspace(0.02, 2.2, 400)
    # Backbone curve: delta_f(A) = (3/8) * beta_3 * f0 * A^2
    df_backbone_hz = (3.0 / 8.0) * beta_3 * f0 * (amps**2)

    # Analytical Duffing response (implicit: dw(A) = backbone +/- (w0/(2Q)) * sqrt((F_peak/A)^2 - 1))
    drive_levels = [0.4, 0.9, 1.4, 1.9]
    curves = {}
    for a_peak in drive_levels:
        a = np.linspace(0.05, a_peak * 0.9999, 250)
        rad = np.sqrt(np.maximum((a_peak / a) ** 2 - 1.0, 0.0))
        half_bw = f0 / (2.0 * q_eff)
        df_bb = (3.0 / 8.0) * beta_3 * f0 * (a**2)
        f_left = df_bb - half_bw * rad
        f_right = df_bb + half_bw * rad
        curves[f"{a_peak:.1f}"] = (a, f_left, f_right)

    # Critical bifurcation amplitude A_crit = sqrt(8 / (3*sqrt(3) * Q * |beta_3|))
    a_crit = math.sqrt(8.0 / (3.0 * math.sqrt(3.0) * q_eff * abs(beta_3)))
    a_crit_q100k = math.sqrt(8.0 / (3.0 * math.sqrt(3.0) * 100_000.0 * abs(beta_3)))

    return {
        "amps": amps,
        "df_backbone_hz": df_backbone_hz,
        "curves": curves,
        "a_crit": np.array([a_crit]),
        "a_crit_q100k": np.array([a_crit_q100k]),
        "beta_3": np.array([beta_3]),
    }


def simulate_whole_angle_hrg(duration_s: float = 0.6, fs_hz: float = 200_000.0) -> dict[str, np.ndarray]:
    """Simulate 2-DOF Micro-HRG (Q=1M, Ag=0.28) in Whole-Angle (Lynch E/Q) mode.

    Uses exact Strang-split carrier physics with Lynch energy (E) and quadrature (Q)
    control forces projected smoothly onto the rotating pattern angle theta(t):
      - E = x^2 + vx^2 + y^2 + vy^2        (maintained at E0 = 1.0)
      - Q = 2 * (x*vy - y*vx)              (nulled to 0.0 -> zero elliptical precession drift)
      - theta = 0.5 * atan2(S, C)          (FREE to precess at dtheta/dt = -Ag * Omega_z!)
    """
    p = PRESETS["hrg_micro_11khz_q1m"]
    dt = 1.0 / fs_hz
    n_steps = int(round(duration_s * fs_hz))
    decim = 100  # 2 kHz telemetry
    n_out = n_steps // decim

    w0 = p.w0
    ag = p.angular_gain  # 0.28 for n=2 HRG
    gx = w0 / p.q_x
    gy = w0 / p.q_y
    # Mode-matched HRG with residual 30 deg/s quadrature stiffness!
    k_xy = 2.0 * ag * (p.quad_equiv_dps * DEG2RAD)

    c_r = math.cos(w0 * dt)
    s_r = math.sin(w0 * dt)

    # Start standing wave aligned along x-axis (theta = 0, E = 1.0, Q = 0)
    x, vx, y, vy = 0.0, 1.0, 0.0, 0.0
    e_ref = 1.0
    ke = 120.0   # Lynch energy loop gain (rad/s)
    kq = 350.0   # Lynch quadrature nulling P gain (rad/s)
    ki_q = 8_000.0  # Lynch quadrature integral gain (rad/s^2) -> zero steady-state Q!
    q_int = 0.0

    # Smooth violent spin profile: 0 -> +500 deg/s burst -> -300 deg/s reversal -> 0
    def omega_profile(t: float) -> float:
        if 0.08 <= t < 0.28:
            # Smooth raised-cosine +500 deg/s burst
            u = (t - 0.08) / 0.20
            return 500.0 * math.sin(math.pi * u) ** 2
        if 0.34 <= t < 0.52:
            u = (t - 0.34) / 0.18
            return -300.0 * math.sin(math.pi * u) ** 2
        return 0.0

    t_out = np.zeros(n_out)
    omega_true = np.zeros(n_out)
    ang_true = np.zeros(n_out)
    ang_wa = np.zeros(n_out)
    lynch_e = np.zeros(n_out)
    lynch_q = np.zeros(n_out)
    theta_wave = np.zeros(n_out)
    x_env = np.zeros(n_out)
    y_env = np.zeros(n_out)

    # Unwrapped true angle & pattern angle
    ang_true_acc = 0.0
    theta_prev = 0.0
    theta_unwrap = 0.0
    idx = 0

    for k in range(n_steps):
        t = k * dt
        om_dps = omega_profile(t)
        om_rad = om_dps * DEG2RAD
        cor = 2.0 * ag * om_rad

        # Instantaneous Lynch invariants (carrier-invariant in normalized coords!)
        e_now = x * x + vx * vx + y * y + vy * vy
        q_now = 2.0 * (x * vy - y * vx)

        # Lynch control accelerations:
        #   Energy force:     u_E = ke * (E_ref - E) * [vx, vy]          (dE/dt = +2*ke*(E_ref-E)*(vx^2+vy^2))
        #   Quadrature force: u_Q = (kq*Q + q_int)   * [+y, -x]          (dQ/dt = -2*(kq*Q+q_int)*(x^2+y^2) -> STABLE!)
        de = e_ref - e_now
        q_int += ki_q * q_now * dt
        q_cmd = kq * q_now + q_int
        ux = ke * de * vx + q_cmd * (+y)
        uy = ke * de * vy + q_cmd * (-x)

        # Exact Strang-split step
        hdt = 0.5 * dt
        vx_h = vx + hdt * (-gx * vx - k_xy * y + cor * vy + ux)
        vy_h = vy + hdt * (-gy * vy - k_xy * x - cor * vx + uy)

        x_n = x * c_r + vx_h * s_r
        vx_r = -x * s_r + vx_h * c_r
        y_n = y * c_r + vy_h * s_r
        vy_r = -y * s_r + vy_h * c_r

        e_n = x_n * x_n + vx_r * vx_r + y_n * y_n + vy_r * vy_r
        q_n = 2.0 * (x_n * vy_r - y_n * vx_r)
        q_cmd1 = kq * q_n + q_int
        ux1 = ke * (e_ref - e_n) * vx_r + q_cmd1 * (+y_n)
        uy1 = ke * (e_ref - e_n) * vy_r + q_cmd1 * (-x_n)

        vx = vx_r + hdt * (-gx * vx_r - k_xy * y_n + cor * vy_r + ux1)
        vy = vy_r + hdt * (-gy * vy_r - k_xy * x_n - cor * vx_r + uy1)
        x, y = x_n, y_n

        ang_true_acc += om_dps * dt

        if (k % decim) == 0 and idx < n_out:
            # Pattern orientation angle theta = 0.5 * atan2(2*(x*y + vx*vy), (x^2+vx^2)-(y^2+vy^2))
            s_var = 2.0 * (x * y + vx * vy)
            c_var = (x * x + vx * vx) - (y * y + vy * vy)
            th = 0.5 * math.atan2(s_var, c_var)
            dth = th - theta_prev
            if dth > 0.5 * math.pi:
                dth -= math.pi
            elif dth < -0.5 * math.pi:
                dth += math.pi
            theta_unwrap += dth
            theta_prev = th

            t_out[idx] = t
            omega_true[idx] = om_dps
            ang_true[idx] = ang_true_acc
            # Bryan's effect: standing wave lags body rotation by -Ag * Theta_body
            ang_wa[idx] = -(theta_unwrap * RAD2DEG) / ag
            lynch_e[idx] = e_now
            lynch_q[idx] = q_now
            theta_wave[idx] = theta_unwrap * RAD2DEG
            x_env[idx] = math.hypot(x, vx)
            y_env[idx] = math.hypot(y, vy)
            idx += 1

    return {
        "t_s": t_out,
        "omega_true_dps": omega_true,
        "ang_true_deg": ang_true,
        "ang_wa_deg": ang_wa,
        "ang_err_mdeg": (ang_wa - ang_true) * 1e3,
        "lynch_e": lynch_e,
        "lynch_q": lynch_q,
        "theta_wave_deg": theta_wave,
        "x_env": x_env,
        "y_env": y_env,
    }


def main() -> None:
    out_dir = Path("studies")
    out_dir.mkdir(parents=True, exist_ok=True)

    soft = simulate_electrostatic_softening()
    duff = simulate_duffing_backbone()
    wa = simulate_whole_angle_hrg()

    max_wa_err_mdeg = float(np.max(np.abs(wa["ang_err_mdeg"])))
    max_q_leak = float(np.max(np.abs(wa["lynch_q"])))
    print(f"[SOFTENING] DC mode-match voltage V_tune = {soft['v_match'][0]:.2f} V (collapses 18.0 Hz split -> 0.00 Hz)")
    print(f"[DUFFING]   A_crit(Q=20k) = {duff['a_crit'][0]:.2f} norm | A_crit(Q=100k) = {duff['a_crit_q100k'][0]:.2f} norm")
    print(f"[WHOLE-ANG] 500 deg/s burst -> Pattern precessed {np.max(np.abs(wa['theta_wave_deg'])):.2f} deg | "
          f"Peak Angle Err = {max_wa_err_mdeg:.2f} mdeg | Max |Q| = {max_q_leak:.5f}")

    # Plot 1: Electrostatic Softening + Duffing Nonlinearity (2x2 figure)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5))

    ax = axes[0, 0]
    ax.plot(soft["vp"], soft["fx_vp"], lw=2.0, label="Drive Mode f_x(V_p)")
    ax.plot(soft["vp"], soft["fy_vp"], "--", lw=2.0, label="Sense Mode f_y(V_p) (+18 Hz split)")
    ax.set_title("Electrostatic Spring Softening vs. Polarization Bias V_p")
    ax.set_xlabel("DC Polarization Voltage V_p (V)")
    ax.set_ylabel("Resonant Frequency (Hz)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(soft["vt"], soft["df_vt"], lw=2.2, color="#059669", label="Mode Split Δf = f_y(V_tune) − f_x")
    ax.axhline(0.0, color="#111827", ls="--", lw=1.0)
    vm = float(soft["v_match"][0])
    ax.axvline(vm, color="#dc2626", ls=":", lw=1.4, label=f"Mode-Match V_tune = {vm:.2f} V")
    ax.scatter([vm], [0.0], color="#dc2626", zorder=5, s=45)
    ax.set_title("DC Mode-Matching: Collapsing Fabrication Δf → 0 Hz")
    ax.set_xlabel("Sense Tuning Electrode Voltage V_tune (V)")
    ax.set_ylabel("Mode Split Δf (Hz)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    cols = ["#3b82f6", "#10b981", "#f59e0b", "#ef4444"]
    for c, (lbl, (a, fl, fr)) in zip(cols, duff["curves"].items()):
        ax.plot(fl, a, color=c, lw=1.8, label=f"Drive Peak A = {lbl}")
        ax.plot(fr, a, color=c, lw=1.8)
    ax.plot(duff["df_backbone_hz"], duff["amps"], "k--", lw=1.6, label="A–f Backbone (β₃ < 0 softening)")
    ax.axhline(float(duff["a_crit"][0]), color="#dc2626", ls=":", lw=1.2,
               label=f"Jump Bifurcation A_crit = {duff['a_crit'][0]:.2f}")
    ax.set_title("Duffing Shark-Fin Resonance & AGC Overdrive Bifurcation")
    ax.set_xlabel("Frequency Offset f − f₀ (Hz)")
    ax.set_ylabel("Normalized Drive Amplitude A")
    ax.set_xlim(-1.2, 0.8)
    ax.set_ylim(0, 2.2)
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    # AGC overdrive amplitude-to-frequency leakage into self-thermometry
    a_sweep = np.linspace(0.2, 2.0, 200)
    df_pull_hz = (3.0 / 8.0) * float(duff["beta_3"][0]) * 11_000.0 * (a_sweep**2)
    # Equivalent fake temperature error on Silicon (-0.33 Hz/C)
    fake_temp_c = df_pull_hz / (-0.33)
    ax.plot(a_sweep, fake_temp_c, lw=2.2, color="#7c3aed", label="Apparent ΔT Error from Duffing Pull")
    ax.axvline(float(duff["a_crit_q100k"][0]), color="#dc2626", ls="--", lw=1.3,
               label=f"Q=100k Bifurcation Limit A_crit = {duff['a_crit_q100k'][0]:.2f}")
    ax.set_title("Why AGC Overdrive Corrupts PLL Self-Thermometry")
    ax.set_xlabel("AGC Drive Amplitude Setpoint (Normalized)")
    ax.set_ylabel("Apparent Temperature Bias (°C)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    fig.suptitle("MEMS / DRG Electrostatic Softening & Duffing Nonlinearity (Python Golden Model)", fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    p1 = out_dir / "advanced_softening_duffing.png"
    fig.savefig(p1, dpi=150)
    plt.close(fig)
    print(f"Saved -> {p1}")

    # Plot 2: Whole-Angle (WA) HRG Mode under 500 deg/s Spin Burst (2x2 figure)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.2))
    t_ms = wa["t_s"] * 1e3

    ax = axes[0, 0]
    ax.plot(t_ms, wa["omega_true_dps"], lw=2.0, color="#2563eb", label="True Body Rate Ω_z(t)")
    ax.axhline(250, color="#dc2626", ls=":", lw=1.2, label="Typical FTR Loop Saturation Limit")
    ax.axhline(-250, color="#dc2626", ls=":", lw=1.2)
    ax.set_title("High-Dynamic Spin Burst (+500 / −300 °/s)")
    ax.set_ylabel("Angular Rate (°/s)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(t_ms, wa["ang_true_deg"], "k--", lw=2.0, label="True Body Angle Θ(t)")
    ax.plot(t_ms, wa["ang_wa_deg"], lw=1.8, color="#10b981", alpha=0.9, label="Whole-Angle HRG Readout (−θ/A_g)")
    ax.plot(t_ms, wa["theta_wave_deg"], lw=1.6, color="#7c3aed", label="Standing-Wave Angle θ(t) (A_g = 0.28)")
    ax.set_title("Whole-Angle Free Pattern Precession (Zero Rate Saturation!)")
    ax.set_ylabel("Angle (°)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(t_ms, wa["x_env"], lw=1.8, color="#2563eb", label="|x(t)| Envelope (cos θ)")
    ax.plot(t_ms, wa["y_env"], lw=1.8, color="#f59e0b", label="|y(t)| Envelope (sin θ)")
    ax.plot(t_ms, wa["lynch_e"], "k--", lw=1.4, label="Lynch Energy E = x²+vx²+y²+vy²")
    ax.set_title("Modal Energy Sharing as Pattern Rotates Around the Shell")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Normalized Amplitude / Energy")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(t_ms, wa["ang_err_mdeg"], lw=1.8, color="#dc2626", label=f"WA Angle Error (peak {max_wa_err_mdeg:.1f} mdeg)")
    ax.set_title("Whole-Angle Readout Error (With 30 °/s Quad Nulling Active)")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Angle Error (mdeg)")
    ax.legend(fontsize=8.5)
    ax.grid(True, alpha=0.3)

    fig.suptitle("Whole-Angle (WA) Micro-HRG Mode — Lynch E/Q Control Under 500 °/s Burst", fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    p2 = out_dir / "advanced_whole_angle_hrg.png"
    fig.savefig(p2, dpi=150)
    plt.close(fig)
    print(f"Saved -> {p2}")


if __name__ == "__main__":
    main()
