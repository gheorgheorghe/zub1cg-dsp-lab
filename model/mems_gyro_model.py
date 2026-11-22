#!/usr/bin/env python3
"""Stage 1 Golden Model: 2-DOF Coriolis Vibratory Gyroscope (DRG & HRG).

Models both:
  1. Silicon MEMS Disk Resonator Gyroscope (DRG): f0 = 11 kHz, Q = 100,000
  2. Fused-Silica Micro-HRG:                      f0 = 11 kHz, Q = 1,000,000
  3. Space-Grade Macro HRG (30 mm shell):         f0 = 4.2 kHz, Q = 5,000,000

Physics formulation (normalized modal coordinates x, y; v = dx/(w0*dt)):
  dx/dt   =  w0 * vx
  dvx/dt  = -(wx^2/w0)*x - g_xx*vx - (w_xy/w0)*y - g_xy*vy + 2*Ag*Omega_z*vy + u_x
  dy/dt   =  w0 * vy
  dvy/dt  = -(wy^2/w0)*y - g_yy*vy - (w_xy/w0)*x - g_xy*vx - 2*Ag*Omega_z*vx + u_y

Discretization:
  Strang-split exact harmonic rotation + exact exponential damping + symplectic
  coupling/Coriolis/actuation update. Preserves Q = 1e5 .. 1e7 with ZERO numerical
  damping and maps 1-to-1 to the Stage 1.3 SystemVerilog `mems_plant_2dof.sv`.

Closed-Loop Control Architecture (Force-to-Rebalance + Lynch Diagnostics):
  - Drive-Axis PLL:        tracks resonant frequency fx(T) under thermal drift
  - Drive-Axis AGC:        regulates drive amplitude x0 -> A_ref = 1.0
  - Sense Lock-In Demod:   separates Coriolis rate (in-phase with vx) from
                           quadrature stiffness coupling (in-phase with x)
  - Quadrature Nulling:    cancels stiffness cross-coupling w_xy
  - Force-to-Rebalance:    nulls sense displacement y -> 0; FTR command = Omega_hat
  - Lynch Variables:       computes Energy E, Quadrature Q, Pattern Angle theta
"""

from __future__ import annotations
import argparse
from dataclasses import dataclass, replace
import math
from typing import Callable
import numpy as np


DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


@dataclass(frozen=True)
class GyroParams:
    """Physical parameters for a 2-DOF Coriolis Vibratory Gyroscope."""
    name: str
    f0_hz: float              # Nominal resonant frequency (Hz)
    q_x: float                # Drive-axis quality factor
    q_y: float                # Sense-axis quality factor
    mode_split_hz: float      # df = fy - fx (Hz); 0 = mode-matched
    angular_gain: float       # Bryan's factor Ag (DRG n=2 ~0.40, HRG n=2 ~0.28)
    quad_equiv_dps: float     # Fabrication stiffness quadrature expressed in deg/s
    anisodamp_bias_dps: float # Damping asymmetry rate bias in deg/s
    arw_deg_sqrt_hr: float    # Angle Random Walk (deg / sqrt(hr))
    readout_noise_rms: float  # Electronic front-end noise (normalized displacement/sqrt(Hz))

    @property
    def w0(self) -> float:
        return 2.0 * math.pi * self.f0_hz

    @property
    def tau_x(self) -> float:
        """Amplitude ringdown time constant tau = Q / (pi * f0)."""
        return self.q_x / (math.pi * self.f0_hz)

    @property
    def tau_y(self) -> float:
        return self.q_y / (math.pi * self.f0_hz)


# ----------------------------------------------------------------------------
# Standard Presets: Cheap Silicon DRG vs. High-Q Micro-HRG vs. Space Macro-HRG
# ----------------------------------------------------------------------------
PRESETS: dict[str, GyroParams] = {
    "drg_11khz_q100k": GyroParams(
        name="Silicon MEMS DRG (11 kHz, Q=100k)",
        f0_hz=11_000.0,
        q_x=100_000.0,
        q_y=95_000.0,
        mode_split_hz=0.50,        # Typical residual split before/during electrostatic tuning
        angular_gain=0.40,         # n=2 wineglass disk resonator
        quad_equiv_dps=120.0,      # Large realistic MEMS quadrature coupling
        anisodamp_bias_dps=0.05,   # Small damping mismatch bias
        arw_deg_sqrt_hr=0.015,     # Tactical/near-navigation MEMS DRG
        readout_noise_rms=1e-7,
    ),
    "hrg_micro_11khz_q1m": GyroParams(
        name="Fused-Silica Micro-HRG (11 kHz, Q=1M)",
        f0_hz=11_000.0,
        q_x=1_000_000.0,
        q_y=980_000.0,
        mode_split_hz=0.05,        # Tightly mode-matched fused-silica micro-shell
        angular_gain=0.28,         # n=2 hemispherical shell Bryan's factor
        quad_equiv_dps=30.0,       # Lower quadrature due to isotropic fused silica
        anisodamp_bias_dps=0.005,
        arw_deg_sqrt_hr=0.002,
        readout_noise_rms=2e-8,
    ),
    "hrg_space_4khz_q5m": GyroParams(
        name="Macro Space-Grade HRG (4.2 kHz, Q=5M)",
        f0_hz=4_200.0,
        q_x=5_000_000.0,
        q_y=4_900_000.0,
        mode_split_hz=0.005,       # Nearly degenerate fused-silica 30 mm hemisphere
        angular_gain=0.28,
        quad_equiv_dps=10.0,
        anisodamp_bias_dps=0.0005,
        arw_deg_sqrt_hr=0.0002,
        readout_noise_rms=5e-9,
    ),
}


@dataclass
class ControllerGains:
    """PI gains for the four closed-loop CVG control loops (baseband rate)."""
    # Drive PLL (rad/s per rad phase error) — ~120 Hz bandwidth keeps phase lag < 0.05 deg
    pll_kp: float = 800.0
    pll_ki: float = 200_000.0
    # Drive AGC (normalized acceleration per unit amplitude error)
    agc_kp: float = 100.0
    agc_ki: float = 4_000.0
    # Sense FTR (Coriolis rate rebalance)
    ftr_kp: float = 200.0
    ftr_ki: float = 25_000.0
    # Sense Quadrature Nulling
    quad_kp: float = 200.0
    quad_ki: float = 25_000.0
    # Lock-in low-pass cutoff (Hz)
    demod_bw_hz: float = 500.0
    # Target normalized drive amplitude
    amp_ref: float = 1.0


@dataclass
class SimResult:
    """Decimated baseband telemetry matching what ZU1CG streams over GbE."""
    t_s: np.ndarray
    omega_true_dps: np.ndarray
    omega_est_dps: np.ndarray
    freq_true_hz: np.ndarray
    freq_pll_hz: np.ndarray
    drive_amp: np.ndarray
    drive_phase_err_deg: np.ndarray
    sense_amp: np.ndarray
    quad_cmd_dps: np.ndarray
    lynch_energy: np.ndarray
    lynch_quad: np.ndarray
    lynch_theta_deg: np.ndarray


def allan_deviation(rate_dps: np.ndarray, fs_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """Compute overlapping Allan Deviation (ADEV) in deg/hr vs cluster time tau (s)."""
    n = len(rate_dps)
    max_m = n // 4
    if max_m < 2:
        return np.array([]), np.array([])
    m_vals = np.unique(np.logspace(0, math.log10(max_m), num=40).astype(int))
    taus = []
    adevs = []
    # Integrate rate to angle (deg)
    theta = np.cumsum(rate_dps) / fs_hz
    for m in m_vals:
        tau = m / fs_hz
        diff2 = theta[2 * m :] - 2.0 * theta[m:-m] + theta[: -2 * m]
        avar = 0.5 * np.mean(diff2**2) / (tau**2)
        taus.append(tau)
        adevs.append(math.sqrt(max(avar, 0.0)) * 3600.0)  # deg/hr
    return np.array(taus), np.array(adevs)


def simulate_cvg_closed_loop(
    params: GyroParams,
    duration_s: float = 0.4,
    fs_hz: float = 200_000.0,
    decim_hz: float = 2_000.0,
    gains: ControllerGains | None = None,
    omega_fn: Callable[[float], float] | None = None,
    df_drift_fn: Callable[[float], float] | None = None,
    enable_noise: bool = True,
    enable_ftr: bool = True,
    enable_quad_null: bool = True,
    seed: int = 42,
) -> SimResult:
    """Run carrier-level Strang-split 2-DOF CVG simulation with closed-loop control.

    Args:
      params:           GyroParams preset (DRG or HRG)
      duration_s:       Simulation length in seconds
      fs_hz:            Carrier physics/controller tick rate (e.g., 200 kHz or 1 MHz)
      decim_hz:         Baseband telemetry output rate (Hz)
      omega_fn:         Callable t_s -> true input angular rate in deg/s
      df_drift_fn:      Callable t_s -> drive resonance thermal drift in Hz
      enable_noise:     Include Brownian thermal rate noise & readout noise
      enable_ftr:       Close the Force-to-Rebalance loop (y -> 0)
      enable_quad_null: Close the quadrature stiffness nulling loop
    """
    if gains is None:
        gains = ControllerGains()
    if omega_fn is None:
        omega_fn = lambda t: 100.0 if t >= 0.15 else 0.0
    if df_drift_fn is None:
        df_drift_fn = lambda t: 0.0

    rng = np.random.default_rng(seed)
    dt = 1.0 / fs_hz
    n_steps = int(round(duration_s * fs_hz))
    decim_stride = max(1, int(round(fs_hz / decim_hz)))
    n_out = n_steps // decim_stride

    w0 = params.w0
    ag = params.angular_gain

    # Convert physical quadrature & anisodamping specifications into modal couplings:
    # Quadrature acceleration = -k_xy * x where k_xy = 2 * Ag * (quad_equiv_rad/s)
    k_xy = 2.0 * ag * (params.quad_equiv_dps * DEG2RAD)
    # Anisodamping rate bias: g_xy = 2 * Ag * (bias_rad/s)
    g_xy = 2.0 * ag * (params.anisodamp_bias_dps * DEG2RAD)

    # White noise std per carrier step from ARW (deg/sqrt(hr) -> rad/s / sqrt(Hz))
    arw_rad_sqrt_s = (params.arw_deg_sqrt_hr * DEG2RAD) / 60.0
    # Equivalent acceleration noise std in normalized modal units
    u_noise_std = 2.0 * ag * arw_rad_sqrt_s * math.sqrt(fs_hz) if enable_noise else 0.0
    readout_std = params.readout_noise_rms * math.sqrt(fs_hz) if enable_noise else 0.0

    # Lock-in 3-stage cascaded 1-pole IIR low-pass (models 3rd-order CIC/FIR decimator)
    a_lpf = 1.0 - math.exp(-2.0 * math.pi * gains.demod_bw_hz * dt)

    # Plant state (initialized near nominal drive oscillation so acquisition is fast)
    x = 0.0
    vx = 0.85 * gains.amp_ref
    y = 0.0
    vy = 0.0

    # NCO & PLL state (start slightly detuned by -15 Hz to test autonomous PLL lock!)
    f_nco = params.f0_hz - 15.0
    phi = 0.0
    pll_int = 0.0
    agc_int = (w0 / params.q_x) * gains.amp_ref  # nominal sustaining drive
    ftr_int = 0.0
    quad_int = 0.0

    # Lock-in 3-stage LPF states (Xc, Xs for drive; Yc, Ys for sense)
    xc1 = xs1 = xc2 = xs2 = xc3 = xs3 = 0.0
    yc1 = ys1 = yc2 = ys2 = yc3 = ys3 = 0.0

    # Output telemetry buffers
    t_out = np.zeros(n_out)
    omega_true_out = np.zeros(n_out)
    omega_est_out = np.zeros(n_out)
    freq_true_out = np.zeros(n_out)
    freq_pll_out = np.zeros(n_out)
    drive_amp_out = np.zeros(n_out)
    drive_perr_out = np.zeros(n_out)
    sense_amp_out = np.zeros(n_out)
    quad_cmd_out = np.zeros(n_out)
    lynch_e_out = np.zeros(n_out)
    lynch_q_out = np.zeros(n_out)
    lynch_th_out = np.zeros(n_out)

    # Pre-generate noise in one vectorized call for speed
    noise_uy = rng.normal(0.0, u_noise_std, size=n_steps) if u_noise_std > 0 else None
    noise_rx = rng.normal(0.0, readout_std, size=n_steps) if readout_std > 0 else None
    noise_ry = rng.normal(0.0, readout_std, size=n_steps) if readout_std > 0 else None

    out_idx = 0
    two_pi = 2.0 * math.pi

    for k in range(n_steps):
        t = k * dt
        omega_dps = omega_fn(t)
        omega_rad = omega_dps * DEG2RAD
        df_t = df_drift_fn(t)

        fx_curr = params.f0_hz + df_t
        fy_curr = fx_curr + params.mode_split_hz
        wx = two_pi * fx_curr
        wy = two_pi * fy_curr

        # 1. NCO Carrier Generation (sin, cos)
        c_phi = math.cos(phi)
        s_phi = math.sin(phi)

        # 2. Sensor readout + Lock-In Demodulation (2x mixer + 3-stage LPF)
        x_meas = x + (noise_rx[k] if noise_rx is not None else 0.0)
        y_meas = y + (noise_ry[k] if noise_ry is not None else 0.0)

        xc1 += a_lpf * (2.0 * x_meas * c_phi - xc1)
        xs1 += a_lpf * (2.0 * x_meas * s_phi - xs1)
        xc2 += a_lpf * (xc1 - xc2)
        xs2 += a_lpf * (xs1 - xs2)
        xc3 += a_lpf * (xc2 - xc3)
        xs3 += a_lpf * (xs2 - xs3)

        yc1 += a_lpf * (2.0 * y_meas * c_phi - yc1)
        ys1 += a_lpf * (2.0 * y_meas * s_phi - ys1)
        yc2 += a_lpf * (yc1 - yc2)
        ys2 += a_lpf * (ys1 - ys2)
        yc3 += a_lpf * (yc2 - yc3)
        ys3 += a_lpf * (ys2 - ys3)

        # 3. Drive-Axis PLL & AGC
        # At resonance under u_x = u_d*cos(phi), x = +A*sin(phi) => xs3 = +A, xc3 = 0.
        amp_x = math.hypot(xc3, xs3)
        phase_err = math.atan2(xc3, max(xs3, 1e-6))  # >0 if NCO lags resonance -> speed up NCO
        pll_int += gains.pll_ki * phase_err * dt
        pll_int = max(-2000.0, min(2000.0, pll_int))
        df_pll_rad = gains.pll_kp * phase_err + pll_int
        f_nco_eff = f_nco + df_pll_rad / two_pi

        amp_err = gains.amp_ref - xs3
        agc_int += gains.agc_ki * amp_err * dt
        agc_int = max(-500.0, min(500.0, agc_int))
        u_d = gains.agc_kp * amp_err + agc_int
        u_d = max(-500.0, min(500.0, u_d))
        u_x = u_d * c_phi

        # 4. Sense-Axis FTR & Quadrature Nulling
        if enable_ftr:
            err_rate = -ys3
            ftr_int += gains.ftr_ki * err_rate * dt
            u_rate = gains.ftr_kp * err_rate + ftr_int
        else:
            u_rate = 0.0

        if enable_quad_null:
            err_quad = yc3
            quad_int += gains.quad_ki * err_quad * dt
            u_quad = gains.quad_kp * err_quad + quad_int
        else:
            u_quad = 0.0

        u_y = u_rate * c_phi + u_quad * s_phi
        if noise_uy is not None:
            u_y += noise_uy[k]

        # 5. Strang-Split Exact-Rotation 2-DOF Resonator Step
        # First half-kick uses carrier phase at t_k (c_phi, s_phi);
        # second half-kick uses carrier phase at t_{k+1} (c_phi_next, s_phi_next)
        # so actuation stays phase-aligned to the exact harmonic rotation!
        half_dt = 0.5 * dt
        cor = 2.0 * ag * omega_rad
        gx = wx / params.q_x
        gy = wy / params.q_y

        phi_next = phi + two_pi * f_nco_eff * dt
        if phi_next >= two_pi:
            phi_next -= two_pi
        c_phi_next = math.cos(phi_next)
        s_phi_next = math.sin(phi_next)

        u_x_next = u_d * c_phi_next
        u_y_next = u_rate * c_phi_next + u_quad * s_phi_next
        if noise_uy is not None:
            u_y_next += noise_uy[k]

        ax = -gx * vx - k_xy * y - g_xy * vy + cor * vy + u_x
        ay = -gy * vy - k_xy * x - g_xy * vx - cor * vx + u_y
        vx_h = vx + half_dt * ax
        vy_h = vy + half_dt * ay

        # Exact harmonic rotations (zero numerical damping, zero freq warping!)
        tx = wx * dt
        ty = wy * dt
        cx_r, sx_r = math.cos(tx), math.sin(tx)
        cy_r, sy_r = math.cos(ty), math.sin(ty)

        x_new = x * cx_r + vx_h * sx_r
        vx_r = -x * sx_r + vx_h * cx_r
        y_new = y * cy_r + vy_h * sy_r
        vy_r = -y * sy_r + vy_h * cy_r

        # Second half-step coupling, Coriolis, damping, and actuation at t_{k+1}
        ax2 = -gx * vx_r - k_xy * y_new - g_xy * vy_r + cor * vy_r + u_x_next
        ay2 = -gy * vy_r - k_xy * x_new - g_xy * vx_r - cor * vx_r + u_y_next
        vx = vx_r + half_dt * ax2
        vy = vy_r + half_dt * ay2
        x = x_new
        y = y_new

        # Advance NCO phase
        phi = phi_next

        # 6. Decimated Baseband Telemetry + Lynch Variables (E, Q, theta)
        if (k % decim_stride) == 0 and out_idx < n_out:
            amp_denom = max(amp_x, 1e-3)
            # Project (u_rate, u_quad) onto the exact instantaneous drive phase (phi + phase_err)
            # so residual PLL tracking lag during thermal ramps causes zero quadrature-to-rate leakage!
            cd = xs3 / amp_denom  # cos(phase_err)
            sd = xc3 / amp_denom  # sin(phase_err)
            u_rate_aligned = u_rate * cd - u_quad * sd
            u_quad_aligned = u_rate * sd + u_quad * cd

            # Calibrated rate estimate from FTR command: u_rate = 2 * Ag * Omega_rad * A
            omega_est_rad = u_rate_aligned / (2.0 * ag * amp_denom)
            quad_est_rad = u_quad_aligned / (2.0 * ag * amp_denom)

            # Lynch slow variables from quadrature demod components (cx, sx, cy, sy)
            lynch_e = xc3 * xc3 + xs3 * xs3 + yc3 * yc3 + ys3 * ys3
            lynch_q = 2.0 * (xc3 * ys3 - xs3 * yc3)
            lynch_r = (xc3 * xc3 + xs3 * xs3) - (yc3 * yc3 + ys3 * ys3)
            lynch_s = 2.0 * (xc3 * yc3 + xs3 * ys3)
            lynch_theta = 0.5 * math.atan2(lynch_s, lynch_r)

            t_out[out_idx] = t
            omega_true_out[out_idx] = omega_dps
            omega_est_out[out_idx] = omega_est_rad * RAD2DEG
            freq_true_out[out_idx] = fx_curr
            freq_pll_out[out_idx] = f_nco_eff
            drive_amp_out[out_idx] = amp_x
            drive_perr_out[out_idx] = phase_err * RAD2DEG
            sense_amp_out[out_idx] = math.hypot(yc3, ys3)
            quad_cmd_out[out_idx] = quad_est_rad * RAD2DEG
            lynch_e_out[out_idx] = lynch_e
            lynch_q_out[out_idx] = lynch_q
            lynch_th_out[out_idx] = lynch_theta * RAD2DEG
            out_idx += 1

    return SimResult(
        t_s=t_out,
        omega_true_dps=omega_true_out,
        omega_est_dps=omega_est_out,
        freq_true_hz=freq_true_out,
        freq_pll_hz=freq_pll_out,
        drive_amp=drive_amp_out,
        drive_phase_err_deg=drive_perr_out,
        sense_amp=sense_amp_out,
        quad_cmd_dps=quad_cmd_out,
        lynch_energy=lynch_e_out,
        lynch_quad=lynch_q_out,
        lynch_theta_deg=lynch_th_out,
    )


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 1 CVG Golden Model (DRG & HRG)")
    ap.add_argument("--preset", choices=list(PRESETS.keys()), default="drg_11khz_q100k")
    ap.add_argument("--duration", type=float, default=0.4, help="Simulation duration (s)")
    ap.add_argument("--fs", type=float, default=200_000.0, help="Carrier sample rate (Hz)")
    ap.add_argument("--rate-step", type=float, default=100.0, help="Angular rate step (deg/s)")
    ap.add_argument("--drift-hz", type=float, default=20.0, help="Thermal frequency drift ramp (Hz)")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    params = PRESETS[args.preset]
    print(f"Running {params.name}")
    print(f"  f0={params.f0_hz:.1f} Hz | Qx={params.q_x:,.0f} (tau={params.tau_x:.2f} s) | "
          f"df={params.mode_split_hz} Hz | Ag={params.angular_gain} | Quad={params.quad_equiv_dps} deg/s")

    def omega_profile(t: float) -> float:
        if t < 0.15:
            return 0.0
        if t < 0.30:
            return args.rate_step
        return -0.5 * args.rate_step

    def drift_profile(t: float) -> float:
        return args.drift_hz * (t / max(args.duration, 1e-6))

    res = simulate_cvg_closed_loop(
        params=params,
        duration_s=args.duration,
        fs_hz=args.fs,
        omega_fn=omega_profile,
        df_drift_fn=drift_profile,
    )

    # Report steady-state accuracy during the rate step [0.22, 0.29] s
    m_step = (res.t_s >= 0.22) & (res.t_s <= 0.29)
    mean_est = float(np.mean(res.omega_est_dps[m_step]))
    err_pct = 100.0 * (mean_est - args.rate_step) / args.rate_step
    pll_err = float(np.mean(np.abs(res.freq_pll_hz[m_step] - res.freq_true_hz[m_step])))
    quad_est = float(np.mean(res.quad_cmd_dps[m_step]))
    print(f"  Step response [0.22..0.29s]: true={args.rate_step:.2f} dps, est={mean_est:.3f} dps "
          f"(err={err_pct:+.3f}%)")
    print(f"  PLL tracking error: {pll_err:.4f} Hz | Quadrature null cmd: {quad_est:.2f} dps "
          f"(true={params.quad_equiv_dps:.2f} dps) | Residual sense |y|={np.mean(res.sense_amp[m_step]):.2e}")

    if not args.no_plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
        axes[0].plot(res.t_s, res.omega_true_dps, "k--", label="True Omega_z")
        axes[0].plot(res.t_s, res.omega_est_dps, label="FTR Estimate")
        axes[0].set_ylabel("Rate (deg/s)")
        axes[0].set_title(f"{params.name} — Closed-Loop FTR + PLL + AGC + Quad Null")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(res.t_s, res.freq_true_hz, "k--", label="True f_x(T)")
        axes[1].plot(res.t_s, res.freq_pll_hz, label="PLL NCO f")
        axes[1].set_ylabel("Freq (Hz)")
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(res.t_s, res.drive_amp, label="Drive Amp |x|")
        axes[2].plot(res.t_s, res.sense_amp * 100.0, label="Sense |y| x100")
        axes[2].set_ylabel("Norm Amp")
        axes[2].legend()
        axes[2].grid(True)

        axes[3].plot(res.t_s, res.quad_cmd_dps, label="Quad Null Cmd (deg/s eq)")
        axes[3].axhline(params.quad_equiv_dps, color="k", linestyle="--", label="True Quad")
        axes[3].set_ylabel("Quad (deg/s)")
        axes[3].set_xlabel("Time (s)")
        axes[3].legend()
        axes[3].grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
