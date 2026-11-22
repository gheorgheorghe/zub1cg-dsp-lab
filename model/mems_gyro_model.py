#!/usr/bin/env python3
"""Stage 1 Golden Model: 2-DOF Coriolis Vibratory Gyroscope (DRG & HRG).

Models three calibrated CVG presets:
  1. Silicon MEMS Disk Resonator Gyroscope (DRG): f0 = 11.0 kHz, Q = 100,000
  2. Fused-Silica Micro-HRG (3D micro-shell):     f0 = 11.0 kHz, Q = 1,000,000
  3. Macro Space-Grade HRG (30 mm hemisphere):    f0 =  4.2 kHz, Q = 5,000,000

Physics formulation (normalized modal coordinates x, y; v = dx/(w0*dt)):
  dx/dt   =  w0 * vx
  dvx/dt  = -(wx^2/w0)*x - g_xx*vx - (w_xy/w0)*y - g_xy*vy + 2*Ag*Omega_z*vy + u_x
  dy/dt   =  w0 * vy
  dvy/dt  = -(wy^2/w0)*y - g_yy*vy - (w_xy/w0)*x - g_xy*vx - 2*Ag*Omega_z*vx + u_y

Signal Naming Convention (standard CVG closed-loop architecture):
  DX, DY  : Demodulated Drive pickoff (DX = amplitude -> AGC, DY = phase error -> PLL)
  SX, SY  : Demodulated Sense pickoff (SX = rate error -> FTR, SY = quadrature -> CLQ)
  PLLAdj  : Drive PLL frequency adjustment (Hz relative to nominal f0)
  AGCAdj  : Drive AGC sustaining force command
  CLQAdj  : Closed-Loop Quadrature (CLQ) nulling force command
  FTRAdj  : Force-to-Rebalance (FTR) Coriolis rebalance force command

Feedforward Rate & Integrated Angle (Bandwidth-Extended FTR):
  From the sense-axis slow envelope equation:
      2 * d(SX)/dt + gamma_y * SX = FTRAdj - 2 * Ag * DX * Omega_z
  Solving for instantaneous rate & integrated angle using FTRAdj and SX error-from-setpoint:
      Omega_FF(t) = [FTRAdj - gamma_y * e_SX - 2 * d(e_SX)/dt] / (2 * Ag * DX)
      Theta_FF(t) = int[(FTRAdj - gamma_y * e_SX) / (2 * Ag * DX)] dt - 2 * e_SX / (2 * Ag * DX)
"""

from __future__ import annotations
import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable
import numpy as np


DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


@dataclass(frozen=True)
class GyroParams:
    """Physical parameters for a 2-DOF Coriolis Vibratory Gyroscope."""
    name: str
    f0_hz: float              # Nominal resonant frequency at t0_c (Hz)
    q_x: float                # Drive-axis quality factor
    q_y: float                # Sense-axis quality factor
    mode_split_hz: float      # df = fy - fx (Hz); 0 = mode-matched
    angular_gain: float       # Bryan's factor Ag (DRG n=2 ~0.40, HRG n=2 ~0.28)
    quad_equiv_dps: float     # Fabrication stiffness quadrature expressed in deg/s
    anisodamp_bias_dps: float # Damping asymmetry rate bias in deg/s
    tcf_ppm_per_c: float      # Temp Coeff of Frequency (Si ~ -30 ppm/C, SiO2 ~ +80 ppm/C)
    arw_deg_sqrt_hr: float    # Angle Random Walk (deg / sqrt(hr))
    readout_noise_rms: float  # Electronic front-end noise (normalized disp / sqrt(Hz))
    t0_c: float = 25.0        # Reference temperature (deg C)

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

    @property
    def df_per_c_hz(self) -> float:
        return self.f0_hz * self.tcf_ppm_per_c * 1e-6


PRESETS: dict[str, GyroParams] = {
    "drg_11khz_q100k": GyroParams(
        name="Silicon MEMS DRG (11 kHz, Q=100k)",
        f0_hz=11_000.0,
        q_x=100_000.0,
        q_y=95_000.0,
        mode_split_hz=0.50,
        angular_gain=0.40,
        quad_equiv_dps=120.0,
        anisodamp_bias_dps=0.05,
        tcf_ppm_per_c=-30.0,      # Silicon Young's modulus TCF (~ -0.33 Hz/C)
        arw_deg_sqrt_hr=0.015,
        readout_noise_rms=1e-7,
    ),
    "hrg_micro_11khz_q1m": GyroParams(
        name="Fused-Silica Micro-HRG (11 kHz, Q=1M)",
        f0_hz=11_000.0,
        q_x=1_000_000.0,
        q_y=980_000.0,
        mode_split_hz=0.05,
        angular_gain=0.28,
        quad_equiv_dps=30.0,
        anisodamp_bias_dps=0.005,
        tcf_ppm_per_c=+80.0,      # Fused silica positive TCF (~ +0.88 Hz/C)
        arw_deg_sqrt_hr=0.002,
        readout_noise_rms=2e-8,
    ),
    "hrg_space_4khz_q5m": GyroParams(
        name="Macro Space-Grade HRG (4.2 kHz, Q=5M)",
        f0_hz=4_200.0,
        q_x=5_000_000.0,
        q_y=4_900_000.0,
        mode_split_hz=0.005,
        angular_gain=0.28,
        quad_equiv_dps=10.0,
        anisodamp_bias_dps=0.0005,
        tcf_ppm_per_c=+80.0,      # Fused silica positive TCF (~ +0.336 Hz/C)
        arw_deg_sqrt_hr=0.0002,
        readout_noise_rms=5e-9,
    ),
}


@dataclass
class ControllerGains:
    """PI gains for the four closed-loop CVG control loops."""
    pll_kp: float = 800.0
    pll_ki: float = 200_000.0
    agc_kp: float = 100.0
    agc_ki: float = 4_000.0
    ftr_kp: float = 120.0
    ftr_ki: float = 8_000.0
    clq_kp: float = 180.0
    clq_ki: float = 20_000.0
    demod_bw_hz: float = 500.0
    dx_setpoint: float = 1.0
    sx_setpoint: float = 0.0
    sy_setpoint: float = 0.0


@dataclass
class SimResult:
    """Decimated baseband telemetry matching ZUBoard 1CG Stage 1 streams."""
    t_s: np.ndarray
    # Demodulated Drive & Sense channels
    dx: np.ndarray
    dy: np.ndarray
    sx: np.ndarray
    sy: np.ndarray
    # Loop adjustments
    pll_adj_hz: np.ndarray
    agc_adj: np.ndarray
    clq_adj: np.ndarray
    ftr_adj: np.ndarray
    # Frequency & Temperature
    freq_true_hz: np.ndarray
    freq_pll_hz: np.ndarray
    temp_true_c: np.ndarray
    temp_est_c: np.ndarray
    # Rate & Integrated Angle (True vs Raw FTR vs Feedforward)
    omega_true_dps: np.ndarray
    omega_ftr_dps: np.ndarray
    omega_ff_dps: np.ndarray
    angle_true_deg: np.ndarray
    angle_ftr_deg: np.ndarray
    angle_ff_deg: np.ndarray
    # Calibrated quadrature readout (deg/s eq) & Lynch slow variables
    quad_cmd_dps: np.ndarray
    lynch_energy: np.ndarray
    lynch_quad: np.ndarray
    lynch_theta_deg: np.ndarray

    @property
    def omega_est_dps(self) -> np.ndarray:
        return self.omega_ftr_dps

    @property
    def drive_amp(self) -> np.ndarray:
        return np.hypot(self.dx, self.dy)

    @property
    def sense_amp(self) -> np.ndarray:
        return np.hypot(self.sx, self.sy)


def allan_deviation(rate_dps: np.ndarray, fs_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """Compute overlapping Allan Deviation (ADEV) in deg/hr vs cluster time tau (s)."""
    n = len(rate_dps)
    max_m = n // 4
    if max_m < 2:
        return np.array([]), np.array([])
    m_vals = np.unique(np.logspace(0, math.log10(max_m), num=40).astype(int))
    taus = []
    adevs = []
    theta = np.cumsum(rate_dps) / fs_hz
    for m in m_vals:
        tau = m / fs_hz
        diff2 = theta[2 * m :] - 2.0 * theta[m:-m] + theta[: -2 * m]
        avar = 0.5 * np.mean(diff2**2) / (tau**2)
        taus.append(tau)
        adevs.append(math.sqrt(max(avar, 0.0)) * 3600.0)
    return np.array(taus), np.array(adevs)


def simulate_cvg_closed_loop(
    params: GyroParams,
    duration_s: float = 0.4,
    fs_hz: float = 200_000.0,
    decim_hz: float = 2_000.0,
    gains: ControllerGains | None = None,
    omega_fn: Callable[[float], float] | None = None,
    temp_fn: Callable[[float], float] | None = None,
    df_drift_fn: Callable[[float], float] | None = None,
    enable_noise: bool = True,
    enable_ftr: bool = True,
    enable_clq: bool = True,
    seed: int = 42,
) -> SimResult:
    """Run carrier-level Strang-split 2-DOF CVG simulation with closed-loop control."""
    if gains is None:
        gains = ControllerGains()
    if omega_fn is None:
        omega_fn = lambda t: 100.0 if t >= 0.15 else 0.0
    if temp_fn is None:
        temp_fn = lambda t: params.t0_c + 5.0 * (t / max(duration_s, 1e-6))

    rng = np.random.default_rng(seed)
    dt = 1.0 / fs_hz
    n_steps = int(round(duration_s * fs_hz))
    decim_stride = max(1, int(round(fs_hz / decim_hz)))
    n_out = n_steps // decim_stride

    w0 = params.w0
    ag = params.angular_gain
    k_xy = 2.0 * ag * (params.quad_equiv_dps * DEG2RAD)
    g_xy = 2.0 * ag * (params.anisodamp_bias_dps * DEG2RAD)
    gamma_y_nom = w0 / params.q_y

    arw_rad_sqrt_s = (params.arw_deg_sqrt_hr * DEG2RAD) / 60.0
    u_noise_std = 2.0 * ag * arw_rad_sqrt_s * math.sqrt(fs_hz) if enable_noise else 0.0
    readout_std = params.readout_noise_rms * math.sqrt(fs_hz) if enable_noise else 0.0

    # 3-stage cascaded 1-pole LPF (models 3rd-order CIC/FIR decimation)
    a_lpf = 1.0 - math.exp(-2.0 * math.pi * gains.demod_bw_hz * dt)

    # Plant state (start near nominal drive amplitude)
    x = 0.0
    vx = 0.85 * gains.dx_setpoint
    y = 0.0
    vy = 0.0

    # NCO starts at nominal f0_hz; PLLAdj represents frequency offset from f0_hz
    f_nco_base = params.f0_hz
    phi = 0.0
    # Start slightly detuned (-5 Hz) to demonstrate autonomous PLL acquisition
    pll_int = -2.0 * math.pi * 5.0
    agc_int = (w0 / params.q_x) * gains.dx_setpoint
    ftr_int = 0.0
    clq_int = 0.0

    # 3-stage LPF demod states:
    #   DX = xs3 (in-phase drive amplitude), DY = xc3 (drive phase error -> 0)
    #   SX = ys3 (sense Coriolis/rate -> 0), SY = yc3 (sense quadrature -> 0)
    xc1 = xs1 = xc2 = xs2 = xc3 = xs3 = 0.0
    yc1 = ys1 = yc2 = ys2 = yc3 = ys3 = 0.0

    # Continuous running angle integrators (true, raw FTR, and feedforward)
    ang_true_deg = 0.0
    ang_ftr_deg = 0.0
    ang_ff_int_deg = 0.0  # integral part of feedforward angle
    sx_err_prev = 0.0
    # Smoothed d(SX)/dt for clean feedforward rate display (cutoff ~ 350 Hz)
    a_diff = 1.0 - math.exp(-2.0 * math.pi * 350.0 * dt)
    dsx_dt_filt = 0.0

    # Output telemetry buffers
    t_out = np.zeros(n_out)
    dx_out = np.zeros(n_out)
    dy_out = np.zeros(n_out)
    sx_out = np.zeros(n_out)
    sy_out = np.zeros(n_out)
    pll_adj_out = np.zeros(n_out)
    agc_adj_out = np.zeros(n_out)
    clq_adj_out = np.zeros(n_out)
    ftr_adj_out = np.zeros(n_out)
    freq_true_out = np.zeros(n_out)
    freq_pll_out = np.zeros(n_out)
    temp_true_out = np.zeros(n_out)
    temp_est_out = np.zeros(n_out)
    omega_true_out = np.zeros(n_out)
    omega_ftr_out = np.zeros(n_out)
    omega_ff_out = np.zeros(n_out)
    ang_true_out = np.zeros(n_out)
    ang_ftr_out = np.zeros(n_out)
    ang_ff_out = np.zeros(n_out)
    quad_cmd_out = np.zeros(n_out)
    lynch_e_out = np.zeros(n_out)
    lynch_q_out = np.zeros(n_out)
    lynch_th_out = np.zeros(n_out)

    noise_uy = rng.normal(0.0, u_noise_std, size=n_steps) if u_noise_std > 0 else None
    noise_rx = rng.normal(0.0, readout_std, size=n_steps) if readout_std > 0 else None
    noise_ry = rng.normal(0.0, readout_std, size=n_steps) if readout_std > 0 else None

    out_idx = 0
    two_pi = 2.0 * math.pi
    df_per_c = params.df_per_c_hz

    for k in range(n_steps):
        t = k * dt
        omega_dps = omega_fn(t)
        omega_rad = omega_dps * DEG2RAD

        temp_c = temp_fn(t)
        if df_drift_fn is not None:
            df_t = df_drift_fn(t)
            temp_c = params.t0_c + (df_t / df_per_c if abs(df_per_c) > 1e-12 else 0.0)
        else:
            df_t = df_per_c * (temp_c - params.t0_c)

        fx_curr = params.f0_hz + df_t
        fy_curr = fx_curr + params.mode_split_hz
        wx = two_pi * fx_curr
        wy = two_pi * fy_curr

        # 1. NCO Carrier
        c_phi = math.cos(phi)
        s_phi = math.sin(phi)

        # 2. Pickoff Readout + 3-Stage Lock-In Demodulation
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

        dx_val, dy_val = xs3, xc3
        sx_val, sy_val = ys3, yc3

        # 3. Drive-Axis PLL (drives DY -> 0) & AGC (drives DX -> dx_setpoint)
        amp_x = math.hypot(dx_val, dy_val)
        phase_err = math.atan2(dy_val, max(dx_val, 1e-6))
        pll_int += gains.pll_ki * phase_err * dt
        pll_int = max(-2000.0, min(2000.0, pll_int))
        pll_adj_rad = gains.pll_kp * phase_err + pll_int
        pll_adj_hz = pll_adj_rad / two_pi
        f_nco_eff = f_nco_base + pll_adj_hz

        amp_err = gains.dx_setpoint - dx_val
        agc_int += gains.agc_ki * amp_err * dt
        agc_int = max(-500.0, min(500.0, agc_int))
        agc_adj = max(-500.0, min(500.0, gains.agc_kp * amp_err + agc_int))
        u_x = agc_adj * c_phi

        # 4. Sense-Axis FTR (drives SX -> sx_setpoint) & CLQ (drives SY -> sy_setpoint)
        sx_err = sx_val - gains.sx_setpoint
        sy_err = sy_val - gains.sy_setpoint

        if enable_ftr:
            ftr_int += gains.ftr_ki * (-sx_err) * dt
            ftr_adj = gains.ftr_kp * (-sx_err) + ftr_int
        else:
            ftr_adj = 0.0

        if enable_clq:
            clq_int += gains.clq_ki * sy_err * dt
            clq_adj = gains.clq_kp * sy_err + clq_int
        else:
            clq_adj = 0.0

        u_y = ftr_adj * c_phi + clq_adj * s_phi
        if noise_uy is not None:
            u_y += noise_uy[k]

        # 5. Phase-Aligned FTR + Feedforward Rate & Feedforward Integrated Angle
        amp_denom = max(amp_x, 1e-3)
        cd = dx_val / amp_denom
        sd = dy_val / amp_denom
        ftr_aligned = ftr_adj * cd - clq_adj * sd
        clq_aligned = ftr_adj * sd + clq_adj * cd
        sx_err_aligned = sx_err * cd - sy_err * sd

        # Filtered derivative of SX error-from-setpoint for feedforward rate
        dsx_raw = (sx_err_aligned - sx_err_prev) * fs_hz
        dsx_dt_filt += a_diff * (dsx_raw - dsx_dt_filt)
        sx_err_prev = sx_err_aligned

        scale_inv_dps = RAD2DEG / (2.0 * ag * amp_denom)
        omega_ftr_dps = ftr_aligned * scale_inv_dps
        # Exact sense envelope inversion: 2*Ag*A*Omega = FTRAdj - gamma_y*e_SX - 2*d(e_SX)/dt
        omega_ff_dps = (ftr_aligned - gamma_y_nom * sx_err_aligned - 2.0 * dsx_dt_filt) * scale_inv_dps

        # Integrate angles at full carrier tick rate
        ang_true_deg += omega_dps * dt
        ang_ftr_deg += omega_ftr_dps * dt
        ang_ff_int_deg += (ftr_aligned - gamma_y_nom * sx_err_aligned) * scale_inv_dps * dt
        # Instantaneous feedforward angle = integral(FTR - damping*e_SX) - 2*e_SX/(2*Ag*A)
        ang_ff_deg = ang_ff_int_deg - 2.0 * sx_err_aligned * scale_inv_dps

        # 6. Strang-Split Exact-Rotation 2-DOF Resonator Step
        half_dt = 0.5 * dt
        cor = 2.0 * ag * omega_rad
        gx = wx / params.q_x
        gy = wy / params.q_y

        phi_next = phi + two_pi * f_nco_eff * dt
        if phi_next >= two_pi:
            phi_next -= two_pi
        c_phi_next = math.cos(phi_next)
        s_phi_next = math.sin(phi_next)

        u_x_next = agc_adj * c_phi_next
        u_y_next = ftr_adj * c_phi_next + clq_adj * s_phi_next
        if noise_uy is not None:
            u_y_next += noise_uy[k]

        ax = -gx * vx - k_xy * y - g_xy * vy + cor * vy + u_x
        ay = -gy * vy - k_xy * x - g_xy * vx - cor * vx + u_y
        vx_h = vx + half_dt * ax
        vy_h = vy + half_dt * ay

        tx = wx * dt
        ty = wy * dt
        cx_r, sx_r = math.cos(tx), math.sin(tx)
        cy_r, sy_r = math.cos(ty), math.sin(ty)

        x_new = x * cx_r + vx_h * sx_r
        vx_r = -x * sx_r + vx_h * cx_r
        y_new = y * cy_r + vy_h * sy_r
        vy_r = -y * sy_r + vy_h * cy_r

        ax2 = -gx * vx_r - k_xy * y_new - g_xy * vy_r + cor * vy_r + u_x_next
        ay2 = -gy * vy_r - k_xy * x_new - g_xy * vx_r - cor * vx_r + u_y_next
        vx = vx_r + half_dt * ax2
        vy = vy_r + half_dt * ay2
        x = x_new
        y = y_new
        phi = phi_next

        # 7. Decimated Baseband Telemetry
        if (k % decim_stride) == 0 and out_idx < n_out:
            temp_est_c = params.t0_c + (pll_adj_hz / df_per_c if abs(df_per_c) > 1e-12 else 0.0)

            lynch_e = dy_val * dy_val + dx_val * dx_val + sy_val * sy_val + sx_val * sx_val
            lynch_q = 2.0 * (dy_val * sx_val - dx_val * sy_val)
            lynch_r = (dy_val * dy_val + dx_val * dx_val) - (sy_val * sy_val + sx_val * sx_val)
            lynch_s = 2.0 * (dy_val * sy_val + dx_val * sx_val)
            lynch_theta = 0.5 * math.atan2(lynch_s, lynch_r)

            t_out[out_idx] = t
            dx_out[out_idx] = dx_val
            dy_out[out_idx] = dy_val
            sx_out[out_idx] = sx_val
            sy_out[out_idx] = sy_val
            pll_adj_out[out_idx] = pll_adj_hz
            agc_adj_out[out_idx] = agc_adj
            clq_adj_out[out_idx] = clq_adj
            ftr_adj_out[out_idx] = ftr_adj
            freq_true_out[out_idx] = fx_curr
            freq_pll_out[out_idx] = f_nco_eff
            temp_true_out[out_idx] = temp_c
            temp_est_out[out_idx] = temp_est_c
            omega_true_out[out_idx] = omega_dps
            omega_ftr_out[out_idx] = omega_ftr_dps
            omega_ff_out[out_idx] = omega_ff_dps
            ang_true_out[out_idx] = ang_true_deg
            ang_ftr_out[out_idx] = ang_ftr_deg
            ang_ff_out[out_idx] = ang_ff_deg
            quad_cmd_out[out_idx] = clq_aligned * scale_inv_dps
            lynch_e_out[out_idx] = lynch_e
            lynch_q_out[out_idx] = lynch_q
            lynch_th_out[out_idx] = lynch_theta * RAD2DEG
            out_idx += 1

    return SimResult(
        t_s=t_out,
        dx=dx_out,
        dy=dy_out,
        sx=sx_out,
        sy=sy_out,
        pll_adj_hz=pll_adj_out,
        agc_adj=agc_adj_out,
        clq_adj=clq_adj_out,
        ftr_adj=ftr_adj_out,
        freq_true_hz=freq_true_out,
        freq_pll_hz=freq_pll_out,
        temp_true_c=temp_true_out,
        temp_est_c=temp_est_out,
        omega_true_dps=omega_true_out,
        omega_ftr_dps=omega_ftr_out,
        omega_ff_dps=omega_ff_out,
        angle_true_deg=ang_true_out,
        angle_ftr_deg=ang_ftr_out,
        angle_ff_deg=ang_ff_out,
        quad_cmd_dps=quad_cmd_out,
        lynch_energy=lynch_e_out,
        lynch_quad=lynch_q_out,
        lynch_theta_deg=lynch_th_out,
    )


def plot_cvg_dashboard(res: SimResult, params: GyroParams, out_path: Path) -> None:
    """Save a 12-panel engineering dashboard covering all demod, loop, temp, and FF channels."""
    import matplotlib
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(4, 3, figsize=(16, 11), sharex=True)
    fig.suptitle(
        f"{params.name}  |  f0={params.f0_hz/1e3:.2f} kHz, Q={params.q_x:,.0f}, "
        f"df={params.mode_split_hz} Hz, Quad={params.quad_equiv_dps} dps, TCF={params.tcf_ppm_per_c:+.0f} ppm/°C",
        fontsize=12,
        fontweight="bold",
    )

    t_ms = res.t_s * 1e3

    # Row 0: Demodulated Drive & Sense (DX, DY, SX)
    axes[0, 0].plot(t_ms, res.dx, color="#1f77b4", label="DX (Drive Amp)")
    axes[0, 0].axhline(1.0, color="k", ls="--", lw=0.8, label="Setpoint")
    axes[0, 0].set_title("DX (Drive In-Phase)")
    axes[0, 0].set_ylabel("Norm")
    axes[0, 0].legend(fontsize=8)

    axes[0, 1].plot(t_ms, res.dy, color="#ff7f0e", label="DY (Phase Err)")
    axes[0, 1].axhline(0.0, color="k", ls="--", lw=0.8)
    axes[0, 1].set_title("DY (Drive Quadrature)")
    axes[0, 1].set_ylabel("Norm")

    axes[0, 2].plot(t_ms, res.sx, color="#2ca02c", label="SX (Rate Err)")
    axes[0, 2].axhline(0.0, color="k", ls="--", lw=0.8)
    axes[0, 2].set_title("SX (Sense Rate Error)")
    axes[0, 2].set_ylabel("Norm")

    # Row 1: SY, PLLAdj, AGCAdj
    axes[1, 0].plot(t_ms, res.sy, color="#d62728", label="SY (Quad Err)")
    axes[1, 0].axhline(0.0, color="k", ls="--", lw=0.8)
    axes[1, 0].set_title("SY (Sense Quadrature Error)")
    axes[1, 0].set_ylabel("Norm")

    axes[1, 1].plot(t_ms, res.freq_true_hz - params.f0_hz, "k--", lw=1.0, label="True Δf(T)")
    axes[1, 1].plot(t_ms, res.pll_adj_hz, color="#9467bd", label="PLLAdj")
    axes[1, 1].set_title("PLLAdj (Freq Offset from f0)")
    axes[1, 1].set_ylabel("Hz")
    axes[1, 1].legend(fontsize=8)

    axes[1, 2].plot(t_ms, res.agc_adj, color="#8c564b")
    axes[1, 2].set_title("AGCAdj (Drive Force Cmd)")
    axes[1, 2].set_ylabel("Norm Acc")

    # Row 2: CLQAdj, FTRAdj, Temperature
    axes[2, 0].plot(t_ms, res.clq_adj, color="#e377c2", label="CLQAdj")
    axes[2, 0].set_title("CLQAdj (Quadrature Null Cmd)")
    axes[2, 0].set_ylabel("Norm Acc")

    axes[2, 1].plot(t_ms, res.ftr_adj, color="#17becf", label="FTRAdj")
    axes[2, 1].set_title("FTRAdj (Force-to-Rebalance Cmd)")
    axes[2, 1].set_ylabel("Norm Acc")

    axes[2, 2].plot(t_ms, res.temp_true_c, "k--", lw=1.0, label="True Temp")
    axes[2, 2].plot(t_ms, res.temp_est_c, color="#bcbd22", label="Est Temp (from PLLAdj)")
    axes[2, 2].set_title("Temperature")
    axes[2, 2].set_ylabel("°C")
    axes[2, 2].legend(fontsize=8)

    # Row 3: Feedforward Rate, Feedforward Integrated Angle, Integrated Angle Error Comparison
    axes[3, 0].plot(t_ms, res.omega_true_dps, "k--", lw=1.2, label="True Rate")
    axes[3, 0].plot(t_ms, res.omega_ftr_dps, color="#17becf", alpha=0.7, label="FTR Rate (Raw)")
    axes[3, 0].plot(t_ms, res.omega_ff_dps, color="#1f77b4", lw=1.2, label="Feedforward Rate")
    axes[3, 0].set_title("Feedforward Rate")
    axes[3, 0].set_ylabel("deg/s")
    axes[3, 0].set_xlabel("Time (ms)")
    axes[3, 0].legend(fontsize=8)

    axes[3, 1].plot(t_ms, res.angle_true_deg, "k--", lw=1.2, label="True Angle")
    axes[3, 1].plot(t_ms, res.angle_ftr_deg, color="#17becf", alpha=0.7, label="FTR Int Angle")
    axes[3, 1].plot(t_ms, res.angle_ff_deg, color="#2ca02c", lw=1.2, label="Feedforward Int Angle")
    axes[3, 1].set_title("Feedforward Integrated Angle")
    axes[3, 1].set_ylabel("deg")
    axes[3, 1].set_xlabel("Time (ms)")
    axes[3, 1].legend(fontsize=8)

    # Zero-reference angle errors at t = 140 ms (after initial startup transient settles)
    ref_idx = int(np.argmin(np.abs(res.t_s - 0.14)))
    err_ftr_mdeg = ((res.angle_ftr_deg - res.angle_ftr_deg[ref_idx]) - (res.angle_true_deg - res.angle_true_deg[ref_idx])) * 1e3
    err_ff_mdeg = ((res.angle_ff_deg - res.angle_ff_deg[ref_idx]) - (res.angle_true_deg - res.angle_true_deg[ref_idx])) * 1e3
    mask_post = res.t_s >= 0.14
    axes[3, 2].plot(t_ms[mask_post], err_ftr_mdeg[mask_post], color="#d62728", alpha=0.8, label="Raw FTR Angle Err")
    axes[3, 2].plot(t_ms[mask_post], err_ff_mdeg[mask_post], color="#2ca02c", lw=1.4, label="FF Angle Err")
    axes[3, 2].axhline(0.0, color="k", ls="--", lw=0.8)
    axes[3, 2].set_title("Transient Angle Error (Post-Lock)")
    axes[3, 2].set_ylabel("mdeg")
    axes[3, 2].set_xlabel("Time (ms)")
    axes[3, 2].legend(fontsize=8)

    for ax in axes.flat:
        ax.grid(True, alpha=0.35)

    plt.tight_layout(rect=(0, 0, 1, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    print(f"  Saved plot -> {out_path}")

    if matplotlib.get_backend().lower() != "agg":
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 1 CVG Golden Model (DRG & HRG)")
    ap.add_argument("--preset", choices=list(PRESETS.keys()), default="drg_11khz_q100k")
    ap.add_argument("--duration", type=float, default=0.4, help="Simulation duration (s)")
    ap.add_argument("--fs", type=float, default=200_000.0, help="Carrier sample rate (Hz)")
    ap.add_argument("--rate-step", type=float, default=100.0, help="Angular rate step (deg/s)")
    ap.add_argument("--temp-delta", type=float, default=5.0, help="Temperature ramp delta (°C)")
    ap.add_argument("--save-plot", type=str, default="", help="Output PNG path")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    params = PRESETS[args.preset]
    print(f"Running {params.name}")
    print(f"  f0={params.f0_hz:.1f} Hz | Qx={params.q_x:,.0f} (tau={params.tau_x:.2f} s) | "
          f"df={params.mode_split_hz} Hz | Ag={params.angular_gain} | "
          f"Quad={params.quad_equiv_dps} dps | TCF={params.tcf_ppm_per_c:+.0f} ppm/°C")

    def omega_profile(t: float) -> float:
        if t < 0.15:
            return 0.0
        if t < 0.30:
            return args.rate_step
        return -0.5 * args.rate_step

    def temp_profile(t: float) -> float:
        return params.t0_c + args.temp_delta * (t / max(args.duration, 1e-6))

    res = simulate_cvg_closed_loop(
        params=params,
        duration_s=args.duration,
        fs_hz=args.fs,
        omega_fn=omega_profile,
        temp_fn=temp_profile,
    )

    m_step = (res.t_s >= 0.22) & (res.t_s <= 0.29)
    ref_idx = int(np.argmin(np.abs(res.t_s - 0.14)))
    m_post = res.t_s >= 0.14
    peak_ftr_ang_err = float(np.max(np.abs((res.angle_ftr_deg[m_post] - res.angle_ftr_deg[ref_idx]) - (res.angle_true_deg[m_post] - res.angle_true_deg[ref_idx])))) * 1e3
    peak_ff_ang_err = float(np.max(np.abs((res.angle_ff_deg[m_post] - res.angle_ff_deg[ref_idx]) - (res.angle_true_deg[m_post] - res.angle_true_deg[ref_idx])))) * 1e3

    print(f"  Step [0.22..0.29s]: true={args.rate_step:.2f} dps | FTR={np.mean(res.omega_ftr_dps[m_step]):.3f} dps | "
          f"FF={np.mean(res.omega_ff_dps[m_step]):.3f} dps")
    print(f"  Peak post-lock angle error: Raw FTR = {peak_ftr_ang_err:.2f} mdeg  ->  Feedforward = {peak_ff_ang_err:.2f} mdeg "
          f"({peak_ftr_ang_err / max(peak_ff_ang_err, 1e-6):.1f}x reduction!)")
    print(f"  PLL err: {np.mean(np.abs(res.freq_pll_hz[m_step] - res.freq_true_hz[m_step])):.4f} Hz | "
          f"CLQ quad: {np.mean(res.quad_cmd_dps[m_step]):.2f}/{params.quad_equiv_dps:.2f} dps")

    if not args.no_plot:
        out_path = Path(args.save_plot) if args.save_plot else Path(__file__).resolve().parent / "plots" / f"{args.preset}.png"
        plot_cvg_dashboard(res, params, out_path)


if __name__ == "__main__":
    main()
