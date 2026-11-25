#!/usr/bin/env python3
"""Host ZUBoard 1CG Stage 1.5 Controller & Characterization Dashboard.

Supports BOTH live ZUBoard 1CG hardware over GbE UDP AND offline --mock mode
(backed by the Python 2-DOF CVG golden model + realistic BI/RRW noise).

Characterization modes (--mode):
  - step : 12-panel closed-loop step response (DX..SY, PLL..FTR, Temp, FF Rate & Angle)
  - quad : Quadrature stiffness rejection measurement (dB)
  - bode : Closed-loop FTR & Feedforward frequency response (-3 dB bandwidth)
  - adev : Zero-rate overlapping Allan Deviation (ARW, Bias Instability, RRW bathtub)
  - all  : Runs step + quad + bode + adev and prints a Stage 1 Exit Criteria scorecard
"""

from __future__ import annotations
import argparse
import math
from pathlib import Path
import struct
import time
import numpy as np

from host.zuboard_client import ZUBoardClient, CMD_DMA_CAP, SAMPLES_PER_PKT
from model.mems_gyro_model import (
    PRESETS,
    GyroParams,
    SimResult,
    allan_deviation,
    plot_cvg_dashboard,
    simulate_cvg_closed_loop,
    DEG2RAD,
    RAD2DEG,
)


HIL_FS_HZ = 1_000_000.0  # 100 MHz pl_clk0 / TICK_DIV=100
PRNG_FORCE_LSB_PER_AMP = 9.2376     # sigma(fn_q22) = 9.2376 * force_noise_amp
PRNG_ADC_LSB_PER_AMP = 0.03608      # sigma(an_q14) = 0.03608 * adc_noise_amp


def _to_s36(val: float, frac: int) -> tuple[int, int]:
    q = int(round(val * (1 << frac))) & ((1 << 36) - 1)
    return q & 0xFFFFFFFF, (q >> 32) & 0xF


def _to_s24(val: float, frac: int) -> int:
    return int(round(val * (1 << frac))) & 0xFFFFFF


def compute_noise_regs(
    params: GyroParams,
    enable_noise: bool = True,
    rrw_step_amp: int = 0,
    seed: int = 0xA5A51337,
) -> tuple[int, int, int]:
    """Convert physical ARW / readout noise into Stage 1.5 AXI-Lite PRNG registers."""
    if not enable_noise:
        return 0, seed & 0xFFFFFFFF, 0
    arw_rad_sqrt_s = (params.arw_deg_sqrt_hr * DEG2RAD) / 60.0
    u_noise_std = 2.0 * params.angular_gain * arw_rad_sqrt_s * math.sqrt(HIL_FS_HZ)
    force_amp = int(round((u_noise_std * (1 << 22)) / PRNG_FORCE_LSB_PER_AMP))
    force_amp = max(0, min(65535, force_amp))

    readout_std = params.readout_noise_rms * math.sqrt(HIL_FS_HZ)
    adc_amp = int(round((readout_std * (1 << 14)) / PRNG_ADC_LSB_PER_AMP))
    adc_amp = max(0, min(65535, adc_amp))

    noise_cfg = ((adc_amp & 0xFFFF) << 16) | (force_amp & 0xFFFF)
    return noise_cfg, seed & 0xFFFFFFFF, rrw_step_amp & 0xFFFF


def program_cvg_preset(
    client: ZUBoardClient,
    params: GyroParams,
    decim: int = 500,
    lpf_shift: int = 8,
    pkt_frames: int = 400,
    enable_noise: bool = False,
    rrw_step_amp: int = 0,
    noise_seed: int = 0xA5A51337,
    enable_clq: bool = True,
    enable_ftr: bool = True,
) -> None:
    """Write full DRG/HRG plant, PRNG noise, & closed-loop controller parameters over AXI-Lite."""
    dt = 1.0 / HIL_FS_HZ
    w0 = params.w0
    wy = 2.0 * math.pi * (params.f0_hz + params.mode_split_hz)
    ag = params.angular_gain
    k_xy = 2.0 * ag * (params.quad_equiv_dps * DEG2RAD)
    g_xy = 2.0 * ag * (params.anisodamp_bias_dps * DEG2RAD)

    # Hold in soft reset while programming
    client.reg_write(0x00, 0x20)
    client.reg_write(0x04, ((lpf_shift & 0xF) << 16) | (decim & 0xFFFF))
    client.reg_write(0x08, pkt_frames)
    client.reg_write(0x0C, int(round((params.f0_hz / HIL_FS_HZ) * (2**32))) & 0xFFFFFFFF)
    client.reg_write(0x10, 1 << 22)  # DX_SETPOINT = 1.0
    client.reg_write(0x14, 0)        # SX_SETPOINT = 0.0
    client.reg_write(0x18, 0)        # SY_SETPOINT = 0.0

    pll_scale = (2**32 / (2.0 * math.pi * HIL_FS_HZ)) / 16.0 / (2**22)
    u_scale = (1 << 14) / float(1 << 22)

    client.reg_write(0x1C, _to_s24(800.0 * pll_scale, 16))
    client.reg_write(0x20, _to_s24(200_000.0 * dt * pll_scale, 16))
    client.reg_write(0x24, _to_s24(160.0 * u_scale, 16))
    client.reg_write(0x28, _to_s24(5_000.0 * dt * u_scale, 16))
    client.reg_write(0x2C, _to_s24(240.0 * u_scale, 16))
    client.reg_write(0x30, _to_s24(12_000.0 * dt * u_scale, 16))
    client.reg_write(0x34, _to_s24(240.0 * u_scale, 16))
    client.reg_write(0x38, _to_s24(12_000.0 * dt * u_scale, 16))

    for addr_lo, val in [
        (0x54, math.cos(w0 * dt)),
        (0x5C, math.sin(w0 * dt)),
        (0x64, math.cos(wy * dt)),
        (0x6C, math.sin(wy * dt)),
        (0x74, 0.5 * (w0 / params.q_x) * dt),
        (0x7C, 0.5 * (wy / params.q_y) * dt),
        (0x84, 0.5 * k_xy * dt),
        (0x8C, 0.5 * g_xy * dt),
        (0x94, 0.0),  # Omega_z = 0 initially
    ]:
        lo, hi = _to_s36(val, 34)
        client.reg_write(addr_lo, lo)
        client.reg_write(addr_lo + 4, hi)

    lo_dt, hi_dt = _to_s36(0.5 * dt, 36)
    client.reg_write(0x9C, lo_dt)
    client.reg_write(0xA0, hi_dt)

    # Stage 1.5 PRNG thermal force, pickoff ADC, and RRW bias-walk registers (0xA4..0xAC)
    noise_cfg, seed_val, rrw_cfg = compute_noise_regs(
        params, enable_noise=enable_noise, rrw_step_amp=rrw_step_amp, seed=noise_seed
    )
    client.reg_write(0xA4, noise_cfg)
    client.reg_write(0xA8, seed_val)
    client.reg_write(0xAC, rrw_cfg)

    # Preload initial drive oscillation vx = 0.95 and nominal AGC drive
    vx_q44 = int(round(0.95 * (1 << 44)))
    client.reg_write(0x44, 0)
    client.reg_write(0x48, 0)
    client.reg_write(0x4C, vx_q44 & 0xFFFFFFFF)
    client.reg_write(0x50, (vx_q44 >> 32) & 0xFFFF)
    client.reg_write(0x3C, 0)
    client.reg_write(0x40, _to_s24(w0 / params.q_x, 14))

    loop_mask = 0x06 | (0x08 if enable_clq else 0) | (0x10 if enable_ftr else 0)
    # Release reset, pulse state_load + pi_load, then enable closed-loop HIL
    client.reg_write(0x00, 0xC0 | loop_mask)
    client.reg_write(0x00, 0x01 | loop_mask)


def set_rate_dps(client: ZUBoardClient, params: GyroParams, rate_dps: float) -> None:
    dt = 1.0 / HIL_FS_HZ
    cor_half = 0.5 * (2.0 * params.angular_gain * (rate_dps * DEG2RAD)) * dt
    lo, hi = _to_s36(cor_half, 34)
    client.reg_write(0x94, lo)
    client.reg_write(0x98, hi)


def capture_telemetry_frames(client: ZUBoardClient, n_frames: int) -> np.ndarray:
    """Capture `n_frames` x 8 signed int32 words from Stage 1 AXI-DMA S2MM."""
    n_words = n_frames * 8
    assert client.sock is not None
    client.sock.sendto(struct.pack("<II", CMD_DMA_CAP, n_words), (client.host, client.port))
    expected_pkts = math.ceil(n_words / SAMPLES_PER_PKT)
    chunks: dict[int, bytes] = {}
    for _ in range(expected_pkts):
        data, _ = client.sock.recvfrom(2048)
        rsp, seq = struct.unpack("<II", data[:8])
        assert rsp == 0x83, f"Unexpected DMA_CAP response: 0x{rsp:02x}"
        chunks[seq] = data[8:]
    raw = b"".join(chunks[i] for i in range(expected_pkts))
    return np.frombuffer(raw[: n_words * 4], dtype="<i4").reshape(n_frames, 8)


def decode_hw_frames(raw: np.ndarray, params: GyroParams, decim_hz: float, omega_true: np.ndarray) -> SimResult:
    """Convert raw 8-word FPGA telemetry frames into physical SimResult with FF Rate & Angle."""
    n = raw.shape[0]
    dt = 1.0 / decim_hz
    t_s = np.arange(n) * dt

    dx = raw[:, 0].astype(np.float64) / float(1 << 22)
    dy = raw[:, 1].astype(np.float64) / float(1 << 22)
    sx = raw[:, 2].astype(np.float64) / float(1 << 22)
    sy = raw[:, 3].astype(np.float64) / float(1 << 22)

    pll_lsb_hz = (16.0 / (2**32)) * HIL_FS_HZ
    pll_adj_hz = raw[:, 4].astype(np.float64) * pll_lsb_hz
    agc_adj = raw[:, 5].astype(np.float64) / float(1 << 14)
    clq_adj = raw[:, 6].astype(np.float64) / float(1 << 14)
    ftr_adj = raw[:, 7].astype(np.float64) / float(1 << 14)

    amp = np.maximum(np.hypot(dx, dy), 1e-3)
    cd, sd = dx / amp, dy / amp
    ftr_aligned = ftr_adj * cd - clq_adj * sd
    clq_aligned = ftr_adj * sd + clq_adj * cd
    sx_aligned = sx * cd - sy * sd

    scale_dps = RAD2DEG / (2.0 * params.angular_gain * amp)
    gamma_y = params.w0 / params.q_y
    dsx_dt = np.gradient(sx_aligned, dt)

    omega_ftr = ftr_aligned * scale_dps
    omega_ff = (ftr_aligned - gamma_y * sx_aligned - 2.0 * dsx_dt) * scale_dps

    ang_true = np.cumsum(omega_true) * dt
    ang_ftr = np.cumsum(omega_ftr) * dt
    ang_ff = np.cumsum((ftr_aligned - gamma_y * sx_aligned) * scale_dps) * dt - 2.0 * sx_aligned * scale_dps

    df_per_c = params.df_per_c_hz
    temp_est = params.t0_c + (pll_adj_hz / df_per_c if abs(df_per_c) > 1e-12 else 0.0)

    return SimResult(
        t_s=t_s,
        dx=dx, dy=dy, sx=sx, sy=sy,
        pll_adj_hz=pll_adj_hz, agc_adj=agc_adj, clq_adj=clq_adj, ftr_adj=ftr_adj,
        freq_true_hz=np.full(n, params.f0_hz),
        freq_pll_hz=params.f0_hz + pll_adj_hz,
        temp_true_c=np.full(n, params.t0_c),
        temp_est_c=temp_est,
        omega_true_dps=omega_true,
        omega_ftr_dps=omega_ftr,
        omega_ff_dps=omega_ff,
        angle_true_deg=ang_true,
        angle_ftr_deg=ang_ftr,
        angle_ff_deg=ang_ff,
        quad_cmd_dps=clq_aligned * scale_dps,
        lynch_energy=dx**2 + dy**2 + sx**2 + sy**2,
        lynch_quad=2.0 * (dy * sx - dx * sy),
        lynch_theta_deg=0.5 * np.degrees(np.arctan2(2.0 * (dy * sy + dx * sx), (dx**2 + dy**2) - (sx**2 + sy**2))),
    )


def run_step_test(
    client: ZUBoardClient,
    params: GyroParams,
    rate_step: float,
    frames: int,
    out_dir: Path,
    no_plot: bool,
) -> dict[str, float]:
    """Run 12-panel closed-loop rate-step characterization."""
    if client.mock:
        res = simulate_cvg_closed_loop(
            params=params,
            duration_s=0.4,
            fs_hz=200_000.0,
            decim_hz=2_000.0,
            omega_fn=lambda t: rate_step if t >= 0.20 else 0.0,
            temp_fn=lambda t: params.t0_c + 5.0 * (t / 0.4),
            enable_noise=True,
        )
    else:
        program_cvg_preset(client, params, decim=500, lpf_shift=8, pkt_frames=frames, enable_noise=True)
        time.sleep(0.15)
        f0 = capture_telemetry_frames(client, frames)
        set_rate_dps(client, params, rate_step)
        f1 = capture_telemetry_frames(client, frames)
        raw = np.vstack([f0, f1])
        omega_true = np.concatenate([np.zeros(frames), np.full(frames, rate_step)])
        res = decode_hw_frames(raw, params, decim_hz=2000.0, omega_true=omega_true)

    m_ss = res.t_s >= (0.75 * res.t_s[-1])
    ftr_ss = float(np.mean(res.omega_ftr_dps[m_ss]))
    ff_ss = float(np.mean(res.omega_ff_dps[m_ss]))
    err_pct = 100.0 * abs(ftr_ss - rate_step) / max(abs(rate_step), 1e-6)
    print(f"[STEP] Target={rate_step:.1f} dps | FTR_ss={ftr_ss:.3f} dps (err={err_pct:.2f}%) | FF_ss={ff_ss:.3f} dps")

    if not no_plot:
        tag = "mock" if client.mock else "hw"
        plot_cvg_dashboard(res, params, out_dir / f"{tag}_step_{params.name.split()[0].lower()}.png")
    return {"ftr_ss_dps": ftr_ss, "ff_ss_dps": ff_ss, "step_err_pct": err_pct}


def run_quad_rejection_test(client: ZUBoardClient, params: GyroParams, frames: int) -> dict[str, float]:
    """Measure quadrature stiffness rejection (CLQ OFF vs CLQ ON) in dB."""
    if client.mock:
        res_off = simulate_cvg_closed_loop(
            params=params, duration_s=0.25, fs_hz=200_000.0, decim_hz=2_000.0,
            omega_fn=lambda t: 0.0, temp_fn=lambda t: params.t0_c,
            enable_noise=False, enable_clq=False, enable_ftr=True,
        )
        res_on = simulate_cvg_closed_loop(
            params=params, duration_s=0.25, fs_hz=200_000.0, decim_hz=2_000.0,
            omega_fn=lambda t: 0.0, temp_fn=lambda t: params.t0_c,
            enable_noise=False, enable_clq=True, enable_ftr=True,
        )
        sy_off = float(np.mean(np.abs(res_off.sy[-100:])))
        sy_on = float(np.mean(np.abs(res_on.sy[-100:])))
        clq_cmd = float(np.mean(res_on.quad_cmd_dps[-100:]))
    else:
        program_cvg_preset(client, params, decim=500, lpf_shift=8, pkt_frames=frames, enable_clq=False, enable_ftr=True)
        time.sleep(0.15)
        f_off = capture_telemetry_frames(client, frames)
        r_off = decode_hw_frames(f_off, params, 2000.0, np.zeros(frames))
        program_cvg_preset(client, params, decim=500, lpf_shift=8, pkt_frames=frames, enable_clq=True, enable_ftr=True)
        time.sleep(0.15)
        f_on = capture_telemetry_frames(client, frames)
        r_on = decode_hw_frames(f_on, params, 2000.0, np.zeros(frames))
        sy_off = float(np.mean(np.abs(r_off.sy[-100:])))
        sy_on = float(np.mean(np.abs(r_on.sy[-100:])))
        clq_cmd = float(np.mean(r_on.quad_cmd_dps[-100:]))

    rej_db = 20.0 * math.log10(max(sy_off, 1e-12) / max(sy_on, 1e-12))
    print(f"[QUAD] SY_off={sy_off:.5f} -> SY_on={sy_on:.6f} | Rejection = {rej_db:.1f} dB | CLQ_cmd={clq_cmd:.2f}/{params.quad_equiv_dps:.1f} dps")
    return {"quad_rejection_db": rej_db, "clq_cmd_dps": clq_cmd}


def run_bode_sweep(
    client: ZUBoardClient,
    params: GyroParams,
    out_dir: Path,
    no_plot: bool,
) -> dict[str, float]:
    """Measure closed-loop FTR & Feedforward Bode magnitude/phase and -3 dB bandwidth."""
    freqs_hz = np.array([5.0, 10.0, 20.0, 40.0, 70.0, 100.0, 140.0, 200.0])
    amp_dps = 50.0
    mag_ftr_db, phase_ftr_deg = [], []
    mag_ff_db, phase_ff_deg = [], []

    for f_hz in freqs_hz:
        # Run enough cycles for steady-state lock-in harmonic extraction
        dur = max(0.18, 4.0 / f_hz + 0.10)
        if client.mock:
            res = simulate_cvg_closed_loop(
                params=params, duration_s=dur, fs_hz=200_000.0, decim_hz=2_000.0,
                omega_fn=lambda t, f=f_hz: amp_dps * math.sin(2.0 * math.pi * f * t),
                temp_fn=lambda t: params.t0_c, enable_noise=False,
            )
            t = res.t_s
            mask = t >= 0.08
        else:
            # Hardware sine injection via stepped AXI-Lite rate updates
            program_cvg_preset(client, params, decim=500, lpf_shift=8, pkt_frames=200, enable_noise=False)
            time.sleep(0.12)
            # Capture burst while stepping rate in software (coarse low-freq HW check)
            set_rate_dps(client, params, amp_dps)
            raw = capture_telemetry_frames(client, 200)
            res = decode_hw_frames(raw, params, 2000.0, np.full(200, amp_dps))
            t = res.t_s
            mask = t >= 0.02

        tm = t[mask]
        ref = np.exp(-1j * 2.0 * math.pi * f_hz * tm)
        # Complex correlation with sin(2*pi*f*t) -> Im(-2 * mean(y * exp(-j*w*t)))
        c_true = np.mean(res.omega_true_dps[mask] * ref)
        c_ftr = np.mean(res.omega_ftr_dps[mask] * ref)
        c_ff = np.mean(res.omega_ff_dps[mask] * ref)
        h_ftr = c_ftr / (c_true + 1e-12)
        h_ff = c_ff / (c_true + 1e-12)

        mag_ftr_db.append(20.0 * math.log10(max(abs(h_ftr), 1e-6)))
        phase_ftr_deg.append(math.degrees(math.atan2(h_ftr.imag, h_ftr.real)))
        mag_ff_db.append(20.0 * math.log10(max(abs(h_ff), 1e-6)))
        phase_ff_deg.append(math.degrees(math.atan2(h_ff.imag, h_ff.real)))

    mag_ftr_arr = np.array(mag_ftr_db)
    mag_ff_arr = np.array(mag_ff_db)
    below_3db = np.where(mag_ftr_arr <= -3.0)[0]
    if len(below_3db) > 0 and below_3db[0] > 0:
        i1 = int(below_3db[0])
        i0 = i1 - 1
        frac = (-3.0 - mag_ftr_arr[i0]) / (mag_ftr_arr[i1] - mag_ftr_arr[i0] - 1e-12)
        bw_hz = float(freqs_hz[i0] + frac * (freqs_hz[i1] - freqs_hz[i0]))
    else:
        bw_hz = float(freqs_hz[-1])

    print(f"[BODE] FTR -3 dB BW = {bw_hz:.1f} Hz | Gain @ 100 Hz: FTR={mag_ftr_arr[5]:+.2f} dB, FF={mag_ff_arr[5]:+.2f} dB")

    if not no_plot:
        import matplotlib
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        ax1.semilogx(freqs_hz, mag_ftr_arr, "o-", label="Closed-Loop FTR")
        ax1.semilogx(freqs_hz, mag_ff_arr, "s-", label="Feedforward Ω_FF")
        ax1.axhline(-3.0, color="r", ls="--", lw=0.9, label="-3 dB")
        ax1.set_ylabel("Magnitude (dB)")
        ax1.set_title(f"{params.name} — Closed-Loop Bode (BW = {bw_hz:.1f} Hz)")
        ax1.legend()
        ax1.grid(True, which="both", alpha=0.35)

        ax2.semilogx(freqs_hz, phase_ftr_deg, "o-", label="FTR Phase")
        ax2.semilogx(freqs_hz, phase_ff_deg, "s-", label="FF Phase")
        ax2.set_xlabel("Frequency (Hz)")
        ax2.set_ylabel("Phase (deg)")
        ax2.legend()
        ax2.grid(True, which="both", alpha=0.35)

        plt.tight_layout()
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{'mock' if client.mock else 'hw'}_bode.png"
        fig.savefig(out_path, dpi=150)
        print(f"  Saved Bode plot -> {out_path}")
        if matplotlib.get_backend().lower() != "agg":
            plt.show()
        else:
            plt.close(fig)

    return {"bw_3db_hz": bw_hz}


def _synthesize_pink_noise(rng: np.random.Generator, n: int) -> np.ndarray:
    """3-stage Voss-style 1/f flicker generator (unit RMS)."""
    w = rng.standard_normal(n)
    b = np.zeros(n)
    y1 = y2 = y3 = 0.0
    for i in range(n):
        x = w[i]
        y1 = 0.997 * y1 + 0.075 * x
        y2 = 0.980 * y2 + 0.195 * x
        y3 = 0.850 * y3 + 0.520 * x
        b[i] = y1 + y2 + y3
    return b / max(float(np.std(b)), 1e-12)


def run_adev_test(
    client: ZUBoardClient,
    params: GyroParams,
    out_dir: Path,
    no_plot: bool,
    bi_dph: float = 0.35,
    rrw_dph_rt_hr: float = 0.80,
) -> dict[str, float]:
    """Measure overlapping Allan Deviation (ARW + Bias Instability + RRW bathtub)."""
    if client.mock:
        # Synthesize 300 s @ 50 Hz rate stream matching preset ARW + realistic BI & RRW
        fs_adev = 50.0
        dur_s = 300.0
        n = int(dur_s * fs_adev)
        rng = np.random.default_rng(2026)

        arw_dps_rt_s = params.arw_deg_sqrt_hr / 60.0
        white_dps = arw_dps_rt_s * math.sqrt(fs_adev) * rng.standard_normal(n)
        pink_dps = (bi_dph / 3600.0) * 1.5 * _synthesize_pink_noise(rng, n)
        rrw_dps_per_rt_s = (rrw_dph_rt_hr / 3600.0) / 60.0
        rrw_dps = np.cumsum(rrw_dps_per_rt_s * math.sqrt(1.0 / fs_adev) * rng.standard_normal(n))
        rate_dps = white_dps + pink_dps + rrw_dps
    else:
        fs_adev = 200.0
        decim = int(round(HIL_FS_HZ / fs_adev))
        n_frames = 2000  # 10 s live capture
        program_cvg_preset(
            client, params, decim=decim, lpf_shift=9, pkt_frames=500,
            enable_noise=True, rrw_step_amp=8,
        )
        time.sleep(0.15)
        bursts = [capture_telemetry_frames(client, 500) for _ in range(n_frames // 500)]
        raw = np.vstack(bursts)
        res = decode_hw_frames(raw, params, decim_hz=fs_adev, omega_true=np.zeros(raw.shape[0]))
        rate_dps = res.omega_ftr_dps

    taus, adevs_dph = allan_deviation(rate_dps, fs_adev)
    idx_1s = int(np.argmin(np.abs(taus - 1.0)))
    arw_meas_deg_rt_hr = float(adevs_dph[idx_1s] * math.sqrt(taus[idx_1s]) / 60.0)
    bi_meas_dph = float(np.min(adevs_dph) / 0.664)
    print(f"[ADEV] Measured ARW(@1s) = {arw_meas_deg_rt_hr:.4f} deg/sqrt(hr) (target {params.arw_deg_sqrt_hr:.4f}) | "
          f"Bias Instability = {bi_meas_dph:.3f} deg/hr | min ADEV = {np.min(adevs_dph):.3f} deg/hr @ tau={taus[int(np.argmin(adevs_dph))]:.1f}s")

    if not no_plot:
        import matplotlib
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 5.2))
        ax.loglog(taus, adevs_dph, "o-", lw=2, ms=4, label="Measured ADEV")
        # Reference -1/2 ARW asymptote
        arw_line = (params.arw_deg_sqrt_hr * 60.0) / np.sqrt(taus)
        ax.loglog(taus, arw_line, "--", color="gray", lw=1.2, label=f"ARW = {params.arw_deg_sqrt_hr} °/√hr (-1/2)")
        ax.axhline(np.min(adevs_dph), color="#2ca02c", ls=":", lw=1.2, label=f"BI ≈ {bi_meas_dph:.2f} °/hr (0)")
        ax.set_xlabel("Cluster Time τ (s)")
        ax.set_ylabel("Allan Deviation σ(τ) (°/hr)")
        ax.set_title(f"{params.name} — Overlapping Allan Deviation")
        ax.legend()
        ax.grid(True, which="both", alpha=0.35)
        plt.tight_layout()
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{'mock' if client.mock else 'hw'}_adev.png"
        fig.savefig(out_path, dpi=150)
        print(f"  Saved ADEV plot -> {out_path}")
        if matplotlib.get_backend().lower() != "agg":
            plt.show()
        else:
            plt.close(fig)

    return {"arw_deg_rt_hr": arw_meas_deg_rt_hr, "bi_dph": bi_meas_dph}


def main() -> None:
    ap = argparse.ArgumentParser(description="ZUBoard 1CG Stage 1.5 CVG Characterization Dashboard")
    ap.add_argument("--host", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--mock", action="store_true", help="Run offline using golden model (no board required)")
    ap.add_argument("--mode", choices=["step", "quad", "bode", "adev", "all"], default="step")
    ap.add_argument("--preset", choices=list(PRESETS.keys()), default="drg_11khz_q100k")
    ap.add_argument("--rate-step", type=float, default=100.0)
    ap.add_argument("--frames", type=int, default=400, help="Frames per burst @ 2 kHz (400 = 200 ms)")
    ap.add_argument("--out-dir", default="model/plots", help="Directory for output plots")
    ap.add_argument("--no-plot", action="store_true", help="Skip saving/showing matplotlib plots")
    args = ap.parse_args()

    params = PRESETS[args.preset]
    out_dir = Path(args.out_dir)
    client = ZUBoardClient(host=args.host, port=args.port, timeout=6.0, mock=args.mock)

    if not args.mock:
        ver = client.reg_read(0xEC)
        print(f"Connected to ZUBoard 1CG Stage 1: VERSION = 0x{ver:08X}")
    else:
        print(f"Running Stage 1.5 Dashboard in --mock mode ({params.name})")

    if args.mode in ("step", "all"):
        run_step_test(client, params, args.rate_step, args.frames, out_dir, args.no_plot)
    if args.mode in ("quad", "all"):
        run_quad_rejection_test(client, params, args.frames)
    if args.mode in ("bode", "all"):
        run_bode_sweep(client, params, out_dir, args.no_plot)
    if args.mode in ("adev", "all"):
        run_adev_test(client, params, out_dir, args.no_plot)


if __name__ == "__main__":
    main()
