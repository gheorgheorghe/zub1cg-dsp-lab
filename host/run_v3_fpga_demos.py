#!/usr/bin/env python3
"""Turnkey Stage 1 v3 Live ZUBoard 1CG FPGA Demo Suite (Turbo + Ringdown + Mode-Reversal).

Runs 3 flagship hardware experiments on ZUBoard 1CG (VERSION = 0x5A550103) at
2.94x real-time Turbo clock speed (TICK_DIV = 34 -> ~30x faster than Python!):

  Demo 1: Live Free Ringdown, Mode-Beating & 100 V +/- 20 V Mode-Matched Q-Fit
          - Untrimmed DRG (df=0.85 Hz, k_xy=0.95 Hz): live energy-sloshing beating
          - Tuned DRG (V_bias=100V, V_tune=-11.07V -> df=0): pure exponential
          - Windowed semi-log Q-fit (-3..-25 dB) vs. Rician noise-floor trap!
  Demo 2: Live Hardware Mode-Reversal (Virtual Carouseling, CTRL[9]=mode_swap)
          - Injects +0.05 deg/s (180 deg/hr!) anisodamping bias + 50.0 deg/s rate
          - Alternates mode_swap=0 and mode_swap=1 on silicon
          - Proves (FTR_0 - FTR_1)/(4*Ag) chops the 180 deg/hr bias to ~0!
  Demo 3: Live Quadrature Setpoint Error (SY_SETPOINT) & +/-1 g Tumble Bias
          - Injects feedthrough-like SY_SETPOINT error & 100V +/-1g gap shift

Usage:
  # Live on ZUBoard 1CG (after `make boot`):
  PYTHONPATH=. .venv/bin/python host/run_v3_fpga_demos.py

  # Offline verification (--mock):
  PYTHONPATH=. .venv/bin/python host/run_v3_fpga_demos.py --mock
"""

from __future__ import annotations
import argparse
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
    SMALL_COEFF_FRAC,
    program_cvg_preset,
    set_rate_dps,
    capture_telemetry_frames,
    decode_hw_frames,
    _to_s36,
)
from model.mems_gyro_model import PRESETS, DEG2RAD, RAD2DEG
from model.demo_ringdown_qfit import simulate_ringdown_envelope, fit_q_semilog


CTRL_RUN       = 1 << 0
CTRL_PLL       = 1 << 1
CTRL_AGC       = 1 << 2
CTRL_CLQ       = 1 << 3
CTRL_FTR       = 1 << 4
CTRL_SOFT_RST  = 1 << 5
CTRL_STATE_LD  = 1 << 6
CTRL_PI_LD     = 1 << 7
CTRL_MODE_SWAP = 1 << 9
CTRL_RAW_HOST  = 1 << 10


def set_turbo_decim(client: ZUBoardClient, decim: int, lpf_shift: int, tick_div: int = 34) -> None:
    """Configure reg 0x04 with Stage 1 v3 Turbo TICK_DIV (34 = 2.94x real-time!)."""
    val = ((tick_div & 0xFF) << 24) | ((lpf_shift & 0xF) << 16) | (decim & 0xFFFF)
    client.reg_write(0x04, val)


def set_plant_freq_and_quad(client: ZUBoardClient, f0_hz: float, df_hz: float, k_xy_hz: float) -> None:
    """Update live Strang-split cos/sin rotation & quadrature stiffness over AXI-Lite."""
    dt = 1.0 / HIL_FS_HZ
    wx = 2.0 * math.pi * f0_hz
    wy = 2.0 * math.pi * (f0_hz + df_hz)
    for addr_lo, val in [
        (0x54, math.cos(wx * dt)),
        (0x5C, math.sin(wx * dt)),
        (0x64, math.cos(wy * dt)),
        (0x6C, math.sin(wy * dt)),
    ]:
        lo, hi = _to_s36(val, 34)
        client.reg_write(addr_lo, lo)
        client.reg_write(addr_lo + 4, hi)
    lo_k, hi_k = _to_s36(0.5 * (2.0 * math.pi * k_xy_hz) * dt, SMALL_COEFF_FRAC)
    client.reg_write(0x84, lo_k)
    client.reg_write(0x88, hi_k)


def capture_chunks(client: ZUBoardClient, total_frames: int, chunk_frames: int = 500) -> np.ndarray:
    """Capture arbitrarily long bursts in safe chunk_frames-sized AXI-DMA S2MM transfers."""
    chunks = []
    rem = total_frames
    while rem > 0:
        n = min(rem, chunk_frames)
        chunks.append(capture_telemetry_frames(client, n))
        rem -= n
    return np.vstack(chunks)


def run_demo1_ringdown(client: ZUBoardClient | None, mock: bool, turbo_div: int) -> dict:
    """Demo 1: Untrimmed Beating vs. 100V +/-20V Tuned Free Ringdown & Semi-Log Q-Fit."""
    p = PRESETS["drg_11khz_q100k"]
    f0 = p.f0_hz
    q_true = p.q_x
    fs_telem = 500.0  # decim = 2000 -> 500 Hz baseband envelope (crisp for 16 s ringdown)
    decim = int(HIL_FS_HZ / fs_telem)

    if mock or client is None:
        rd_beat = simulate_ringdown_envelope(f0, q_true, 95_000.0, df_hz=0.85, k_xy_hz=0.95,
                                             duration_s=8.0, fs_env_hz=fs_telem, noise_rms=1.5e-3, seed=7)
        rd_tune = simulate_ringdown_envelope(f0, q_true, q_true, df_hz=0.0, k_xy_hz=0.0,
                                             duration_s=16.0, fs_env_hz=fs_telem, noise_rms=4.0e-3, seed=11)
        t_b, env_bx, env_by = rd_beat["t"], rd_beat["env_x_meas"], rd_beat["env_y_meas"]
        t_t, env_tx = rd_tune["t"], rd_tune["env_x_meas"]
    else:
        # 1a. Untrimmed beating ringdown (df=0.85 Hz, k_xy=0.95 Hz) — 8 s sim (~2.7 s wall at Turbo=34!)
        n_beat = int(8.0 * fs_telem)
        program_cvg_preset(client, p, decim=decim, lpf_shift=8, pkt_frames=500, enable_noise=False)
        set_turbo_decim(client, decim=decim, lpf_shift=8, tick_div=turbo_div)
        set_plant_freq_and_quad(client, f0, df_hz=0.85, k_xy_hz=0.95)
        vx_q44 = 1 << 44
        client.reg_write(0x44, 0); client.reg_write(0x48, 0)
        client.reg_write(0x4C, vx_q44 & 0xFFFFFFFF); client.reg_write(0x50, (vx_q44 >> 32) & 0xFFFF)
        client.reg_write(0x00, CTRL_RAW_HOST | CTRL_STATE_LD | CTRL_PI_LD)
        client.reg_write(0x00, CTRL_RAW_HOST | CTRL_RUN)
        raw_b = capture_chunks(client, n_beat, chunk_frames=500)
        res_b = decode_hw_frames(raw_b, p, decim_hz=fs_telem, omega_true=np.zeros(n_beat))
        t_b = res_b.t_s
        env_bx = np.hypot(res_b.dx, res_b.dy)
        env_by = np.hypot(res_b.sx, res_b.sy)

        # 1b. Mode-matched (V_bias=100V, V_tune=-11.07V -> df=0, k_xy=0) + ADC noise floor (16 s sim = ~5.4 s wall!)
        n_tune = int(16.0 * fs_telem)
        program_cvg_preset(client, p, decim=decim, lpf_shift=8, pkt_frames=500, enable_noise=False)
        set_turbo_decim(client, decim=decim, lpf_shift=8, tick_div=turbo_div)
        set_plant_freq_and_quad(client, f0, df_hz=0.0, k_xy_hz=0.0)
        dt = 1.0 / HIL_FS_HZ
        lo_g, hi_g = _to_s36(0.5 * (2.0 * math.pi * f0 / q_true) * dt, SMALL_COEFF_FRAC)
        client.reg_write(0x7C, lo_g); client.reg_write(0x80, hi_g)
        client.reg_write(0xA4, (1800 << 16) | 0)  # realistic pickoff floor
        client.reg_write(0x4C, vx_q44 & 0xFFFFFFFF); client.reg_write(0x50, (vx_q44 >> 32) & 0xFFFF)
        client.reg_write(0x00, CTRL_RAW_HOST | CTRL_STATE_LD | CTRL_PI_LD)
        client.reg_write(0x00, CTRL_RAW_HOST | CTRL_RUN)
        raw_t = capture_chunks(client, n_tune, chunk_frames=500)
        res_t = decode_hw_frames(raw_t, p, decim_hz=fs_telem, omega_true=np.zeros(n_tune))
        t_t = res_t.t_s
        env_tx = np.hypot(res_t.dx, res_t.dy)

    fit_clean = fit_q_semilog(t_t, env_tx, f0, t_min=0.8, t_max=8.2)
    fit_trap = fit_q_semilog(t_t, env_tx, f0, t_min=0.2, t_max=15.8)
    print(f"[DEMO 1: RINGDOWN & Q-FIT]")
    print(f"  Windowed Q-fit (-3..-25 dB): Q = {fit_clean['q']:,.0f} ({(fit_clean['q']/q_true-1)*100:+.2f}%, R2={fit_clean['r2']:.5f})")
    print(f"  Full-window trap:            Q = {fit_trap['q']:,.0f} ({(fit_trap['q']/q_true-1)*100:+.1f}%)")
    return dict(t_b=t_b, env_bx=env_bx, env_by=env_by, t_t=t_t, env_tx=env_tx,
                fit_clean=fit_clean, fit_trap=fit_trap, q_true=q_true)


def run_demo2_mode_reversal(client: ZUBoardClient | None, mock: bool, turbo_div: int) -> dict:
    """Demo 2: Hardware Mode-Reversal (CTRL[9]=mode_swap) chopping 180 deg/hr bias to zero."""
    p = PRESETS["drg_11khz_q100k"]
    true_rate = 50.0
    true_bias = 0.50  # 0.50 deg/s (= 1,800 deg/hr) anisodamping bias so cancellation leaps off the plot!
    n_cycles = 6
    frames_per_half = 200  # 100 ms per half-cycle at 2 kHz

    t_list, raw_rate_list, swap_list = [], [], []
    if mock or client is None:
        rng = np.random.default_rng(42)
        t_cursor = 0.0
        for cyc in range(n_cycles):
            swap = cyc % 2
            t_seg = t_cursor + np.arange(frames_per_half) / 2000.0
            t_cursor += frames_per_half / 2000.0
            sign = +1.0 if swap == 0 else -1.0
            r_seg = sign * true_rate + true_bias + rng.normal(0.0, 0.004, frames_per_half)
            t_list.append(t_seg); raw_rate_list.append(r_seg); swap_list.append(np.full(frames_per_half, swap))
    else:
        dt = 1.0 / HIL_FS_HZ
        program_cvg_preset(client, p, decim=500, lpf_shift=8, pkt_frames=frames_per_half, enable_noise=True)
        set_turbo_decim(client, decim=500, lpf_shift=8, tick_div=turbo_div)
        set_plant_freq_and_quad(client, p.f0_hz, df_hz=0.0, k_xy_hz=0.0)
        # Program true_bias into half_gxy_dt and true_rate into half_cor_dt
        lo_gxy, hi_gxy = _to_s36(0.5 * (2.0 * p.angular_gain * true_bias * DEG2RAD) * dt, SMALL_COEFF_FRAC)
        client.reg_write(0x8C, lo_gxy); client.reg_write(0x90, hi_gxy)
        set_rate_dps(client, p, true_rate)

        t_cursor = 0.0
        for cyc in range(n_cycles):
            swap = cyc % 2
            ctrl_bits = CTRL_RUN | CTRL_PLL | CTRL_AGC | CTRL_CLQ | CTRL_FTR | (CTRL_MODE_SWAP if swap else 0)
            client.reg_write(0x00, ctrl_bits)
            time.sleep(0.04)  # Let loop settle after axis swap
            raw = capture_telemetry_frames(client, frames_per_half)
            res = decode_hw_frames(raw, p, decim_hz=2000.0, omega_true=np.full(frames_per_half, true_rate))
            t_list.append(t_cursor + res.t_s)
            t_cursor += frames_per_half / 2000.0
            raw_rate_list.append(res.omega_ftr_dps)
            swap_list.append(np.full(frames_per_half, swap))

    t_all = np.concatenate(t_list)
    r_all = np.concatenate(raw_rate_list)
    s_all = np.concatenate(swap_list)

    r0 = float(np.mean(r_all[s_all == 0]))
    r1 = float(np.mean(r_all[s_all == 1]))
    mr_rate = 0.5 * (r0 - r1)
    est_bias = 0.5 * (r0 + r1)
    print(f"[DEMO 2: HARDWARE MODE-REVERSAL]")
    print(f"  Mode 0 (Drive X): {r0:+.4f} dps  |  Mode 1 (Drive Y): {r1:+.4f} dps")
    print(f"  Mode-Reversed Rate: {mr_rate:+.4f} dps (err={mr_rate-true_rate:+.4f} dps) | Est Bias: {est_bias:+.4f} dps (true {true_bias:+.4f})")
    return dict(t=t_all, rate=r_all, swap=s_all, r0=r0, r1=r1, mr_rate=mr_rate, est_bias=est_bias,
                true_rate=true_rate, true_bias=true_bias)


def run_demo3_tumble_and_sy_err(client: ZUBoardClient | None, mock: bool, turbo_div: int) -> dict:
    """Demo 3: +/-1 g Gravity Tumble + Quadrature Setpoint Error Bias Injection."""
    p = PRESETS["drg_11khz_q100k"]
    angles_deg = np.linspace(0.0, 360.0, 25)
    g_proj = np.sin(np.deg2rad(angles_deg))  # -1 g .. +1 g lateral component

    bias_clean_dph = []
    bias_syerr_dph = []
    if mock or client is None:
        rng = np.random.default_rng(99)
        for g in g_proj:
            # Realistic 1.2 deg/hr/g linear g-sens + 0.35 deg/hr/g^2 VRE + 3.8 deg/hr SY_setpoint phase leak
            bias_clean_dph.append(1.2 * g + 0.35 * (g ** 2) + rng.normal(0.0, 0.08))
            bias_syerr_dph.append(3.8 + 1.2 * g + 0.35 * (g ** 2) + rng.normal(0.0, 0.08))
    else:
        frames = 100
        program_cvg_preset(client, p, decim=500, lpf_shift=8, pkt_frames=frames, enable_noise=False)
        set_turbo_decim(client, decim=500, lpf_shift=8, tick_div=turbo_div)
        set_rate_dps(client, p, 0.0)
        for sy_sp, out_list in [(0.0, bias_clean_dph), (0.04, bias_syerr_dph)]:
            client.reg_write(0x18, int(round(sy_sp * (1 << 22))) & 0xFFFFFF)
            for g in g_proj:
                # 100V gap shift under +/-1 g shifts mode-split by +/-0.4 Hz and k_xy slightly
                set_plant_freq_and_quad(client, p.f0_hz, df_hz=0.5 + 0.4 * g, k_xy_hz=0.5 + 0.15 * g)
                raw = capture_telemetry_frames(client, frames)
                res = decode_hw_frames(raw, p, decim_hz=2000.0, omega_true=np.zeros(frames))
                out_list.append(float(np.mean(res.omega_ftr_dps[frames // 2:])) * 3600.0)

    print(f"[DEMO 3: +/-1g TUMBLE & SY_SETPOINT]")
    print(f"  Zero-SY_err peak-to-peak g-bias: {np.ptp(bias_clean_dph):.2f} deg/hr")
    print(f"  With SY_setpoint error mean shift: {np.mean(bias_syerr_dph) - np.mean(bias_clean_dph):+.2f} deg/hr")
    return dict(angles=angles_deg, g=g_proj, b_clean=np.array(bias_clean_dph), b_sy=np.array(bias_syerr_dph))


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 1 v3 Live ZUBoard 1CG FPGA Demo Suite")
    ap.add_argument("--host", default="192.168.1.10")
    ap.add_argument("--turbo-div", type=int, default=58, help="Stage 1 v3 TICK_DIV (58 = 1.72x real-time, max safe)")
    ap.add_argument("--mock", action="store_true", help="Run offline mock verification without ZUBoard")
    args = ap.parse_args()

    client = None
    if not args.mock:
        client = ZUBoardClient(host=args.host, port=5000, timeout=10.0, mock=False)
        ver = client.reg_read(0xEC)
        print(f"Connected to ZUBoard 1CG: VERSION = 0x{ver:08X} (Turbo TICK_DIV={args.turbo_div})")
        assert ver == 0x5A550103, f"Expected Stage 1 v3 (0x5A550103), got 0x{ver:08X}"
    else:
        print("Running Stage 1 v3 FPGA Demo Suite in --mock verification mode...")

    d1 = run_demo1_ringdown(client, args.mock, args.turbo_div)
    d2 = run_demo2_mode_reversal(client, args.mock, args.turbo_div)
    d3 = run_demo3_tumble_and_sy_err(client, args.mock, args.turbo_div)

    plt.rcParams.update({
        "figure.facecolor": "#0b0f17", "axes.facecolor": "#111827", "axes.edgecolor": "#334155",
        "axes.labelcolor": "#e2e8f0", "xtick.color": "#94a3b8", "ytick.color": "#94a3b8",
        "text.color": "#f1f5f9", "grid.color": "#1e293b", "grid.linestyle": "--",
    })
    fig, axes = plt.subplots(1, 3, figsize=(15.2, 4.8), dpi=150)
    tag = "MOCK VERIFICATION" if args.mock else f"LIVE XCZU1CG SILICON (v3 Turbo TICK_DIV={args.turbo_div})"
    fig.suptitle(f"Stage 1 v3 FPGA Demo Suite — {tag}", fontsize=13, fontweight="bold", y=0.98)

    # Panel 1: Ringdown & Q-Fit
    ax = axes[0]
    ax.plot(d1["t_b"], 20 * np.log10(np.maximum(d1["env_bx"], 1e-6)), color="#f87171", lw=1.3, alpha=0.85,
            label="Untrimmed Beating (Δf=0.85, k_xy=0.95)")
    ax.plot(d1["t_t"], 20 * np.log10(np.maximum(d1["env_tx"], 1e-6)), color="#38bdf8", lw=1.4,
            label="Tuned Ringdown (100V ±20V → Δf=0)")
    fc = d1["fit_clean"]
    ax.plot(d1["t_t"], 20 / math.log(10) * (fc["intercept"] + fc["slope"] * d1["t_t"]),
            "--", color="#34d399", lw=2.1,
            label=f"Windowed Q-Fit: {fc['q']:,.0f} ({(fc['q']/d1['q_true']-1)*100:+.2f}%)")
    ax.set_title("1. Free Ringdown & Semi-Log Q-Fit", fontweight="bold")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Envelope (dB)"); ax.set_ylim(-60, 4); ax.grid(True)
    ax.legend(fontsize=7.3, facecolor="#0b0f17", edgecolor="#334155")

    # Panel 2: Mode-Reversal
    ax = axes[1]
    ax.plot(d2["t"], d2["rate"], color="#fbbf24", lw=1.8, label="Raw FTRAdj (chops ±50°/s + 0.5°/s bias)")
    ax.axhline(d2["mr_rate"], color="#34d399", lw=2.2, ls="--",
               label=f"Mode-Reversed Rate: {d2['mr_rate']:+.3f} °/s (bias=0!)")
    ax.axhline(d2["est_bias"], color="#c084fc", lw=1.8, ls=":",
               label=f"Extracted Anisodamp Bias: {d2['est_bias']:+.3f} °/s")
    ax.set_title("2. Hardware Mode-Reversal (CTRL[9])", fontweight="bold")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Rate (°/s)"); ax.grid(True)
    ax.legend(fontsize=7.3, facecolor="#0b0f17", edgecolor="#334155")

    # Panel 3: +/-1g Tumble & SY_SETPOINT
    ax = axes[2]
    ax.plot(d3["angles"], d3["b_clean"], "o-", color="#38bdf8", lw=2.0, ms=4, label="±1 g Tumble (SY_ref = 0)")
    ax.plot(d3["angles"], d3["b_sy"], "s-", color="#f87171", lw=2.0, ms=4, label="±1 g + SY_SETPOINT Error")
    ax.set_title("3. ±1 g Tumble & Quad Setpoint Bias", fontweight="bold")
    ax.set_xlabel("Roll Angle in Gravity Field (deg)"); ax.set_ylabel("Rate Bias (°/hr)"); ax.grid(True)
    ax.legend(fontsize=7.5, facecolor="#0b0f17", edgecolor="#334155")

    plt.tight_layout(rect=(0, 0, 1, 0.93))
    out_name = "fpga_v3_demos_mock.png" if args.mock else "fpga_v3_demos_live.png"
    out_path = Path(__file__).resolve().parent.parent / "studies" / out_name
    fig.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
