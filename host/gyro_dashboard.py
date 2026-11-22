#!/usr/bin/env python3
"""Host ZUBoard 1CG Stage 1 Controller & 12-Panel Live CVG Dashboard.

Programs any calibrated DRG / HRG preset into `mems_hil_axis_wrap.sv` over GbE
UDP, injects angular rate steps, captures 8-word AXI-DMA telemetry bursts
(DX, DY, SX, SY, PLLAdj, AGCAdj, CLQAdj, FTRAdj), computes Feedforward Rate &
Feedforward Integrated Angle, and plots the 3-column 12-panel dashboard.
"""

from __future__ import annotations
import argparse
import math
from pathlib import Path
import struct
import time
import numpy as np

from host.zuboard_client import ZUBoardClient, CMD_DMA_CAP, SAMPLES_PER_PKT
from model.mems_gyro_model import PRESETS, GyroParams, SimResult, plot_cvg_dashboard, DEG2RAD, RAD2DEG


HIL_FS_HZ = 1_000_000.0  # 100 MHz pl_clk0 / TICK_DIV=100


def _to_s36(val: float, frac: int) -> tuple[int, int]:
    q = int(round(val * (1 << frac))) & ((1 << 36) - 1)
    return q & 0xFFFFFFFF, (q >> 32) & 0xF


def _to_s24(val: float, frac: int) -> int:
    return int(round(val * (1 << frac))) & 0xFFFFFF


def program_cvg_preset(
    client: ZUBoardClient,
    params: GyroParams,
    decim: int = 500,
    lpf_shift: int = 8,
    pkt_frames: int = 400,
) -> None:
    """Write full DRG/HRG plant & closed-loop controller parameters over AXI-Lite."""
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

    # Preload initial drive oscillation vx = 0.95 and nominal AGC drive
    vx_q44 = int(round(0.95 * (1 << 44)))
    client.reg_write(0x44, 0)
    client.reg_write(0x48, 0)
    client.reg_write(0x4C, vx_q44 & 0xFFFFFFFF)
    client.reg_write(0x50, (vx_q44 >> 32) & 0xFFFF)
    client.reg_write(0x3C, 0)
    client.reg_write(0x40, _to_s24(w0 / params.q_x, 14))

    # Release reset, pulse state_load + pi_load, then enable closed-loop HIL
    client.reg_write(0x00, 0xDE)
    client.reg_write(0x00, 0x1F)


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

    # PLLAdj LSB = 16 / 2^32 * HIL_FS_HZ Hz
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


def main() -> None:
    ap = argparse.ArgumentParser(description="ZUBoard 1CG Stage 1 Live CVG Dashboard")
    ap.add_argument("--host", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--preset", choices=list(PRESETS.keys()), default="drg_11khz_q100k")
    ap.add_argument("--rate-step", type=float, default=100.0)
    ap.add_argument("--frames", type=int, default=400, help="Frames per burst @ 2 kHz (400 = 200 ms)")
    ap.add_argument("--out", default="", help="Output PNG path")
    args = ap.parse_args()

    params = PRESETS[args.preset]
    client = ZUBoardClient(host=args.host, port=args.port, mock=False)
    ver = client.reg_read(0xEC)
    print(f"Connected to ZUBoard 1CG Stage 1: VERSION = 0x{ver:08X}")

    program_cvg_preset(client, params, decim=500, lpf_shift=8, pkt_frames=args.frames)
    time.sleep(0.15)  # Let PLL/AGC/CLQ settle

    # Capture 200 ms at 0 dps, then inject rate step and capture 200 ms at +rate_step
    f0 = capture_telemetry_frames(client, args.frames)
    set_rate_dps(client, params, args.rate_step)
    f1 = capture_telemetry_frames(client, args.frames)

    raw = np.vstack([f0, f1])
    omega_true = np.concatenate([np.zeros(args.frames), np.full(args.frames, args.rate_step)])
    res = decode_hw_frames(raw, params, decim_hz=2000.0, omega_true=omega_true)

    out_path = Path(args.out) if args.out else Path("model/plots") / f"hw_{args.preset}.png"
    plot_cvg_dashboard(res, params, out_path)


if __name__ == "__main__":
    main()
