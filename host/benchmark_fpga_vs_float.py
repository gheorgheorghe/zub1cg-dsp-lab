#!/usr/bin/env python3
"""Live ZUBoard 1CG Fixed-Point RTL vs 64-bit Float64 Golden Model Benchmark.

Measures exact fixed-point rounding/quantization error on physical XCZU1CG silicon
across all 3 calibrated CVG presets (DRG Q=100k, Micro-HRG Q=1M, Space HRG Q=5M),
plus a clean 30 s live hardware ADEV comparing PRNG-ON vs pure fixed-point (PRNG-OFF).
"""

from __future__ import annotations
import json
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
    program_cvg_preset,
    set_rate_dps,
    capture_telemetry_frames,
    decode_hw_frames,
)
from model.mems_gyro_model import (
    PRESETS,
    GyroParams,
    DEG2RAD,
    RAD2DEG,
    allan_deviation,
    simulate_cvg_closed_loop,
)


def coeff_quant_report(p: GyroParams) -> dict[str, float]:
    """Compute exact quantization error of Stage 1 v2 High-Q Q-6.42 plant & Q10.22 output registers."""
    dt = 1.0 / HIL_FS_HZ
    w0 = p.w0
    gx = 0.5 * (w0 / p.q_x) * dt
    gy = 0.5 * (w0 / p.q_y) * dt
    gxy = 0.5 * (2.0 * p.angular_gain * (p.anisodamp_bias_dps * DEG2RAD)) * dt
    cor_1dps = 0.5 * (2.0 * p.angular_gain * (1.0 * DEG2RAD)) * dt
    q42 = float(1 << 42)

    gx_lsb = gx * q42
    gx_err_pct = abs(round(gx_lsb) - gx_lsb) / max(gx_lsb, 1e-12) * 100.0
    gxy_lsb = gxy * q42
    gxy_err_pct = abs(round(gxy_lsb) - gxy_lsb) / max(gxy_lsb, 1e-12) * 100.0

    # 1 LSB of Q10.22 FTRAdj mapped to deg/s and deg/hr
    ftr_lsb_dps = (1.0 / float(1 << 22)) * RAD2DEG / (2.0 * p.angular_gain)
    ftr_lsb_dph = ftr_lsb_dps * 3600.0
    # 1 LSB of Q2.22 demod SX and Q2.22 Delta-Sigma pickoff ADC
    sx_lsb = 1.0 / float(1 << 22)
    adc_lsb = 1.0 / float(1 << 22)

    return {
        "gamma_x_q42_lsbs": gx_lsb,
        "gamma_x_quant_err_pct": gx_err_pct,
        "anisodamp_q42_lsbs": gxy_lsb,
        "anisodamp_quant_err_pct": gxy_err_pct,
        "cor_1dps_q42_lsbs": cor_1dps * q42,
        "ftr_q22_lsb_dps": ftr_lsb_dps,
        "ftr_q22_lsb_dph": ftr_lsb_dph,
        "sx_q22_lsb": sx_lsb,
        "adc_q22_lsb": adc_lsb,
    }


def main() -> None:
    out_dir = Path("studies")
    out_dir.mkdir(parents=True, exist_ok=True)
    client = ZUBoardClient(host="192.168.1.10", port=5000, timeout=8.0, mock=False)
    ver = client.reg_read(0xEC)
    print(f"Connected to ZUBoard 1CG: VERSION = 0x{ver:08X}")

    preset_keys = ["drg_11khz_q100k", "hrg_micro_11khz_q1m", "hrg_space_4khz_q5m"]
    results: dict[str, dict] = {}
    traces: dict[str, dict] = {}

    rate_step = 100.0
    frames = 400  # 200 ms @ 2 kHz per segment

    for key in preset_keys:
        p = PRESETS[key]
        qrep = coeff_quant_report(p)
        print(f"\n=== Preset: {p.name} ({key}) ===")
        print(f"  Q-6.42 gamma_x = {qrep['gamma_x_q42_lsbs']:.1f} LSBs (err {qrep['gamma_x_quant_err_pct']:.4f}%) | "
              f"anisodamp g_xy = {qrep['anisodamp_q42_lsbs']:.2f} LSBs | "
              f"Q10.22 FTR 1 LSB = {qrep['ftr_q22_lsb_dps']:.7f} dps ({qrep['ftr_q22_lsb_dph']:.3f} dph)")

        # 1. Noise-free Live FPGA 0 -> 100 dps step
        program_cvg_preset(client, p, decim=500, lpf_shift=8, pkt_frames=frames, enable_noise=False)
        time.sleep(0.25)  # allow clean lock before capturing zero-rate baseline
        f0 = capture_telemetry_frames(client, frames)
        set_rate_dps(client, p, rate_step)
        f1 = capture_telemetry_frames(client, frames)
        raw_hw = np.vstack([f0, f1])
        omega_true = np.concatenate([np.zeros(frames), np.full(frames, rate_step)])
        res_hw = decode_hw_frames(raw_hw, p, decim_hz=2000.0, omega_true=omega_true)

        # 2. Noise-free 64-bit float64 Python Golden Model (matching exact 0.2 s step)
        res_py = simulate_cvg_closed_loop(
            params=p,
            duration_s=0.40,
            fs_hz=200_000.0,
            decim_hz=2_000.0,
            omega_fn=lambda t: rate_step if t >= 0.20 else 0.0,
            temp_fn=lambda t: p.t0_c,
            enable_noise=False,
        )

        # Align lengths (800 samples)
        n = min(len(res_hw.t_s), len(res_py.t_s))
        t_ms = res_hw.t_s[:n] * 1e3
        hw_ftr = res_hw.omega_ftr_dps[:n]
        py_ftr = res_py.omega_ftr_dps[:n]
        hw_ff = res_hw.omega_ff_dps[:n]
        py_ff = res_py.omega_ff_dps[:n]

        # Zero-rate region: samples 100..380 (settled at 0 dps)
        z_hw = hw_ftr[100:380]
        z_py = py_ftr[100:380]
        zero_bias_hw_dps = float(np.mean(z_hw))
        zero_bias_py_dps = float(np.mean(z_py))
        zero_rms_hw_dps = float(np.std(z_hw))
        zero_p2p_hw_dps = float(np.ptp(z_hw))

        # Steady-state 100 dps region: last 150 samples (325..400 ms)
        ss_hw = float(np.mean(hw_ftr[650:n]))
        ss_py = float(np.mean(py_ftr[650:n]))
        sf_err_hw_pct = (ss_hw - rate_step) / rate_step * 100.0
        sf_err_py_pct = (ss_py - rate_step) / rate_step * 100.0
        diff_ss_dps = ss_hw - ss_py
        diff_ss_ppm = (diff_ss_dps / rate_step) * 1e6

        # Post-step transient RMS diff (200..400 ms)
        step_rms_diff_dps = float(np.sqrt(np.mean((hw_ftr[400:n] - py_ftr[400:n]) ** 2)))
        sx_peak_hw = float(np.max(np.abs(res_hw.sx[400:n])))
        sx_peak_py = float(np.max(np.abs(res_py.sx[400:n])))

        # 3. Live Hardware Quadrature Rejection
        program_cvg_preset(client, p, decim=500, lpf_shift=8, pkt_frames=frames, enable_clq=False, enable_noise=False)
        time.sleep(0.20)
        q_off = decode_hw_frames(capture_telemetry_frames(client, frames), p, 2000.0, np.zeros(frames))
        program_cvg_preset(client, p, decim=500, lpf_shift=8, pkt_frames=frames, enable_clq=True, enable_noise=False)
        time.sleep(0.20)
        q_on = decode_hw_frames(capture_telemetry_frames(client, frames), p, 2000.0, np.zeros(frames))
        sy_off = float(np.mean(np.abs(q_off.sy[-100:])))
        sy_on = float(np.mean(np.abs(q_on.sy[-100:])))
        quad_rej_db = 20.0 * math.log10(max(sy_off, 1e-12) / max(sy_on, 1e-12))

        print(f"  Zero-rate (0 dps, no noise): HW mean={zero_bias_hw_dps:+.5f} dps ({zero_bias_hw_dps*3600:+.1f} dph), "
              f"RMS={zero_rms_hw_dps:.5f} dps ({zero_rms_hw_dps*3600:.2f} dph), p2p={zero_p2p_hw_dps:.5f} dps | "
              f"Float64 mean={zero_bias_py_dps:+.5f} dps")
        print(f"  100 dps SS: HW={ss_hw:.4f} dps ({sf_err_hw_pct:+.3f}%) vs Float64={ss_py:.4f} dps ({sf_err_py_pct:+.3f}%) | "
              f"delta={diff_ss_dps:+.4f} dps ({diff_ss_ppm:+.0f} ppm)")
        print(f"  Transient SX peak: HW={sx_peak_hw:.5f} vs Float64={sx_peak_py:.5f} | Quad Rejection={quad_rej_db:.1f} dB")

        results[key] = {
            "name": p.name,
            "q": p.q_x,
            "f0_hz": p.f0_hz,
            "zero_bias_hw_dps": zero_bias_hw_dps,
            "zero_bias_py_dps": zero_bias_py_dps,
            "zero_rms_hw_dps": zero_rms_hw_dps,
            "zero_p2p_hw_dps": zero_p2p_hw_dps,
            "ss_hw_dps": ss_hw,
            "ss_py_dps": ss_py,
            "sf_err_hw_pct": sf_err_hw_pct,
            "sf_err_py_pct": sf_err_py_pct,
            "diff_ss_dps": diff_ss_dps,
            "diff_ss_ppm": diff_ss_ppm,
            "step_rms_diff_dps": step_rms_diff_dps,
            "sx_peak_hw": sx_peak_hw,
            "sx_peak_py": sx_peak_py,
            "quad_rej_db": quad_rej_db,
            **qrep,
        }
        traces[key] = {
            "t_ms": t_ms,
            "hw_ftr": hw_ftr,
            "py_ftr": py_ftr,
            "hw_ff": hw_ff,
            "py_ff": py_ff,
            "z_hw": z_hw,
        }

    # 4. Clean 30 s Live Hardware ADEV on Silicon DRG: PRNG-OFF (pure fixed-point) vs PRNG-ON
    print("\n=== Clean 30 s Live Hardware ADEV (Silicon DRG: Pure Fixed-Point vs PRNG Thermal+RRW) ===")
    p_drg = PRESETS["drg_11khz_q100k"]
    fs_adev = 200.0
    decim_adev = int(round(HIL_FS_HZ / fs_adev))  # 5000 -> 200 Hz
    burst_frames = 500  # 2.5 s per burst
    n_bursts = 12       # 30.0 s total

    adev_data = {}
    for label, en_noise, rrw_amp in [("fixed_point_only", False, 0), ("prng_thermal_rrw", True, 2048)]:
        program_cvg_preset(
            client, p_drg, decim=decim_adev, lpf_shift=9, pkt_frames=burst_frames,
            enable_noise=en_noise, rrw_step_amp=rrw_amp,
        )
        time.sleep(0.35)
        # Discard 1 warmup burst (2.5 s) so initial lock transient is completely gone!
        _ = capture_telemetry_frames(client, burst_frames)
        bursts = [capture_telemetry_frames(client, burst_frames) for _ in range(n_bursts)]
        raw = np.vstack(bursts)
        res = decode_hw_frames(raw, p_drg, decim_hz=fs_adev, omega_true=np.zeros(raw.shape[0]))
        taus, adevs_dph = allan_deviation(res.omega_ftr_dps, fs_adev)
        idx_1s = int(np.argmin(np.abs(taus - 1.0)))
        arw_1s = float(adevs_dph[idx_1s] * math.sqrt(taus[idx_1s]) / 60.0)
        min_adev = float(np.min(adevs_dph))
        tau_min = float(taus[int(np.argmin(adevs_dph))])
        std_dps = float(np.std(res.omega_ftr_dps))
        print(f"  [{label}] 30 s @ 200 Hz: std={std_dps:.5f} dps ({std_dps*3600:.2f} dph) | "
              f"ARW(@1s)={arw_1s:.4f} deg/sqrt(hr) | min ADEV={min_adev:.3f} dph @ tau={tau_min:.2f}s")
        adev_data[label] = {
            "taus": taus,
            "adevs_dph": adevs_dph,
            "arw_1s": arw_1s,
            "min_adev_dph": min_adev,
            "tau_min_s": tau_min,
            "std_dps": std_dps,
        }

    # --- Plot 1: FPGA Fixed-Point vs Float64 Golden Model across all 3 presets ---
    fig, axes = plt.subplots(3, 2, figsize=(13, 10))
    colors = ["#2563eb", "#059669", "#7c3aed"]
    for idx, key in enumerate(preset_keys):
        r = results[key]
        tr = traces[key]
        ax_l = axes[idx, 0]
        ax_r = axes[idx, 1]

        ax_l.plot(tr["t_ms"], tr["py_ftr"], "--", color="#111827", lw=1.6, label="Float64 Golden Model")
        ax_l.plot(tr["t_ms"], tr["hw_ftr"], "-", color=colors[idx], lw=1.4, alpha=0.9, label="Live ZUBoard 1CG Fixed-Point")
        ax_l.set_title(f"{r['name']} — Step (HW err={r['sf_err_hw_pct']:+.2f}%, Δ vs f64={r['diff_ss_ppm']:+.0f} ppm)", fontsize=10, fontweight="bold")
        ax_l.set_ylabel("FTR Rate (°/s)")
        ax_l.set_xlim(180, 400)
        ax_l.grid(True, alpha=0.3)
        ax_l.legend(loc="lower right", fontsize=8)

        # Zoomed zero-rate quantization floor (in deg/hr!)
        t_z = np.arange(len(tr["z_hw"])) * 0.5  # ms
        z_dph = (tr["z_hw"] - np.mean(tr["z_hw"])) * 3600.0
        ax_r.plot(t_z, z_dph, color=colors[idx], lw=1.0, label=f"HW Zero-Rate Ripple (RMS={r['zero_rms_hw_dps']*3600:.2f} °/hr)")
        lsb_dph = r["ftr_q22_lsb_dph"]
        ax_r.axhline(+0.5 * lsb_dph, color="#dc2626", ls=":", lw=1.1, label=f"±0.5 LSB of Q10.22 FTR (±{0.5*lsb_dph:.3f} °/hr)")
        ax_r.axhline(-0.5 * lsb_dph, color="#dc2626", ls=":", lw=1.1)
        ax_r.set_title(f"Zero-Rate Fixed-Point Floor (v2 High-Q) | Quad Rej = {r['quad_rej_db']:.1f} dB", fontsize=10)
        ax_r.set_ylabel("Residual Rate (°/hr)")
        ax_r.grid(True, alpha=0.3)
        ax_r.legend(loc="upper right", fontsize=8)

    axes[2, 0].set_xlabel("Time (ms)")
    axes[2, 1].set_xlabel("Zero-Rate Capture Time (ms)")
    fig.suptitle("ZUBoard 1CG Live Stage 1 v2 (40b NCO + Q10.22 PI + 24b ADC + Q-6.42) vs Float64", fontsize=12.5, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    p1 = out_dir / "hw_fpga_vs_float_error_v2.png"
    fig.savefig(p1, dpi=150)
    plt.close(fig)
    print(f"\nSaved -> {p1}")

    # --- Plot 2: Clean 30 s Live Hardware ADEV (Pure Fixed-Point vs PRNG Thermal+RRW) ---
    fig, ax = plt.subplots(figsize=(9, 5.6))
    fp = adev_data["fixed_point_only"]
    pr = adev_data["prng_thermal_rrw"]
    ax.loglog(pr["taus"], pr["adevs_dph"], "o-", color="#2563eb", lw=2.0, ms=4,
              label=f"Live FPGA v2 PRNG ON (ARW={pr['arw_1s']:.4f} °/√hr, min={pr['min_adev_dph']:.3f} °/hr)")
    ax.loglog(fp["taus"], fp["adevs_dph"], "s-", color="#10b981", lw=2.0, ms=4,
              label=f"Live FPGA v2 PRNG OFF — Pure Fixed-Point Floor (min={fp['min_adev_dph']:.4f} °/hr)")
    arw_ref = (p_drg.arw_deg_sqrt_hr * 60.0) / np.sqrt(pr["taus"])
    ax.loglog(pr["taus"], arw_ref, "--", color="#6b7280", lw=1.3,
              label=f"Silicon DRG Target ARW = {p_drg.arw_deg_sqrt_hr} °/√hr (-1/2 slope)")
    q22_floor = results["drg_11khz_q100k"]["ftr_q22_lsb_dph"] / math.sqrt(12.0)
    ax.axhline(q22_floor, color="#f59e0b", ls=":", lw=1.3,
               label=f"Q10.22 FTR 1-sample quant floor (LSB/√12 = {q22_floor:.4f} °/hr)")
    ax.set_xlabel("Cluster Time τ (s)")
    ax.set_ylabel("Allan Deviation σ(τ) (°/hr)")
    ax.set_title("ZUBoard 1CG Live 30 s ADEV (Stage 1 v2 High-Q Precision) — Fixed-Point Floor vs PRNG")
    ax.legend(fontsize=8.5)
    ax.grid(True, which="both", alpha=0.35)
    plt.tight_layout()
    p2 = out_dir / "hw_adev_30s_v2.png"
    fig.savefig(p2, dpi=150)
    plt.close(fig)
    print(f"Saved -> {p2}")

    # Dump JSON summary for study docs
    summary = {
        "version_hex": f"0x{ver:08X}",
        "presets": results,
        "adev_30s": {
            k: {kk: vv for kk, vv in v.items() if kk not in ("taus", "adevs_dph")}
            for k, v in adev_data.items()
        },
    }
    (out_dir / "hw_fixed_point_benchmark_v2.json").write_text(json.dumps(summary, indent=2))
    print(f"Saved -> {out_dir / 'hw_fixed_point_benchmark_v2.json'}")


if __name__ == "__main__":
    main()
