"""Full RTL-in-the-loop cocotb verification of mems_hil_top (plant + controller)."""

import math
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
try:
    from cocotb_tools.runner import get_runner
except ImportError:
    from cocotb.runner import get_runner


S_FRAC = 44
C_FRAC = 34
SMALL_FRAC = 42
DT_FRAC = 36


def to_fixed(val: float, bits: int, frac: int) -> int:
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    q = max(lo, min(hi, int(round(val * (1 << frac)))))
    return q & ((1 << bits) - 1)


def from_signed(val: int, bits: int) -> int:
    v = val & ((1 << bits) - 1)
    return v - (1 << bits) if (v & (1 << (bits - 1))) else v


@cocotb.test()
async def tb_drg_closed_loop_rtl(dut):
    """Run DRG (11 kHz, Q=100k, 120 dps quad) closed-loop in pure SystemVerilog!"""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # Use TICK_DIV = 40 in sim (fast Verilator sim; physics configured for fs = 200 kHz)
    fs = 200_000.0
    dt = 1.0 / fs
    f0 = 11_000.0
    w0 = 2.0 * math.pi * f0
    ag = 0.40
    q_x = 100_000.0
    q_y = 95_000.0
    k_xy = 2.0 * ag * (120.0 * math.pi / 180.0)

    dut.rst_n.value = 0
    dut.run_enable.value = 0
    dut.tick_div_cfg.value = 0
    dut.state_load.value = 0
    dut.pi_load.value = 0

    dut.cos_tx.value = to_fixed(math.cos(w0 * dt), 36, C_FRAC)
    dut.sin_tx.value = to_fixed(math.sin(w0 * dt), 36, C_FRAC)
    dut.cos_ty.value = to_fixed(math.cos(2 * math.pi * (f0 + 0.5) * dt), 36, C_FRAC)
    dut.sin_ty.value = to_fixed(math.sin(2 * math.pi * (f0 + 0.5) * dt), 36, C_FRAC)
    dut.half_gx_dt.value = to_fixed(0.5 * (w0 / q_x) * dt, 36, SMALL_FRAC)
    dut.half_gy_dt.value = to_fixed(0.5 * (w0 / q_y) * dt, 36, SMALL_FRAC)
    dut.half_kxy_dt.value = to_fixed(0.5 * k_xy * dt, 36, SMALL_FRAC)
    dut.half_gxy_dt.value = 0
    dut.half_cor_dt.value = 0
    dut.half_dt_q36.value = to_fixed(0.5 * dt, 36, DT_FRAC)
    dut.force_noise_amp.value = 0
    dut.adc_noise_amp.value = 0
    dut.rrw_step_amp.value = 0
    dut.noise_seed.value = 0xA5A51337

    # NCO base frequency word for 11 kHz at fs = 200 kHz
    dut.base_freq_word.value = int(round((f0 / fs) * (2**32)))
    dut.lpf_shift.value = 6  # fc ~ 500 Hz at 200 kHz
    dut.enable_pll.value = 1
    dut.enable_agc.value = 1
    dut.enable_clq.value = 1
    dut.enable_ftr.value = 1
    dut.mode_swap.value = 0

    dut.dx_setpoint.value = 1 << 22  # 1.0 in Q2.22
    dut.sx_setpoint.value = 0
    dut.sy_setpoint.value = 0

    # Stage 1 v2 High-Q gains: Q2.22 err -> 32-bit Q10.22 out (u_scale = 1.0, GAIN_FRAC = 14)
    # For 40-bit PLL: 1 rad/s = (2^40 / (2*pi*fs)) PLLAdj counts
    pll_scale = (2**40 / (2.0 * math.pi * fs)) / (2**22)

    dut.pll_kp.value = to_fixed(600.0 * pll_scale, 24, 14)
    dut.pll_ki.value = to_fixed(150_000.0 * dt * pll_scale, 24, 14)
    dut.agc_kp.value = to_fixed(160.0, 24, 14)
    dut.agc_ki.value = to_fixed(5_000.0 * dt, 24, 14)
    dut.clq_kp.value = to_fixed(240.0, 24, 14)
    dut.clq_ki.value = to_fixed(12_000.0 * dt, 24, 14)
    dut.ftr_kp.value = to_fixed(240.0, 24, 14)
    dut.ftr_ki.value = to_fixed(12_000.0 * dt, 24, 14)

    for _ in range(5):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Preload drive resonator near nominal amplitude & preload AGC nominal drive (Q10.22)
    dut.x_init.value = 0
    dut.vx_init.value = to_fixed(0.95, 48, S_FRAC)
    dut.y_init.value = 0
    dut.vy_init.value = 0
    dut.state_load.value = 1
    dut.pll_init.value = 0
    dut.agc_init.value = to_fixed(w0 / q_x, 32, 22)
    dut.pi_load.value = 1
    await RisingEdge(dut.clk)
    dut.state_load.value = 0
    dut.pi_load.value = 0
    dut.run_enable.value = 1

    # Run 30 ms settle at Omega = 0 (6,000 ticks), then 90 ms at Omega = +100 dps (18,000 ticks)
    omega_step_rad = 100.0 * math.pi / 180.0
    ticks = 0
    ftr_samples = []
    clq_samples = []
    dx_samples = []
    ang_true = 0.0
    ang_ftr = 0.0
    ang_ff_int = 0.0
    peak_ftr_err = 0.0
    peak_ff_err = 0.0
    gamma_y = w0 / q_y

    while ticks < 24_000:
        await RisingEdge(dut.clk)
        if dut.tick_pulse.value:
            ticks += 1
            if ticks == 6_000:
                dut.half_cor_dt.value = to_fixed(0.5 * (2.0 * ag * omega_step_rad) * dt, 36, SMALL_FRAC)

            if ticks >= 6_000:
                dx_val = from_signed(int(dut.dx_out.value), 24) / float(1 << 22)
                dy_val = from_signed(int(dut.dy_out.value), 24) / float(1 << 22)
                sx_val = from_signed(int(dut.sx_out.value), 24) / float(1 << 22)
                sy_val = from_signed(int(dut.sy_out.value), 24) / float(1 << 22)
                amp = max(math.hypot(dx_val, dy_val), 0.1)
                cd, sd = dx_val / amp, dy_val / amp
                ftr_u = from_signed(int(dut.ftr_adj_out.value), 32) / float(1 << 22)
                clq_u = from_signed(int(dut.clq_adj_out.value), 32) / float(1 << 22)
                ftr_aligned = ftr_u * cd - clq_u * sd
                clq_aligned = ftr_u * sd + clq_u * cd
                sx_aligned = sx_val * cd - sy_val * sd
                scale_dps = (180.0 / math.pi) / (2.0 * ag * amp)

                rate_ftr = ftr_aligned * scale_dps
                ang_true += 100.0 * dt
                ang_ftr += rate_ftr * dt
                ang_ff_int += (ftr_aligned - gamma_y * sx_aligned) * scale_dps * dt
                ang_ff = ang_ff_int - 2.0 * sx_aligned * scale_dps

                peak_ftr_err = max(peak_ftr_err, abs(ang_ftr - ang_true))
                peak_ff_err = max(peak_ff_err, abs(ang_ff - ang_true))

                if ticks >= 22_000:
                    dx_samples.append(amp)
                    ftr_samples.append(rate_ftr)
                    clq_samples.append(clq_aligned * scale_dps)

    dx_mean = sum(dx_samples) / len(dx_samples)
    rate_mean = sum(ftr_samples) / len(ftr_samples)
    quad_mean = sum(clq_samples) / len(clq_samples)
    cocotb.log.info(
        f"RTL Closed-Loop DRG: DX={dx_mean:.4f} (exp 1.0), "
        f"Rate={rate_mean:.2f} dps (exp 100.0), Quad={quad_mean:.2f} dps (exp 120.0) | "
        f"Peak Angle Err: FTR={peak_ftr_err*1e3:.1f} mdeg -> FF={peak_ff_err*1e3:.1f} mdeg"
    )
    assert abs(dx_mean - 1.0) < 0.01
    assert abs(rate_mean - 100.0) < 1.5
    assert abs(quad_mean - 120.0) < 1.5
    assert peak_ff_err < peak_ftr_err / 5.0


@cocotb.test()
async def tb_v3_mode_reversal_cancels_anisodamping(dut):
    """Verify Stage 1 v3 hardware mode_swap=0 vs 1 chops huge anisodamping bias to zero!"""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    fs = 200_000.0
    dt = 1.0 / fs
    f0 = 11_000.0
    w0 = 2.0 * math.pi * f0
    ag = 0.40
    q_res = 100_000.0
    true_rate_dps = 50.0
    bias_dps = 5.0  # 5.0 deg/s (= 18,000 deg/hr) anisodamping bias for crisp short-sim proof!

    dut.rst_n.value = 0
    dut.run_enable.value = 0
    dut.tick_div_cfg.value = 58  # Exercise Turbo TICK_DIV=58 (max safe: 20-cyc NCO + 32-cyc plant)!
    dut.state_load.value = 0
    dut.pi_load.value = 0

    dut.cos_tx.value = to_fixed(math.cos(w0 * dt), 36, C_FRAC)
    dut.sin_tx.value = to_fixed(math.sin(w0 * dt), 36, C_FRAC)
    dut.cos_ty.value = to_fixed(math.cos(w0 * dt), 36, C_FRAC)
    dut.sin_ty.value = to_fixed(math.sin(w0 * dt), 36, C_FRAC)
    dut.half_gx_dt.value = to_fixed(0.5 * (w0 / q_res) * dt, 36, SMALL_FRAC)
    dut.half_gy_dt.value = to_fixed(0.5 * (w0 / q_res) * dt, 36, SMALL_FRAC)
    dut.half_kxy_dt.value = 0
    dut.half_gxy_dt.value = to_fixed(0.5 * (2.0 * ag * bias_dps * math.pi / 180.0) * dt, 36, SMALL_FRAC)
    dut.half_cor_dt.value = to_fixed(0.5 * (2.0 * ag * true_rate_dps * math.pi / 180.0) * dt, 36, SMALL_FRAC)
    dut.half_dt_q36.value = to_fixed(0.5 * dt, 36, DT_FRAC)
    dut.force_noise_amp.value = 0
    dut.adc_noise_amp.value = 0
    dut.rrw_step_amp.value = 0
    dut.noise_seed.value = 0xA5A51337

    dut.base_freq_word.value = int(round((f0 / fs) * (2**32)))
    dut.lpf_shift.value = 6
    dut.enable_pll.value = 1
    dut.enable_agc.value = 1
    dut.enable_clq.value = 1
    dut.enable_ftr.value = 1
    dut.mode_swap.value = 0
    dut.dx_setpoint.value = 1 << 22
    dut.sx_setpoint.value = 0
    dut.sy_setpoint.value = 0

    pll_scale = (2**40 / (2.0 * math.pi * fs)) / (2**22)
    dut.pll_kp.value = to_fixed(600.0 * pll_scale, 24, 14)
    dut.pll_ki.value = to_fixed(150_000.0 * dt * pll_scale, 24, 14)
    dut.agc_kp.value = to_fixed(160.0, 24, 14)
    dut.agc_ki.value = to_fixed(5_000.0 * dt, 24, 14)
    dut.clq_kp.value = to_fixed(240.0, 24, 14)
    dut.clq_ki.value = to_fixed(12_000.0 * dt, 24, 14)
    dut.ftr_kp.value = to_fixed(240.0, 24, 14)
    dut.ftr_ki.value = to_fixed(12_000.0 * dt, 24, 14)

    async def run_phase(swap: int) -> float:
        dut.rst_n.value = 0
        dut.run_enable.value = 0
        dut.mode_swap.value = swap
        dut.x_init.value = 0
        dut.vx_init.value = (1 << S_FRAC) if swap == 0 else 0
        dut.y_init.value = 0
        dut.vy_init.value = (1 << S_FRAC) if swap == 1 else 0
        dut.pll_init.value = 0
        dut.agc_init.value = to_fixed(w0 / q_res, 32, 22)
        for _ in range(4):
            await RisingEdge(dut.clk)
        dut.rst_n.value = 1
        await RisingEdge(dut.clk)
        dut.half_cor_dt.value = 0
        dut.state_load.value = 1
        dut.pi_load.value = 1
        await RisingEdge(dut.clk)
        dut.state_load.value = 0
        dut.pi_load.value = 0
        dut.run_enable.value = 1

        ftr_vals = []
        steps = 0
        while steps < 18_000:  # 20 ms AGC settle + 70 ms FTR settle at 200 kHz
            await RisingEdge(dut.clk)
            if dut.step_done.value:
                steps += 1
                if steps == 4_000:
                    dut.half_cor_dt.value = to_fixed(
                        0.5 * (2.0 * ag * true_rate_dps * math.pi / 180.0) * dt, 36, SMALL_FRAC
                    )
                if steps > 16_000:
                    dx_v = from_signed(int(dut.dx_out.value), 24) / float(1 << 22)
                    dy_v = from_signed(int(dut.dy_out.value), 24) / float(1 << 22)
                    ftr_v = from_signed(int(dut.ftr_adj_out.value), 32) / float(1 << 22)
                    clq_v = from_signed(int(dut.clq_adj_out.value), 32) / float(1 << 22)
                    amp = max(math.hypot(dx_v, dy_v), 0.1)
                    ftr_aligned = (ftr_v * dx_v - clq_v * dy_v) / amp
                    ftr_vals.append(ftr_aligned / amp)
        return sum(ftr_vals) / len(ftr_vals)

    ftr_0 = await run_phase(0)  # Normal:  +2*Ag*Omega + g_xy  (~ +55 deg/s)
    ftr_1 = await run_phase(1)  # Swapped: -2*Ag*Omega + g_xy  (~ -45 deg/s)

    scale = 180.0 / (math.pi * 2.0 * ag)
    omega_0 = ftr_0 * scale
    omega_1 = ftr_1 * scale
    omega_mr = 0.5 * (omega_0 - omega_1)
    bias_est = 0.5 * (omega_0 + omega_1)

    cocotb.log.info(
        f"Mode-Reversal RTL PASS: mode0={omega_0:+.3f} dps, mode1={omega_1:+.3f} dps -> "
        f"MR_rate={omega_mr:.3f} dps (true {true_rate_dps}), est_bias={bias_est:.3f} dps (true {bias_dps})"
    )
    assert abs(omega_0 - (true_rate_dps + bias_dps)) < 1.0
    assert abs(omega_1 - (-true_rate_dps + bias_dps)) < 1.0
    assert abs(omega_mr - true_rate_dps) < 1.0, f"Mode-reversal rate error: {omega_mr}"
    assert abs(bias_est - bias_dps) < 0.10, f"Anisodamping bias extraction error: {bias_est}"


def test_runner():
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("verilator")
    runner.build(
        verilog_sources=[
            root / "rtl" / "common" / "sv_nco_cordic.sv",
            root / "rtl" / "common" / "sv_lockin.sv",
            root / "rtl" / "common" / "sv_pi_ctrl.sv",
            root / "rtl" / "mems_hil" / "mems_plant_2dof.sv",
            root / "rtl" / "mems_hil" / "mems_ctrl_top.sv",
            root / "rtl" / "mems_hil" / "mems_hil_top.sv",
        ],
        hdl_toplevel="mems_hil_top",
        parameters={"TICK_DIV": 60},
        build_dir=root / "sim" / "sim_build" / "mems_closed_loop",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="mems_hil_top",
        test_module="test_mems_closed_loop",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )
