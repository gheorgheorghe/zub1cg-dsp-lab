"""cocotb + Verilator verification of 48-bit Strang-split mems_plant_2dof.sv.

Tests:
  1. DRG (f0 = 11 kHz, Q = 100,000) open-loop ringdown -> asserts RTL Q error < 0.1%
  2. HRG (f0 = 11 kHz, Q = 1,000,000) open-loop ringdown -> asserts RTL Q error < 0.2%
  3. Coriolis cross-axis energy transfer under Omega_z = 100 deg/s
"""

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
DT_FRAC = 36


def to_fixed(val: float, bits: int, frac: int) -> int:
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    q = max(lo, min(hi, int(round(val * (1 << frac)))))
    return q & ((1 << bits) - 1)


def from_signed(val: int, bits: int) -> int:
    v = val & ((1 << bits) - 1)
    return v - (1 << bits) if (v & (1 << (bits - 1))) else v


async def step_plant_once(dut):
    dut.tick_en.value = 1
    await RisingEdge(dut.clk)
    dut.tick_en.value = 0
    while True:
        await RisingEdge(dut.clk)
        if dut.step_done.value:
            break


async def run_ringdown(dut, q_target: float, f0_hz: float = 11_000.0, fs_hz: float = 1_000_000.0, n_steps: int = 10_000):
    dt = 1.0 / fs_hz
    w = 2.0 * math.pi * f0_hz
    tx = w * dt

    dut.rst_n.value = 0
    dut.tick_en.value = 0
    dut.state_load.value = 0
    dut.u_x_k.value = 0
    dut.u_y_k.value = 0
    dut.u_x_k1.value = 0
    dut.u_y_k1.value = 0
    dut.force_noise_amp.value = 0
    dut.adc_noise_amp.value = 0
    dut.rrw_step_amp.value = 0
    dut.noise_seed.value = 0xA5A51337

    dut.cos_tx.value = to_fixed(math.cos(tx), 36, C_FRAC)
    dut.sin_tx.value = to_fixed(math.sin(tx), 36, C_FRAC)
    dut.cos_ty.value = to_fixed(math.cos(tx), 36, C_FRAC)
    dut.sin_ty.value = to_fixed(math.sin(tx), 36, C_FRAC)
    dut.half_gx_dt.value = to_fixed(0.5 * (w / q_target) * dt, 36, C_FRAC)
    dut.half_gy_dt.value = to_fixed(0.5 * (w / q_target) * dt, 36, C_FRAC)
    dut.half_kxy_dt.value = 0
    dut.half_gxy_dt.value = 0
    dut.half_cor_dt.value = 0
    dut.half_dt_q36.value = to_fixed(0.5 * dt, 36, DT_FRAC)

    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Load initial drive displacement x = 1.0, vx = y = vy = 0
    dut.x_init.value = to_fixed(1.0, 48, S_FRAC)
    dut.vx_init.value = 0
    dut.y_init.value = 0
    dut.vy_init.value = 0
    dut.state_load.value = 1
    await RisingEdge(dut.clk)
    dut.state_load.value = 0
    await RisingEdge(dut.clk)

    for _ in range(n_steps):
        await step_plant_once(dut)

    x_f = from_signed(int(dut.x_state.value), 48) / float(1 << S_FRAC)
    vx_f = from_signed(int(dut.vx_state.value), 48) / float(1 << S_FRAC)
    amp = math.hypot(x_f, vx_f)
    dur = n_steps * dt
    q_meas = -w * dur / (2.0 * math.log(amp))
    err_pct = 100.0 * (q_meas - q_target) / q_target
    return q_meas, err_pct, amp


@cocotb.test()
async def tb_drg_and_hrg_ringdown_and_coriolis(dut):
    """Verify exact DRG (Q=100k), HRG (Q=1M) ringdown Q and Coriolis coupling in RTL."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # 1. DRG Q = 100,000 ringdown (10 ms = 10,000 steps @ 1 MHz)
    q_drg, err_drg, _ = await run_ringdown(dut, q_target=100_000.0, n_steps=10_000)
    cocotb.log.info(f"RTL DRG Q=100k: measured={q_drg:.1f} (err={err_drg:+.4f}%)")
    assert abs(err_drg) < 0.1, f"DRG Q error too high: {err_drg:.4f}%"

    # 2. HRG Q = 1,000,000 ringdown (10 ms = 10,000 steps @ 1 MHz)
    q_hrg, err_hrg, _ = await run_ringdown(dut, q_target=1_000_000.0, n_steps=10_000)
    cocotb.log.info(f"RTL HRG Q=1M:   measured={q_hrg:.1f} (err={err_hrg:+.4f}%)")
    assert abs(err_hrg) < 0.2, f"HRG Q error too high: {err_hrg:.4f}%"

    # 3. Coriolis coupling: mode-matched DRG under Omega_z = 200 deg/s for 5 ms
    # Analytical sense envelope growth: |y(t)| ~ Ag * Omega_rad * t (for small angle)
    f0 = 11_000.0
    fs = 1_000_000.0
    dt = 1.0 / fs
    ag = 0.40
    omega_rad = 200.0 * math.pi / 180.0
    n_cor = 5000  # 5 ms

    dut.half_cor_dt.value = to_fixed(0.5 * (2.0 * ag * omega_rad) * dt, 36, C_FRAC)
    dut.half_gx_dt.value = to_fixed(0.5 * (2 * math.pi * f0 / 100_000.0) * dt, 36, C_FRAC)
    dut.half_gy_dt.value = to_fixed(0.5 * (2 * math.pi * f0 / 100_000.0) * dt, 36, C_FRAC)
    dut.x_init.value = to_fixed(1.0, 48, S_FRAC)
    dut.vx_init.value = 0
    dut.y_init.value = 0
    dut.vy_init.value = 0
    dut.state_load.value = 1
    await RisingEdge(dut.clk)
    dut.state_load.value = 0
    await RisingEdge(dut.clk)

    for _ in range(n_cor):
        await step_plant_once(dut)

    y_f = from_signed(int(dut.y_state.value), 48) / float(1 << S_FRAC)
    vy_f = from_signed(int(dut.vy_state.value), 48) / float(1 << S_FRAC)
    sense_amp = math.hypot(y_f, vy_f)
    expected_amp = math.sin(ag * omega_rad * (n_cor * dt))
    cocotb.log.info(f"Coriolis 5 ms sense amp: RTL={sense_amp:.6f}, analytical={expected_amp:.6f}")
    assert abs(sense_amp - expected_amp) / expected_amp < 0.01


def test_runner():
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("verilator")
    runner.build(
        verilog_sources=[root / "rtl" / "mems_hil" / "mems_plant_2dof.sv"],
        hdl_toplevel="mems_plant_2dof",
        build_dir=root / "sim" / "sim_build" / "mems_plant",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="mems_plant_2dof",
        test_module="test_mems_plant",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )
