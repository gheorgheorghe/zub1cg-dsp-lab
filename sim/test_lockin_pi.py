"""cocotb + Verilator testbenches for sv_lockin and sv_pi_ctrl."""

import math
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
try:
    from cocotb_tools.runner import get_runner
except ImportError:
    from cocotb.runner import get_runner


def to_signed(val: int, bits: int) -> int:
    mask = (1 << bits) - 1
    v = val & mask
    return v - (1 << bits) if (v & (1 << (bits - 1))) else v


@cocotb.test()
async def tb_lockin_iq_recovery(dut):
    """Feed A*sin(wt) + B*cos(wt) into sv_lockin and verify exact I/Q recovery."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    dut.rst_n.value = 0
    dut.tick_en.value = 0
    dut.lpf_shift.value = 6
    dut.sig_in.value = 0
    dut.cos_in.value = 0
    dut.sin_in.value = 0

    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Carrier = 11 kHz at fs = 200 kHz; signal = 0.8*sin(phi) - 0.35*cos(phi)
    f0 = 11_000.0
    fs = 200_000.0
    a_sin = 0.80
    b_cos = -0.35

    for k in range(1500):
        phi = 2.0 * math.pi * f0 * k / fs
        sig = a_sin * math.sin(phi) + b_cos * math.cos(phi)
        dut.sig_in.value = int(round(sig * 16384)) & 0xFFFF
        dut.sin_in.value = int(round(math.sin(phi) * 32760)) & 0xFFFF
        dut.cos_in.value = int(round(math.cos(phi) * 32760)) & 0xFFFF
        dut.tick_en.value = 1
        await RisingEdge(dut.clk)
        dut.tick_en.value = 0
        await RisingEdge(dut.clk)

    out_i = to_signed(int(dut.out_i.value), 24) / float(1 << 22)
    out_q = to_signed(int(dut.out_q.value), 24) / float(1 << 22)
    cocotb.log.info(f"Lock-in recovered I={out_i:.5f} (exp {a_sin}), Q={out_q:.5f} (exp {b_cos})")
    assert abs(out_i - a_sin) < 0.005
    assert abs(out_q - b_cos) < 0.005


@cocotb.test()
async def tb_pi_anti_windup(dut):
    """Verify PI integration, output clamping, and instant recovery (no windup lag)."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    dut.rst_n.value = 0
    dut.tick_en.value = 0
    dut.int_load.value = 0
    dut.int_init.value = 0
    dut.err.value = 0
    dut.kp.value = 1 << 16       # Kp = 1.0
    dut.ki.value = 1 << 14       # Ki*Ts = 0.25
    dut.out_min.value = (-1000) & 0xFFFFFF
    dut.out_max.value = 1000

    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Drive large positive error -> hits +1000 saturation for 50 ticks
    dut.err.value = 400
    for _ in range(50):
        dut.tick_en.value = 1
        await RisingEdge(dut.clk)
        dut.tick_en.value = 0
        await RisingEdge(dut.clk)

    assert to_signed(int(dut.ctrl_out.value), 24) == 1000
    assert int(dut.sat_flag.value) == 1

    # Reverse error sign -> must exit saturation within a few ticks (anti-windup!)
    dut.err.value = (-400) & 0xFFFFFF
    for _ in range(4):
        dut.tick_en.value = 1
        await RisingEdge(dut.clk)
        dut.tick_en.value = 0
        await RisingEdge(dut.clk)

    val_after = to_signed(int(dut.ctrl_out.value), 24)
    cocotb.log.info(f"PI output 4 ticks after error reversal: {val_after}")
    assert val_after < 500, f"Integrator wound up! ctrl_out={val_after}"


def test_lockin_runner():
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("verilator")
    runner.build(
        verilog_sources=[root / "rtl" / "common" / "sv_lockin.sv"],
        hdl_toplevel="sv_lockin",
        build_dir=root / "sim" / "sim_build" / "lockin",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="sv_lockin",
        test_module="test_lockin_pi",
        testcase="tb_lockin_iq_recovery",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )


def test_pi_runner():
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("verilator")
    runner.build(
        verilog_sources=[root / "rtl" / "common" / "sv_pi_ctrl.sv"],
        hdl_toplevel="sv_pi_ctrl",
        build_dir=root / "sim" / "sim_build" / "pi_ctrl",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="sv_pi_ctrl",
        test_module="test_lockin_pi",
        testcase="tb_pi_anti_windup",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )
