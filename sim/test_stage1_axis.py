"""cocotb + Verilator verification of Stage 1.4 AXI4-Lite + AXI4-Stream wrapper."""

import math
from pathlib import Path
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
try:
    from cocotb_tools.runner import get_runner
except ImportError:
    from cocotb.runner import get_runner


def to_s36(val: float, frac: int) -> tuple[int, int]:
    q = int(round(val * (1 << frac))) & ((1 << 36) - 1)
    return q & 0xFFFFFFFF, (q >> 32) & 0xF


def to_s24(val: float, frac: int) -> int:
    return int(round(val * (1 << frac))) & 0xFFFFFF


def from_s32(v: int) -> int:
    v &= 0xFFFFFFFF
    return v - 0x100000000 if (v & 0x80000000) else v


async def axil_write(dut, addr: int, data: int):
    dut.s_axi_awaddr.value = addr
    dut.s_axi_awvalid.value = 1
    dut.s_axi_wdata.value = data & 0xFFFFFFFF
    dut.s_axi_wstrb.value = 0xF
    dut.s_axi_wvalid.value = 1
    dut.s_axi_bready.value = 1

    aw_done = False
    w_done = False
    while not (aw_done and w_done):
        await RisingEdge(dut.aclk)
        if dut.s_axi_awvalid.value and dut.s_axi_awready.value:
            aw_done = True
            dut.s_axi_awvalid.value = 0
        if dut.s_axi_wvalid.value and dut.s_axi_wready.value:
            w_done = True
            dut.s_axi_wvalid.value = 0

    while not dut.s_axi_bvalid.value:
        await RisingEdge(dut.aclk)
    await RisingEdge(dut.aclk)
    dut.s_axi_bready.value = 0


async def axil_read(dut, addr: int) -> int:
    dut.s_axi_araddr.value = addr
    dut.s_axi_arvalid.value = 1
    dut.s_axi_rready.value = 1
    while True:
        await RisingEdge(dut.aclk)
        if dut.s_axi_arvalid.value and dut.s_axi_arready.value:
            dut.s_axi_arvalid.value = 0
            break
    while not dut.s_axi_rvalid.value:
        await RisingEdge(dut.aclk)
    val = int(dut.s_axi_rdata.value)
    await RisingEdge(dut.aclk)
    dut.s_axi_rready.value = 0
    return val


@cocotb.test()
async def tb_stage1_axil_and_axis_telemetry(dut):
    """Configure DRG preset over AXI4-Lite and capture 8-word AXI4-Stream DMA frames."""
    cocotb.start_soon(Clock(dut.aclk, 10, units="ns").start())

    dut.aresetn.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 1

    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)

    # 1. Verify Stage 1 v2 High-Q VERSION register (0xEC)
    ver = await axil_read(dut, 0xEC)
    assert ver == 0x5A550102, f"Unexpected VERSION: 0x{ver:08X}"

    # 2. Program DRG 11 kHz Q=100k parameters over AXI-Lite (fs = 1 MHz)
    fs = 1_000_000.0
    dt = 1.0 / fs
    f0 = 11_000.0
    w0 = 2.0 * math.pi * f0
    ag = 0.40
    k_xy = 2.0 * ag * (120.0 * math.pi / 180.0)

    await axil_write(dut, 0x04, (6 << 16) | 50)          # lpf_shift=6, decim=50 -> 400 ticks across 8 frames
    await axil_write(dut, 0x08, 4)                       # 4 frames (32 words) per TLAST burst
    await axil_write(dut, 0x0C, int(round((f0 / fs) * (2**32))))
    await axil_write(dut, 0x10, 1 << 22)                 # DX setpoint = 1.0

    # Stage 1 v2 High-Q gains (Q2.22 err -> 32-bit Q10.22 out / 40b NCO, GAIN_FRAC = 14)
    pll_scale = (2**40 / (2.0 * math.pi * fs)) / (2**22)
    await axil_write(dut, 0x1C, to_s24(600.0 * pll_scale, 14))
    await axil_write(dut, 0x20, to_s24(150_000.0 * dt * pll_scale, 14))
    await axil_write(dut, 0x24, to_s24(160.0, 14))
    await axil_write(dut, 0x28, to_s24(5_000.0 * dt, 14))
    await axil_write(dut, 0x2C, to_s24(240.0, 14))
    await axil_write(dut, 0x30, to_s24(12_000.0 * dt, 14))
    await axil_write(dut, 0x34, to_s24(240.0, 14))
    await axil_write(dut, 0x38, to_s24(12_000.0 * dt, 14))

    # Plant cos/sin (36-bit Q2.34)
    for addr_lo, val in [
        (0x54, math.cos(w0 * dt)),
        (0x5C, math.sin(w0 * dt)),
        (0x64, math.cos(2 * math.pi * (f0 + 0.5) * dt)),
        (0x6C, math.sin(2 * math.pi * (f0 + 0.5) * dt)),
    ]:
        lo, hi = to_s36(val, 34)
        await axil_write(dut, addr_lo, lo)
        await axil_write(dut, addr_lo + 4, hi)

    # Plant high-Q kick coefficients (36-bit Q-6.42)
    for addr_lo, val in [
        (0x74, 0.5 * (w0 / 100_000.0) * dt),
        (0x7C, 0.5 * (w0 / 95_000.0) * dt),
        (0x84, 0.5 * k_xy * dt),
        (0x8C, 0.0),
        (0x94, 0.5 * (2.0 * ag * (100.0 * math.pi / 180.0)) * dt),
    ]:
        lo, hi = to_s36(val, 42)
        await axil_write(dut, addr_lo, lo)
        await axil_write(dut, addr_lo + 4, hi)

    lo_dt, hi_dt = to_s36(0.5 * dt, 36)
    await axil_write(dut, 0x9C, lo_dt)
    await axil_write(dut, 0xA0, hi_dt)

    # Preload vx = 1.0 in Q4.44 (1 << 44 -> lo=0, hi=0x1000) and pulse state_load + pi_load
    await axil_write(dut, 0x44, 0)
    await axil_write(dut, 0x48, 0)
    await axil_write(dut, 0x4C, 0)
    await axil_write(dut, 0x50, 0x1000)
    await axil_write(dut, 0x40, int(round((w0 / 100_000.0) * (1 << 22))) & 0xFFFFFFFF)
    await axil_write(dut, 0x00, 0xDE)  # state_load=1, pi_load=1, loops=1, run=0
    await axil_write(dut, 0x00, 0x1F)  # run=1, PLL/AGC/CLQ/FTR enabled

    # 3. Capture 2 complete bursts = 8 frames = 64 words and check TLAST at words 31 and 63
    words = []
    tlasts = []
    while len(words) < 64:
        await RisingEdge(dut.aclk)
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            words.append(from_s32(int(dut.m_axis_tdata.value)))
            tlasts.append(int(dut.m_axis_tlast.value))

    tlast_idx = [i for i, v in enumerate(tlasts) if v == 1]
    assert tlast_idx == [31, 63], f"Unexpected TLAST positions: {tlast_idx}"

    # Verify AXI-Lite RO telemetry registers are alive and non-zero
    frame_cnt = await axil_read(dut, 0xE8)
    status = await axil_read(dut, 0xE4)
    dx_ro = from_s32(await axil_read(dut, 0xC0)) / float(1 << 22)
    cocotb.log.info(f"Stage 1 AXI wrap OK: frames={frame_cnt}, status=0x{status:08X}, DX_RO={dx_ro:.4f}")
    assert frame_cnt >= 8
    assert (status & 0xFFFF0000) == 0x00010000
    assert (status & 0x8) == 0, "Telemetry stream overflowed!"
    assert dx_ro > 0.5, f"Expected settled DX_RO > 0.5, got {dx_ro:.4f}"
    # Verify AXI-Stream word 0 of frame 7 (words[56]) matches DX_RO scale
    dx_stream_last = words[56] / float(1 << 22)
    assert abs(dx_stream_last - dx_ro) < 0.02

    # 4. Verify Stage 1.5 PRNG thermal/ADC/RRW noise registers (0xA4..0xAC)
    await axil_write(dut, 0xA4, (400 << 16) | 2000)   # adc_amp=400, force_amp=2000
    await axil_write(dut, 0xA8, 0x12345678)            # custom PRNG seed
    await axil_write(dut, 0xAC, 50)                    # rrw_step_amp=50
    assert await axil_read(dut, 0xA4) == ((400 << 16) | 2000)
    assert await axil_read(dut, 0xA8) == 0x12345678
    assert await axil_read(dut, 0xAC) == 50

    # Capture 2 more bursts with noise enabled and verify pickoff/telemetry stays live & bounded
    noisy_words = []
    while len(noisy_words) < 64:
        await RisingEdge(dut.aclk)
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            noisy_words.append(from_s32(int(dut.m_axis_tdata.value)))
    dx_noisy = [noisy_words[i * 8] / float(1 << 22) for i in range(8)]
    ftr_noisy = [noisy_words[i * 8 + 7] / float(1 << 22) for i in range(8)]
    cocotb.log.info(f"Stage 1.5 PRNG noise OK: DX_mean={sum(dx_noisy)/8:.4f}, FTR_span={max(ftr_noisy)-min(ftr_noisy):.4f}")
    assert 0.5 < (sum(dx_noisy) / 8.0) < 1.5
    assert (max(ftr_noisy) - min(ftr_noisy)) > 0.0


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
            root / "rtl" / "mems_hil" / "mems_hil_axis_wrap.sv",
        ],
        hdl_toplevel="mems_hil_axis_wrap",
        parameters={"TICK_DIV": 60},
        build_dir=root / "sim" / "sim_build" / "stage1_axis",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="mems_hil_axis_wrap",
        test_module="test_stage1_axis",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )
