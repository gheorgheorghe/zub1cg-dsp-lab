"""cocotb + Verilator testbench for stage0_dsp_slot (NCO packet framing + MM2S loopback)."""

from pathlib import Path
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
try:
    from cocotb_tools.runner import get_runner
except ImportError:
    from cocotb.runner import get_runner


@cocotb.test()
async def tb_nco_tlast_framing(dut):
    """Verify TLAST pulses at exact pkt_len boundaries for AXI-DMA S2MM."""
    cocotb.start_soon(Clock(dut.aclk, 10, units="ns").start())

    dut.aresetn.value = 0
    dut.ctrl_enable.value = 0
    dut.ctrl_loopback.value = 0
    dut.ctrl_soft_rst.value = 0
    dut.nco_freq_word.value = 1 << 24
    dut.pkt_len.value = 64
    dut.nco_phase_off.value = 0
    dut.s_axis_tdata.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 1

    for _ in range(5):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    dut.ctrl_enable.value = 1

    tlast_positions = []
    seen = 0
    while seen < 192:
        await RisingEdge(dut.aclk)
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            if dut.m_axis_tlast.value:
                tlast_positions.append(seen)
            seen += 1

    assert tlast_positions == [63, 127, 191], f"Unexpected TLAST indices: {tlast_positions}"
    await RisingEdge(dut.aclk)
    assert int(dut.sample_cnt_out.value) >= 192


@cocotb.test()
async def tb_mm2s_loopback_mode(dut):
    """Verify bit-exact AXI-Stream loopback with random backpressure."""
    cocotb.start_soon(Clock(dut.aclk, 10, units="ns").start())
    rng = random.Random(7)

    dut.aresetn.value = 0
    dut.ctrl_enable.value = 1
    dut.ctrl_loopback.value = 1
    dut.ctrl_soft_rst.value = 0
    dut.nco_freq_word.value = 0
    dut.pkt_len.value = 32
    dut.nco_phase_off.value = 0
    dut.s_axis_tvalid.value = 0
    dut.s_axis_tlast.value = 0
    dut.m_axis_tready.value = 1

    for _ in range(4):
        await RisingEdge(dut.aclk)
    dut.aresetn.value = 1

    tx_words = [0xA500_0000 + i for i in range(64)]
    rx_words = []
    rx_tlasts = []
    idx = 0

    while len(rx_words) < len(tx_words):
        dut.m_axis_tready.value = 1 if rng.random() > 0.3 else 0
        if idx < len(tx_words):
            dut.s_axis_tdata.value = tx_words[idx]
            dut.s_axis_tvalid.value = 1
            dut.s_axis_tlast.value = 1 if idx == len(tx_words) - 1 else 0
        else:
            dut.s_axis_tvalid.value = 0
            dut.s_axis_tlast.value = 0

        await RisingEdge(dut.aclk)
        if dut.s_axis_tvalid.value and dut.s_axis_tready.value:
            idx += 1
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            rx_words.append(int(dut.m_axis_tdata.value))
            rx_tlasts.append(int(dut.m_axis_tlast.value))

    assert rx_words == tx_words
    assert rx_tlasts[-1] == 1 and sum(rx_tlasts) == 1


def test_runner():
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("verilator")
    runner.build(
        verilog_sources=[
            root / "rtl" / "common" / "sv_nco_cordic.sv",
            root / "rtl" / "common" / "stage0_dsp_slot.sv",
        ],
        hdl_toplevel="stage0_dsp_slot",
        build_dir=root / "sim" / "sim_build" / "stage0_slot",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="stage0_dsp_slot",
        test_module="test_stage0_slot",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )
