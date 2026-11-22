"""cocotb + Verilator testbench for sv_nco_cordic."""

from pathlib import Path
import random
import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge
from cocotb.runner import get_runner


def unpack_iq(word: int) -> tuple[int, int]:
    """Unpack 32-bit {sin_q15, cos_q15} into signed 16-bit (I, Q)."""
    i_raw = word & 0xFFFF
    q_raw = (word >> 16) & 0xFFFF
    i_val = i_raw - 0x10000 if (i_raw & 0x8000) else i_raw
    q_val = q_raw - 0x10000 if (q_raw & 0x8000) else q_raw
    return i_val, q_val


@cocotb.test()
async def test_nco_sfdr_and_accuracy(dut):
    """Verify Q1.15 accuracy and SFDR > 90 dBc against Python golden sine/cosine."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())  # 100 MHz

    dut.rst_n.value = 0
    dut.enable.value = 0
    dut.freq_word.value = 0
    dut.phase_offset.value = 0
    dut.m_axis_tready.value = 1

    for _ in range(5):
        await RisingEdge(dut.clk)

    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # Choose coherent bin k = 137 out of N = 4096 samples (prime, hits all phases)
    n_samples = 4096
    bin_k = 137
    freq_word = round(bin_k / n_samples * (2**32))
    dut.freq_word.value = freq_word
    dut.enable.value = 1

    iq = np.zeros(n_samples, dtype=np.complex128)
    count = 0
    while count < n_samples:
        await RisingEdge(dut.clk)
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            i_val, q_val = unpack_iq(int(dut.m_axis_tdata.value))
            iq[count] = i_val + 1j * q_val
            count += 1

    # Compare against ideal complex exponential
    phases = ((np.arange(n_samples, dtype=np.uint64) * freq_word) & 0xFFFFFFFF) / (2**32) * 2 * np.pi
    ideal = 32760.0 * np.exp(1j * phases)
    max_lsb_err = float(np.max(np.abs(iq - ideal)))
    cocotb.log.info(f"Max complex error vs ideal: {max_lsb_err:.3f} LSB")
    assert max_lsb_err < 2.5, f"CORDIC error too large: {max_lsb_err:.3f} LSB"

    # Compute SFDR (coherent Hann-windowed spectrum)
    win = np.hanning(n_samples)
    spec = np.abs(np.fft.fft(iq * win))
    spec_db = 20.0 * np.log10(spec / np.max(spec) + 1e-15)
    peak_idx = int(np.argmax(spec))

    # Mask +-4 bins around fundamental mainlobe
    mask = np.ones(n_samples, dtype=bool)
    for offset in range(-4, 5):
        mask[(peak_idx + offset) % n_samples] = False
    sfdr_db = -float(np.max(spec_db[mask]))

    cocotb.log.info(f"Measured SFDR = {sfdr_db:.2f} dBc (peak bin = {peak_idx}, expected = {bin_k})")
    assert peak_idx == bin_k
    assert sfdr_db > 90.0, f"Expected SFDR > 90 dBc, got {sfdr_db:.2f} dBc"


@cocotb.test()
async def test_nco_backpressure(dut):
    """Ensure random AXI-Stream tready stalls drop or duplicate zero samples."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    rng = random.Random(42)

    dut.rst_n.value = 0
    dut.enable.value = 0
    dut.freq_word.value = 1 << 24  # 256 samples/cycle
    dut.phase_offset.value = 0
    dut.m_axis_tready.value = 1

    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    dut.enable.value = 1

    n_samples = 512
    captured = []
    while len(captured) < n_samples:
        dut.m_axis_tready.value = 1 if rng.random() > 0.35 else 0
        await RisingEdge(dut.clk)
        if dut.m_axis_tvalid.value and dut.m_axis_tready.value:
            captured.append(unpack_iq(int(dut.m_axis_tdata.value)))

    arr = np.array([c[0] + 1j * c[1] for c in captured])
    phases = np.arange(n_samples) * (2 * np.pi / 256.0)
    ideal = 32760.0 * np.exp(1j * phases)
    max_err = float(np.max(np.abs(arr - ideal)))
    cocotb.log.info(f"Backpressure max error: {max_err:.3f} LSB")
    assert max_err < 2.5


def test_runner():
    """Pytest entry point invoking Verilator via cocotb.runner."""
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("verilator")
    runner.build(
        verilog_sources=[root / "rtl" / "common" / "sv_nco_cordic.sv"],
        hdl_toplevel="sv_nco_cordic",
        build_dir=root / "sim" / "sim_build" / "nco",
        build_args=["--sv", "-Wall"],
        always=True,
    )
    runner.test(
        hdl_toplevel="sv_nco_cordic",
        test_module="test_nco",
        extra_env={"PYTHONPATH": str(root / "sim")},
    )
