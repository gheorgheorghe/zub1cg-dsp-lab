#!/usr/bin/env python3
"""Host UDP client & live spectrum viewer for ZUBoard 1CG Stage 0 shell.

Supports --mock mode so you can verify plotting and spectrum analysis offline
before connecting the physical ZUBoard 1CG.
"""

from __future__ import annotations
import argparse
import math
import socket
import struct
import numpy as np

PL_CLK_HZ = 100_000_000.0
SAMPLES_PER_PKT = 256

# Register byte offsets (matching rtl/common/axil_regfile.sv)
REG_CTRL       = 0x00
REG_FREQ_WORD  = 0x04
REG_PKT_LEN    = 0x08
REG_PHASE_OFF  = 0x0C
REG_STATUS     = 0x30
REG_SAMPLE_CNT = 0x34
REG_VERSION    = 0x38

CMD_REG_READ  = 0x01
CMD_REG_WRITE = 0x02
CMD_DMA_CAP   = 0x03


class ZUBoardClient:
    def __init__(self, host: str = "192.168.1.10", port: int = 5000, timeout: float = 2.0, mock: bool = False):
        self.host = host
        self.port = port
        self.mock = mock
        self._mock_regs = {
            REG_CTRL: 0x01,
            REG_FREQ_WORD: round(16_000.0 / PL_CLK_HZ * (2**32)),
            REG_PKT_LEN: 1024,
            REG_PHASE_OFF: 0,
            REG_STATUS: 0x03,
            REG_SAMPLE_CNT: 0,
            REG_VERSION: 0x5A550001,
        }
        self.sock = None
        if not mock:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(timeout)

    def reg_read(self, offset: int) -> int:
        if self.mock:
            return self._mock_regs.get(offset, 0)
        assert self.sock is not None
        self.sock.sendto(struct.pack("<II", CMD_REG_READ, offset), (self.host, self.port))
        data, _ = self.sock.recvfrom(64)
        rsp, val = struct.unpack("<II", data[:8])
        assert rsp == 0x81, f"Unexpected REG_READ response: 0x{rsp:02x}"
        return val

    def reg_write(self, offset: int, value: int) -> int:
        if self.mock:
            self._mock_regs[offset] = value & 0xFFFFFFFF
            return self._mock_regs[offset]
        assert self.sock is not None
        self.sock.sendto(struct.pack("<III", CMD_REG_WRITE, offset, value & 0xFFFFFFFF), (self.host, self.port))
        data, _ = self.sock.recvfrom(64)
        rsp, val = struct.unpack("<II", data[:8])
        assert rsp == 0x82, f"Unexpected REG_WRITE response: 0x{rsp:02x}"
        return val

    def set_nco_freq(self, freq_hz: float, clk_hz: float = PL_CLK_HZ) -> int:
        ftw = round((freq_hz / clk_hz) * (2**32)) & 0xFFFFFFFF
        return self.reg_write(REG_FREQ_WORD, ftw)

    def capture_iq(self, n_samples: int) -> np.ndarray:
        """Capture `n_samples` packed Q1.15 {sin, cos} samples from AXI-DMA S2MM."""
        if self.mock:
            ftw = self._mock_regs[REG_FREQ_WORD]
            phases = ((np.arange(n_samples, dtype=np.uint64) * ftw) & 0xFFFFFFFF) / (2**32) * 2.0 * np.pi
            iq = np.round(32760.0 * np.cos(phases)) + 1j * np.round(32760.0 * np.sin(phases))
            self._mock_regs[REG_SAMPLE_CNT] += n_samples
            return iq

        assert self.sock is not None
        self.sock.sendto(struct.pack("<II", CMD_DMA_CAP, n_samples), (self.host, self.port))
        expected_pkts = math.ceil(n_samples / SAMPLES_PER_PKT)
        chunks: dict[int, bytes] = {}
        for _ in range(expected_pkts):
            data, _ = self.sock.recvfrom(2048)
            rsp, seq = struct.unpack("<II", data[:8])
            assert rsp == 0x83, f"Unexpected DMA_CAP response: 0x{rsp:02x}"
            chunks[seq] = data[8:]

        raw = b"".join(chunks[i] for i in range(expected_pkts))
        words = np.frombuffer(raw[: n_samples * 4], dtype="<i2").reshape(-1, 2)
        return words[:, 0].astype(np.float64) + 1j * words[:, 1].astype(np.float64)


def main() -> None:
    ap = argparse.ArgumentParser(description="ZUBoard 1CG Stage 0 UDP Client")
    ap.add_argument("--host", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--freq-hz", type=float, default=1_000_000.0, help="NCO frequency in Hz")
    ap.add_argument("--samples", type=int, default=2048)
    ap.add_argument("--mock", action="store_true", help="Run offline without physical board")
    ap.add_argument("--no-plot", action="store_true", help="Skip matplotlib window")
    args = ap.parse_args()

    client = ZUBoardClient(host=args.host, port=args.port, mock=args.mock)
    ver = client.reg_read(REG_VERSION)
    print(f"Connected ({'MOCK' if args.mock else args.host}): VERSION = 0x{ver:08X}")

    ftw = client.set_nco_freq(args.freq_hz)
    actual_hz = ftw / (2**32) * PL_CLK_HZ
    print(f"Programmed NCO: target={args.freq_hz:.2f} Hz, actual={actual_hz:.4f} Hz (FTW=0x{ftw:08X})")

    iq = client.capture_iq(args.samples)
    win = np.hanning(len(iq))
    spec = np.fft.fftshift(np.fft.fft(iq * win))
    freqs_mhz = np.fft.fftshift(np.fft.fftfreq(len(iq), d=1.0 / PL_CLK_HZ)) / 1e6
    spec_db = 20.0 * np.log10(np.abs(spec) / np.max(np.abs(spec)) + 1e-15)
    peak_mhz = freqs_mhz[int(np.argmax(spec_db))]
    print(f"Captured {len(iq)} I/Q samples | Peak spectrum = {peak_mhz:.6f} MHz | Peak mag = {np.max(np.abs(iq)):.1f} LSB")

    if not args.no_plot:
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6))
        ax1.plot(iq.real[:256], label="I (cos)")
        ax1.plot(iq.imag[:256], label="Q (sin)")
        ax1.set_title(f"ZUBoard 1CG NCO Capture ({actual_hz/1e3:.3f} kHz)")
        ax1.set_xlabel("Sample")
        ax1.set_ylabel("Q1.15 LSB")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(freqs_mhz, spec_db)
        ax2.set_xlabel("Frequency (MHz)")
        ax2.set_ylabel("Magnitude (dBFS)")
        ax2.set_ylim(-120, 5)
        ax2.grid(True)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
