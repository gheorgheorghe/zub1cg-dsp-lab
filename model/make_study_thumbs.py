#!/usr/bin/env python3
"""Generate crisp 280x180 dark-mode figure thumbnails for all 15 study cards.

- For the 10 studies with existing Matplotlib / live silicon figures, crops the
  most iconic plot panel directly from the real PNG at high resolution.
- For the 5 pure interactive HTML studies, renders a faithful dark-mode mini-figure
  matching the exact centerpiece visual inside each study page.
- Also writes /tmp/thumbs_contact_sheet.png for visual verification.
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
from PIL import Image, ImageOps
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


STUDIES_DIR = Path(__file__).resolve().parent.parent / "studies"
THUMBS_DIR = STUDIES_DIR / "thumbs"
THUMBS_DIR.mkdir(parents=True, exist_ok=True)
TW, TH = 280, 180


def crop_panel(src_name: str, box_frac: tuple[float, float, float, float], dst_name: str) -> None:
    """Crop normalized (x0, y0, x1, y1) region from an existing study PNG and resize to TWxTH."""
    im = Image.open(STUDIES_DIR / src_name).convert("RGB")
    w, h = im.size
    x0, y0, x1, y1 = box_frac
    cropped = im.crop((int(w * x0), int(h * y0), int(w * x1), int(h * y1)))
    thumb = ImageOps.fit(cropped, (TW, TH), method=Image.Resampling.LANCZOS)
    thumb.save(THUMBS_DIR / dst_name, optimize=True)


def init_fig() -> tuple[plt.Figure, plt.Axes]:
    fig, ax = plt.subplots(figsize=(2.8, 1.8), dpi=100)
    fig.patch.set_facecolor("#0b0f17")
    ax.set_facecolor("#111827")
    for sp in ax.spines.values():
        sp.set_color("#26334a")
    ax.tick_params(colors="#64748b", labelsize=6, length=2)
    return fig, ax


def make_summary_thumb() -> None:
    """Clean dark-mode ZUBoard 1CG Master Summary figure (SoC + Live Step + KPIs)."""
    fig, ax = init_fig()
    # Clean 100 deg/s closed-loop FTR + Feedforward step curve + KPI banner
    t = np.linspace(0, 400, 260)
    u = np.where(t >= 180, 100.0, 0.0)
    dt_step = np.maximum(t - 180.0, 0.0)
    ftr = np.where(t >= 180, 100.0 * (1.0 - np.exp(-dt_step / 14.0) * (np.cos(dt_step / 14.0) - 0.6 * np.sin(dt_step / 14.0))), 0.0)
    ff = np.where(t >= 180, 100.0 * (1.0 + 0.06 * np.exp(-dt_step / 10.0)), 0.0)

    ax.fill_between(t, 0, ff, color="#34d399", alpha=0.11)
    ax.plot(t, u, ":", color="#94a3b8", lw=1.2)
    ax.plot(t, ftr, color="#38bdf8", lw=1.9, label="FTR")
    ax.plot(t, ff, color="#34d399", lw=2.3, label="FF")

    # Sleek header pills inside thumbnail (well clear of the step curve!)
    ax.text(14, 134, "ZUBoard 1CG · Stage 1 v3", color="#34d399", fontsize=7.3, fontweight="bold")
    ax.text(14, 46, "0.027 °/hr\nQ = 100,003\n0.006 °C", color="#cbd5e1", fontsize=6.4, fontweight="bold")
    ax.text(290, 30, "+100 °/s\nLive Step", color="#bae6fd", fontsize=6.8, fontweight="bold", ha="center")

    ax.set_xlim(0, 400); ax.set_ylim(-8, 154)
    ax.grid(True, color="#1e293b", ls="--", lw=0.6)
    ax.set_xticks([]); ax.set_yticks([])
    plt.subplots_adjust(0.04, 0.06, 0.96, 0.94)
    fig.savefig(THUMBS_DIR / "summary.png", dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def make_cvg_crash_thumb() -> None:
    """n=2 wineglass breathing + Coriolis mode mini-figure."""
    fig, ax = init_fig()
    th = np.linspace(0, 2 * math.pi, 240)
    r0 = np.ones_like(th)
    r_drv = 1.0 + 0.24 * np.cos(2 * th)
    r_sns = 1.0 + 0.24 * np.sin(2 * th)
    ax.plot(r0 * np.cos(th), r0 * np.sin(th), ":", color="#475569", lw=1.1)
    ax.fill(r_drv * np.cos(th), r_drv * np.sin(th), color="#38bdf8", alpha=0.16)
    ax.plot(r_drv * np.cos(th), r_drv * np.sin(th), color="#38bdf8", lw=2.2)
    ax.plot(r_sns * np.cos(th), r_sns * np.sin(th), "--", color="#fbbf24", lw=1.7)
    ax.scatter([1.24, -1.24, 0, 0], [0, 0, 0.76, -0.76], s=18, color="#38bdf8", zorder=5)
    ax.text(-1.55, 1.08, "n=2 Wineglass", color="#e2e8f0", fontsize=7.5, fontweight="bold")
    ax.text(0.45, -1.28, "DX / SX / SY", color="#fbbf24", fontsize=7.0, fontweight="bold")
    ax.set_aspect("equal")
    ax.set_xlim(-1.65, 1.65); ax.set_ylim(-1.40, 1.35)
    ax.axis("off")
    plt.subplots_adjust(0.02, 0.02, 0.98, 0.98)
    fig.savefig(THUMBS_DIR / "cvg_crash.png", dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def make_pi_vs_pid_thumb() -> None:
    """PI (smooth green) vs PID (noisy red derivative blowup) mini-figure."""
    fig, ax = init_fig()
    rng = np.random.default_rng(7)
    t = np.linspace(0, 1.0, 200)
    step = 1.0 - np.exp(-t / 0.14)
    pi_sig = step + rng.normal(0, 0.015, len(t))
    pid_sig = step + 0.12 * np.sin(28 * t) * np.exp(-t * 3) + rng.normal(0, 0.11, len(t))
    ax.axhline(1.0, color="#475569", ls=":", lw=1.0)
    ax.plot(t, pid_sig, color="#f87171", lw=1.2, alpha=0.88, label="PID (D noise!)")
    ax.plot(t, pi_sig, color="#34d399", lw=2.2, label="PI (Clean)")
    ax.set_xlim(0, 1.0); ax.set_ylim(-0.15, 1.48)
    ax.grid(True, color="#1e293b", ls="--", lw=0.6)
    ax.legend(loc="lower right", fontsize=6.5, framealpha=0.90, facecolor="#0b0f17", edgecolor="#334155", labelcolor="#e2e8f0")
    ax.set_xticks([]); ax.set_yticks([])
    plt.subplots_adjust(0.05, 0.07, 0.96, 0.93)
    fig.savefig(THUMBS_DIR / "pi_vs_pid.png", dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def make_quad_vacuum_thumb() -> None:
    """LCC Gold Getter Vacuum Q(P) S-curve mini-figure."""
    fig, ax = init_fig()
    p_torr = np.logspace(-7, -1, 200)
    q_ted = 1.2e5
    q_gas = 4.0e2 / p_torr
    q_tot = 1.0 / (1.0 / q_ted + 1.0 / q_gas)
    ax.loglog(p_torr, q_tot, color="#fbbf24", lw=2.4)
    ax.axvspan(1e-7, 1e-5, color="#34d399", alpha=0.16)
    ax.axhline(q_ted, color="#38bdf8", ls="--", lw=1.1)
    ax.text(2e-7, 1.5e5, "Q_TED Plateau", color="#bae6fd", fontsize=6.8, fontweight="bold")
    ax.text(1.5e-7, 8e3, "NEG Getter\n<1 µTorr", color="#a7f3d0", fontsize=6.5, fontweight="bold")
    ax.set_xlim(1e-7, 1e-1); ax.set_ylim(3e3, 3e5)
    ax.grid(True, which="both", color="#1e293b", ls="--", lw=0.5)
    ax.set_xticks([]); ax.set_yticks([])
    plt.subplots_adjust(0.06, 0.08, 0.96, 0.93)
    fig.savefig(THUMBS_DIR / "quad_vacuum.png", dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def make_bug02_rgmii_thumb() -> None:
    """RGMII TXC 2ns DLL Eye Diagram mini-figure."""
    fig, ax = init_fig()
    t = np.linspace(0, 2 * math.pi, 200)
    eye_hi = np.tanh(3.5 * np.sin(t))
    ax.fill_between(t, -eye_hi, eye_hi, color="#34d399", alpha=0.14)
    ax.plot(t, eye_hi, color="#34d399", lw=2.0)
    ax.plot(t, -eye_hi, color="#34d399", lw=2.0)
    # Bad edge-aligned TXC (red) vs fixed 2ns shifted TXC (cyan)
    ax.axvline(math.pi, color="#f87171", ls="--", lw=1.8)
    ax.axvline(math.pi * 0.5, color="#38bdf8", ls="-", lw=2.2)
    ax.text(0.45, 0.72, "2ns DLL\n0x06D1", color="#bae6fd", fontsize=6.7, fontweight="bold")
    ax.text(3.35, -0.82, "0x16D1\nFAIL", color="#fecaca", fontsize=6.7, fontweight="bold")
    ax.set_xlim(0, 2 * math.pi); ax.set_ylim(-1.25, 1.25)
    ax.set_xticks([]); ax.set_yticks([])
    plt.subplots_adjust(0.05, 0.07, 0.96, 0.93)
    fig.savefig(THUMBS_DIR / "bug02_rgmii.png", dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def make_bug01_ddr_thumb() -> None:
    """A53 / R5F LPDDR4 Memory Collision -> Partitioned Map mini-figure."""
    fig, ax = init_fig()
    # Left bar: Collision; Right bar: Fixed partition
    ax.barh([1.15], [0.75], left=0.05, height=0.38, color="#38bdf8", edgecolor="#0b0f17")
    ax.barh([1.15], [0.25], left=0.05, height=0.38, color="#f87171", hatch="///", edgecolor="#fff")
    ax.text(0.48, 1.15, "COLLISION @ 0x100000!", color="#fecaca", fontsize=6.8, fontweight="bold", va="center")

    ax.barh([0.45], [0.60], left=0.05, height=0.38, color="#34d399", edgecolor="#0b0f17")
    ax.barh([0.45], [0.26], left=0.68, height=0.38, color="#c084fc", edgecolor="#0b0f17")
    ax.text(0.35, 0.45, "A53 (0x0..768M)", color="#062019", fontsize=6.6, fontweight="bold", ha="center", va="center")
    ax.text(0.81, 0.45, "R5F\n0x30M", color="#1a0b2e", fontsize=6.3, fontweight="bold", ha="center", va="center")

    ax.set_xlim(0, 1.0); ax.set_ylim(0.05, 1.60)
    ax.axis("off")
    plt.subplots_adjust(0.03, 0.05, 0.97, 0.95)
    fig.savefig(THUMBS_DIR / "bug01_ddr.png", dpi=100, facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> None:
    # 1. Crop flagship panels from the 9 existing high-res PNGs + dedicated Master Summary thumb
    make_summary_thumb()
    crop_panel("cloverleaf_64probe_pinning.png", (0.04, 0.08, 0.48, 0.54), "cloverleaf64.png")
    crop_panel("ringdown_qfit_and_tuning.png", (0.50, 0.08, 0.99, 0.53), "ringdown.png")
    crop_panel("nav_grade_drg_hurdles_adev.png", (0.01, 0.08, 0.65, 0.96), "nav_hurdles.png")
    crop_panel("silicon_drg_paths.png",        (0.01, 0.08, 0.50, 0.53), "silicon_paths.png")
    crop_panel("electrode_allocation_trades.png", (0.01, 0.08, 0.50, 0.53), "electrode_pins.png")
    crop_panel("differential_electrode_pairs.png", (0.50, 0.08, 0.99, 0.53), "diff_pairs.png")
    crop_panel("adev_presets_comparison.png",  (0.02, 0.04, 0.98, 0.96), "adev_compare.png")
    crop_panel("hw_adev_30s_v2.png",           (0.02, 0.04, 0.98, 0.96), "fpga_v2.png")
    crop_panel("advanced_softening_duffing.png", (0.01, 0.49, 0.50, 0.97), "advanced_phys.png")

    # 2. Render crisp matching dark-mode figures for the 5 pure interactive HTML studies
    make_cvg_crash_thumb()
    make_pi_vs_pid_thumb()
    make_quad_vacuum_thumb()
    make_bug02_rgmii_thumb()
    make_bug01_ddr_thumb()

    # 3. Build a 5x3 contact sheet in /tmp to visually inspect all 15 thumbnails in one read!
    names = [
        "summary.png", "cvg_crash.png", "cloverleaf64.png", "ringdown.png", "pi_vs_pid.png",
        "quad_vacuum.png", "nav_hurdles.png", "silicon_paths.png", "electrode_pins.png", "diff_pairs.png",
        "adev_compare.png", "fpga_v2.png", "advanced_phys.png", "bug02_rgmii.png", "bug01_ddr.png",
    ]
    sheet = Image.new("RGB", (5 * TW + 6 * 8, 3 * TH + 4 * 8), (11, 15, 23))
    for i, nm in enumerate(names):
        im = Image.open(THUMBS_DIR / nm)
        r, c = divmod(i, 5)
        sheet.paste(im, (8 + c * (TW + 8), 8 + r * (TH + 8)))
    sheet.save("/tmp/thumbs_contact_sheet.png")
    print(f"Generated {len(names)} thumbnails in {THUMBS_DIR} + /tmp/thumbs_contact_sheet.png")


if __name__ == "__main__":
    main()
