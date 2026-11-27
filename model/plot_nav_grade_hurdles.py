#!/usr/bin/env python3
"""Generate quantitative ADEV waterfall showing the biggest hurdles to Nav-Grade
(<= 0.01 deg/hr) MEMS DRG bias stability and the exact impact of each improvement.

Stages modeled (cumulative engineering improvements from Tactical Si DRG -> Nav-Grade DRG):
  0. Baseline Silicon MEMS DRG (Q=100k, open-loop quad, uncomp. temp)
       ARW = 0.015 deg/rt-hr | BI ~ 0.25 deg/hr  | RRW = 1.6 deg/hr/rt-hr
  1. + Low-TED Quartz DRG & <1 uTorr NEG Vacuum (Q: 100k -> 800k)
       ARW = 0.0045 deg/rt-hr | BI ~ 0.085 deg/hr | RRW = 0.55 deg/hr/rt-hr
  2. + Closed-Loop Quad Nulling (>68 dB CLQ + shared-clock digital demod)
       ARW = 0.0040 deg/rt-hr | BI ~ 0.032 deg/hr | RRW = 0.22 deg/hr/rt-hr
  3. + Mode Reversal / Virtual Carouseling (cancels in-phase anisodamping!)
       ARW = 0.0032 deg/rt-hr | BI ~ 0.0085 deg/hr | RRW = 0.055 deg/hr/rt-hr  <-- CROSSES NAV-GRADE!
  4. + Self-Thermometry (0.006 C) + Micro-Oven + HF Carrier & 24-bit DSP
       ARW = 0.0020 deg/rt-hr | BI ~ 0.0032 deg/hr | RRW = 0.012 deg/hr/rt-hr  <-- SOLID NAV-GRADE!
"""

from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from model.plot_adev_comparison import generate_adev_curve


def main() -> None:
    root = Path(__file__).resolve().parents[1]

    stages = [
        {
            "short": "0. Baseline Si DRG\n(Q=100k, raw)",
            "label": "0. Baseline Silicon DRG (Q=100k, raw quad & temp)",
            "arw": 0.015,
            "bi": 0.24,
            "rrw": 1.60,
            "color": "#f87171",
            "seed": 201,
        },
        {
            "short": "1. + Quartz High-Q\n& <1 µTorr Vacuum",
            "label": "1. + Quartz Low-TED (Q=800k) & <1 µTorr Getter",
            "arw": 0.0045,
            "bi": 0.080,
            "rrw": 0.55,
            "color": "#fbbf24",
            "seed": 202,
        },
        {
            "short": "2. + Closed-Loop\nQuad Null (CLQ)",
            "label": "2. + Closed-Loop Quad Nulling (kills Δφ leakage)",
            "arw": 0.0040,
            "bi": 0.030,
            "rrw": 0.22,
            "color": "#c084fc",
            "seed": 203,
        },
        {
            "short": "3. + Mode Reversal\n(Virtual Carousel)",
            "label": "3. + Mode Reversal / Carousel (cancels anisodamping)",
            "arw": 0.0032,
            "bi": 0.0080,
            "rrw": 0.050,
            "color": "#38bdf8",
            "seed": 204,
        },
        {
            "short": "4. + Self-Temp Oven\n& HF-Carrier 24b DSP",
            "label": "4. + Self-Thermometry Oven & HF-Carrier 24b DSP",
            "arw": 0.0020,
            "bi": 0.0035,
            "rrw": 0.022,
            "color": "#34d399",
            "seed": 205,
        },
    ]

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(14.5, 6.6), facecolor="#0b0f17",
        gridspec_kw={"width_ratios": [1.55, 1.0]},
    )

    # --- Panel A: Overlapping ADEV Bathtub Waterfall ---
    ax1.set_facecolor("#0f172a")
    measured_bis = []
    for st in stages:
        taus, adevs, bi_est, tau_min = generate_adev_curve(
            st["arw"], st["bi"], st["rrw"], fs_hz=20.0, dur_s=4000.0, seed=st["seed"]
        )
        measured_bis.append(bi_est)
        lbl = f"{st['label']}  (BI≈{bi_est:.4f} °/hr)"
        ax1.loglog(taus, adevs, lw=2.8, color=st["color"], label=lbl, zorder=4)
        idx_min = int(np.argmin(adevs))
        ax1.scatter(
            [taus[idx_min]], [adevs[idx_min]],
            s=56, facecolor=st["color"], edgecolor="#0b0f17", linewidth=1.6, zorder=6,
        )
        print(f"{st['label']:54s} | BI≈{bi_est:.4f} °/hr (min ADEV={adevs[idx_min]:.4f} @ tau={tau_min:.0f}s)")

    # Shaded Navigation-Grade target zone (<= 0.01 deg/hr)
    ax1.axhspan(1e-4, 0.01, color="#10b981", alpha=0.10, zorder=1)
    ax1.axhline(0.01, color="#34d399", ls="--", lw=1.6, zorder=3)
    ax1.text(0.058, 0.0112, "NAVIGATION-GRADE THRESHOLD (0.01 °/hr)", color="#34d399",
             fontsize=9.5, fontweight="bold", va="bottom")

    ax1.axhline(0.1, color="#94a3b8", ls=":", lw=1.0, zorder=2)
    ax1.text(0.058, 0.115, "Tactical Grade (~0.1 °/hr)", color="#94a3b8", fontsize=9.0, va="bottom")

    ax1.set_xlim(0.05, 1000.0)
    ax1.set_ylim(1.2e-3, 12.0)
    ax1.set_xlabel("Cluster Averaging Time τ (seconds)", color="#e6edf7", fontsize=11.5, fontweight="bold")
    ax1.set_ylabel("Overlapping Allan Deviation σ(τ)  [°/hr]", color="#e6edf7", fontsize=11.5, fontweight="bold")
    ax1.set_title("A. Cumulative ADEV Waterfall: Tactical → Nav-Grade DRG", color="#ffffff", fontsize=12.5, fontweight="bold", pad=10)
    ax1.tick_params(colors="#cbd5e1", which="both")
    for spine in ax1.spines.values():
        spine.set_color("#334155")
    ax1.grid(True, which="major", color="#1e293b", lw=0.9)
    ax1.grid(True, which="minor", color="#1e293b", lw=0.45, alpha=0.7)

    leg = ax1.legend(loc="lower left", frameon=True, facecolor="#0b1220", edgecolor="#334155", fontsize=8.3)
    for txt in leg.get_texts():
        txt.set_color("#e6edf7")

    # --- Panel B: Bias Instability Waterfall Bar Chart (Log Scale) ---
    ax2.set_facecolor("#0f172a")
    y_pos = np.arange(len(stages))[::-1]
    colors = [st["color"] for st in stages]
    labels = [st["short"] for st in stages]

    bars = ax2.barh(y_pos, measured_bis, height=0.58, color=colors, edgecolor="#0b0f17", linewidth=1.2, zorder=4)
    ax2.set_xscale("log")
    ax2.set_xlim(1e-3, 3.5)
    ax2.set_ylim(-0.65, 4.65)
    ax2.axvline(0.01, color="#34d399", ls="--", lw=1.6, zorder=5)
    ax2.axvspan(1e-3, 0.01, color="#10b981", alpha=0.10, zorder=1)
    ax2.text(0.0082, 4.55, "Nav-Grade ≤ 0.01 °/hr", color="#34d399", fontsize=8.5,
             fontweight="bold", ha="right", va="top")

    base_bi = measured_bis[0]
    for bar, bi_val in zip(bars, measured_bis):
        gain = base_bi / bi_val
        tag = f" {bi_val:.4f} °/hr ({gain:.0f}×)" if gain > 1.2 else f" {bi_val:.3f} °/hr (1×)"
        ax2.text(bi_val * 1.15, bar.get_y() + bar.get_height() / 2.0, tag,
                 color="#f1f5f9", fontsize=9.0, fontweight="bold", va="center", ha="left")

    ax2.set_yticks(y_pos)
    ax2.set_yticklabels(labels, color="#e6edf7", fontsize=9.2, fontweight="bold")
    ax2.set_xlabel("Bias Instability BI  [°/hr]  (Lower is Better)", color="#e6edf7", fontsize=11.0, fontweight="bold")
    total_gain = base_bi / measured_bis[-1]
    ax2.set_title(f"B. Total BI Reduction ({total_gain:.0f}× Gain)", color="#ffffff", fontsize=12.5, fontweight="bold", pad=10)
    ax2.tick_params(colors="#cbd5e1", which="both")
    for spine in ax2.spines.values():
        spine.set_color("#334155")
    ax2.grid(True, axis="x", which="major", color="#1e293b", lw=0.9)
    ax2.grid(True, axis="x", which="minor", color="#1e293b", lw=0.45, alpha=0.7)

    fig.suptitle(
        "Closing the Gap to Navigation-Grade MEMS DRGs (≤ 0.01 °/hr Bias Instability)",
        color="#ffffff", fontsize=14.0, fontweight="bold", y=0.98,
    )
    plt.tight_layout(rect=[0, 0, 1, 0.94])

    for out_path in [
        root / "studies" / "nav_grade_drg_hurdles_adev.png",
        root / "model" / "plots" / "nav_grade_drg_hurdles_adev.png",
    ]:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=160, facecolor=fig.get_facecolor())
        print(f"Saved -> {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
