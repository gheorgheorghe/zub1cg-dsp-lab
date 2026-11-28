# ZUBoard 1CG DSP & Control Lab

A modular, post-MATLAB FPGA SoC engineering platform for the **Avnet ZUBoard 1CG** (AMD Zynq UltraScale+ `XCZU1CG`) built on **Linux + Vivado/Vitis 2024.2 + SystemVerilog + `cocotb`/Verilator + Python**.

See [docs/PLAN.md](docs/PLAN.md) for the complete Stage 0 → Stage 3 roadmap (Reusable Shell → MEMS Vibratory Gyroscope HIL → Radar/RF DSP + Tracking → Real IMU Sensor Fusion & Gimbal Control).

---

## Interactive Study Notes & Live Silicon Reports

All engineering explainers, interactive browser simulators, hardware bring-up postmortems, and live ZUBoard 1CG plots are hosted live on **GitHub Pages**:

👉 **[Browse All Study Notes (Mobile-Friendly Index)](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/)**

| Topic | Live Interactive Page | Source |
|---|---|---|
| **Start Here — Gyro-Land Master Summary** | [Stage 0 & Stage 1 Executive Summary](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/gyro_land_summary.html) | [studies/gyro_land_summary.html](studies/gyro_land_summary.html) |
| **CVG Crash Course (Newcomer Primer)** | [2D Wineglass Physics, DX/DY/SX/SY & ADEV Intuition](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/cvg_crash_course.html) | [studies/cvg_crash_course.html](studies/cvg_crash_course.html) |
| **64-Probe Cloverleaf Mode Pinning** | [64-ADC Die Station: Cloverleaf Ring-Up & Fab Pinning](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/cloverleaf_64probe_pinning.html) | [studies/cloverleaf_64probe_pinning.html](studies/cloverleaf_64probe_pinning.html) |
| **Live Silicon Results (v1 → v2 High-Q)** | [Fixed-Point Rounding, High-Q v2 & +30 °C Thermal Ramp](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/fpga_fixed_point_vs_float64.html) | [studies/fpga_fixed_point_vs_float64.html](studies/fpga_fixed_point_vs_float64.html) |
| **Nav-Grade MEMS DRG Hurdles** | [Why Si DRGs Stall at 0.2 °/hr + Interactive ADEV Lab](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/nav_grade_drg_hurdles.html) | [studies/nav_grade_drg_hurdles.html](studies/nav_grade_drg_hurdles.html) |
| **Improving Silicon DRGs (Beyond Quartz)** | [Beating TED Geometrically + Interactive Zener Q(f₀, b) Lab](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/silicon_drg_improvement_paths.html) | [studies/silicon_drg_improvement_paths.html](studies/silicon_drg_improvement_paths.html) |
| **Electrode & Pin Allocation Trades** | [More Contact Points & Sense vs. Drive Pin Trades](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/drg_electrode_pin_trades.html) | [studies/drg_electrode_pin_trades.html](studies/drg_electrode_pin_trades.html) |
| **Differential Electrode Pairs** | [Push-Pull Sensing/Actuation, Gap Mismatch & 4-Quad Shock](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/differential_electrode_pairs.html) | [studies/differential_electrode_pairs.html](studies/differential_electrode_pairs.html) |
| **Ringdown, Q-Fit & 100V Tuning Demo** | [Interactive Ringdown Q-Fit Bench + g-Sens & Realism Drivers](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/ringdown_qfit_and_bias_realism.html) | [studies/ringdown_qfit_and_bias_realism.html](studies/ringdown_qfit_and_bias_realism.html) |
| **Advanced CVG Physics** | [Electrostatic Softening, Duffing Overdrive & Whole-Angle HRG](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/advanced_gyro_physics.html) | [studies/advanced_gyro_physics.html](studies/advanced_gyro_physics.html) |
| **Control Theory (Interactive Lab)** | [Wait: PI, but no D? (Demodulation + Feedforward)](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/pi_vs_pid_cvg_explainer.html) | [studies/pi_vs_pid_cvg_explainer.html](studies/pi_vs_pid_cvg_explainer.html) |
| **MEMS Physics & Vacuum Packaging** | [100+ °/s Quadrature & Gold Getter Vacuum Packages](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/drg_quadrature_and_vacuum_explainer.html) | [studies/drg_quadrature_and_vacuum_explainer.html](studies/drg_quadrature_and_vacuum_explainer.html) |
| **ADEV Comparison Chart** | [Silicon vs. Quartz DRG vs. Micro-HRG vs. Space HRG](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/adev_presets_comparison.png) | [studies/adev_presets_comparison.png](studies/adev_presets_comparison.png) |
| **Bring-Up Postmortem · Bug #01** | [A53 / R5F LPDDR4 Memory Collision & Fix](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/bugs/bug01_a53_r5f_ddr_collision.html) | [studies/bugs/bug01_a53_r5f_ddr_collision.html](studies/bugs/bug01_a53_r5f_ddr_collision.html) |
| **Bring-Up Postmortem · Bug #02** | [KSZ9131RNX PHY & RGMII TXC 2 ns DLL Bypass](https://gheorgheorghe.github.io/zub1cg-dsp-lab/studies/bugs/bug02_ksz9131_rgmii_txc_dll.html) | [studies/bugs/bug02_ksz9131_rgmii_txc_dll.html](studies/bugs/bug02_ksz9131_rgmii_txc_dll.html) |

---

## Quick Start (Linux)

### 1. Install Open-Source RTL Simulation & Python Tools
```bash
# Ubuntu / Debian example
sudo apt update && sudo apt install -y verilator gtkwave python3-venv python3-pip build-essential

# Create Python environment with cocotb + scientific stack
make venv
```

### 2. Run Stage 0 RTL Verification (`cocotb` + Verilator)
```bash
# Runs sv_nco_cordic and stage0_dsp_slot testbenches (verifies >90 dBc SFDR & AXI-Stream framing)
make sim
```

### 3. Build the ZUBoard 1CG Bitstream & `.xsa` (Vivado 2024.2)
1. Source Vivado 2024.2:
   ```bash
   source /tools/Xilinx/Vivado/2024.2/settings64.sh
   ```
2. Install Avnet `zub1cg` board files (choose one):
   - **GUI:** Vivado → *Tools → Vivado Store → Boards* → install **Avnet ZUBoard 1CG**.
   - **CLI / Reproducible:**
     ```bash
     git clone --branch 2024.2 --depth 1 https://github.com/Xilinx/XilinxBoardStore.git ../XilinxBoardStore
     export BOARD_REPO=../XilinxBoardStore/boards/Avnet/zub1cg
     ```
3. Run the Tcl build:
   ```bash
   make bit
   # Outputs: hw/build/zub1cg_stage0.xsa + timing & utilization reports
   ```

### 4. Build Firmware in Vitis Unified IDE 2024.2
1. Launch Vitis 2024.2, create a **Platform Component** from `hw/build/zub1cg_stage0.xsa`.
2. Add two application domains:
   - `a53_net_bridge` (Cortex-A53 `standalone` + `lwip220` library enabled) using `sw/a53_net_bridge/src/main.c`.
   - `r5f_realtime` (Cortex-R5F `standalone`) using `sw/r5f_realtime/src/main.c`.
3. Boot the board over JTAG/SD, connect Gigabit Ethernet (`192.168.1.10`), and run the host client:
   ```bash
   # Test host client offline first (mock mode, no board required):
   .venv/bin/python host/zuboard_client.py --mock --freq-hz 16000 --samples 2048

   # Run against live ZUBoard 1CG over GbE:
   .venv/bin/python host/zuboard_client.py --host 192.168.1.10 --freq-hz 16000 --samples 2048
   ```
