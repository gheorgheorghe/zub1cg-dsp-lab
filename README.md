# ZUBoard 1CG DSP & Control Lab

A modular, post-MATLAB FPGA SoC engineering platform for the **Avnet ZUBoard 1CG** (AMD Zynq UltraScale+ `XCZU1CG`) built on **Linux + Vivado/Vitis 2024.2 + SystemVerilog + `cocotb`/Verilator + Python**.

See [docs/PLAN.md](docs/PLAN.md) for the complete Stage 0 → Stage 3 roadmap (Reusable Shell → MEMS Vibratory Gyroscope HIL → Radar/RF DSP + Tracking → Real IMU Sensor Fusion & Gimbal Control).

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
