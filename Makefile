SHELL := /bin/bash
VENV  := .venv
PY    := $(VENV)/bin/python
PIP   := $(VENV)/bin/pip
SIM   ?= verilator
VIVADO ?= vivado
VITIS  ?= vitis
XSDB   ?= xsdb
BOARD_REPO ?=
PROJECT ?= stage1_mems
JOBS ?= 8

.PHONY: help venv board-files lint sim sim-nco sim-slot sim-lockin sim-plant sim-closed-loop sim-stage1-axis test-model plot-drg plot-hrg plot-space-hrg bd bit sw boot mock-host clean

help:
	@echo "Targets:"
	@echo "  make venv          Create Python venv and install cocotb/numpy/scipy/pytest"
	@echo "  make board-files   Download Avnet ZUBoard 1CG board files for Vivado 2024.2"
	@echo "  make test-model    Run Stage 1 DRG/HRG golden model & bit-width pytest suite"
	@echo "  make plot-drg      Plot DRG (11 kHz, Q=100k) closed-loop simulation"
	@echo "  make plot-hrg      Plot Micro-HRG (11 kHz, Q=1M) closed-loop simulation"
	@echo "  make plot-space-hrg Plot Space HRG (4.2 kHz, Q=5M) closed-loop simulation"
	@echo "  make lint          Run Verilator lint on all SystemVerilog RTL"
	@echo "  make sim           Run all cocotb + Verilator RTL testbenches"
	@echo "  make bd            Generate Vivado 2024.2 BD (PROJECT=stage0|stage1_mems)"
	@echo "  make bit           Full Vivado synth/impl/bitstream + .xsa (PROJECT=$(PROJECT))"
	@echo "  make mock-host     Run host UDP client in offline --mock mode"
	@echo "  make clean         Remove simulation and Vivado build outputs"

venv:
	python3 -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt

board-files:
	rm -rf hw/board_files/.tmp_xbs
	git clone --depth 1 --filter=blob:none --sparse --branch 2024.2 \
		https://github.com/Xilinx/XilinxBoardStore.git hw/board_files/.tmp_xbs
	git -C hw/board_files/.tmp_xbs sparse-checkout set boards/Avnet/zub1cg
	mkdir -p hw/board_files
	rm -rf hw/board_files/zub1cg
	mv hw/board_files/.tmp_xbs/boards/Avnet/zub1cg hw/board_files/zub1cg
	rm -rf hw/board_files/.tmp_xbs
	@echo "Installed ZUBoard 1CG board files in hw/board_files/zub1cg"

lint:
	verilator --lint-only -Wall --sv --top-module axil_regfile rtl/common/axil_regfile.sv
	verilator --lint-only -Wall --sv --top-module stage0_dsp_slot \
		rtl/common/sv_nco_cordic.sv rtl/common/stage0_dsp_slot.sv
	verilator --lint-only -Wall --sv --top-module mems_hil_axis_wrap \
		rtl/common/sv_nco_cordic.sv \
		rtl/common/sv_pi_ctrl.sv \
		rtl/common/sv_lockin.sv \
		rtl/mems_hil/mems_plant_2dof.sv \
		rtl/mems_hil/mems_ctrl_top.sv \
		rtl/mems_hil/mems_hil_top.sv \
		rtl/mems_hil/mems_hil_axis_wrap.sv

sim: sim-nco sim-slot sim-lockin sim-plant sim-closed-loop sim-stage1-axis

sim-nco:
	@echo "=== Running cocotb: sv_nco_cordic ==="
	$(PY) -m pytest -s sim/test_nco.py

sim-slot:
	@echo "=== Running cocotb: stage0_dsp_slot ==="
	$(PY) -m pytest -s sim/test_stage0_slot.py

sim-lockin:
	@echo "=== Running cocotb: sv_lockin & sv_pi_ctrl ==="
	$(PY) -m pytest -s sim/test_lockin_pi.py

sim-plant:
	@echo "=== Running cocotb: mems_plant_2dof (DRG Q=100k & HRG Q=1M) ==="
	$(PY) -m pytest -s sim/test_mems_plant.py

sim-closed-loop:
	@echo "=== Running cocotb: mems_hil_top closed-loop DRG ==="
	$(PY) -m pytest -s sim/test_mems_closed_loop.py

sim-stage1-axis:
	@echo "=== Running cocotb: mems_hil_axis_wrap (AXI-Lite + 8-word AXI-Stream) ==="
	$(PY) -m pytest -s sim/test_stage1_axis.py

test-model:
	@echo "=== Running Stage 1 DRG/HRG golden model & bit-width study ==="
	PYTHONPATH=. $(PY) model/fixed_point.py
	PYTHONPATH=. $(PY) -m pytest -v model/test_mems_model.py

plot-drg:
	PYTHONPATH=. $(PY) model/mems_gyro_model.py --preset drg_11khz_q100k

plot-hrg:
	PYTHONPATH=. $(PY) model/mems_gyro_model.py --preset hrg_micro_11khz_q1m

plot-space-hrg:
	PYTHONPATH=. $(PY) model/mems_gyro_model.py --preset hrg_space_4khz_q5m

bd:
	mkdir -p hw/build
	PROJECT=$(PROJECT) BOARD_REPO=$(BOARD_REPO) $(VIVADO) -mode batch -source hw/scripts/create_bd.tcl \
		-log hw/build/create_bd_$(PROJECT).log -journal hw/build/create_bd_$(PROJECT).jou

bit:
	mkdir -p hw/build
	PROJECT=$(PROJECT) BOARD_REPO=$(BOARD_REPO) JOBS=$(JOBS) $(VIVADO) -mode batch -source hw/scripts/build_bit.tcl \
		-log hw/build/build_bit_$(PROJECT).log -journal hw/build/build_bit_$(PROJECT).jou

sw:
	PROJECT=$(PROJECT) $(VITIS) -s sw/build_vitis.py $(PROJECT)

boot:
	PROJECT=$(PROJECT) $(XSDB) sw/boot_jtag.tcl

mock-host:
	$(PY) host/zuboard_client.py --mock --freq-hz 16000 --samples 2048 --no-plot

clean:
	rm -rf sim/sim_build sim/results.xml sim/*.vcd sim/*.fst hw/build .Xil *.log *.jou
