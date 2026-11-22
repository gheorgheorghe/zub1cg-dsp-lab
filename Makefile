SHELL := /bin/bash
VENV  := .venv
PY    := $(VENV)/bin/python
PIP   := $(VENV)/bin/pip
SIM   ?= verilator
VIVADO ?= vivado
BOARD_REPO ?=
JOBS ?= 8

.PHONY: help venv board-files lint sim sim-nco sim-slot test-model plot-drg plot-hrg plot-space-hrg bd bit mock-host clean

help:
	@echo "Targets:"
	@echo "  make venv          Create Python venv and install cocotb/numpy/scipy/pytest"
	@echo "  make board-files   Download Avnet ZUBoard 1CG board files for Vivado 2024.2"
	@echo "  make test-model    Run Stage 1 DRG/HRG golden model & bit-width pytest suite"
	@echo "  make plot-drg      Plot DRG (11 kHz, Q=100k) closed-loop simulation"
	@echo "  make plot-hrg      Plot Micro-HRG (11 kHz, Q=1M) closed-loop simulation"
	@echo "  make plot-space-hrg Plot Space HRG (4.2 kHz, Q=5M) closed-loop simulation"
	@echo "  make lint          Run Verilator lint on all SystemVerilog RTL"
	@echo "  make sim         Run all cocotb + Verilator RTL testbenches"
	@echo "  make bd          Generate Vivado 2024.2 Block Design project (hw/build)"
	@echo "  make bit         Full Vivado synth/impl/bitstream + export zub1cg_stage0.xsa"
	@echo "  make mock-host   Run host UDP client in offline --mock mode"
	@echo "  make clean       Remove simulation and Vivado build outputs"

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
	verilator --lint-only -Wall --sv \
		rtl/common/sv_nco_cordic.sv \
		rtl/common/axil_regfile.sv \
		rtl/common/stage0_dsp_slot.sv

sim: sim-nco sim-slot

sim-nco:
	@echo "=== Running cocotb: sv_nco_cordic ==="
	$(PY) -m pytest -s sim/test_nco.py

sim-slot:
	@echo "=== Running cocotb: stage0_dsp_slot ==="
	$(PY) -m pytest -s sim/test_stage0_slot.py

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
	BOARD_REPO=$(BOARD_REPO) $(VIVADO) -mode batch -source hw/scripts/create_bd.tcl \
		-log hw/build/create_bd.log -journal hw/build/create_bd.jou

bit:
	mkdir -p hw/build
	BOARD_REPO=$(BOARD_REPO) JOBS=$(JOBS) $(VIVADO) -mode batch -source hw/scripts/build_bit.tcl \
		-log hw/build/build_bit.log -journal hw/build/build_bit.jou

mock-host:
	$(PY) host/zuboard_client.py --mock --freq-hz 16000 --samples 2048 --no-plot

clean:
	rm -rf sim/sim_build sim/results.xml sim/*.vcd sim/*.fst hw/build .Xil *.log *.jou
