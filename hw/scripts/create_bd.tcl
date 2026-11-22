# ============================================================================
# Vivado 2024.2 Reproducible Block Design Script for Avnet ZUBoard 1CG
# Supports monorepo PROJECT selection:
#   PROJECT=stage0       -> Stage 0 NCO + Loopback Shell (zub1cg_stage0)
#   PROJECT=stage1_mems  -> Stage 1 48-bit DRG/HRG HIL   (zub1cg_stage1_mems)
# ============================================================================
set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize "$script_dir/../.."]
set build_dir  "$repo_root/hw/build"

set project_sel "stage0"
if {[info exists ::env(PROJECT)] && $::env(PROJECT) ne ""} {
    set project_sel $::env(PROJECT)
}

if {$project_sel eq "stage0"} {
    set proj_name   "zub1cg_stage0"
    set slot_module "stage0_bd_wrap"
    set rtl_files [list \
        "$repo_root/rtl/common/sv_nco_cordic.sv" \
        "$repo_root/rtl/common/axil_regfile.sv" \
        "$repo_root/rtl/common/stage0_dsp_slot.sv" \
        "$repo_root/rtl/common/stage0_bd_wrap.v" \
    ]
} elseif {$project_sel eq "stage1_mems"} {
    set proj_name   "zub1cg_stage1_mems"
    set slot_module "stage1_bd_wrap"
    set rtl_files [list \
        "$repo_root/rtl/common/sv_nco_cordic.sv" \
        "$repo_root/rtl/common/sv_lockin.sv" \
        "$repo_root/rtl/common/sv_pi_ctrl.sv" \
        "$repo_root/rtl/mems_hil/mems_plant_2dof.sv" \
        "$repo_root/rtl/mems_hil/mems_ctrl_top.sv" \
        "$repo_root/rtl/mems_hil/mems_hil_top.sv" \
        "$repo_root/rtl/mems_hil/mems_hil_axis_wrap.sv" \
        "$repo_root/rtl/mems_hil/stage1_bd_wrap.v" \
    ]
} else {
    error "Unknown PROJECT='$project_sel'. Valid options: stage0, stage1_mems"
}

puts "INFO: Building ZUBoard 1CG Block Design for PROJECT=$project_sel ($proj_name)"

# Locate Avnet ZUBoard 1CG board files automatically (BOARD_REPO, local hw/board_files, or XHub)
set repo_paths {}
if {[info exists ::env(BOARD_REPO)] && $::env(BOARD_REPO) ne ""} {
    lappend repo_paths $::env(BOARD_REPO)
}
if {[file isdirectory "$repo_root/hw/board_files"]} {
    lappend repo_paths "$repo_root/hw/board_files"
}
if {[llength $repo_paths] > 0} {
    set_param board.repoPaths $repo_paths
}

file mkdir $build_dir
create_project -force $proj_name "$build_dir/$proj_name" -part xczu1cg-sbva484-1-e

# Attach Avnet ZUBoard 1CG board part (required for LPDDR4, GbE GEM3, UART1, SD presets)
set zub_boards [get_board_parts -quiet *zub1cg*]
if {[llength $zub_boards] > 0} {
    set board_part [lindex $zub_boards 0]
    puts "INFO: Selecting board part: $board_part"
    set_property board_part $board_part [current_project]
} else {
    puts "WARNING: Avnet ZUBoard 1CG board files not found! Run 'make board-files' or install via Vivado Store."
    if {![info exists ::env(ALLOW_NO_BOARD)]} {
        error "Aborting because ZUBoard 1CG PS presets (LPDDR4/GbE/UART) require board files. Run 'make board-files' first (or set ALLOW_NO_BOARD=1)."
    }
}

# Add RTL sources & constraints
add_files -norecurse $rtl_files
add_files -fileset constrs_1 -norecurse "$repo_root/hw/constraints/zub1cg_base.xdc"
update_compile_order -fileset sources_1

# Create Block Design
create_bd_design "zub1cg_bd"

# 1. Zynq UltraScale+ MPSoC Processing System
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 zynq_ps
if {[llength $zub_boards] > 0} {
    apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
        -config {apply_board_preset "1"} [get_bd_cells zynq_ps]
}

# Configure 100 MHz PL clock, AXI-Lite master (HPM0_LPD), and High-Perf DDR slave (HP0_FPD)
set_property -dict [list \
    CONFIG.PSU__USE__M_AXI_GP0 {0} \
    CONFIG.PSU__USE__M_AXI_GP1 {0} \
    CONFIG.PSU__USE__M_AXI_GP2 {1} \
    CONFIG.PSU__USE__S_AXI_GP2 {1} \
    CONFIG.PSU__USE__IRQ0 {1} \
    CONFIG.PSU__FPGA_PL0_ENABLE {1} \
    CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100} \
] [get_bd_cells zynq_ps]

# 2. Processor System Reset for 100 MHz pl_clk0
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst_pl_100m
connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins rst_pl_100m/slowest_sync_clk]
connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins rst_pl_100m/ext_reset_in]

# 3. AXI-DMA (Simple Register Mode, 32-bit AXI-Stream MM2S & S2MM)
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_0
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_length_width {23} \
    CONFIG.c_m_axi_mm2s_data_width {64} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_m_axi_s2mm_data_width {64} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
    CONFIG.c_include_mm2s {1} \
    CONFIG.c_include_s2mm {1} \
] [get_bd_cells axi_dma_0]

# 4. Swappable Project DSP Slot (stage0_bd_wrap or stage1_bd_wrap)
create_bd_cell -type module -reference $slot_module dsp_slot_0

# 5. Interconnects (SmartConnect for AXI-Lite control and HP0 DDR data)
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smc_ctrl
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {2}] [get_bd_cells smc_ctrl]

create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 smc_dma
set_property -dict [list CONFIG.NUM_SI {2} CONFIG.NUM_MI {1}] [get_bd_cells smc_dma]

# Interrupt concat (MM2S + S2MM -> PS IRQ0)
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 xlconcat_irq
set_property -dict [list CONFIG.NUM_PORTS {2}] [get_bd_cells xlconcat_irq]

# Clocks & Resets
foreach clk_pin [list \
    zynq_ps/maxihpm0_lpd_aclk \
    zynq_ps/saxihp0_fpd_aclk \
    axi_dma_0/s_axi_lite_aclk \
    axi_dma_0/m_axi_mm2s_aclk \
    axi_dma_0/m_axi_s2mm_aclk \
    dsp_slot_0/aclk \
    smc_ctrl/aclk \
    smc_dma/aclk \
] {
    connect_bd_net [get_bd_pins zynq_ps/pl_clk0] [get_bd_pins $clk_pin]
}

foreach rst_pin [list \
    axi_dma_0/axi_resetn \
    dsp_slot_0/aresetn \
    smc_ctrl/aresetn \
    smc_dma/aresetn \
] {
    connect_bd_net [get_bd_pins rst_pl_100m/peripheral_aresetn] [get_bd_pins $rst_pin]
}

# AXI-Lite Control Plane
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_LPD] [get_bd_intf_pins smc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M00_AXI] [get_bd_intf_pins axi_dma_0/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins smc_ctrl/M01_AXI] [get_bd_intf_pins dsp_slot_0/s_axi]

# AXI-DMA Data Plane <-> DDR4
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_MM2S] [get_bd_intf_pins smc_dma/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXI_S2MM] [get_bd_intf_pins smc_dma/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins smc_dma/M00_AXI]      [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

# AXI4-Stream Pipeline
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXIS_MM2S] [get_bd_intf_pins dsp_slot_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins dsp_slot_0/m_axis]     [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM]

# Interrupts
connect_bd_net [get_bd_pins axi_dma_0/mm2s_introut] [get_bd_pins xlconcat_irq/In0]
connect_bd_net [get_bd_pins axi_dma_0/s2mm_introut] [get_bd_pins xlconcat_irq/In1]
connect_bd_net [get_bd_pins xlconcat_irq/dout]      [get_bd_pins zynq_ps/pl_ps_irq0]

# Assign Addresses (deterministic map matching sw/ firmware headers)
assign_bd_address
set_property offset 0x80000000 [get_bd_addr_segs {zynq_ps/Data/*axi_dma_0*}]
set_property range  64K        [get_bd_addr_segs {zynq_ps/Data/*axi_dma_0*}]
set_property offset 0x80010000 [get_bd_addr_segs {zynq_ps/Data/*dsp_slot_0*}]
set_property range  4K         [get_bd_addr_segs {zynq_ps/Data/*dsp_slot_0*}]

validate_bd_design
save_bd_design

# Generate HDL top wrapper
make_wrapper -files [get_files "$build_dir/$proj_name/$proj_name.srcs/sources_1/bd/zub1cg_bd/zub1cg_bd.bd"] -top
add_files -norecurse "$build_dir/$proj_name/$proj_name.gen/sources_1/bd/zub1cg_bd/hdl/zub1cg_bd_wrapper.v"
set_property top zub1cg_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts "SUCCESS: ZUBoard 1CG Block Design ($proj_name) created at $build_dir/$proj_name"
