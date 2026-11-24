# ============================================================================
# XSDB JTAG Boot Script for Avnet ZUBoard 1CG (Stage 0 or Stage 1 MEMS HIL)
# Usage:
#   PROJECT=stage1_mems xsdb sw/boot_jtag.tcl
# ============================================================================
set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize "$script_dir/.."]

set project_sel "stage1_mems"
if {[info exists ::env(PROJECT)] && $::env(PROJECT) ne ""} {
    set project_sel $::env(PROJECT)
}

set proj_name "zub1cg_${project_sel}"
set ws_dir    "$repo_root/sw/build/ws_${project_sel}"
set plat_dir  "$ws_dir/${proj_name}_plat/export/${proj_name}_plat/hw"

set bit_file  "$repo_root/hw/build/${proj_name}/${proj_name}.runs/impl_1/zub1cg_bd_wrapper.bit"
set psu_init  "$plat_dir/psu_init.tcl"
set a53_elf   "$ws_dir/a53_net_bridge/build/a53_net_bridge.elf"
set r5f_elf   "$ws_dir/r5f_realtime/build/r5f_realtime.elf"

puts "=== ZUBoard 1CG JTAG Boot ($proj_name) ==="
puts "  BIT : $bit_file"
puts "  A53 : $a53_elf"
puts "  R5F : $r5f_elf"

connect
after 500

# 1. Reset entire system and initialize PS (LPDDR4, clocks, MIO, GbE GEM3, UART1)
targets -set -nocase -filter {name =~ "*PSU*"}
rst -system
after 1000

source $psu_init
psu_init
after 500
psu_ps_pl_isolation_removal
after 200
psu_ps_pl_reset_config

# 2. Program Programmable Logic bitstream
puts "Programming PL bitstream..."
fpga -file $bit_file
after 500

# 3. Load & start Cortex-R5F #0 (1 kHz CVG supervisor)
puts "Loading Cortex-R5F #0..."
targets -set -nocase -filter {name =~ "*R5*#0"}
rst -processor
dow $r5f_elf
con

# 4. Load & start Cortex-A53 #0 (lwIP GbE UDP <-> AXI-DMA bridge)
puts "Loading Cortex-A53 #0..."
targets -set -nocase -filter {name =~ "*A53*#0"}
rst -processor
dow $a53_elf
con

puts "SUCCESS: ZUBoard 1CG booted! Connect host to 192.168.1.10:5000 (UDP)."
disconnect
