# ============================================================================
# Vivado 2024.2 Full Build Script: BD -> Synth -> Impl -> Bitstream -> .xsa
# ============================================================================
set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize "$script_dir/../.."]
set build_dir  "$repo_root/hw/build"
set proj_name  "zub1cg_stage0"

source "$script_dir/create_bd.tcl"

set jobs 8
if {[info exists ::env(JOBS)] && $::env(JOBS) ne ""} {
    set jobs $::env(JOBS)
}

# Run Synthesis
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    error "ERROR: Synthesis failed!"
}

# Run Implementation & Bitstream
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "ERROR: Implementation / write_bitstream failed!"
}

# Open implemented design for reports and timing signoff
open_run impl_1
report_timing_summary -file "$build_dir/timing_summary.rpt"
report_utilization    -file "$build_dir/utilization.rpt"

set wns [get_property STATS.WNS [get_runs impl_1]]
puts "INFO: Final Worst Negative Slack (WNS) = $wns ns"
if {$wns < 0.0} {
    error "ERROR: Timing failed with WNS = $wns ns!"
}

# Export Hardware Platform (.xsa) with Bitstream for Vitis 2024.2
write_hw_platform -fixed -include_bit -force -file "$build_dir/${proj_name}.xsa"
puts "SUCCESS: Exported $build_dir/${proj_name}.xsa"
