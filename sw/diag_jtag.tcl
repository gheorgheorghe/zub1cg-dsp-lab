# ============================================================================
# Quick JTAG Diagnostic for ZUBoard 1CG (A53, R5F, Xil_Assert, PL Stage 1.5)
# Usage: /tools/Xilinx/Vitis/2024.2/bin/xsdb /gemini/zub1cg-dsp-lab/sw/diag_jtag.tcl
# ============================================================================
connect
after 300
configparams force-mem-access 1
puts "=== TARGETS ==="
puts [targets]

puts "\n=== CORTEX-A53 #0 ==="
targets -set -nocase -filter {name =~ "*A53*#0"}
catch {stop}
catch {puts [rrd pc]}
catch {puts [rrd r0]}
catch {puts [rrd r1]}
catch {puts [rrd r30]}
catch {puts [rrd sp]}
puts "Backtrace:"
catch {puts [bt]}
puts "PL VERSION (0x800100EC): [mrd 0x800100EC 1]"
puts "PL STATUS/FRAMES (0x800100E4): [mrd 0x800100E4 2]"
puts "GEM2 CTRL/CFG/STAT/USR (0xFF0D0000): [mrd 0xFF0D0000 4]"
puts "GEM2 TXSTAT/RXQ/TXQ/RXSTAT (0xFF0D0014): [mrd 0xFF0D0014 4]"
puts "GEM2 ISR/IER/IDR/IMR (0xFF0D0024): [mrd 0xFF0D0024 4]"
puts "GEM2 TX_FRAMES (0xFF0D0108): [mrd 0xFF0D0108 1]"
puts "GEM2 RX_FRAMES (0xFF0D0158): [mrd 0xFF0D0158 1]"
puts "GEM2 RX_FCS_ERR (0xFF0D0190): [mrd 0xFF0D0190 4]"
puts "GEM2 TXQ1BASE (0xFF0D0440): [mrd 0xFF0D0440 1]"
puts "GEM2 RXQ1BASE (0xFF0D0480): [mrd 0xFF0D0480 1]"
puts "CRL_APB GEM2_REF_CTRL (0xFF5E0058): [mrd 0xFF5E0058 1]"
puts "A53 link_speed (0x00016e90): [mrd 0x00016e90 1]"
puts "A53 phyaddrforemac (0x002ac3b0): [mrd 0x002ac3b0 1]"
puts "A53 server_netif (0x00200b90): [mrd 0x00200b90 20]"
puts "RX BD ring @ 0x00600280 (first 8 BDs):"
puts [mrd 0x00600280 32]
puts "TX BD ring @ 0x00610000 (first 4 BDs):"
puts [mrd 0x00610000 16]

# Dump ASCII bytes at r0 if it points to valid DDR (Xil_Assert filename!)
if {![catch {set r0_line [rrd r0]}]} {
    if {[regexp {([0-9a-fA-F]{8,16})} $r0_line -> r0_hex]} {
        set r0_val [expr "0x$r0_hex"]
        if {$r0_val > 0x1000 && $r0_val < 0x30000000} {
            puts "String @ r0 (0x$r0_hex):"
            set words [mrd -value $r0_val 16]
            set ascii ""
            foreach w $words {
                for {set b 0} {$b < 4} {incr b} {
                    set ch [expr {($w >> ($b * 8)) & 0xFF}]
                    if {$ch == 0} { break }
                    if {$ch >= 32 && $ch < 127} {
                        append ascii [format %c $ch]
                    }
                }
            }
            puts "  \"$ascii\""
        }
    }
}

puts "\n=== CORTEX-R5F #0 ==="
targets -set -nocase -filter {name =~ "*R5*#0"}
catch {stop}
catch {puts [rrd pc]}
catch {puts [rrd r0]}
catch {puts [rrd r1]}
catch {puts [rrd lr]}
catch {con}

targets -set -nocase -filter {name =~ "*A53*#0"}
catch {con}
disconnect
