# Read KSZ9131RNX PHY @ MDIO addr 7 via GEM2 PHY_MAINT (0xFF0D0034)
proc mdio_wait {} {
    for {set i 0} {$i < 100} {incr i} {
        set st [lindex [mrd -value 0xFF0D0008 1] 0]
        if {$st & 0x4} { return }
        after 1
    }
}
proc mdio_rd {phy reg} {
    set val [expr {(1 << 30) | (2 << 28) | ($phy << 23) | ($reg << 18) | (2 << 16)}]
    mwr 0xFF0D0034 $val
    mdio_wait
    return [expr {[lindex [mrd -value 0xFF0D0034 1] 0] & 0xFFFF}]
}
proc mdio_wr {phy reg data} {
    set val [expr {(1 << 30) | (1 << 28) | ($phy << 23) | ($reg << 18) | (2 << 16) | ($data & 0xFFFF)}]
    mwr 0xFF0D0034 $val
    mdio_wait
}
proc mmd_rd {phy dev reg} {
    mdio_wr $phy 0x0D $dev
    mdio_wr $phy 0x0E $reg
    mdio_wr $phy 0x0D [expr {0x4000 | $dev}]
    return [mdio_rd $phy 0x0E]
}
proc mmd_wr {phy dev reg data} {
    mdio_wr $phy 0x0D $dev
    mdio_wr $phy 0x0E $reg
    mdio_wr $phy 0x0D [expr {0x4000 | $dev}]
    mdio_wr $phy 0x0E $data
}

connect
after 200
targets -set -nocase -filter {name =~ "*A53*#0"}
stop
set phy 7
puts [format "BMCR (0x00) = 0x%04X" [mdio_rd $phy 0x00]]
puts [format "BMSR (0x01) = 0x%04X" [mdio_rd $phy 0x01]]
puts [format "ID1  (0x02) = 0x%04X" [mdio_rd $phy 0x02]]
puts [format "ID2  (0x03) = 0x%04X" [mdio_rd $phy 0x03]]
puts [format "CTRL (0x1F) = 0x%04X" [mdio_rd $phy 0x1F]]
puts [format "MMD2 0x04 (RGMII Ctrl Skew)  = 0x%04X" [mmd_rd $phy 2 0x04]]
puts [format "MMD2 0x05 (RGMII RXD Skew)   = 0x%04X" [mmd_rd $phy 2 0x05]]
puts [format "MMD2 0x06 (RGMII TXD Skew)   = 0x%04X" [mmd_rd $phy 2 0x06]]
puts [format "MMD2 0x08 (RGMII CLK Skew)   = 0x%04X" [mmd_rd $phy 2 0x08]]
puts [format "MMD2 0x4C (RXC DLL Ctrl)     = 0x%04X" [mmd_rd $phy 2 0x4C]]
puts [format "MMD2 0x4D (TXC DLL Ctrl)     = 0x%04X" [mmd_rd $phy 2 0x4D]]

if {[info exists ::env(FIX_TXC)]} {
    set txc [mmd_rd $phy 2 0x4D]
    set rxc [mmd_rd $phy 2 0x4C]
    set new_txc [expr {$txc ^ 0x1000}]
    puts [format "Toggling TXC DLL (0x4D): 0x%04X -> 0x%04X" $txc $new_txc]
    mmd_wr $phy 2 0x4D $new_txc
    puts [format "Readback MMD2 0x4D = 0x%04X" [mmd_rd $phy 2 0x4D]]
}
con
disconnect
