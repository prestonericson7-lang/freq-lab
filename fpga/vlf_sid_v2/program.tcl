#-----------------------------------------------------------------------------
# program.tcl -- load the VLF SID receiver into the PZ7020-StarLite over JTAG
#
# Usage (Vivado Tcl Shell):
#   cd P:/Downloads/freq-lab/fpga/vlf_sid_v2
#   source program.tcl
# or: vivado -mode batch -source program.tcl
#
# This loads the FPGA's RAM configuration only: a power cycle erases it and
# the board comes back exactly as before.
#-----------------------------------------------------------------------------
set SCRIPT_DIR [file dirname [file normalize [info script]]]
set BIT_FILE   ${SCRIPT_DIR}/build/vlf_sid_v2.bit

if {![file exists $BIT_FILE]} {
    puts "ERROR: bitstream not found: $BIT_FILE"
    puts "       run build.tcl first"
    return
}

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set dev [lindex [get_hw_devices xc7z020*] 0]   ;# the PL -- index 0 of the chain is arm_dap_0
current_hw_device $dev
set_property PROGRAM.FILE $BIT_FILE $dev
program_hw_devices $dev

puts "============================================"
puts " Programmed: $BIT_FILE"
puts " LED R19 blinks at 1 Hz = running."
puts " LED V13 blinks once per window; solid = antenna signal clipping."
puts "============================================"

close_hw_target
disconnect_hw_server
close_hw_manager
