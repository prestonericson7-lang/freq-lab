#-----------------------------------------------------------------------------
# program.tcl -- program the PZ7020-StarLite with the photon counter bitstream
#
# Usage:
#   vivado -mode batch -source program.tcl
#   (board must be connected via JTAG/USB)
#-----------------------------------------------------------------------------
set SCRIPT_DIR [file dirname [file normalize [info script]]]
set BIT_FILE   ${SCRIPT_DIR}/build/photon_counter.bit

if {![file exists $BIT_FILE]} {
    puts "ERROR: bitstream not found: $BIT_FILE"
    puts "       run build.tcl first"
    exit 1
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
puts "============================================"

close_hw_target
disconnect_hw_server
close_hw_manager
