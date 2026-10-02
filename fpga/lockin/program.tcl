#-----------------------------------------------------------------------------
# program.tcl -- load lockin_top.bit over the board's own USB-JTAG
#
# Boot jumper on JTAG, the JTAG Type-C port connected. The bitstream lives in
# the fabric only until power-off.
#
#   Vivado Tcl console:   cd <this folder> ; source program.tcl
#   or from a shell:      vivado -mode batch -source program.tcl
#-----------------------------------------------------------------------------
set here [file dirname [file normalize [info script]]]
open_hw_manager
connect_hw_server
open_hw_target
set dev [lindex [get_hw_devices xc7z020*] 0]
current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev
set_property PROGRAM.FILE [file join $here lockin_top.bit] $dev
program_hw_devices $dev
close_hw_target
disconnect_hw_server
puts "Programmed. LED1 should blink about once a second."
