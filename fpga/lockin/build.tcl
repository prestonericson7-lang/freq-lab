#-----------------------------------------------------------------------------
# build.tcl -- create the Vivado project and build the bitstream
#
#   Vivado Tcl console:   cd <this folder> ; source build.tcl
#   or from a shell:      vivado -mode batch -source build.tcl
#
# Result: lockin_top.bit in this folder.
#-----------------------------------------------------------------------------
set here [file dirname [file normalize [info script]]]
set proj lockin
set part xc7z020clg400-2

create_project -force $proj [file join $here vivado] -part $part
add_files [glob [file join $here hdl *.v]]
add_files -fileset constrs_1 [file join $here constraints lockin_jm1.xdc]
set_property top lockin_top [current_fileset]
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "synthesis failed" }

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "implementation failed" }

set bit [file join $here vivado $proj.runs impl_1 lockin_top.bit]
file copy -force $bit [file join $here lockin_top.bit]
puts "Bitstream written: [file join $here lockin_top.bit]"
