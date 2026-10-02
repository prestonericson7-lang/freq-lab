#-----------------------------------------------------------------------------
# build.tcl -- Vivado non-project flow for the VLF SID receiver, version 2
#              (adds sferic_capture: lightning time-of-arrival events)
#
# Usage:  vivado -mode batch -source build.tcl
# Output: build/vlf_sid_v2.bit (+ timing, utilization, DRC, power reports)
#
# Part: xc7z020clg400-2 (PZ7020-StarLite). Tested with Vivado 2026.1.
#-----------------------------------------------------------------------------
set TOP  vlf_top
set PART xc7z020clg400-2

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set HDL_DIR    ${SCRIPT_DIR}/hdl
set XDC_DIR    ${SCRIPT_DIR}/constraints
set BUILD_DIR  ${SCRIPT_DIR}/build

file mkdir $BUILD_DIR

# synthesis
read_verilog [glob ${HDL_DIR}/*.v]
read_xdc     ${XDC_DIR}/vlf_jm1.xdc

synth_design -top $TOP -part $PART

report_utilization    -file ${BUILD_DIR}/util_synth.rpt
report_timing_summary -file ${BUILD_DIR}/timing_synth.rpt

# place and route
opt_design
place_design
phys_opt_design
route_design

# final reports
report_utilization    -file ${BUILD_DIR}/util_impl.rpt
report_utilization -hierarchical -file ${BUILD_DIR}/util_hier.rpt
report_timing_summary -file ${BUILD_DIR}/timing_impl.rpt
report_io             -file ${BUILD_DIR}/io.rpt
report_drc            -file ${BUILD_DIR}/drc.rpt
report_power          -file ${BUILD_DIR}/power.rpt
report_methodology    -file ${BUILD_DIR}/methodology.rpt

# bitstream
write_bitstream -force ${BUILD_DIR}/vlf_sid_v2.bit

puts "============================================"
puts " Build complete: ${BUILD_DIR}/vlf_sid_v2.bit"
puts "============================================"
