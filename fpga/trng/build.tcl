#-----------------------------------------------------------------------------
# build.tcl -- Vivado non-project flow for the ring-oscillator TRNG (#73)
#   vivado -mode batch -source build.tcl
# Output: build/trng.bit (+ timing, utilization, DRC, power reports)
# Part: xc7z020clg400-2 (PZ7020-StarLite). Tested with Vivado 2026.1.
#-----------------------------------------------------------------------------
set TOP  trng_top
set PART xc7z020clg400-2

set SCRIPT_DIR [file dirname [file normalize [info script]]]
set HDL_DIR    ${SCRIPT_DIR}/hdl
set XDC_DIR    ${SCRIPT_DIR}/constraints
set BUILD_DIR  ${SCRIPT_DIR}/build
file mkdir $BUILD_DIR

read_verilog [glob ${HDL_DIR}/*.v]
read_xdc     ${XDC_DIR}/trng_jm1.xdc

# ring oscillators are deliberate combinational loops; the HDL marks the nets
# with ALLOW_COMBINATORIAL_LOOPS. Downgrade the combinational-loop DRC so the
# bitstream can still be written.
synth_design -top $TOP -part $PART
catch { set_property SEVERITY {Warning} [get_drc_checks LUTLP-1] }

report_utilization    -file ${BUILD_DIR}/util_synth.rpt
report_timing_summary -file ${BUILD_DIR}/timing_synth.rpt

opt_design
place_design
phys_opt_design
route_design

report_utilization    -file ${BUILD_DIR}/util_impl.rpt
report_timing_summary -file ${BUILD_DIR}/timing_impl.rpt
report_io             -file ${BUILD_DIR}/io.rpt
report_drc            -file ${BUILD_DIR}/drc.rpt
report_power          -file ${BUILD_DIR}/power.rpt

write_bitstream -force ${BUILD_DIR}/trng.bit
puts "============================================"
puts " Build complete: ${BUILD_DIR}/trng.bit"
puts "============================================"
