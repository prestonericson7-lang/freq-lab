#-----------------------------------------------------------------------------
# photon_jm1.xdc -- PZ7020-StarLite constraints for photon_top.v
#
# Board : Puzhi PZ7020-StarLite, Zynq XC7Z020-CLG400
# Part  : xc7z020clg400-2
#
# JM1 even-numbered row (BANK35, 3.3 V):
#   pin 2  3.3 V        pin 4  GND
#   pin 6  DET_A        (E17)   LVCMOS33 from detector A discriminator
#   pin 8  DET_B        (D18)   LVCMOS33 from detector B discriminator
#   pin 10 GATE         (F16)   counting gate (active-high, tie high if unused)
#   pin 14 UART TX      (B19)   to adapter / Teensy RX
#   pin 16 UART RX      (A20)   from adapter / Teensy TX
#
# WARNING: the detector inputs expect clean LVCMOS33 digital pulses from a
# discriminator/comparator.  NEVER connect raw SiPM or APD signals -- bias
# voltages (25-70 V) WILL destroy the FPGA.
#-----------------------------------------------------------------------------

## Fabric clock -- 50 MHz single-ended, IO_12P_MRCC_34
set_property PACKAGE_PIN U18      [get_ports clk_50m]
set_property IOSTANDARD  LVCMOS33 [get_ports clk_50m]
create_clock -period 20.000 -name clk_50m -waveform {0.000 10.000} [get_ports clk_50m]

## User LEDs (BANK34). High = lit.
set_property PACKAGE_PIN R19      [get_ports led_hb]
set_property IOSTANDARD  LVCMOS33 [get_ports led_hb]
set_property PACKAGE_PIN V13      [get_ports led_act]
set_property IOSTANDARD  LVCMOS33 [get_ports led_act]

## User KEY1 -- active low (BANK35). Held = fan to 100 %.
set_property PACKAGE_PIN G14      [get_ports key_n]
set_property IOSTANDARD  LVCMOS33 [get_ports key_n]
set_property PULLUP      true     [get_ports key_n]

## Fan -- same pins as the lockin design
set_property PACKAGE_PIN H16      [get_ports fan_pwm_o]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_pwm_o]
set_property DRIVE       8        [get_ports fan_pwm_o]
set_property SLEW        SLOW     [get_ports fan_pwm_o]
set_property PACKAGE_PIN H17      [get_ports fan_tach]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_tach]
set_property PULLUP      true     [get_ports fan_tach]
set_false_path -from [get_ports fan_tach]

## Detector A -- JM1 pin 6 (E17)
## Fast digital pulse from SiPM/APD discriminator
set_property PACKAGE_PIN E17      [get_ports det_a]
set_property IOSTANDARD  LVCMOS33 [get_ports det_a]

## Detector B -- JM1 pin 8 (D18)
set_property PACKAGE_PIN D18      [get_ports det_b]
set_property IOSTANDARD  LVCMOS33 [get_ports det_b]

## Counting gate -- JM1 pin 10 (F16) -- active high
## Tie to 3.3 V if not used, or drive from a Teensy to control runs
set_property PACKAGE_PIN F16      [get_ports gate]
set_property IOSTANDARD  LVCMOS33 [get_ports gate]
set_property PULLUP      true     [get_ports gate]

## UART -- JM1 pin 14 (TX out), pin 16 (RX in)
set_property PACKAGE_PIN B19      [get_ports uart_tx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_tx]
set_property DRIVE       8        [get_ports uart_tx]
set_property SLEW        SLOW     [get_ports uart_tx]
set_property PACKAGE_PIN A20      [get_ports uart_rx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_rx]
set_property PULLUP      true     [get_ports uart_rx]

## Asynchronous inputs are synchronised in RTL
set_false_path -from [get_ports uart_rx]
set_false_path -from [get_ports key_n]
set_false_path -from [get_ports det_a]
set_false_path -from [get_ports det_b]
set_false_path -from [get_ports gate]
