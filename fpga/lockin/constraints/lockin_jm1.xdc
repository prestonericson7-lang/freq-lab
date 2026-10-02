#-----------------------------------------------------------------------------
# lockin_jm1.xdc -- PZ7020-StarLite constraints for hdl/lockin_top.v
#
# Board : Puzhi PZ7020-StarLite, Zynq XC7Z020-CLG400
# Part  : xc7z020clg400-2
#
# Ball assignments are from the board reference repo (docs/pinout.md), which
# compiled them from Puzhi's "CON Pins Signal and Equal Length.xlsx" and the
# User Manual. JM1 is entirely on BANK35 (default 3.3 V), so LVCMOS33 is right
# unless the bank resistor option has been changed.
#
# New wiring is all on JM1's even-numbered row:
#   pin 2  3.3 V        pin 4  GND
#   pin 6  ADC in +     (E17, VAUX1P)   0 .. 1.0 V ONLY
#   pin 8  ADC in -     (D18, VAUX1N)   tie to the sensor's ground
#   pin 10 drive out    (F16)           sigma-delta bitstream, needs RC low-pass
#   pin 12 sync out     (F17)           square wave at the drive frequency
#   pin 14 UART TX      (B19)           to adapter / Teensy RX
#   pin 16 UART RX      (A20)           from adapter / Teensy TX
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

## Fan -- same pins as constraints/fan_jm1.xdc in the board repo
set_property PACKAGE_PIN H16      [get_ports fan_pwm]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_pwm]
set_property DRIVE       8        [get_ports fan_pwm]
set_property SLEW        SLOW     [get_ports fan_pwm]
set_property PACKAGE_PIN H17      [get_ports fan_tach]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_tach]
set_property PULLUP      true     [get_ports fan_tach]
set_false_path -from [get_ports fan_tach]

## XADC auxiliary channel 1 -- JM1 pin 6 (P) / pin 8 (N)
set_property PACKAGE_PIN E17      [get_ports vauxp1]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxp1]
set_property PACKAGE_PIN D18      [get_ports vauxn1]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxn1]

## Drive output (sigma-delta) -- JM1 pin 10
set_property PACKAGE_PIN F16      [get_ports dac_out]
set_property IOSTANDARD  LVCMOS33 [get_ports dac_out]
set_property DRIVE       8        [get_ports dac_out]
set_property SLEW        SLOW     [get_ports dac_out]

## Sync output -- JM1 pin 12
set_property PACKAGE_PIN F17      [get_ports sync_out]
set_property IOSTANDARD  LVCMOS33 [get_ports sync_out]
set_property DRIVE       8        [get_ports sync_out]
set_property SLEW        SLOW     [get_ports sync_out]

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
