#-----------------------------------------------------------------------------
# correlator_jm1.xdc -- PZ7020-StarLite constraints for correlator_top.v
#
# Board : Puzhi PZ7020-StarLite, Zynq XC7Z020-CLG400
# Part  : xc7z020clg400-2
#
# JM1 even-numbered row (BANK35, 3.3 V):
#   pin 2  3.3 V        pin 4  GND
#   pin 6  ADC A+       (E17, VAUX1P)   0 .. 1.0 V ONLY
#   pin 8  ADC A-       (D18, VAUX1N)   tie to sensor A ground
#   pin 14 UART TX      (B19)           to adapter / Teensy RX
#   pin 16 UART RX      (A20)           from adapter / Teensy TX
#   pin 18 GPS PPS      (C20)           1PPS from GPS (optional, tie low)
#
# JM1 odd-numbered row:
#   pin 9  ADC B+       (E18, VAUX9P)   0 .. 1.0 V ONLY
#   pin 11 ADC B-       (E19, VAUX9N)   tie to sensor B ground
#
# VAUX9 = IO_L5P/L5N_T0_AD9P/AD9N_35 = balls E18/E19 (PZ-StarLite schematic);
# the Puzhi "CON Pins Signal" sheet puts them on JM1 pins 9 and 11.
# JM1 pins 10/12 are F16/F17 -- plain I/O, no ADC: don't wire channel B there.
# Bank 35 is adjustable 1.8/2.5/3.3 V; these constraints assume the 3.3 V default.
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

## XADC auxiliary channel 1 -- JM1 pin 6 (P) / pin 8 (N) -- sensor A
set_property PACKAGE_PIN E17      [get_ports vauxp1]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxp1]
set_property PACKAGE_PIN D18      [get_ports vauxn1]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxn1]

## XADC auxiliary channel 9 -- JM1 pin 9 (P) / pin 11 (N) -- sensor B
## E18/E19 = AD9P/AD9N (PZ-StarLite schematic + connector pin sheet)
set_property PACKAGE_PIN E18      [get_ports vauxp9]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxp9]
set_property PACKAGE_PIN E19      [get_ports vauxn9]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxn9]

## UART -- JM1 pin 14 (TX out), pin 16 (RX in)
set_property PACKAGE_PIN B19      [get_ports uart_tx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_tx]
set_property DRIVE       8        [get_ports uart_tx]
set_property SLEW        SLOW     [get_ports uart_tx]
set_property PACKAGE_PIN A20      [get_ports uart_rx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_rx]
set_property PULLUP      true     [get_ports uart_rx]

## GPS PPS -- JM1 pin 18 (C20).  Tie low if no GPS connected.
set_property PACKAGE_PIN C20      [get_ports gps_pps]
set_property IOSTANDARD  LVCMOS33 [get_ports gps_pps]
set_property PULLDOWN    true     [get_ports gps_pps]

## Asynchronous inputs are synchronised in RTL
set_false_path -from [get_ports uart_rx]
set_false_path -from [get_ports key_n]
set_false_path -from [get_ports gps_pps]
