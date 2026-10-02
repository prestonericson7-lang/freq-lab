#-----------------------------------------------------------------------------
# vlf_jm1.xdc -- PZ7020-StarLite constraints for vlf_top.v (VLF SID receiver)
#
# Board : Puzhi PZ7020-StarLite, Zynq XC7Z020-CLG400
# Part  : xc7z020clg400-2
#
# JM1 header (BANK35, 3.3 V):
#   pin 2  3.3 V             pin 4  GND
#   pin 5  fan PWM   (H16)   pin 6  ANTENNA+  (E17, VAUX1P)  0 .. 1.0 V ONLY
#   pin 7  fan TACH  (H17)   pin 8  ANTENNA-  (D18, VAUX1N)  preamp ground
#   pin 14 UART TX   (B19)   to USB-serial adapter RX (3.3 V)
#   pin 16 UART RX   (A20)   from USB-serial adapter TX (3.3 V)
#   pin 18 GPS PPS   (C20)   1PPS from a 3.3 V GPS module (optional)
#
# Same pins as the lockin/correlator designs, so the same wiring works.
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

## Fan -- same pins as the other designs
set_property PACKAGE_PIN H16      [get_ports fan_pwm_o]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_pwm_o]
set_property DRIVE       8        [get_ports fan_pwm_o]
set_property SLEW        SLOW     [get_ports fan_pwm_o]
set_property PACKAGE_PIN H17      [get_ports fan_tach]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_tach]
set_property PULLUP      true     [get_ports fan_tach]

## XADC auxiliary channel 1 -- JM1 pin 6 (P) / pin 8 (N) -- antenna preamp
set_property PACKAGE_PIN E17      [get_ports vauxp1]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxp1]
set_property PACKAGE_PIN D18      [get_ports vauxn1]
set_property IOSTANDARD  LVCMOS33 [get_ports vauxn1]

## UART -- JM1 pin 14 (TX out), pin 16 (RX in)
set_property PACKAGE_PIN B19      [get_ports uart_tx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_tx]
set_property DRIVE       8        [get_ports uart_tx]
set_property SLEW        SLOW     [get_ports uart_tx]
set_property PACKAGE_PIN A20      [get_ports uart_rx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_rx]
set_property PULLUP      true     [get_ports uart_rx]

## GPS PPS -- JM1 pin 18 (C20). Pulled down: reads "no GPS" when unconnected.
set_property PACKAGE_PIN C20      [get_ports gps_pps]
set_property IOSTANDARD  LVCMOS33 [get_ports gps_pps]
set_property PULLDOWN    true     [get_ports gps_pps]

## Asynchronous inputs are synchronised in RTL; outputs are slow (LEDs, fan, UART)
set_false_path -from [get_ports {uart_rx key_n gps_pps fan_tach}]
set_false_path -to   [get_ports {uart_tx led_hb led_act fan_pwm_o}]
