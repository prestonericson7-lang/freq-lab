#-----------------------------------------------------------------------------
# trng_jm1.xdc -- PZ7020-StarLite constraints for trng_top.v (ring-osc TRNG)
# Board: Puzhi PZ7020-StarLite, xc7z020clg400-2. Same JM1 pins as the other
# freq-lab designs; this design uses no analog pins (XADC reads on-chip temp).
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

## Fan
set_property PACKAGE_PIN H16      [get_ports fan_pwm_o]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_pwm_o]
set_property DRIVE       8        [get_ports fan_pwm_o]
set_property SLEW        SLOW     [get_ports fan_pwm_o]
set_property PACKAGE_PIN H17      [get_ports fan_tach]
set_property IOSTANDARD  LVCMOS33 [get_ports fan_tach]
set_property PULLUP      true     [get_ports fan_tach]

## UART -- JM1 pin 14 (TX out), pin 16 (RX in)
set_property PACKAGE_PIN B19      [get_ports uart_tx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_tx]
set_property DRIVE       8        [get_ports uart_tx]
set_property SLEW        SLOW     [get_ports uart_tx]
set_property PACKAGE_PIN A20      [get_ports uart_rx]
set_property IOSTANDARD  LVCMOS33 [get_ports uart_rx]
set_property PULLUP      true     [get_ports uart_rx]

## Asynchronous inputs are synchronised in RTL; outputs are slow
set_false_path -from [get_ports {uart_rx key_n fan_tach}]
set_false_path -to   [get_ports {uart_tx led_hb led_act fan_pwm_o}]
