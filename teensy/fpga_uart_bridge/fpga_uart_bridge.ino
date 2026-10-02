/*
 * fpga_uart_bridge.ino -- Teensy 4.1 as the USB <-> FPGA serial link
 *
 * Lets tools/lockin_host.py reach the lock-in in the FPGA through the Teensy's
 * USB port, so no separate USB-serial adapter is needed.
 *
 * WIRING (both boards are 3.3 V logic)
 *   Teensy pin 0 (RX1) <- FPGA JM1 pin 14 (uart_tx, ball B19)
 *   Teensy pin 1 (TX1) -> FPGA JM1 pin 16 (uart_rx, ball A20)
 *   Teensy GND         -- FPGA JM1 pin 4  (GND)
 *
 * STATUS: compiles for Teensy 4.1. Not yet run on hardware.
 */
void setup() {
  Serial.begin(115200);     // USB, baud ignored
  Serial1.begin(115200);    // to the FPGA
}

void loop() {
  while (Serial.available())  Serial1.write(Serial.read());
  while (Serial1.available()) Serial.write(Serial1.read());
}
