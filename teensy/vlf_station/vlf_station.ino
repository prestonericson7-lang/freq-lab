/*
 * vlf_station.ino -- Teensy 4.1 as the GPS-aware link for the VLF SID station
 *
 * WHAT IT DOES
 *   Sits between the PC and the vlf_sid FPGA and adds the GPS. It passes the
 *   FPGA's register traffic straight through (so tools/vlf_host.py talks to the
 *   FPGA exactly as if a plain USB-serial adapter were used), AND it reads the
 *   GPS NMEA stream and the 1PPS, so the host can stamp every window with TRUE
 *   GPS UTC instead of trusting the PC clock. The same 1PPS also goes to the
 *   FPGA's JM1-18 so the FPGA's one-second windows are GPS-aligned.
 *
 *   This reuses the FPGA<->Teensy wiring you already have (the GPU matrix-math
 *   rig): when you switch the FPGA to the vlf_sid bitstream, flash this to the
 *   Teensy and it becomes the station bridge. Nothing here changes the FPGA.
 *
 * WIRING (all 3.3 V logic, common ground)
 *   FPGA UART  : Teensy pin 0 (RX1) <- FPGA JM1-14 (uart_tx, B19)
 *                Teensy pin 1 (TX1) -> FPGA JM1-16 (uart_rx, A20)
 *   GPS        : Teensy pin 7 (RX2) <- GPS TX (NMEA, 9600)      -- same GPS as past .ino builds
 *   GPS 1PPS   : GPS PPS --> Teensy pin 2  AND  --> FPGA JM1-18 (C20)   (one line, two inputs)
 *   GND        : Teensy GND -- FPGA JM1-4 -- GPS GND
 *   (GPS RX is not needed; leave it unconnected.)
 *
 * HOW THE HOST USES IT
 *   The PC opens the Teensy's USB serial port (not a separate adapter). Any line
 *   that does NOT start with '@' is forwarded verbatim to the FPGA and the
 *   FPGA's reply is passed back -- so  R/W  register commands just work. Lines
 *   that start with '@' are answered by this bridge, not the FPGA:
 *     @gps\n  -> "@GPS <utc|NONE> <pps_count> <valid 0/1> <sats>\n"
 *     @pps\n  -> "@PPS <pps_count> <ms_since_last_pps>\n"
 *     @id\n   -> "@VLFSTATION 1\n"
 *   tools/vlf_host.py  log --gps-bridge  uses @gps to stamp windows with GPS time.
 *
 * STATUS: compiles for Teensy 4.1. The NMEA parser (gps_nmea.h) is PC-tested
 *   (test/pc_test.cpp, 11/11). Not yet run on hardware.
 */

#include <Arduino.h>
#include "gps_nmea.h"

const int PIN_PPS = 2;

gps::Fix fix;
volatile uint32_t ppsCount = 0;
volatile uint32_t ppsMicros = 0;

void ppsISR() {
  ppsMicros = micros();
  ppsCount++;
}

// GPS NMEA line assembly on Serial2
char nmea[100];
int  nmeaLen = 0;

// USB->FPGA line assembly, so we can peek the first char for a '@' bridge command
char ucmd[96];
int  ucmdLen = 0;

void handleBridge(char *line) {
  if (!strcmp(line, "@gps")) {
    uint32_t c; noInterrupts(); c = ppsCount; interrupts();
    Serial.printf("@GPS %s %lu %d %d\n", fix.utc[0] ? fix.utc : "NONE",
                  (unsigned long)c, fix.valid ? 1 : 0, fix.sats);
  } else if (!strcmp(line, "@pps")) {
    uint32_t c, m; noInterrupts(); c = ppsCount; m = ppsMicros; interrupts();
    Serial.printf("@PPS %lu %lu\n", (unsigned long)c, (unsigned long)((micros() - m) / 1000));
  } else if (!strcmp(line, "@id")) {
    Serial.println("@VLFSTATION 1");
  } else {
    Serial.println("@E");
  }
}

void setup() {
  Serial.begin(115200);       // USB to the PC (baud ignored)
  Serial1.begin(115200);      // to the FPGA
  Serial2.begin(9600);        // from the GPS (NMEA)
  pinMode(PIN_PPS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PPS), ppsISR, RISING);
}

void loop() {
  // USB -> FPGA, intercepting '@' bridge commands line by line
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (ucmdLen > 0) {
        ucmd[ucmdLen] = 0;
        if (ucmd[0] == '@') {
          handleBridge(ucmd);
        } else {
          Serial1.write((const uint8_t *)ucmd, ucmdLen);
          Serial1.write('\n');
        }
        ucmdLen = 0;
      }
    } else if (ucmdLen < (int)sizeof(ucmd) - 1) {
      ucmd[ucmdLen++] = ch;
    }
  }

  // FPGA -> USB, verbatim
  while (Serial1.available()) Serial.write(Serial1.read());

  // GPS NMEA -> parser
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n' || c == '\r') {
      if (nmeaLen > 0) { nmea[nmeaLen] = 0; nmeaLen = 0; gps::parse(nmea, fix); }
    } else if (nmeaLen < (int)sizeof(nmea) - 1) {
      nmea[nmeaLen++] = c;
    }
  }
}
