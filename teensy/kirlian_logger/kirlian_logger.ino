/*
 * kirlian_logger.ino -- Teensy 4.1 controlled-Kirlian logger (list items #47, #48)
 *
 * WHAT IT DOES
 *   Kirlian (corona-discharge) photography makes a glow around a fingertip on a
 *   high-voltage plate. The 1972 CIA test and the SRI files found the glow
 *   tracks SKIN MOISTURE and CONTACT PRESSURE. This logs those two mundane
 *   causes, plus a corona-brightness measurement, for each exposure, so you can
 *   see how much of the "aura" they explain (kirlian_analysis.h fits it) and
 *   what, if anything, is left over.
 *
 *   It does NOT generate the high voltage. You build the HV corona rig from a
 *   documented circuit (an automotive ignition coil or a cold-cathode / Tesla
 *   driver, a few kV at a few tens of kHz through a dielectric plate). This
 *   board only MEASURES and times, and trips the camera.
 *
 *   Per exposure it records:
 *     - skin conductance (GSR): two dry electrodes on the same hand, a known
 *       series resistor, so a moist finger reads higher conductance;
 *     - contact pressure: a force-sensing resistor (FSR) under the plate;
 *     - corona brightness: a photodiode or a ring of them around the plate,
 *       integrated over the exposure (or an external value you type from an
 *       image if you photograph instead);
 *     - plate HV present / absent, from an isolated pickup, so a mislabelled
 *       exposure is caught.
 *   `vary` walks one variable at a time (press harder, wet the finger) so the
 *   fit is not fooled by moisture and pressure moving together.
 *
 * HIGH-VOLTAGE SAFETY -- READ BEFORE BUILDING THE RIG
 *   The corona plate carries kilovolts. The whole point of a safe build is that
 *   the person's skin NEVER touches the HV conductor: the finger sits on a
 *   grounded-return dielectric and the HV is on the far electrode, current-
 *   limited by the dielectric's capacitance to microamps. Interlock it: the HV
 *   driver is powered only while a momentary foot/hand switch is held AND this
 *   board says the exposure is armed; releasing either cuts the HV. Keep the
 *   energy tiny (small plate capacitance, low duty). This sketch's `arm` /
 *   `expose` sequence is written to drive that interlock (pin 2 = HV enable,
 *   high only during an armed exposure). Build the HV side from a published
 *   circuit, not freehand, and if you are not comfortable with kV, do not.
 *
 * WIRING (Teensy 4.1)
 *   A0 (14)  GSR: electrode -> skin -> electrode -> R_SERIES (100 k) -> 3.3 V;
 *            node (skin/R_SERIES junction) -> A0. Conductance from the divider.
 *   A1 (15)  FSR pressure: FSR from 3.3 V to A1, 10 k from A1 to GND.
 *   A2 (16)  corona photodiode (reverse-biased) into a transimpedance amp -> A2;
 *            integrated over the exposure window.
 *   A3 (17)  HV-present pickup: a small plate near (not on) the HV, through a
 *            1 M / 1 nF divider and a diode detector -> A3 (isolated, tiny).
 *   pin 2    HV ENABLE out (to the interlock; high only during an armed exposure)
 *   pin 3    camera trigger out (high for TRIG_MS at the start of the exposure)
 *   pin 4    foot/hand switch in (active low, INPUT_PULLUP): must be held to expose
 *   SD card: KIRLIAN.CSV
 *
 * COMMANDS (USB serial 115200)
 *   help | stat
 *   gsrcal <dry_counts> <wet_counts>   two-point GSR calibration (optional; raw otherwise)
 *   presscal <zero_counts> <ref_g> <ref_counts>   FSR calibration (optional)
 *   expms <ms>            exposure length (default 500)
 *   arm                   prints the HV warning; `arm yes` enables an exposure
 *   expose [corona_ext]   one exposure: hold the switch; optionally give an external
 *                         corona value (e.g. from an image) instead of the photodiode
 *   vary <moist|press> <n>   prompt-guided series walking one variable (labels the rows)
 *   fit                   run the moisture+pressure fit over the logged exposures, print R^2
 *                         and the residual corona per exposure
 *   clear                 start a new log file
 *   stop
 *
 * FILES
 *   KIRLIAN.CSV  idx,label,gsr_uS,press_g,corona,hv_present,source   per exposure
 *
 * STATUS: written for Teensyduino 1.62. The fit (kirlian_analysis.h) is
 *   PC-tested (test/pc_test.cpp). Not run on hardware. Build the HV rig yourself
 *   from a documented, interlocked, current-limited circuit.
 */

#include <Arduino.h>
#include <SD.h>
#include "kirlian_analysis.h"

const int PIN_GSR = 14, PIN_PRESS = 15, PIN_CORONA = 16, PIN_HV_SENSE = 17;
const int PIN_HV_EN = 2, PIN_TRIG = 3, PIN_SWITCH = 4;
const int TRIG_MS = 30;
const double R_SERIES = 100000.0, VREF = 3.3;
const int MAXEXP = 200;

double corona[MAXEXP], moist[MAXEXP], press[MAXEXP];
int hvFlag[MAXEXP];
char labels[MAXEXP][16];
int nExp = 0;

bool armed = false, sdOk = false;
uint32_t expMs = 500;
double gsrDry = -1, gsrWet = -1, pressZero = -1, pressRefG = 0, pressRefCt = -1;

double readGsrUS() {
  // node voltage divider: V = 3.3 * Rskin / (Rskin + R_SERIES) is wrong orientation;
  // here skin is from node to electrode to 3.3 V through R_SERIES to node... simplest:
  // conductance ~ (counts). With calibration map counts to microsiemens.
  int c = analogRead(PIN_GSR);
  if (gsrDry >= 0 && gsrWet > gsrDry) {
    // linear map: dry -> ~2 uS, wet -> ~20 uS (typical finger range)
    double f = (c - gsrDry) / (gsrWet - gsrDry);
    return 2.0 + f * 18.0;
  }
  // uncalibrated: report conductance of the divider in uS
  double v = c * VREF / 4096.0;
  double rskin = (v > 0.001 && v < VREF - 0.001) ? R_SERIES * v / (VREF - v) : 1e9;
  return 1e6 / rskin;
}

double readPressG() {
  int c = analogRead(PIN_PRESS);
  if (pressZero >= 0 && pressRefCt > pressZero)
    return pressRefG * (c - pressZero) / (pressRefCt - pressZero);
  return c;   // raw counts
}

double integrateCorona() {
  // integrate the photodiode over the exposure window
  uint32_t t0 = millis();
  double acc = 0; long n = 0;
  while (millis() - t0 < expMs) { acc += analogRead(PIN_CORONA); n++; }
  return n ? acc / n : 0;
}

bool hvPresent() { return analogRead(PIN_HV_SENSE) > 200; }

void logExposure(const char *label, double g, double p, double co, int hv, const char *src) {
  if (nExp < MAXEXP) {
    corona[nExp] = co; moist[nExp] = g; press[nExp] = p; hvFlag[nExp] = hv;
    strncpy(labels[nExp], label, 15); labels[nExp][15] = 0;
    nExp++;
  }
  if (sdOk) {
    bool isNew = !SD.exists("KIRLIAN.CSV");
    File f = SD.open("KIRLIAN.CSV", FILE_WRITE);
    if (f) {
      if (isNew) f.println("idx,label,gsr_uS,press_g,corona,hv_present,source");
      f.printf("%d,%s,%.3f,%.2f,%.2f,%d,%s\n", nExp, label, g, p, co, hv, src);
      f.close();
    }
  }
  Serial.printf("# exposure %d [%s]: GSR %.2f uS, pressure %.1f, corona %.1f, HV %s (%s)\n",
                nExp, label, g, p, co, hv ? "present" : "ABSENT", src);
  if (!hv) Serial.println("# WARNING: no HV detected during this exposure -- excluded from the fit");
}

void doExpose(const char *label, double coronaExt, bool haveExt) {
  if (!armed) { Serial.println("# not armed: `arm` then `arm yes`"); return; }
  if (digitalRead(PIN_SWITCH) == HIGH) { Serial.println("# hold the foot/hand switch (pin 4 to GND) to expose"); return; }
  double g = readGsrUS(), p = readPressG();
  digitalWriteFast(PIN_HV_EN, HIGH);
  digitalWriteFast(PIN_TRIG, HIGH);
  uint32_t tt = millis();
  double co = haveExt ? coronaExt : integrateCorona();
  while (millis() - tt < (uint32_t)TRIG_MS) {}
  digitalWriteFast(PIN_TRIG, LOW);
  int hv = hvPresent() ? 1 : 0;
  digitalWriteFast(PIN_HV_EN, LOW);
  logExposure(label, g, p, co, hv, haveExt ? "image" : "photodiode");
}

void runFit() {
  // use only exposures where HV was present
  static double c2[MAXEXP], m2[MAXEXP], p2[MAXEXP];
  int idx[MAXEXP], k = 0;
  for (int i = 0; i < nExp; i++) if (hvFlag[i]) { c2[k] = corona[i]; m2[k] = moist[i]; p2[k] = press[i]; idx[k] = i; k++; }
  if (k < 4) { Serial.printf("# need at least 4 valid exposures, have %d\n", k); return; }
  kirlian::Fit f = kirlian::fit(c2, m2, p2, k);
  if (!f.ok) { Serial.println("# fit failed (vary moisture and pressure independently: use `vary`)"); return; }
  Serial.printf("# fit over %d exposures: corona = %.2f + %.3f*GSR(uS) + %.4f*pressure\n", k, f.b0, f.b_moist, f.b_press);
  Serial.printf("# moisture + pressure explain R^2 = %.3f of the corona variance; residual rms %.2f\n", f.r2, f.rms_resid);
  Serial.println("# residual corona per exposure (what moisture + pressure do NOT explain):");
  Serial.println("idx,label,corona,predicted,residual");
  for (int j = 0; j < k; j++) {
    double pred = f.b0 + f.b_moist * m2[j] + f.b_press * p2[j];
    Serial.printf("%d,%s,%.2f,%.2f,%+.2f\n", idx[j] + 1, labels[idx[j]], c2[j], pred, c2[j] - pred);
  }
  if (f.r2 > 0.8) Serial.println("# most of the 'aura' is moisture + pressure, measured. The residual is the open question.");
}

void printHelp() {
  Serial.println(F("# kirlian_logger: gsrcal <dry> <wet> | presscal <zero> <g> <counts> | expms <ms> |\n"
                   "#   arm | expose [corona] | vary <moist|press> <n> | fit | clear | stop | stat\n"
                   "#   (this board measures and interlocks; build the HV rig from a documented circuit)"));
}

int varyStep = 0, varyLeft = 0; char varyLabel[8] = "";

void handleLine(char *line) {
  char *tok[4]; int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 4; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) printHelp();
  else if (!strcmp(c, "arm")) {
    if (n >= 2 && !strcmp(tok[1], "yes")) { armed = true; Serial.println("# armed: hold the switch and `expose`. HV enable follows the exposure."); }
    else Serial.println(F("# HIGH VOLTAGE on the plate. The skin must never touch the HV conductor:\n"
                          "# finger on a grounded dielectric, HV on the far side, current-limited to microamps,\n"
                          "# interlocked to the switch and to `arm`. Build from a documented circuit. `arm yes` to enable."));
  }
  else if (!strcmp(c, "expose")) {
    char lbl[16]; if (varyLeft > 0) { snprintf(lbl, sizeof(lbl), "%s%d", varyLabel, varyStep); varyStep++; varyLeft--; }
    else snprintf(lbl, sizeof(lbl), "e%d", nExp + 1);
    doExpose(lbl, n >= 2 ? atof(tok[1]) : 0, n >= 2);
  }
  else if (!strcmp(c, "vary") && n >= 3) {
    strncpy(varyLabel, tok[1], 7); varyLabel[7] = 0; varyLeft = atoi(tok[2]); varyStep = 1;
    Serial.printf("# vary %s over %d exposures: change ONLY %s each time (keep the other steady), `expose` each.\n", varyLabel, varyLeft, varyLabel);
  }
  else if (!strcmp(c, "gsrcal") && n >= 3) { gsrDry = atof(tok[1]); gsrWet = atof(tok[2]); Serial.printf("# GSR cal dry %.0f wet %.0f counts\n", gsrDry, gsrWet); }
  else if (!strcmp(c, "presscal") && n >= 4) { pressZero = atof(tok[1]); pressRefG = atof(tok[2]); pressRefCt = atof(tok[3]); Serial.printf("# pressure cal: %.0f counts = 0 g, %.0f counts = %.1f g\n", pressZero, pressRefCt, pressRefG); }
  else if (!strcmp(c, "expms") && n >= 2) { expMs = constrain(atoi(tok[1]), 10, 5000); Serial.printf("# exposure %lu ms\n", (unsigned long)expMs); }
  else if (!strcmp(c, "fit")) runFit();
  else if (!strcmp(c, "clear")) { nExp = 0; if (sdOk) SD.remove("KIRLIAN.CSV"); Serial.println("# log cleared"); }
  else if (!strcmp(c, "stop")) { armed = false; digitalWriteFast(PIN_HV_EN, LOW); Serial.println("# stopped, HV disabled"); }
  else if (!strcmp(c, "stat")) Serial.printf("# %s, %d exposures logged, exposure %lu ms, SD %s, switch %s\n",
                                             armed ? "ARMED" : "not armed", nExp, (unsigned long)expMs, sdOk ? "present" : "none",
                                             digitalRead(PIN_SWITCH) == LOW ? "held" : "open");
  else Serial.println("# unknown command, type help");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_HV_EN, OUTPUT); digitalWriteFast(PIN_HV_EN, LOW);
  pinMode(PIN_TRIG, OUTPUT); digitalWriteFast(PIN_TRIG, LOW);
  pinMode(PIN_SWITCH, INPUT_PULLUP);
  analogReadResolution(12); analogReadAveraging(8);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) {}
  sdOk = SD.begin(BUILTIN_SDCARD);
  Serial.println("# kirlian_logger -- controlled Kirlian: logs moisture + pressure + corona. HV enable is OFF until an armed exposure.");
  printHelp();
}

void loop() {
  static char line[64]; static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (len > 0) { line[len] = 0; len = 0; handleLine(line); } }
    else if (len < (int)sizeof(line) - 1) line[len++] = ch;
  }
}
