/*
 * nuisance_logger.ino -- Teensy 4.1: log a signal next to the ordinary things
 * that fake one, and veto the ordinary cause after the fact (list item #159).
 *
 * WHAT IT DOES
 *   Samples a SIGNAL channel together with four NUISANCE channels -- vibration,
 *   stray magnetic field, sound, and mains pickup -- at the same instants, and
 *   computes, per sample, whether the signal is persistently correlated with any
 *   nuisance over a short window (nuisance_analysis.h, PC-tested 10/10). When it
 *   is, that nuisance is flagged as the likely cause: the 8 Hz mount resonance
 *   (#133) that sank the pilot studies, or the 60 Hz that is everywhere. Every
 *   null in the files came down to exactly this, so logging it alongside the
 *   signal is what lets you rule the ordinary cause in or out.
 *
 * WIRING (Teensy 4.1)
 *   A0 (14)  SIGNAL   whatever you are measuring (0..3.3 V, centred ~1.65 V)
 *   A1 (15)  B-FIELD  a fluxgate analog out or a coil+amp (stray magnetic field)
 *   A2 (16)  SOUND    an electret mic + amp (acoustic/infrasound)
 *   A3 (17)  MAINS    a short wire / 1 pF pickup near the mains, buffered
 *   MPU-6050 on Wire (SDA 18, SCL 19, 3.3 V): VIBRATION (accel magnitude)
 *   SD card: NUISANCE.CSV
 *   The ADC channels and the IMU are read together at FS; the vibration value is
 *   |accel| - 1 g. If the MPU is absent the vibration channel is held at 0.
 *
 * COMMANDS (USB serial 115200)
 *   help | stat
 *   fs <Hz>              sample rate 100..4000 (default 1000)
 *   window <ms>          correlation window (default 256 ms; use ~4 cycles of the
 *                        nuisance you suspect: 64 ms suits 60 Hz, 500 ms suits 8 Hz)
 *   rthr <0..1>          correlation magnitude that vetoes (default 0.5)
 *   zthr <sd>            signal excursion threshold for the contamination report (default 3)
 *   start [seconds]      log; prints a line whenever the signal excurses, naming any veto
 *   stop
 *   report               contamination: of the signal excursions so far, how many each
 *                        nuisance channel explained
 *   names <sig> <vib> <bfield> <sound> <mains>   rename the channels in the log
 *
 * FILES
 *   NUISANCE.CSV  t_s,sig,vib,bfield,sound,mains,z_sig,veto_mask   per excursion
 *                 (veto_mask bit0 vib, bit1 bfield, bit2 sound, bit3 mains)
 *
 * STATUS: written for Teensyduino 1.62. The veto core (nuisance_analysis.h) is
 *   PC-tested. Not run on hardware.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include "nuisance_analysis.h"

using namespace nuisance;

const int PIN_SIG = 14, PIN_BFIELD = 15, PIN_SOUND = 16, PIN_MAINS = 17;
const int MPU_ADDR = 0x68;
const double ADC_LSB = 3.3 / 4096.0;

double fs = 1000.0;
int windowMs = 256;
Veto<4> veto;                 // channels: 0 vib, 1 bfield, 2 sound, 3 mains
bool running = false, sdOk = false, mpuOk = false;
uint32_t stopAtMs = 0;
elapsedMicros sampleTimer;
long nExc = 0, nSamp = 0;
char chName[5][10] = {"sig", "vib", "bfield", "sound", "mains"};

IntervalTimer tick;
volatile bool sampleReady = false;

bool mpuBegin() {
  Wire.begin(); Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00);     // wake
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00);     // +-2 g
  Wire.endTransmission();
  return true;
}

double readVibration() {
  if (!mpuOk) return 0;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return 0;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  double gx = ax / 16384.0, gy = ay / 16384.0, gz = az / 16384.0;
  return sqrt(gx * gx + gy * gy + gz * gz) - 1.0;      // |accel| - 1 g (0 at rest)
}

void onTick() { sampleReady = true; }

void startRun(double seconds) {
  double rthr = veto.r_thr, zthr = veto.z_thr;          // keep the user's thresholds
  veto = Veto<4>();                                     // reset running stats and correlators
  veto.r_thr = rthr; veto.z_thr = zthr;
  veto.init((int)(windowMs / 1000.0 * fs));
  nExc = nSamp = 0;
  running = true;
  stopAtMs = seconds > 0 ? millis() + (uint32_t)(seconds * 1000) : 0;
  tick.begin(onTick, 1000000.0 / fs);
  Serial.printf("# logging: fs %.0f Hz, window %d ms (%d samples), r>=%.2f vetoes, z>=%.1f = excursion\n",
                fs, windowMs, (int)(windowMs / 1000.0 * fs), veto.r_thr, veto.z_thr);
  Serial.println("t_s,sig,vib,bfield,sound,mains,z_sig,veto");
}

void stopRun() {
  if (!running) return;
  running = false; tick.end();
  Serial.println("# stopped");
}

void report() {
  Serial.printf("# %ld samples, %ld signal excursions (|z|>=%.1f)\n", nSamp, nExc, veto.z_thr);
  for (int i = 0; i < 4; i++)
    Serial.printf("#   %-8s explained %ld of %ld excursions (%.0f %%)\n",
                  chName[i + 1], veto.vetoed[i], nExc, nExc ? 100.0 * veto.vetoed[i] / nExc : 0.0);
  long worst = 0; int wi = 0;
  for (int i = 0; i < 4; i++) if (veto.vetoed[i] > worst) { worst = veto.vetoed[i]; wi = i; }
  if (nExc > 0 && worst >= 0.5 * nExc)
    Serial.printf("# >= half the 'signal' is explained by %s: treat it as that, not a new effect.\n", chName[wi + 1]);
}

void logExcursion(double t, const double *v5, double zsig, unsigned mask) {
  char line[128];
  snprintf(line, sizeof(line), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%u",
           t, v5[0], v5[1], v5[2], v5[3], v5[4], zsig, mask);
  Serial.print("# "); Serial.println(line);
  if (sdOk) {
    bool isNew = !SD.exists("NUISANCE.CSV");
    File f = SD.open("NUISANCE.CSV", FILE_WRITE);
    if (f) { if (isNew) f.println("t_s,sig,vib,bfield,sound,mains,z_sig,veto_mask"); f.println(line); f.close(); }
  }
}

void printHelp() {
  Serial.println(F("# nuisance_logger: fs <Hz> | window <ms> | rthr <0..1> | zthr <sd> |\n"
                   "#   start [s] | stop | report | names <sig> <vib> <bfield> <sound> <mains> | stat"));
}

void handleLine(char *line) {
  char *tok[6]; int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 6; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) printHelp();
  else if (!strcmp(c, "fs") && n >= 2) { if (!running) { fs = constrain(atof(tok[1]), 100, 4000); Serial.printf("# fs %.0f Hz\n", fs); } else Serial.println("# stop first"); }
  else if (!strcmp(c, "window") && n >= 2) { windowMs = constrain(atoi(tok[1]), 10, 2000); Serial.printf("# window %d ms\n", windowMs); }
  else if (!strcmp(c, "rthr") && n >= 2) { veto.r_thr = constrain(atof(tok[1]), 0.1, 0.99); Serial.printf("# veto r>=%.2f\n", veto.r_thr); }
  else if (!strcmp(c, "zthr") && n >= 2) { veto.z_thr = constrain(atof(tok[1]), 1.0, 10.0); Serial.printf("# excursion |z|>=%.1f\n", veto.z_thr); }
  else if (!strcmp(c, "start")) startRun(n >= 2 ? atof(tok[1]) : 0);
  else if (!strcmp(c, "stop")) stopRun();
  else if (!strcmp(c, "report")) report();
  else if (!strcmp(c, "names") && n >= 6) { for (int i = 0; i < 5; i++) { strncpy(chName[i], tok[i + 1], 9); chName[i][9] = 0; } Serial.println("# renamed"); }
  else if (!strcmp(c, "stat")) Serial.printf("# %s, fs %.0f Hz, window %d ms, MPU %s, SD %s, %ld samples, %ld excursions\n",
                                             running ? "LOGGING" : "idle", fs, windowMs, mpuOk ? "ok" : "absent", sdOk ? "present" : "none", nSamp, nExc);
  else Serial.println("# unknown command, type help");
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); analogReadAveraging(4);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) {}
  mpuOk = mpuBegin();
  sdOk = SD.begin(BUILTIN_SDCARD);
  veto.init((int)(windowMs / 1000.0 * fs));
  Serial.printf("# nuisance_logger -- signal + vibration/B-field/sound/mains, vetoes the ordinary cause. MPU %s, SD %s.\n",
                mpuOk ? "found" : "absent", sdOk ? "present" : "none");
  printHelp();
}

void loop() {
  static char line[72]; static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (len > 0) { line[len] = 0; len = 0; handleLine(line); } }
    else if (len < (int)sizeof(line) - 1) line[len++] = ch;
  }
  if (running && sampleReady) {
    sampleReady = false;
    double sig = (analogRead(PIN_SIG) - 2048) * ADC_LSB;
    double nui[4];
    nui[0] = readVibration();
    nui[1] = (analogRead(PIN_BFIELD) - 2048) * ADC_LSB;
    nui[2] = (analogRead(PIN_SOUND) - 2048) * ADC_LSB;
    nui[3] = (analogRead(PIN_MAINS) - 2048) * ADC_LSB;
    unsigned mask = veto.step(sig, nui);
    nSamp++;
    if (fabs(veto.z_sig) >= veto.z_thr) {
      nExc++;
      double v5[5] = {sig, nui[0], nui[1], nui[2], nui[3]};
      logExcursion(nSamp / fs, v5, veto.z_sig, mask);
    }
    if (stopAtMs && millis() >= stopAtMs) { stopRun(); report(); }
  }
}
