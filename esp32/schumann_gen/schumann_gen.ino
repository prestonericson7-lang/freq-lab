/*
 * schumann_gen.ino -- ESP32 calibrated ELF magnetic-field generator
 *                     (7.83 Hz "Schumann" fundamental + harmonics), with a
 *                     current-sense calibration loop, a blinded sham mode,
 *                     a magnetophosphene threshold mode, and a phone web page.
 *
 * WHAT IT IS
 *   A signal generator that drives a coil with a sum of sines at the
 *   Earth-ionosphere cavity frequencies (7.83, 14.3, 20.8, 27.3, 33.8 Hz) and
 *   KNOWS the field it makes: it measures the coil current through a sense
 *   resistor and computes the field from the coil geometry, so the display
 *   reads microtesla, not "level 7". Everything a consumer "7.83 Hz device"
 *   does not do: a stated, measured field, a real sham mode, and a log.
 *
 *   The cavity resonances are real (Schumann 1952; measured daily by ELF
 *   observatories at about 1 pT). Whether a generated field at 7.83 Hz does
 *   anything to a person is not established; this device makes no such claim.
 *   It lets you run the blinded test yourself (`blind`).
 *
 * HARDWARE (ESP32 classic: DAC on GPIO25; ESP32-S3 has no DAC -> LEDC PWM + RC)
 *   GPIO25 (DAC1)  -> waveform, 0..3.3 V centred on 1.65 V
 *                     S3: GPIO17 PWM 78 kHz 10-bit -> 10 kOhm -> 100 nF -> same
 *   coil driver: a DC-coupled power buffer (an audio amp will not pass 7.8 Hz):
 *                     DAC -> op-amp (gain 1..3, e.g. half an LM358 from a 9-12 V
 *                     supply, non-inverting, input biased at 1.65 V) -> complementary
 *                     emitter follower (BD139/BD140 or TIP31/TIP32 on a small
 *                     heatsink) -> coil -> R_SENSE -> ground
 *   GPIO34 (ADC)   <- top of R_SENSE through 10 k, 100 nF to ground (0..3.3 V!)
 *                     a 1 Ohm sense at 1 A peak = 1 V peak: fine
 *   GPIO35 (ADC)   <- optional Hall/fluxgate analog output for `verify a`
 *   GPIO27         <- optional FG-3 style fluxgate frequency output for `verify f`
 *   GPIO0 (BOOT)   button: marks the threshold in `phosphene` mode, also `stop`
 *   GPIO2          LED: on while the field is on (and in sham-on sessions)
 *
 *   Coil (default numbers, change with `coil`): 200 turns, 100 mm radius,
 *   24 AWG (about 5 Ohm). 100 mA peak gives 126 uT peak at the centre and
 *   about 45 uT at 100 mm on axis -- the Earth's field is ~50 uT.
 *
 * FIELD MATHS
 *   centre of a circular coil:  B = mu0 * N * I / (2 R)
 *   on axis at distance d:      B(d) = mu0 * N * I * R^2 / (2 (R^2 + d^2)^1.5)
 *   I is measured: I = V_sense / R_sense (ADC, 12 bit, calibrated by the ESP32
 *   core's eFuse values; `cal` trims it against a meter).
 *
 * COMMANDS (USB serial 115200, newline-terminated) -- the web page uses the same
 *   help | stat
 *   freq <Hz>                      fundamental, 0.1..100 (default 7.83)
 *   harm <n> <relative 0..1>       harmonic n = 1..5 (1 = fundamental), default 1,.5,.35,.25,.2
 *   harmonics <on|off>             all harmonics at once (default on)
 *   amp <0..1>                     output scale (DAC swing)
 *   coil <turns> <radius_mm> <rsense_ohm>
 *   target <uT_peak> [distance_mm] closed loop on the sense current to reach that field
 *   on | off
 *   verify <a|f> [uT_per_V | Hz_per_uT and Hz_at_zero]
 *                                  read the external sensor and compare with the computed field
 *   blind <sessions> <minutes>     random on/sham schedule; prints the SHA-256 commitment
 *   reveal                         prints the schedule after the sessions
 *   phosphene <f0> <f1> <seconds>  sweep f0->f1 at the set amp; press BOOT when you see flicker
 *   phosphene ramp <Hz> <seconds>  fixed frequency, amplitude 0->amp; press BOOT at threshold
 *   save | load                    settings in flash
 *   wifi <on|off>                  the phone page (AP "SCHUMANN-xxxx", password schumann1)
 *
 * MAGNETOPHOSPHENES (the `phosphene` mode)
 *   A changing magnetic field makes a faint flicker in the dark at the retina
 *   (Lovsund 1980; ICNIRP 2010): threshold about 10-20 mT peak at 20 Hz,
 *   higher either side. That is 100x this coil's field at 100 mA. To reach it
 *   you need about 5 A peak in the default coil (B = 6.3 mT at the centre;
 *   put the eye at the centre of a 100 mm coil) or 500 turns at 2 A: a real
 *   power stage, thick wire (18 AWG), and the coil gets warm (5 A into 1 Ohm =
 *   25 W peak). The phenomenon itself is harmless and reversible; the
 *   hazards are the coil temperature and the driver. The firmware limits a
 *   phosphene run to 60 s and ramps down on the button.
 *
 * STATUS: written for the ESP32 Arduino core 3.3 (esp32:esp32:esp32 and
 *   esp32:esp32:esp32s3). See the README for the compile check.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include "mbedtls/sha256.h"
#include "driver/timer.h"
#if !defined(CONFIG_IDF_TARGET_ESP32)
#include "driver/ledc.h"
#define HAS_DAC 0
#else
#include "driver/dac.h"
#define HAS_DAC 1
#endif

// ------------------------------------------------------------------ pins
const int PIN_DAC    = 25;     // ESP32 classic DAC1
const int PIN_PWM    = 17;     // S3 / other: LEDC output
const int PIN_SENSE  = 34;     // ADC: coil current sense
const int PIN_HALL   = 35;     // ADC: external analog field sensor
const int PIN_FLUX   = 27;     // digital: fluxgate frequency output
const int PIN_BUTTON = 0;
const int PIN_LED    = 2;

// ------------------------------------------------------------------ synthesis
const double FS = 8000.0;                          // waveform samples per second
const int    NH = 5;
const double MU0 = 4.0e-7 * PI;
const double SCHUMANN[NH] = {7.83, 14.3, 20.8, 27.3, 33.8};   // mode centre frequencies

volatile uint32_t phase[NH] = {0, 0, 0, 0, 0};
volatile uint32_t pinc[NH]  = {0, 0, 0, 0, 0};
volatile float    hrel[NH]  = {1.0f, 0.5f, 0.35f, 0.25f, 0.2f};
volatile bool     harmOn    = true;
volatile float    ampScale  = 0.3f;                // 0..1 of the DAC swing
volatile bool     outputOn  = false;
volatile uint32_t sampleCount = 0;
volatile float    sumNorm = 1.0f;                  // 1 / sum of relative amplitudes (keeps peak <= 1)
static int16_t    sineTab[1024];

hw_timer_t *synthTimer = nullptr;
Preferences prefs;
WebServer web(80);

double fundamental = 7.83;
double coilTurns = 200, coilRadius_m = 0.100, rSense = 1.0;
double adcGainTrim = 1.0;                          // `cal` multiplies the ADC volts by this
double targetUT = 0, targetDist_m = 0;             // closed loop when > 0
bool   wifiOn = false;
bool   blindRunning = false;
String blindSchedule, blindSalt, blindCommit;
int    blindN = 0, blindMinutes = 0, blindIdx = -1;
uint32_t blindStartMs = 0;
bool   phosRunning = false, phosRamp = false;
double phosF0 = 10, phosF1 = 40, phosFixed = 20;
uint32_t phosStartMs = 0, phosDurMs = 0;
float  phosAmpMax = 0.3f;

// ------------------------------------------------------------------ ISR: one sample
void IRAM_ATTR onSynthTimer() {
  int32_t acc = 0;
  int n = harmOn ? NH : 1;
  for (int k = 0; k < n; k++) {
    phase[k] += pinc[k];
    acc += (int32_t)(sineTab[phase[k] >> 22] * hrel[k]);
  }
  float v = acc * sumNorm * ampScale / 32767.0f;      // -1..1
  if (!outputOn) v = 0;
#if HAS_DAC
  int code = 128 + (int)(v * 127.0f);
  dac_output_voltage(DAC_CHAN_0, (uint8_t)constrain(code, 0, 255));    // DAC_CHAN_0 = GPIO25
#else
  int code = 512 + (int)(v * 511.0f);
  ledcWrite(PIN_PWM, constrain(code, 0, 1023));
#endif
  sampleCount = sampleCount + 1;
}

void setFrequency(double f0) {
  fundamental = f0;
  for (int k = 0; k < NH; k++) {
    double fk = (k == 0) ? f0 : f0 * SCHUMANN[k] / SCHUMANN[0];   // harmonics scale with the fundamental
    pinc[k] = (uint32_t)(fk / FS * 4294967296.0);
  }
}

void updateNorm() {
  float s = 0;
  int n = harmOn ? NH : 1;
  for (int k = 0; k < n; k++) s += hrel[k];
  sumNorm = (s > 0) ? 1.0f / s : 1.0f;
}

// ------------------------------------------------------------------ measurement
// peak coil current from the sense resistor: sample one full fundamental period
double measurePeakCurrent() {
  uint32_t n = (uint32_t)(1000000.0 / fundamental) + 2000;   // us of one period + margin
  uint32_t t0 = micros();
  int vmax = 0, vmin = 4095;
  while (micros() - t0 < n) {
    int v = analogReadMilliVolts(PIN_SENSE);
    if (v > vmax) vmax = v;
    if (v < vmin) vmin = v;
    delayMicroseconds(200);
  }
  // the sense node swings around its DC level (unipolar driver) or around 0
  double vpk = (vmax - vmin) / 2.0 / 1000.0 * adcGainTrim;
  return vpk / rSense;
}

double fieldCentre(double ipk) { return MU0 * coilTurns * ipk / (2.0 * coilRadius_m); }
double fieldAxis(double ipk, double d) {
  double r2 = coilRadius_m * coilRadius_m;
  return MU0 * coilTurns * ipk * r2 / (2.0 * pow(r2 + d * d, 1.5));
}

// closed loop: nudge ampScale so the measured field meets the target
void serviceTarget() {
  static uint32_t last = 0;
  if (targetUT <= 0 || !outputOn || millis() - last < 1500) return;
  last = millis();
  double ipk = measurePeakCurrent();
  double b = (targetDist_m > 0 ? fieldAxis(ipk, targetDist_m) : fieldCentre(ipk)) * 1e6;
  if (b < 0.05) { Serial.println("# target: no current measured (driver off? sense wiring?)"); return; }
  double ratio = targetUT / b;
  float next = constrain(ampScale * (float)pow(ratio, 0.5), 0.0f, 1.0f);   // half-step, stable
  ampScale = next;
  if (fabs(ratio - 1.0) > 0.03)
    Serial.printf("# target %.2f uT: measured %.2f uT, amp -> %.3f\n", targetUT, b, ampScale);
  if (ampScale >= 0.999f && ratio > 1.05)
    Serial.println("# WARNING: amp at maximum and still below target: more turns, current or a closer distance");
}

// ------------------------------------------------------------------ blind / sham
String sha256Hex(const String &s) {
  unsigned char out[32];
  mbedtls_sha256((const unsigned char *)s.c_str(), s.length(), out, 0);
  String h;
  for (int i = 0; i < 32; i++) { char b[3]; snprintf(b, 3, "%02x", out[i]); h += b; }
  return h;
}

void startBlind(int n, int minutes) {
  blindSchedule = "";
  int on = n / 2;
  for (int i = 0; i < n; i++) blindSchedule += (i < on) ? 'F' : 'S';   // F = field, S = sham
  for (int i = n - 1; i > 0; i--) {                                      // Fisher-Yates with the hardware RNG
    int j = esp_random() % (i + 1);
    char t = blindSchedule[i]; blindSchedule[i] = blindSchedule[j]; blindSchedule[j] = t;
  }
  blindSalt = String((uint32_t)esp_random(), HEX) + String((uint32_t)esp_random(), HEX);
  blindCommit = sha256Hex(blindSchedule + ":" + blindSalt);
  blindN = n; blindMinutes = minutes; blindIdx = -1; blindRunning = true;
  blindStartMs = millis();
  prefs.putString("b_sched", blindSchedule);
  prefs.putString("b_salt", blindSalt);
  prefs.putString("b_commit", blindCommit);
  Serial.printf("# blind: %d sessions of %d min, half field, half sham, order drawn by the hardware RNG\n", n, minutes);
  Serial.printf("# commitment (SHA-256 of schedule:salt): %s\n", blindCommit.c_str());
  Serial.println("# write that down or send it to someone now; `reveal` after the last session shows the schedule.");
  Serial.println("# the LED and the coil current look the same in both conditions (the driver runs, the amplitude is zero in sham).");
}

void serviceBlind() {
  if (!blindRunning) return;
  uint32_t el = millis() - blindStartMs;
  int idx = el / (blindMinutes * 60000UL);
  if (idx >= blindN) {
    blindRunning = false;
    outputOn = false;
    Serial.println("# blind: all sessions done. Write down what you noticed per session, then `reveal`.");
    return;
  }
  if (idx != blindIdx) {
    blindIdx = idx;
    bool field = blindSchedule[idx] == 'F';
    outputOn = true;
    ampScale = field ? phosAmpMax : 0.0f;          // phosAmpMax doubles as the remembered amp
    Serial.printf("# session %d of %d started\n", idx + 1, blindN);
  }
}

// ------------------------------------------------------------------ phosphene mode
void servicePhosphene() {
  if (!phosRunning) return;
  uint32_t el = millis() - phosStartMs;
  if (el > phosDurMs || el > 60000) {
    phosRunning = false; outputOn = false;
    Serial.println("# phosphene: run ended without a button press (no flicker seen, or too weak)");
    return;
  }
  double x = (double)el / phosDurMs;
  if (phosRamp) {
    ampScale = (float)(phosAmpMax * x);
  } else {
    setFrequency(phosF0 + (phosF1 - phosF0) * x);
  }
  if (digitalRead(PIN_BUTTON) == LOW) {
    double ipk = measurePeakCurrent();
    Serial.printf("# THRESHOLD: %.2f Hz, amp %.3f, I_peak %.3f A, B_centre %.1f uT (%.2f mT)\n",
                  fundamental, ampScale, ipk, fieldCentre(ipk) * 1e6, fieldCentre(ipk) * 1e3);
    prefs.putString("phos_last", String(fundamental, 2) + "Hz " + String(fieldCentre(ipk) * 1e3, 2) + "mT");
    phosRunning = false; outputOn = false;
    setFrequency(7.83);
    delay(300);
  }
}

// ------------------------------------------------------------------ settings
void saveSettings() {
  prefs.putDouble("f0", fundamental);
  prefs.putFloat("amp", ampScale);
  prefs.putBool("harm", harmOn);
  for (int k = 0; k < NH; k++) prefs.putFloat((String("h") + k).c_str(), hrel[k]);
  prefs.putDouble("turns", coilTurns);
  prefs.putDouble("radius", coilRadius_m);
  prefs.putDouble("rsense", rSense);
  prefs.putDouble("trim", adcGainTrim);
  Serial.println("# saved");
}

void loadSettings() {
  setFrequency(prefs.getDouble("f0", 7.83));
  ampScale = prefs.getFloat("amp", 0.3f);
  harmOn = prefs.getBool("harm", true);
  for (int k = 0; k < NH; k++) hrel[k] = prefs.getFloat((String("h") + k).c_str(), hrel[k]);
  coilTurns = prefs.getDouble("turns", 200);
  coilRadius_m = prefs.getDouble("radius", 0.100);
  rSense = prefs.getDouble("rsense", 1.0);
  adcGainTrim = prefs.getDouble("trim", 1.0);
  updateNorm();
}

// ------------------------------------------------------------------ status
String statusJson() {
  double ipk = outputOn ? measurePeakCurrent() : 0;
  String s = "{";
  s += "\"on\":" + String(outputOn ? "true" : "false");
  s += ",\"freq\":" + String(fundamental, 3);
  s += ",\"amp\":" + String(ampScale, 3);
  s += ",\"harmonics\":" + String(harmOn ? "true" : "false");
  s += ",\"i_peak_A\":" + String(ipk, 4);
  s += ",\"b_centre_uT\":" + String(fieldCentre(ipk) * 1e6, 2);
  s += ",\"b_100mm_uT\":" + String(fieldAxis(ipk, 0.1) * 1e6, 2);
  s += ",\"coil\":{\"turns\":" + String(coilTurns, 0) + ",\"radius_mm\":" + String(coilRadius_m * 1000, 1) + ",\"rsense\":" + String(rSense, 3) + "}";
  s += ",\"target_uT\":" + String(targetUT, 2);
  s += ",\"blind\":" + String(blindRunning ? "true" : "false");
  s += ",\"phosphene\":" + String(phosRunning ? "true" : "false");
  s += ",\"samples\":" + String(sampleCount);
  s += "}";
  return s;
}

void printStat() {
  Serial.println(statusJson());
}

// ------------------------------------------------------------------ web page
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Schumann generator</title><style>body{font-family:system-ui;background:#111;color:#eee;margin:16px}
input,button{font-size:18px;margin:4px}button{padding:8px 14px}pre{background:#222;padding:8px}</style></head><body>
<h2>ELF field generator</h2>
<p>Frequency <input id=f type=number step=0.01 value=7.83 style="width:90px"> Hz &nbsp; Amplitude <input id=a type=range min=0 max=1 step=0.01 value=0.3></p>
<p><button onclick="cmd('on')">ON</button><button onclick="cmd('off')">OFF</button>
<button onclick="cmd('harmonics on')">harmonics on</button><button onclick="cmd('harmonics off')">off</button>
<button onclick="cmd('freq '+f.value)">set freq</button><button onclick="cmd('amp '+a.value)">set amp</button></p>
<p>Target field <input id=t type=number step=0.1 value=10 style="width:80px"> uT at <input id=d type=number value=100 style="width:70px"> mm
<button onclick="cmd('target '+t.value+' '+d.value)">hold</button></p>
<pre id=s>...</pre>
<script>function cmd(c){fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.text()).then(t=>{s.textContent=t})}
setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>{s.textContent=t}),2000)</script></body></html>)HTML";

String handleCommandLine(String line);

void startWifi() {
  char ssid[24];
  uint64_t mac = ESP.getEfuseMac();
  snprintf(ssid, sizeof(ssid), "SCHUMANN-%04X", (unsigned)(mac & 0xFFFF));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, "schumann1");
  web.on("/", []() { web.send_P(200, "text/html", PAGE); });
  web.on("/status", []() { web.send(200, "application/json", statusJson()); });
  web.on("/cmd", []() { web.send(200, "text/plain", handleCommandLine(web.arg("c"))); });
  web.begin();
  wifiOn = true;
  Serial.printf("# wifi AP %s password schumann1, page at http://%s/\n", ssid, WiFi.softAPIP().toString().c_str());
}

// ------------------------------------------------------------------ commands
String handleCommandLine(String line) {
  char buf[96];
  line.trim();
  line.toCharArray(buf, sizeof(buf));
  char *tok[6];
  int n = 0;
  for (char *p = strtok(buf, " \t"); p && n < 6; p = strtok(NULL, " \t")) tok[n++] = p;
  if (n == 0) return "";
  const char *c = tok[0];
  String r = "ok";
  if (!strcmp(c, "help")) {
    r = "freq amp harm harmonics on off coil target verify blind reveal phosphene save load wifi stat";
  } else if (!strcmp(c, "stat")) {
    r = statusJson();
  } else if (!strcmp(c, "freq") && n >= 2) {
    double f = atof(tok[1]);
    if (f < 0.1 || f > 100) r = "freq must be 0.1..100 Hz"; else { setFrequency(f); r = "freq " + String(f, 3); }
  } else if (!strcmp(c, "amp") && n >= 2) {
    ampScale = constrain((float)atof(tok[1]), 0.0f, 1.0f);
    phosAmpMax = ampScale;
    targetUT = 0;
    r = "amp " + String(ampScale, 3);
  } else if (!strcmp(c, "harm") && n >= 3) {
    int k = atoi(tok[1]) - 1;
    if (k < 0 || k >= NH) r = "harm 1..5"; else { hrel[k] = constrain((float)atof(tok[2]), 0.0f, 1.0f); updateNorm(); }
  } else if (!strcmp(c, "harmonics") && n >= 2) {
    harmOn = !strcmp(tok[1], "on");
    updateNorm();
  } else if (!strcmp(c, "on")) {
    outputOn = true; digitalWrite(PIN_LED, HIGH);
  } else if (!strcmp(c, "off")) {
    outputOn = false; targetUT = 0; blindRunning = false; phosRunning = false; digitalWrite(PIN_LED, LOW);
  } else if (!strcmp(c, "coil") && n >= 4) {
    coilTurns = atof(tok[1]); coilRadius_m = atof(tok[2]) / 1000.0; rSense = atof(tok[3]);
    r = "coil " + String(coilTurns, 0) + " turns, " + String(coilRadius_m * 1000, 1) + " mm, " + String(rSense, 3) + " ohm sense";
  } else if (!strcmp(c, "target") && n >= 2) {
    targetUT = atof(tok[1]);
    targetDist_m = (n >= 3) ? atof(tok[2]) / 1000.0 : 0;
    outputOn = true; digitalWrite(PIN_LED, HIGH);
    r = "holding " + String(targetUT, 2) + " uT" + (targetDist_m > 0 ? " at " + String(targetDist_m * 1000, 0) + " mm" : " at the centre");
  } else if (!strcmp(c, "cal") && n >= 2) {
    // `cal <true_peak_A>`: a meter in series read this; trim the ADC path to match
    double meas = measurePeakCurrent() / adcGainTrim;
    if (meas > 1e-4) { adcGainTrim = atof(tok[1]) / meas; r = "trim " + String(adcGainTrim, 4); } else r = "no current to calibrate on";
  } else if (!strcmp(c, "verify") && n >= 2) {
    double ipk = measurePeakCurrent();
    double bc = fieldCentre(ipk) * 1e6;
    if (tok[1][0] == 'a') {
      double uTperV = (n >= 3) ? atof(tok[2]) : 1.0;
      int vmax = 0, vmin = 4095;
      uint32_t t0 = millis();
      while (millis() - t0 < 1000) { int v = analogReadMilliVolts(PIN_HALL); if (v > vmax) vmax = v; if (v < vmin) vmin = v; delay(1); }
      double bs = (vmax - vmin) / 2.0 / 1000.0 * uTperV;
      r = "computed " + String(bc, 2) + " uT peak at the centre; sensor " + String(bs, 2) + " uT peak (" + String(uTperV, 1) + " uT/V)";
    } else {
      // fluxgate with a frequency output: count edges for 200 ms with the field off and on
      double hzPeruT = (n >= 3) ? atof(tok[2]) : 1.0;
      bool was = outputOn;
      outputOn = false; delay(300);
      uint32_t t0 = micros(); long n0 = 0; int last = digitalRead(PIN_FLUX);
      while (micros() - t0 < 200000) { int v = digitalRead(PIN_FLUX); if (v != last) { n0++; last = v; } }
      outputOn = was; delay(300);
      t0 = micros(); long n1 = 0; last = digitalRead(PIN_FLUX);
      while (micros() - t0 < 200000) { int v = digitalRead(PIN_FLUX); if (v != last) { n1++; last = v; } }
      double f0 = n0 / 2.0 / 0.2, f1 = n1 / 2.0 / 0.2;
      r = "computed " + String(bc, 2) + " uT peak; fluxgate " + String(f0, 0) + " Hz off, " + String(f1, 0) + " Hz on = " + String((f1 - f0) / hzPeruT, 2) + " uT mean shift (" + String(hzPeruT, 1) + " Hz/uT)";
    }
  } else if (!strcmp(c, "blind") && n >= 3) {
    int ns = atoi(tok[1]), mins = atoi(tok[2]);
    if (ns < 2 || ns > 40 || mins < 1 || mins > 60) r = "blind <2..40 sessions> <1..60 minutes>"; else startBlind(ns, mins);
  } else if (!strcmp(c, "reveal")) {
    if (blindRunning) r = "sessions still running";
    else {
      String sched = prefs.getString("b_sched", ""), salt = prefs.getString("b_salt", "");
      r = "schedule " + sched + " (F = field, S = sham), salt " + salt + ", commitment " + sha256Hex(sched + ":" + salt);
    }
  } else if (!strcmp(c, "phosphene") && n >= 4) {
    if (!strcmp(tok[1], "ramp")) {
      phosRamp = true; phosFixed = atof(tok[2]); phosDurMs = (uint32_t)(atof(tok[3]) * 1000);
      setFrequency(phosFixed); phosAmpMax = ampScale; ampScale = 0;
    } else {
      phosRamp = false; phosF0 = atof(tok[1]); phosF1 = atof(tok[2]); phosDurMs = (uint32_t)(atof(tok[3]) * 1000);
      setFrequency(phosF0);
    }
    harmOn = false; updateNorm();
    phosRunning = true; phosStartMs = millis(); outputOn = true; digitalWrite(PIN_LED, HIGH);
    r = "phosphene run started (max 60 s): press BOOT the moment you see the flicker";
  } else if (!strcmp(c, "save")) {
    saveSettings();
  } else if (!strcmp(c, "load")) {
    loadSettings(); r = "loaded";
  } else if (!strcmp(c, "wifi") && n >= 2) {
    if (!strcmp(tok[1], "on") && !wifiOn) startWifi();
    else if (!strcmp(tok[1], "off") && wifiOn) { web.stop(); WiFi.mode(WIFI_OFF); wifiOn = false; }
  } else {
    r = "unknown command, type help";
  }
  return r;
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_FLUX, INPUT);
  analogReadResolution(12);
  for (int i = 0; i < 1024; i++) sineTab[i] = (int16_t)lround(32767.0 * sin(2.0 * PI * (i + 0.5) / 1024.0));
  prefs.begin("schumann", false);
  loadSettings();
#if HAS_DAC
  dac_output_enable(DAC_CHAN_0);
  dac_output_voltage(DAC_CHAN_0, 128);
#else
  ledcAttach(PIN_PWM, 78125, 10);
  ledcWrite(PIN_PWM, 512);
#endif
  synthTimer = timerBegin(1000000);                   // 1 MHz timer base (core 3.x API)
  timerAttachInterrupt(synthTimer, &onSynthTimer);
  timerAlarm(synthTimer, (uint64_t)(1000000.0 / FS), true, 0);
  Serial.println("# schumann_gen -- calibrated ELF field generator. Type help.");
  printStat();
}

void loop() {
  static char line[96];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len > 0) { line[len] = 0; len = 0; Serial.println(handleCommandLine(String(line))); }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
  if (wifiOn) web.handleClient();
  serviceTarget();
  serviceBlind();
  servicePhosphene();
  if (!phosRunning && digitalRead(PIN_BUTTON) == LOW && outputOn) {   // panic/stop button
    outputOn = false; targetUT = 0; blindRunning = false; digitalWrite(PIN_LED, LOW);
    Serial.println("# button: output off");
    delay(400);
  }
}
