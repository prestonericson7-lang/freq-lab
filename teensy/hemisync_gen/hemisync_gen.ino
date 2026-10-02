/*
 * hemisync_gen.ino -- Teensy 4.1 binaural-beat generator with a blind test mode
 *
 * WHAT IT DOES
 *   Generates up to six tone pairs at once, a different tone in each ear, the
 *   way the Monroe documents describe Hemi-Sync, with exact frequencies:
 *   the left and right tones come from the same sample clock, so a "4 Hz beat"
 *   is 4.00000 Hz. It can also play the two control conditions a real test
 *   needs, and pick between them at random without telling you which is which.
 *
 * WHERE THE NUMBERS COME FROM
 *   CIA-RDP96-00788R001700210004-8 "The Monroe Institute's Hemisync Process":
 *     carriers below about 1000 Hz, beats below about 30 Hz, up to six beat
 *     frequencies in one mix; Focus 10 = a delta-band beat first, then a
 *     low-beta-band beat mixed in. (Bands only. It gives no frequencies.)
 *   CIA-RDP96-00788R001700210017-4 (1982 trip report): surf sound plus a
 *     4 Hz beat. (No carrier given.)
 *   US patent 5,213,562: 100 Hz left / 104 Hz right for a 4 Hz beat; beat sets
 *     1.5/4/6 Hz, 2/4/7 Hz and 0.5/3/4 Hz; pink noise at least 10 dB below the
 *     tones, panned slowly between the ears. (No carriers given for the sets.)
 *   Where a source gives no number, the preset below says so and uses a
 *   placeholder you can change with the `pair` command.
 *
 * TEST CONDITIONS (command `mode`, or chosen at random by `blind`)
 *   binaural : left ear gets the carrier, right ear gets carrier + beat
 *   monaural : both ears get both tones mixed (an ordinary acoustic beat)
 *   sham     : both ears get the carrier only (no beat at all)
 *
 * WIRING (Teensy 4.1)
 *   Default output is MQS, which needs no audio board:
 *     pin 12 -> 220 ohm -> 10 uF (+ side to the pin) -> headphone LEFT
 *     pin 10 -> 220 ohm -> 10 uF (+ side to the pin) -> headphone RIGHT
 *     GND    -> headphone sleeve
 *   For a cleaner output set USE_I2S to 1 and use any I2S DAC
 *   (BCLK 21, LRCLK 20, DATA 7; MCLK 23 if the DAC wants it).
 *   pin 2  -> square wave at the beat frequency of pair 0 (marker for an EEG
 *             front end, a scope or the FPGA). Timing resolution 2.9 ms.
 *   SD card in the Teensy's slot: session log and the blind key.
 *
 * COMMANDS (USB serial, newline-terminated)
 *   help | stat
 *   vol <0..0.8>                       master volume, starts at 0.15
 *   pair <0-5> <carrier_Hz> <beat_Hz> <level 0..1>     level 0 = off
 *   glide <0-5> <new_beat_Hz> <seconds>                slide a beat frequency
 *   noise <off|pink|surf> [dB_below_tones]             default 10
 *   pan <Hz>                           noise panning rate between ears, 0 = centred
 *   mode <binaural|monaural|sham>
 *   preset <name>                      load a preset (type `preset` to list)
 *   run <minutes>                      play, fade out at the end, log it
 *   blind <preset> <minutes>           same, with the condition picked at random
 *                                      by the hardware random generator and hidden
 *   reveal                             print which condition each blind session got
 *   stop
 *
 * KEEP THE VOLUME COMFORTABLE. This is an audio generator, nothing more.
 *
 * STATUS: compiles for Teensy 4.1 (Teensyduino 1.62). Not yet run on hardware.
 */

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include <Entropy.h>

#define USE_I2S 0

const int   PIN_BEAT_SYNC = 2;
const int   NPAIRS        = 6;
const float VOL_MAX       = 0.8f;
const float PINK_NORM     = 0.41136f;   // scales the pink filter to the RMS of a unit sine

enum { MODE_BINAURAL = 0, MODE_MONAURAL = 1, MODE_SHAM = 2 };
enum { NOISE_OFF = 0, NOISE_PINK = 1, NOISE_SURF = 2 };
const char *MODE_NAME[3] = {"binaural", "monaural", "sham"};

// Types used by functions further down. They sit above the first function so
// the Arduino builder's auto-generated prototypes can see them.
struct Step { float t_s; int8_t pair; float carrier; float beat; float level; };
struct Preset {
  const char *name;
  const char *note;
  const Step *steps;
  int n;
  int noise;
  float noiseDb;
  float panHz;
};

// =================================================================== synth
class BeatSynth : public AudioStream {
 public:
  BeatSynth() : AudioStream(0, NULL) {}
  virtual void update(void);

  struct Pair {
    uint32_t phL = 0, phR = 0;
    uint32_t incL = 0;
    double   incR = 0;          // kept as double so a glide can move it smoothly
    double   incRStep = 0;
    uint32_t glideLeft = 0;
    double   incRTarget = 0;
    float    level = 0, levelTarget = 0;
    float    carrier = 0, beat = 0;
  };
  Pair p[NPAIRS];
  volatile int mode = MODE_BINAURAL;
  float master = 0, masterTarget = 0, masterStep = 0;
  int   noiseType = NOISE_OFF;
  float noiseGain = 0, noiseGainTarget = 0;
  uint32_t panPh = 0, panInc = 0;
  uint32_t swellPh = 0, swell2Ph = 0;
  uint32_t rng = 0x1234567u;
  float b0 = 0, b1 = 0, b2 = 0;
  volatile uint32_t clipCount = 0;
  volatile uint64_t samples = 0;
};

static inline float fastSin(uint32_t ph) {
  return sinf((float)(ph >> 8) * (TWO_PI / 16777216.0f));
}

void BeatSynth::update(void) {
  audio_block_t *bl = allocate();
  audio_block_t *br = allocate();
  if (!bl || !br) {
    if (bl) release(bl);
    if (br) release(br);
    return;
  }
  const int m = mode;
  const uint32_t swellInc  = (uint32_t)(4294967296.0 / (AUDIO_SAMPLE_RATE_EXACT * 9.0));   // 9 s swell
  const uint32_t swell2Inc = (uint32_t)(4294967296.0 / (AUDIO_SAMPLE_RATE_EXACT * 31.0));  // 31 s drift

  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    float L = 0, R = 0;
    for (int k = 0; k < NPAIRS; k++) {
      Pair &q = p[k];
      q.level += (q.levelTarget - q.level) * 0.0005f;          // about 45 ms, click-free
      if (q.glideLeft) {
        q.incR += q.incRStep;
        if (--q.glideLeft == 0) q.incR = q.incRTarget;
      }
      q.phL += q.incL;
      q.phR += (uint32_t)q.incR;
      if (q.level < 1e-5f) continue;
      float sl = fastSin(q.phL);
      if (m == MODE_BINAURAL) {
        L += q.level * sl;
        R += q.level * fastSin(q.phR);
      } else if (m == MODE_MONAURAL) {
        float mix = 0.5f * q.level * (sl + fastSin(q.phR));
        L += mix;
        R += mix;
      } else {
        L += q.level * sl;
        R += q.level * sl;
      }
    }

    noiseGain += (noiseGainTarget - noiseGain) * 0.0002f;
    if (noiseGain > 1e-5f) {
      rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
      float w = (float)(int32_t)rng * (1.0f / 2147483648.0f);
      b0 = 0.99765f * b0 + w * 0.0990460f;                     // Paul Kellett's pink filter
      b1 = 0.96300f * b1 + w * 0.2965164f;
      b2 = 0.57000f * b2 + w * 1.0526913f;
      float pink = (b0 + b1 + b2 + w * 0.1848f) * PINK_NORM * noiseGain;
      if (noiseType == NOISE_SURF) {
        swellPh += swellInc;
        swell2Ph += swell2Inc;
        float s1 = 0.5f * (1.0f + fastSin(swellPh));
        float s2 = 0.5f * (1.0f + fastSin(swell2Ph));
        pink *= 0.25f + 0.75f * s1 * s1 * (0.6f + 0.4f * s2);
      }
      panPh += panInc;
      float pan = panInc ? 0.8f * fastSin(panPh) : 0.0f;
      L += pink * sqrtf(0.5f * (1.0f + pan)) * 1.41421f;
      R += pink * sqrtf(0.5f * (1.0f - pan)) * 1.41421f;
    }

    if (master < masterTarget) { master += masterStep; if (master > masterTarget) master = masterTarget; }
    else if (master > masterTarget) { master -= masterStep; if (master < masterTarget) master = masterTarget; }

    float fl = L * master * 30000.0f;
    float fr = R * master * 30000.0f;
    if (fl > 32767.0f) { fl = 32767.0f; clipCount++; } else if (fl < -32767.0f) { fl = -32767.0f; clipCount++; }
    if (fr > 32767.0f) { fr = 32767.0f; clipCount++; } else if (fr < -32767.0f) { fr = -32767.0f; clipCount++; }
    bl->data[i] = (int16_t)fl;
    br->data[i] = (int16_t)fr;
  }
  samples += AUDIO_BLOCK_SAMPLES;
  // marker: high during the first half of pair 0's beat cycle
  digitalWriteFast(PIN_BEAT_SYNC, ((p[0].phR - p[0].phL) & 0x80000000u) ? LOW : HIGH);
  transmit(bl, 0);
  transmit(br, 1);
  release(bl);
  release(br);
}

BeatSynth synth;
#if USE_I2S
AudioOutputI2S audioOut;
#else
AudioOutputMQS audioOut;
#endif
AudioConnection patchL(synth, 0, audioOut, 0);
AudioConnection patchR(synth, 1, audioOut, 1);

// =================================================================== presets
const Step ST_PATENT4[]  = {{0, 0, 100, 4.0f, 0.5f}};
const Step ST_TRIP1982[] = {{0, 0, 100, 4.0f, 0.4f}};
const Step ST_PATENTA[]  = {{0, 0, 100, 1.5f, 0.3f}, {0, 1, 200, 4.0f, 0.3f}, {0, 2, 300, 6.0f, 0.3f}};
const Step ST_PATENTB[]  = {{0, 0, 100, 2.0f, 0.3f}, {0, 1, 200, 4.0f, 0.3f}, {0, 2, 300, 7.0f, 0.3f}};
const Step ST_PATENTC[]  = {{0, 0, 100, 0.5f, 0.3f}, {0, 1, 200, 3.0f, 0.3f}, {0, 2, 300, 4.0f, 0.3f}};
const Step ST_FOCUS10[]  = {{0, 0, 100, 2.0f, 0.4f}, {300, 1, 200, 14.0f, 0.3f}};

const Preset PRESETS[] = {
  {"patent4",  "US 5,213,562 example: 100 Hz left, 104 Hz right, 4 Hz beat",
   ST_PATENT4, 1, NOISE_OFF, 10, 0},
  {"trip1982", "1982 trip report: surf + 4 Hz beat (carrier not given; 100 Hz used)",
   ST_TRIP1982, 1, NOISE_SURF, 6, 0},
  {"patentA",  "patent beat set 1.5 / 4 / 6 Hz, pink noise 10 dB down, panned (carriers are placeholders)",
   ST_PATENTA, 3, NOISE_PINK, 10, 0.1f},
  {"patentB",  "patent beat set 2 / 4 / 7 Hz, pink noise 10 dB down, panned (carriers are placeholders)",
   ST_PATENTB, 3, NOISE_PINK, 10, 0.1f},
  {"patentC",  "patent beat set 0.5 / 3 / 4 Hz, pink noise 10 dB down, panned (carriers are placeholders)",
   ST_PATENTC, 3, NOISE_PINK, 10, 0.1f},
  {"focus10doc", "Hemisync paper's Focus 10 recipe: delta-band beat, low-beta beat added at 5 min "
                 "(paper names bands only; 2 Hz and 14 Hz are placeholders)",
   ST_FOCUS10, 2, NOISE_PINK, 10, 0.1f},
};
const int NPRESETS = sizeof(PRESETS) / sizeof(PRESETS[0]);

// =================================================================== state
float volume = 0.15f;
float noiseDb = 10.0f;
bool  sdOk = false;

bool  running = false;
bool  blindRun = false;
const Preset *activePreset = NULL;
int   nextStep = 0;
elapsedMillis sessionMs;
uint32_t sessionLenMs = 0;
uint32_t sessionId = 0;
const int MAX_KEYS = 64;
uint8_t  keyMode[MAX_KEYS];
uint32_t keyId[MAX_KEYS];
int      nKeys = 0;

// =================================================================== helpers
uint32_t hzToInc(double f) { return (uint32_t)(f * 4294967296.0 / AUDIO_SAMPLE_RATE_EXACT + 0.5); }

void setPair(int k, float carrier, float beat, float level) {
  if (k < 0 || k >= NPAIRS) return;
  if (carrier < 20) carrier = 20;
  if (carrier > 1500) carrier = 1500;
  if (beat < 0) beat = 0;
  if (beat > 40) beat = 40;
  if (level < 0) level = 0;
  if (level > 1) level = 1;
  uint32_t il = hzToInc(carrier);
  uint32_t ir = hzToInc((double)carrier + (double)beat);
  AudioNoInterrupts();
  BeatSynth::Pair &q = synth.p[k];
  q.incL = il;
  q.incR = (double)ir;
  q.glideLeft = 0;
  q.phL = 0;
  q.phR = 0;                          // beat marker starts in phase
  q.levelTarget = level;
  q.carrier = carrier;
  q.beat = beat;
  AudioInterrupts();
}

void glidePair(int k, float newBeat, float seconds) {
  if (k < 0 || k >= NPAIRS) return;
  if (newBeat < 0) newBeat = 0;
  if (newBeat > 40) newBeat = 40;
  uint32_t n = (uint32_t)(seconds * AUDIO_SAMPLE_RATE_EXACT);
  double target = (double)hzToInc((double)synth.p[k].carrier + (double)newBeat);
  AudioNoInterrupts();
  BeatSynth::Pair &q = synth.p[k];
  if (n == 0) {
    q.incR = target;
    q.glideLeft = 0;
  } else {
    q.incRTarget = target;
    q.incRStep = (target - q.incR) / (double)n;
    q.glideLeft = n;
  }
  q.beat = newBeat;
  AudioInterrupts();
}

float toneRef() {
  float ref = 0;
  for (int k = 0; k < NPAIRS; k++) if (synth.p[k].levelTarget > ref) ref = synth.p[k].levelTarget;
  return ref > 0 ? ref : 0.3f;
}

void applyNoise(int type, float db) {
  noiseDb = db;
  float g = (type == NOISE_OFF) ? 0.0f : toneRef() * powf(10.0f, -db / 20.0f);
  AudioNoInterrupts();
  synth.noiseType = type;
  synth.noiseGainTarget = g;
  AudioInterrupts();
}

void setPan(float hz) {
  if (hz < 0) hz = 0;
  if (hz > 5) hz = 5;
  uint32_t inc = hzToInc(hz);
  AudioNoInterrupts();
  synth.panInc = inc;
  AudioInterrupts();
}

void fadeTo(float target, float seconds) {
  float step = (seconds > 0) ? fabsf(target - synth.master) / (seconds * AUDIO_SAMPLE_RATE_EXACT) : 1.0f;
  if (step < 1e-9f) step = 1e-9f;
  AudioNoInterrupts();
  synth.masterTarget = target;
  synth.masterStep = step;
  AudioInterrupts();
}

void allOff() {
  for (int k = 0; k < NPAIRS; k++) {
    AudioNoInterrupts();
    synth.p[k].levelTarget = 0;
    AudioInterrupts();
  }
}

const Preset *findPreset(const char *name) {
  for (int i = 0; i < NPRESETS; i++) if (!strcmp(name, PRESETS[i].name)) return &PRESETS[i];
  return NULL;
}

// Apply every preset step whose time has come.
void runSteps(uint32_t nowMs) {
  if (!activePreset) return;
  while (nextStep < activePreset->n && (uint32_t)(activePreset->steps[nextStep].t_s * 1000.0f) <= nowMs) {
    const Step &s = activePreset->steps[nextStep++];
    setPair(s.pair, s.carrier, s.beat, s.level);
    Serial.printf("# t=%lus: pair %d on, carrier %.2f Hz, beat %.3f Hz, level %.2f\n",
                  (unsigned long)(nowMs / 1000), s.pair, s.carrier, s.beat, s.level);
    applyNoise(synth.noiseType, noiseDb);      // keep the noise level tied to the tones
  }
}

void loadPreset(const Preset *pr) {
  allOff();
  activePreset = pr;
  nextStep = 0;
  runSteps(0);
  applyNoise(pr->noise, pr->noiseDb);
  setPan(pr->panHz);
}

void logLine(const char *file, const char *text) {
  if (!sdOk) return;
  File f = SD.open(file, FILE_WRITE);
  if (f) {
    f.println(text);
    f.close();
  }
}

void startSession(float minutes, bool blind) {
  if (minutes <= 0) minutes = 10;
  sessionId++;
  blindRun = blind;
  if (blind) {
    int m = (int)Entropy.random(3);            // hardware random generator
    AudioNoInterrupts();
    synth.mode = m;
    AudioInterrupts();
    if (nKeys < MAX_KEYS) { keyId[nKeys] = sessionId; keyMode[nKeys] = (uint8_t)m; nKeys++; }
    char b[48];
    snprintf(b, sizeof(b), "%lu,%s", (unsigned long)sessionId, MODE_NAME[m]);
    logLine("BLINDKEY.CSV", b);
  }
  sessionLenMs = (uint32_t)(minutes * 60000.0f);
  sessionMs = 0;
  if (activePreset) { nextStep = 0; runSteps(0); }
  synth.clipCount = 0;
  running = true;
  fadeTo(volume, 5.0f);
  char b[160];
  snprintf(b, sizeof(b), "%lu,%lu,%s,%.1f,%s,%.2f", (unsigned long)sessionId, (unsigned long)millis(),
           activePreset ? activePreset->name : "manual", minutes,
           blind ? "blind" : MODE_NAME[synth.mode], volume);
  logLine("SESSIONS.CSV", b);
  Serial.printf("# session %lu started: %.1f min, %s%s\n", (unsigned long)sessionId, minutes,
                blind ? "condition hidden" : MODE_NAME[synth.mode], sdOk ? "" : " (no SD card: not logged)");
}

void stopSession(const char *why) {
  if (!running) { fadeTo(0, 2.0f); return; }
  running = false;
  fadeTo(0, 8.0f);
  Serial.printf("# session %lu %s after %lu s, clipped samples: %lu\n", (unsigned long)sessionId, why,
                (unsigned long)(sessionMs / 1000), (unsigned long)synth.clipCount);
}

void printStat() {
  Serial.printf("# volume %.2f, mode %s, %s, output %s\n", volume,
                blindRun && running ? "hidden (blind session)" : MODE_NAME[synth.mode],
                running ? "RUNNING" : "stopped", USE_I2S ? "I2S" : "MQS (pins 12 L, 10 R)");
  for (int k = 0; k < NPAIRS; k++) {
    if (synth.p[k].levelTarget > 0)
      Serial.printf("#  pair %d: left %.3f Hz, right %.3f Hz, beat %.4f Hz, level %.2f\n", k,
                    synth.p[k].carrier, synth.p[k].carrier + synth.p[k].beat, synth.p[k].beat,
                    synth.p[k].levelTarget);
  }
  const char *nz = synth.noiseType == NOISE_OFF ? "off" : synth.noiseType == NOISE_PINK ? "pink" : "surf";
  Serial.printf("# noise %s, %.0f dB below the loudest tone; SD card %s; audio CPU %.1f%%\n", nz, noiseDb,
                sdOk ? "present" : "none", AudioProcessorUsageMax());
  if (running) Serial.printf("# session %lu: %lu of %lu s\n", (unsigned long)sessionId,
                             (unsigned long)(sessionMs / 1000), (unsigned long)(sessionLenMs / 1000));
}

void printHelp() {
  Serial.println(F(
    "# hemisync_gen commands:\n"
    "#  vol <0..0.8> | mode <binaural|monaural|sham> | stat | stop\n"
    "#  pair <0-5> <carrier_Hz> <beat_Hz> <level 0..1>\n"
    "#  glide <0-5> <new_beat_Hz> <seconds>\n"
    "#  noise <off|pink|surf> [dB_below_tones] | pan <Hz>\n"
    "#  preset [name] | run <minutes> | blind <preset> <minutes> | reveal"));
}

void handleLine(char *line) {
  char *tok[6];
  int n = 0;
  for (char *p = strtok(line, " \t\r\n"); p && n < 6; p = strtok(NULL, " \t\r\n")) tok[n++] = p;
  if (n == 0) return;
  const char *c = tok[0];
  if (!strcmp(c, "help") || !strcmp(c, "?")) {
    printHelp();
  } else if (!strcmp(c, "stat")) {
    printStat();
  } else if (!strcmp(c, "vol") && n >= 2) {
    float v = atof(tok[1]);
    if (v < 0) v = 0;
    if (v > VOL_MAX) v = VOL_MAX;
    volume = v;
    if (running) fadeTo(volume, 1.0f);
    Serial.printf("# volume %.2f\n", volume);
  } else if (!strcmp(c, "pair") && n >= 5) {
    setPair(atoi(tok[1]), atof(tok[2]), atof(tok[3]), atof(tok[4]));
    applyNoise(synth.noiseType, noiseDb);
    printStat();
  } else if (!strcmp(c, "glide") && n >= 4) {
    glidePair(atoi(tok[1]), atof(tok[2]), atof(tok[3]));
    Serial.println("# gliding");
  } else if (!strcmp(c, "noise") && n >= 2) {
    int t = !strcmp(tok[1], "pink") ? NOISE_PINK : !strcmp(tok[1], "surf") ? NOISE_SURF : NOISE_OFF;
    applyNoise(t, (n >= 3) ? atof(tok[2]) : noiseDb);
    Serial.println("# noise set");
  } else if (!strcmp(c, "pan") && n >= 2) {
    setPan(atof(tok[1]));
    Serial.println("# pan set");
  } else if (!strcmp(c, "mode") && n >= 2) {
    if (running && blindRun) { Serial.println("# a blind session is running: stop it first"); return; }
    int m = !strcmp(tok[1], "monaural") ? MODE_MONAURAL : !strcmp(tok[1], "sham") ? MODE_SHAM : MODE_BINAURAL;
    AudioNoInterrupts();
    synth.mode = m;
    AudioInterrupts();
    Serial.printf("# mode %s\n", MODE_NAME[m]);
  } else if (!strcmp(c, "preset")) {
    if (n < 2) {
      for (int i = 0; i < NPRESETS; i++) Serial.printf("#  %-11s %s\n", PRESETS[i].name, PRESETS[i].note);
      return;
    }
    const Preset *pr = findPreset(tok[1]);
    if (!pr) { Serial.println("# no such preset"); return; }
    loadPreset(pr);
    Serial.printf("# preset %s loaded: %s\n", pr->name, pr->note);
  } else if (!strcmp(c, "run")) {
    startSession((n >= 2) ? atof(tok[1]) : 10, false);
  } else if (!strcmp(c, "blind") && n >= 3) {
    const Preset *pr = findPreset(tok[1]);
    if (!pr) { Serial.println("# no such preset"); return; }
    loadPreset(pr);
    startSession(atof(tok[2]), true);
  } else if (!strcmp(c, "reveal")) {
    if (running && blindRun) { Serial.println("# a blind session is running: stop it first"); return; }
    Serial.println("session,condition");
    for (int i = 0; i < nKeys; i++) Serial.printf("%lu,%s\n", (unsigned long)keyId[i], MODE_NAME[keyMode[i]]);
    if (sdOk) Serial.println("# sessions from before the last power-up are in BLINDKEY.CSV on the SD card");
  } else if (!strcmp(c, "stop")) {
    stopSession("stopped");
  } else {
    Serial.println("# unknown command, type help");
  }
}

// =================================================================== setup/loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BEAT_SYNC, OUTPUT);
  AudioMemory(12);
  Entropy.Initialize();
  synth.rng = Entropy.random() | 1u;
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  sdOk = SD.begin(BUILTIN_SDCARD);
  Serial.println("# hemisync_gen -- binaural-beat generator with blind test mode");
  printHelp();
  loadPreset(&PRESETS[0]);
  Serial.printf("# preset %s loaded, volume %.2f. Type `run 10` to play for ten minutes.\n",
                PRESETS[0].name, volume);
}

void loop() {
  static char line[96];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len > 0) {
        line[len] = 0;
        len = 0;
        handleLine(line);
      }
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
  if (running) {
    runSteps(sessionMs);
    if (sessionMs >= sessionLenMs) stopSession("finished");
  }
}
