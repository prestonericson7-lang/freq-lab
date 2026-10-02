// torsion_cam.ino -- ESP32-S3 CAM angle tracker for the 2-liter-bottle torsion pendulum
//
// The camera looks down through the lid at the backlit rotor and two reference
// squares on the paper disc. Every frame it measures the rotor's angle (to about
// 0.2 mrad per frame) and prints one line over USB serial. tools/torsion_host.py
// on the PC does the rest (live view, calibration, sessions, analysis).
//
// Board : ESP32-S3 CAM, 16 MB flash / 8 MB OPI PSRAM (Meshnology GC2145 kit and the
//         common dual-USB-C clones). Camera pins are found automatically: the
//         ESP32S3_EYE layout used by most of these boards is tried first, then the
//         XIAO-style layout some GC2145 boards use.
// Arduino IDE: Board "ESP32S3 Dev Module", PSRAM "OPI PSRAM", Flash Size "16MB",
//         USB CDC On Boot "Disabled" (plug into the USB-C port marked COM/UART;
//         with "Enabled", use the other port). Serial Monitor 921600 baud.
//
// Output, one line per frame:
//   A,<ms>,<frame>,<angle_urad>,<rotor_urad>,<ref_urad>,<flags>,<cx*10>,<cy*10>,<area>,<thr>,<acq>
//     angle = rotor - reference (camera movement cancelled) when flags bit1 is set
//     flags bit0 rotor found, bit1 reference squares found
// Commands (type a line):
//   snap      160x120 preview + the regions being measured (host saves a PNG)
//   snapfull  full-resolution preview (focus check)
//   acq       find rotor and squares again
//   info      sensor, pin layout, frame size, frame rate
//   res vga | res qvga   change resolution (VGA default)
#include "esp_camera.h"
#include "tracker.h"

// ------------------------------------------------------------------ camera pin layouts
struct CamPins {
  const char* name;
  int pwdn, reset, xclk, sda, scl, y9, y8, y7, y6, y5, y4, y3, y2, vsync, href, pclk;
};
static const CamPins PIN_LAYOUTS[] = {
  // ESP32S3_EYE layout: Freenove / Keyestudio / most "ESP32-S3 CAM N16R8" boards
  {"ESP32S3_EYE", -1, -1, 15, 4, 5, 16, 17, 18, 12, 10, 8, 9, 11, 6, 7, 13},
  // XIAO ESP32S3 Sense layout, used by some GC2145 boards
  {"XIAO_S3", -1, -1, 10, 40, 39, 48, 11, 12, 14, 16, 18, 17, 15, 38, 47, 13},
};
#define FORCE_LAYOUT -1          // 0 or 1 to skip the automatic search

static int layout = -1;
static framesize_t fsize = FRAMESIZE_VGA;
static int yoff = -1;
static Tracker tracker;
static uint32_t frameNo = 0;
static uint32_t lastFrameMs = 0;
static float fps = 0;

static bool cameraStart(int which, framesize_t fs) {
  const CamPins& p = PIN_LAYOUTS[which];
  camera_config_t c = {};
  c.pin_pwdn = p.pwdn;
  c.pin_reset = p.reset;
  c.pin_xclk = p.xclk;
  c.pin_sccb_sda = p.sda;
  c.pin_sccb_scl = p.scl;
  c.pin_d7 = p.y9; c.pin_d6 = p.y8; c.pin_d5 = p.y7; c.pin_d4 = p.y6;
  c.pin_d3 = p.y5; c.pin_d2 = p.y4; c.pin_d1 = p.y3; c.pin_d0 = p.y2;
  c.pin_vsync = p.vsync;
  c.pin_href = p.href;
  c.pin_pclk = p.pclk;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_YUV422;        // GC2145 has no JPEG/GRAYSCALE; Y is every other byte
  c.frame_size = fs;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) return false;
  // make sure frames really arrive at this size
  for (int i = 0; i < 4; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      bool ok = fb->len >= fb->width * fb->height * 2;
      esp_camera_fb_return(fb);
      if (ok) return true;
    }
  }
  esp_camera_deinit();
  return false;
}

static bool cameraBegin() {
  int first = FORCE_LAYOUT >= 0 ? FORCE_LAYOUT : 0;
  int last = FORCE_LAYOUT >= 0 ? FORCE_LAYOUT : 1;
  for (int k = first; k <= last; k++) {
    for (int attempt = 0; attempt < 2; attempt++) {
      framesize_t fs = attempt == 0 ? fsize : FRAMESIZE_QVGA;
      if (cameraStart(k, fs)) {
        layout = k;
        fsize = fs;
        return true;
      }
    }
  }
  return false;
}

// ------------------------------------------------------------------ base64 for previews
static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static void sendBase64(const uint8_t* d, size_t n) {
  char line[80];
  int pos = 0;
  for (size_t i = 0; i < n; i += 3) {
    uint32_t v = (uint32_t)d[i] << 16;
    if (i + 1 < n) v |= (uint32_t)d[i + 1] << 8;
    if (i + 2 < n) v |= d[i + 2];
    line[pos++] = B64[(v >> 18) & 63];
    line[pos++] = B64[(v >> 12) & 63];
    line[pos++] = i + 1 < n ? B64[(v >> 6) & 63] : '=';
    line[pos++] = i + 2 < n ? B64[v & 63] : '=';
    if (pos >= 76) { line[pos] = 0; Serial.println(line); pos = 0; }
  }
  if (pos) { line[pos] = 0; Serial.println(line); }
}

static void sendSnap(camera_fb_t* fb, bool full) {
  int w = fb->width, h = fb->height;
  int f = full ? 1 : (w / 160 > 0 ? w / 160 : 1);
  int ow = w / f, oh = h / f;
  uint8_t* img = (uint8_t*)malloc((size_t)ow * oh);
  if (!img) { Serial.println("E,no memory for snap"); return; }
  for (int j = 0; j < oh; j++)
    for (int i = 0; i < ow; i++) {
      int s = 0;
      for (int yy = 0; yy < f; yy++)
        for (int xx = 0; xx < f; xx++) s += fb->buf[((j * f + yy) * w + i * f + xx) * 2 + yoff];
      img[j * ow + i] = s / (f * f);
    }
  Serial.printf("SNAP %d %d %d %.1f %.1f %.1f %.1f %ld %d %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",
                ow, oh, f, tracker.rx, tracker.ry, tracker.half_len, tracker.half_w,
                (long)(tracker.axis_angle * 1e6), tracker.nref, tracker.qx[0], tracker.qy[0],
                tracker.qx[1], tracker.qy[1], tracker.qh, tracker.px, tracker.py,
                tracker.pr > 1e8 ? -1.0 : tracker.pr);
  sendBase64(img, (size_t)ow * oh);
  Serial.println("SNAPEND");
  free(img);
}

static void printInfo() {
  sensor_t* s = esp_camera_sensor_get();
  camera_fb_t* fb = esp_camera_fb_get();
  Serial.printf("I,sensor_pid=0x%04x,layout=%s,frame=%dx%d,yoff=%d,fps=%.1f,psram=%u,acq=%d,refs=%d\n",
                s ? s->id.PID : 0, layout >= 0 ? PIN_LAYOUTS[layout].name : "none",
                fb ? fb->width : 0, fb ? fb->height : 0, yoff, fps, (unsigned)ESP.getPsramSize(),
                tracker.acquisitions, tracker.nref);
  if (fb) esp_camera_fb_return(fb);
}

static String cmd;
static int pendingSnap = 0;      // 1 = small, 2 = full

static void handleSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') { if (cmd.length() < 40) cmd += ch; continue; }
    cmd.trim();
    if (cmd == "snap") pendingSnap = 1;
    else if (cmd == "snapfull") pendingSnap = 2;
    else if (cmd == "acq") { tracker.rotor_area0 = 0; Serial.println("K,acquire"); }
    else if (cmd == "info") printInfo();
    else if (cmd == "res vga" || cmd == "res qvga") {
      esp_camera_deinit();
      fsize = cmd == "res vga" ? FRAMESIZE_VGA : FRAMESIZE_QVGA;
      if (cameraStart(layout, fsize)) { tracker = Tracker(); yoff = -1; Serial.println("K,resolution"); }
      else Serial.println("E,resolution change failed");
    } else if (cmd.length()) Serial.println("E,unknown command");
    cmd = "";
  }
}

void setup() {
  Serial.setRxBufferSize(512);
  Serial.begin(921600);
  delay(300);
  Serial.println("I,torsion_cam v1 starting");
  if (!psramFound()) Serial.println("E,no PSRAM: set Tools > PSRAM > OPI PSRAM");
  if (!cameraBegin()) {
    Serial.println("E,camera not found on either pin layout (check the ribbon cable, PSRAM = OPI)");
    return;
  }
  printInfo();
}

void loop() {
  handleSerial();
  if (layout < 0) { delay(1000); Serial.println("E,no camera"); return; }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    if (millis() - lastFrameMs > 2000) { Serial.println("E,camera timeout"); lastFrameMs = millis(); }
    return;
  }
  uint32_t now = millis();
  if (lastFrameMs) fps = 0.9f * fps + 0.1f * (1000.0f / (float)(now - lastFrameMs + 1));
  lastFrameMs = now;
  if (yoff < 0) yoff = detectYOffset(fb->buf, fb->width, fb->height);
  Lum lum = {fb->buf, (int)fb->width, (int)fb->height, 2, yoff};
  TrackOut o = tracker.update(lum);
  frameNo++;
  Serial.printf("A,%lu,%lu,%ld,%ld,%ld,%d,%ld,%ld,%ld,%d,%d\n", (unsigned long)now, (unsigned long)frameNo,
                (long)lround(o.angle * 1e6), (long)lround(o.rotor_angle * 1e6), (long)lround(o.ref_angle * 1e6),
                (o.rotor_ok ? 1 : 0) | (o.ref_ok ? 2 : 0), (long)lround(o.cx * 10), (long)lround(o.cy * 10),
                (long)lround(o.area), o.thr, tracker.acquisitions);
  if (pendingSnap) { sendSnap(fb, pendingSnap == 2); pendingSnap = 0; }
  esp_camera_fb_return(fb);
}
