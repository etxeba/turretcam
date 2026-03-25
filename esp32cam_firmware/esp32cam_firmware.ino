// esp32cam_firmware.ino
// HackPack Turret Tracker — ESP32-CAM firmware
//
// Motion detection via frame differencing. Sends proportional X/Y tracking
// errors to the Arduino Nano over UART (GPIO14 TX only — one-way link).
// The Nano owns all servo state and translates errors into servo commands.
//
// Protocol: X<-100..100>Y<-100..100>\n   (0,0 = no target / stop)
//
// HTTP endpoints:
//   http://<IP>:81/stream  — MJPEG live stream
//   http://<IP>/capture    — single JPEG snapshot
//   http://<IP>/status     — JSON tracking stats
//   http://<IP>/config     — GET / SET tracking parameters

#include "esp_camera.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

Preferences prefs;

// =========================================================
// WiFi credentials — update before flashing
// =========================================================
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// =========================================================
// AI Thinker ESP32-CAM pin map
// =========================================================
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// UART to Nano (TX only — one-way command link)
#define UART_TX_PIN  14   // ESP32-CAM GPIO14 → Nano RX
#define UART_BAUD    9600

// ── Adaptive settling ─────────────────────────────────────────────────────────
// After sending a move command the whole frame shifts. We discard frames until
// the global pixel-diff drops back near the learned quiet baseline.
#define SETTLE_LOW_THRESH   8      // pixel diff counted toward global motion score
#define SETTLE_MULTIPLIER   2.5f   // "settled" when global diff < baseline * this
#define SETTLE_MAX_FRAMES   10     // hard cap so we never stall indefinitely
#define SETTLE_EMA_ALPHA    0.20f  // how fast the learned frame count adapts
#define BASELINE_EMA_ALPHA  0.03f  // how fast the quiet baseline updates

// =========================================================
// Tunable tracking config (adjustable at runtime via /config)
// =========================================================
struct Config {
  int  threshold  = 25;    // pixel-diff threshold for motion detection
  int  min_pixels = 50;    // minimum motion pixels required to track
  bool enabled    = true;  // master on/off
  bool hmirror    = false; // flip camera horizontally
  bool vflip      = false; // flip camera vertically
  // Camera-to-barrel alignment offset in pixels.
  // Shifts the "aim point" away from frame centre to compensate for a
  // camera that isn't perfectly aligned with the barrel.
  // Positive offset_x → aim point moves right; positive offset_y → moves down.
  // Calibrate by pointing at a stationary target and tweaking via /config
  // until the turret centres on it without hunting.
  int  offset_x   = 0;
  int  offset_y   = 0;
};

Config cfg;

// =========================================================
// State
// =========================================================
uint8_t* prev_frame   = nullptr;
int      frame_width  = 0;
int      frame_height = 0;
size_t   frame_bytes  = 0;

// Stats (written by tracking task, read by HTTP handlers)
volatile unsigned long stat_frames        = 0;
volatile int           stat_motion_pixels = 0;
volatile float         stat_cx = -1.0f;   // motion centroid, pixels
volatile float         stat_cy = -1.0f;
volatile int           stat_ex =  0;      // last sent error X (-100..100)
volatile int           stat_ey =  0;      // last sent error Y (-100..100)
volatile unsigned long stat_last_cmd_ms   = 0;

// Settling state (written by tracking task, read by HTTP handler)
volatile float stat_settle_ema      = 3.0f;  // learned settle frame count
volatile float stat_baseline_diff   = -1.0f; // learned quiet global diff (-1 = init)

SemaphoreHandle_t camMutex;
SemaphoreHandle_t statMutex;

WebServer server(80);
WebServer streamServer(81);

// =========================================================
// Camera initialisation
// =========================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QVGA;  // 320 × 240
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_hmirror(s, cfg.hmirror ? 1 : 0);
  s->set_vflip(s,   cfg.vflip   ? 1 : 0);
  return true;
}

// =========================================================
// Motion detection
// Single pass: counts high-threshold pixels (target motion centroid) and
// low-threshold pixels (global frame motion used by the settling detector).
// =========================================================
bool detectMotion(const uint8_t* curr, const uint8_t* prev,
                  int w, int h,
                  float& cx, float& cy,
                  int& motion_pixels, int& global_diff_pixels) {
  long sum_x = 0, sum_y = 0;
  motion_pixels      = 0;
  global_diff_pixels = 0;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int idx  = y * w + x;
      int diff = abs((int)curr[idx] - (int)prev[idx]);
      if (diff > SETTLE_LOW_THRESH) global_diff_pixels++;
      if (diff > cfg.threshold) {
        sum_x += x;
        sum_y += y;
        motion_pixels++;
      }
    }
  }

  if (motion_pixels < cfg.min_pixels) return false;

  cx = (float)sum_x / motion_pixels;
  cy = (float)sum_y / motion_pixels;
  return true;
}

// =========================================================
// Send tracking error to Nano
// ex, ey: -100..100 (% of half-frame; positive = right / down)
// Sending 0,0 signals no target — Nano's watchdog stops the yaw.
// =========================================================
void sendTrackingError(int ex, int ey) {
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "X%dY%d\n", ex, ey);
  Serial2.print(cmd);
  Serial.printf("→ Nano: %s", cmd);

  xSemaphoreTake(statMutex, portMAX_DELAY);
  stat_ex          = ex;
  stat_ey          = ey;
  stat_last_cmd_ms = millis();
  xSemaphoreGive(statMutex);
}

// =========================================================
// Tracking task — runs on core 0, frees core 1 for web servers
// =========================================================
void trackingTask(void* pvParameters) {
  // Settling state — local to this task, no mutex needed
  float settleEMA    = 3.0f;  // learned frame count (EMA)
  float baselineDiff = -1.0f; // quiet-frame global diff EMA; -1 = not yet init
  bool  settling     = false;
  int   settleCounter = 0;

  for (;;) {
    if (!cfg.enabled) {
      sendTrackingError(0, 0);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(200)) != pdTRUE) continue;
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    if (fb->len != frame_bytes || !prev_frame) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    float cx = 0.0f, cy = 0.0f;
    int   motion_pixels = 0, global_diff_pixels = 0;

    bool motion = detectMotion(fb->buf, prev_frame,
                               frame_width, frame_height,
                               cx, cy, motion_pixels, global_diff_pixels);

    memcpy(prev_frame, fb->buf, frame_bytes);

    xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    // Seed baseline from the very first frame
    if (baselineDiff < 0.0f) baselineDiff = (float)global_diff_pixels;

    float settleThreshold = baselineDiff * SETTLE_MULTIPLIER;

    if (settling) {
      settleCounter++;

      bool frameSettled = (float)global_diff_pixels <= settleThreshold;
      bool timedOut     = settleCounter >= SETTLE_MAX_FRAMES;

      if (frameSettled || timedOut) {
        // Update learned frame count with what we actually observed
        settleEMA = settleEMA * (1.0f - SETTLE_EMA_ALPHA)
                  + settleCounter  * SETTLE_EMA_ALPHA;
        settling = false;
        Serial.printf("Settled in %d frames (learned avg: %.1f)%s\n",
                      settleCounter, settleEMA,
                      timedOut && !frameSettled ? " [timeout]" : "");
      }

      // Discard this frame — send stop so the Nano watchdog doesn't spin
      sendTrackingError(0, 0);

    } else {
      // Not settling: update the quiet baseline (slow EMA so occasional
      // target motion doesn't skew it meaningfully)
      baselineDiff = baselineDiff * (1.0f - BASELINE_EMA_ALPHA)
                   + global_diff_pixels * BASELINE_EMA_ALPHA;

      int ex = 0, ey = 0;
      if (motion) {
        float err_x = (cx - (frame_width  * 0.5f + cfg.offset_x)) / (frame_width  * 0.5f);
        float err_y = (cy - (frame_height * 0.5f + cfg.offset_y)) / (frame_height * 0.5f);
        ex = constrain((int)(err_x * 100.0f), -100, 100);
        ey = constrain((int)(err_y * 100.0f), -100, 100);
      }
      sendTrackingError(ex, ey);

      // Begin settling after any non-zero command
      if (ex != 0 || ey != 0) {
        settling      = true;
        settleCounter = 0;
      }
    }

    xSemaphoreTake(statMutex, portMAX_DELAY);
    stat_frames++;
    stat_motion_pixels  = motion_pixels;
    stat_settle_ema     = settleEMA;
    stat_baseline_diff  = baselineDiff;
    if (motion) { stat_cx = cx; stat_cy = cy; }
    xSemaphoreGive(statMutex);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =========================================================
// HTTP: MJPEG stream  (port 81, /stream)
// =========================================================
void handleStream() {
  WiFiClient client = streamServer.client();

  streamServer.sendHeader("Access-Control-Allow-Origin", "*");
  streamServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  streamServer.sendHeader("Pragma", "no-cache");
  streamServer.sendHeader("Expires", "-1");
  streamServer.send(200, "multipart/x-mixed-replace; boundary=frame");

  while (client.connected()) {
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(500)) != pdTRUE) continue;
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) { delay(10); continue; }

    uint8_t* jpg_buf = nullptr;
    size_t   jpg_len = 0;
    bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);

    xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    if (ok && jpg_buf) {
      client.print("--frame\r\n");
      client.print("Content-Type: image/jpeg\r\n");
      client.printf("Content-Length: %u\r\n\r\n", jpg_len);
      client.write(jpg_buf, jpg_len);
      client.print("\r\n");
      free(jpg_buf);
    }

    delay(50);
  }
}

// HTTP: single JPEG snapshot
void handleCapture() {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    server.send(503, "text/plain", "Camera busy");
    return;
  }
  camera_fb_t* fb = esp_camera_fb_get();
  xSemaphoreGive(camMutex);

  if (!fb) { server.send(500, "text/plain", "Camera error"); return; }

  uint8_t* jpg_buf = nullptr;
  size_t   jpg_len = 0;
  bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);

  xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  if (!ok || !jpg_buf) { server.send(500, "text/plain", "JPEG encode error"); return; }

  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char*)jpg_buf, jpg_len);
  free(jpg_buf);
}

// HTTP: JSON status
void handleStatus() {
  xSemaphoreTake(statMutex, portMAX_DELAY);
  unsigned long frames        = stat_frames;
  int           motion_pixels = stat_motion_pixels;
  float         cx            = stat_cx;
  float         cy            = stat_cy;
  int           ex            = stat_ex;
  int           ey            = stat_ey;
  unsigned long cmd_ms        = stat_last_cmd_ms;
  float         settle_ema    = stat_settle_ema;
  float         baseline_diff = stat_baseline_diff;
  xSemaphoreGive(statMutex);

  char buf[640];
  snprintf(buf, sizeof(buf),
    "{"
      "\"frames\":%lu,"
      "\"motion_pixels\":%d,"
      "\"target_x\":%.1f,"
      "\"target_y\":%.1f,"
      "\"error_x\":%d,"
      "\"error_y\":%d,"
      "\"tracking\":%s,"
      "\"ms_since_cmd\":%lu,"
      "\"settle_frames_learned\":%.1f,"
      "\"baseline_diff\":%.0f,"
      "\"ip\":\"%s\","
      "\"rssi\":%d,"
      "\"heap\":%lu"
    "}",
    frames, motion_pixels, cx, cy, ex, ey,
    cfg.enabled ? "true" : "false",
    millis() - cmd_ms,
    settle_ema, baseline_diff,
    WiFi.localIP().toString().c_str(),
    WiFi.RSSI(),
    (unsigned long)ESP.getFreeHeap()
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// HTTP: GET/SET config
void handleConfig() {
  if (server.hasArg("threshold"))  cfg.threshold  = server.arg("threshold").toInt();
  if (server.hasArg("min_pixels")) cfg.min_pixels = server.arg("min_pixels").toInt();
  if (server.hasArg("enabled"))    cfg.enabled    = server.arg("enabled").toInt() != 0;
  if (server.hasArg("offset_x"))   cfg.offset_x   = server.arg("offset_x").toInt();
  if (server.hasArg("offset_y"))   cfg.offset_y   = server.arg("offset_y").toInt();

  if (server.hasArg("hmirror")) {
    cfg.hmirror = server.arg("hmirror").toInt() != 0;
    sensor_t* s = esp_camera_sensor_get();
    s->set_hmirror(s, cfg.hmirror ? 1 : 0);
  }
  if (server.hasArg("vflip")) {
    cfg.vflip = server.arg("vflip").toInt() != 0;
    sensor_t* s = esp_camera_sensor_get();
    s->set_vflip(s, cfg.vflip ? 1 : 0);
  }

  bool saved = false;
  if (server.hasArg("save") && server.arg("save").toInt() == 1) {
    saveConfig();
    saved = true;
  }

  char buf[280];
  snprintf(buf, sizeof(buf),
    "{"
      "\"threshold\":%d,"
      "\"min_pixels\":%d,"
      "\"enabled\":%s,"
      "\"hmirror\":%s,"
      "\"vflip\":%s,"
      "\"offset_x\":%d,"
      "\"offset_y\":%d,"
      "\"saved\":%s"
    "}",
    cfg.threshold, cfg.min_pixels,
    cfg.enabled ? "true" : "false",
    cfg.hmirror  ? "true" : "false",
    cfg.vflip    ? "true" : "false",
    cfg.offset_x, cfg.offset_y,
    saved ? "true" : "false"
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// =========================================================
// NVS persistence
// =========================================================
void saveConfig() {
  prefs.begin("turret", false);
  prefs.putInt("threshold",  cfg.threshold);
  prefs.putInt("min_pixels", cfg.min_pixels);
  prefs.putBool("enabled",   cfg.enabled);
  prefs.putBool("hmirror",   cfg.hmirror);
  prefs.putBool("vflip",     cfg.vflip);
  prefs.putInt("offset_x",   cfg.offset_x);
  prefs.putInt("offset_y",   cfg.offset_y);
  prefs.end();
  Serial.println("Config saved to NVS");
}

void loadConfig() {
  prefs.begin("turret", true);  // read-only
  cfg.threshold  = prefs.getInt("threshold",  cfg.threshold);
  cfg.min_pixels = prefs.getInt("min_pixels", cfg.min_pixels);
  cfg.enabled    = prefs.getBool("enabled",   cfg.enabled);
  cfg.hmirror    = prefs.getBool("hmirror",   cfg.hmirror);
  cfg.vflip      = prefs.getBool("vflip",     cfg.vflip);
  cfg.offset_x   = prefs.getInt("offset_x",  cfg.offset_x);
  cfg.offset_y   = prefs.getInt("offset_y",  cfg.offset_y);
  prefs.end();
  Serial.printf("Config loaded from NVS (offset %d, %d)\n", cfg.offset_x, cfg.offset_y);
}

// =========================================================
// Setup
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== HackPack Turret Tracker — ESP32-CAM ===");

  loadConfig();

  // UART2 → Nano (TX only; RX unused)
  Serial2.begin(UART_BAUD, SERIAL_8N1, -1, UART_TX_PIN);
  Serial.println("UART2 to Nano ready");

  if (!initCamera()) {
    Serial.println("FATAL: camera failed — rebooting in 5 s");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Camera OK");

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb) {
    frame_width  = fb->width;
    frame_height = fb->height;
    frame_bytes  = fb->len;
    prev_frame   = (uint8_t*)malloc(frame_bytes);
    if (prev_frame) memcpy(prev_frame, fb->buf, frame_bytes);
    esp_camera_fb_return(fb);
    Serial.printf("Frame: %d x %d (%d bytes)\n", frame_width, frame_height, frame_bytes);
  } else {
    Serial.println("FATAL: cannot read first frame — rebooting");
    delay(5000);
    ESP.restart();
  }

  camMutex  = xSemaphoreCreateMutex();
  statMutex = xSemaphoreCreateMutex();

  Serial.printf("Connecting to %s", ssid);
  WiFi.begin(ssid, password);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Stream:  http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Capture: http://%s/capture\n",   WiFi.localIP().toString().c_str());
    Serial.printf("  Status:  http://%s/status\n",    WiFi.localIP().toString().c_str());
    Serial.printf("  Config:  http://%s/config\n",    WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi FAILED — offline mode (tracking still works)");
  }

  ArduinoOTA.setHostname("turret-cam");
  ArduinoOTA.onStart([]()            { Serial.println("OTA: start");    });
  ArduinoOTA.onEnd([]()              { Serial.println("OTA: end");      });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\n", p * 100 / t);
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();
  Serial.println("OTA ready — hostname: turret-cam");

  server.on("/status",  handleStatus);
  server.on("/config",  handleConfig);
  server.on("/capture", handleCapture);
  server.begin();
  Serial.println("HTTP server started on port 80");

  streamServer.on("/stream", handleStream);
  streamServer.begin();
  Serial.println("Stream server started on port 81");

  xTaskCreatePinnedToCore(
    trackingTask,
    "tracking",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );

  Serial.println("Tracking task started — turret is live!\n");
}

// =========================================================
// Loop — core 1: OTA + HTTP servers
// =========================================================
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  streamServer.handleClient();
  delay(1);
}
