// esp32cam_firmware.ino
// HackPack Turret Tracker — ESP32-CAM firmware
//
// Motion detection via frame differencing. Sends servo commands to the
// Arduino Nano over UART (GPIO14 TX, GPIO15 RX).
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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

// UART to Nano
#define UART_TX_PIN  14   // ESP32-CAM GPIO14 → Nano D2 (RX)
#define UART_RX_PIN  15   // ESP32-CAM GPIO15 ← Nano D3 (TX)
#define UART_BAUD    9600

// =========================================================
// Tunable tracking config (all adjustable at runtime via /config)
// =========================================================
struct Config {
  int   threshold       = 25;    // pixel-diff threshold for motion detection
  int   min_pixels      = 50;    // minimum motion pixels required to track
  float pan_kp          = 0.08f; // proportional gain — pan axis
  float tilt_kp         = 0.08f; // proportional gain — tilt axis
  int   pan_center      = 90;    // rest angle — pan  (0-180°)
  int   tilt_center     = 90;    // rest angle — tilt (0-180°)
  int   smoothing_frames = 3;    // exponential smoothing window (future use)
  bool  enabled         = true;  // master on/off
  bool  hmirror         = false; // flip camera horizontally
  bool  vflip           = false; // flip camera vertically
};

Config cfg;

// =========================================================
// State
// =========================================================
volatile float current_pan  = 90.0f;
volatile float current_tilt = 90.0f;

uint8_t* prev_frame  = nullptr;
int      frame_width  = 0;
int      frame_height = 0;
size_t   frame_bytes  = 0;

// Stats (written by tracking task, read by HTTP handlers)
volatile unsigned long stat_frames        = 0;
volatile int           stat_motion_pixels = 0;
volatile float         stat_cx = -1.0f;
volatile float         stat_cy = -1.0f;
volatile unsigned long stat_last_cmd_ms   = 0;

SemaphoreHandle_t camMutex;   // guards esp_camera_fb_get/return calls
SemaphoreHandle_t statMutex;  // guards stat_* and current_pan/tilt writes

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
  config.fb_count     = 2;               // double-buffer for speed

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
// Computes centroid of pixels that differ from the previous frame.
// Returns true when enough motion is found.
// =========================================================
bool detectMotion(const uint8_t* curr, const uint8_t* prev,
                  int w, int h,
                  float& cx, float& cy, int& motion_pixels) {
  long sum_x = 0, sum_y = 0;
  motion_pixels = 0;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int idx  = y * w + x;
      int diff = abs((int)curr[idx] - (int)prev[idx]);
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
// Send servo command to Nano
// Format: P<pan>T<tilt>\n   e.g. "P90T90\n"
// =========================================================
void sendServoCommand(float pan, float tilt) {
  pan  = constrain(pan,  0.0f, 180.0f);
  tilt = constrain(tilt, 0.0f, 180.0f);

  char cmd[24];
  snprintf(cmd, sizeof(cmd), "P%dT%d\n", (int)pan, (int)tilt);
  Serial2.print(cmd);

  Serial.printf("→ Nano: %s", cmd);

  xSemaphoreTake(statMutex, portMAX_DELAY);
  stat_last_cmd_ms = millis();
  xSemaphoreGive(statMutex);
}

// =========================================================
// Tracking task — runs on core 0, frees core 1 for web servers
// =========================================================
void trackingTask(void* pvParameters) {
  for (;;) {
    if (!cfg.enabled) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Grab frame
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
    int   motion_pixels = 0;

    bool motion = detectMotion(fb->buf, prev_frame,
                               frame_width, frame_height,
                               cx, cy, motion_pixels);

    memcpy(prev_frame, fb->buf, frame_bytes);

    xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    xSemaphoreTake(statMutex, portMAX_DELAY);
    stat_frames++;
    stat_motion_pixels = motion_pixels;
    if (motion) {
      stat_cx = cx;
      stat_cy = cy;
    }
    xSemaphoreGive(statMutex);

    if (motion) {
      // Normalised error: -1 (left/up) to +1 (right/down)
      float err_x = (cx - frame_width  * 0.5f) / (frame_width  * 0.5f);
      float err_y = (cy - frame_height * 0.5f) / (frame_height * 0.5f);

      xSemaphoreTake(statMutex, portMAX_DELAY);
      // Pan: target right → error positive → rotate right (increase pan)
      current_pan  += err_x * cfg.pan_kp  * 90.0f;
      // Tilt: target low  → error positive → tilt down (decrease tilt)
      current_tilt -= err_y * cfg.tilt_kp * 90.0f;
      current_pan  = constrain(current_pan,  0.0f, 180.0f);
      current_tilt = constrain(current_tilt, 0.0f, 180.0f);
      float pan  = current_pan;
      float tilt = current_tilt;
      xSemaphoreGive(statMutex);

      sendServoCommand(pan, tilt);
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // yield; tracking runs ~30 fps
  }
}

// =========================================================
// HTTP: MJPEG stream  (port 81, /stream)
// Streams grayscale frames converted to JPEG until client disconnects.
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

    delay(50);  // ~20 fps stream cap
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
  float         pan           = current_pan;
  float         tilt          = current_tilt;
  unsigned long cmd_ms        = stat_last_cmd_ms;
  xSemaphoreGive(statMutex);

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{"
      "\"frames\":%lu,"
      "\"motion_pixels\":%d,"
      "\"target_x\":%.1f,"
      "\"target_y\":%.1f,"
      "\"pan\":%.1f,"
      "\"tilt\":%.1f,"
      "\"tracking\":%s,"
      "\"ms_since_cmd\":%lu,"
      "\"ip\":\"%s\","
      "\"rssi\":%d,"
      "\"heap\":%lu"
    "}",
    frames, motion_pixels, cx, cy, pan, tilt,
    cfg.enabled ? "true" : "false",
    millis() - cmd_ms,
    WiFi.localIP().toString().c_str(),
    WiFi.RSSI(),
    (unsigned long)ESP.getFreeHeap()
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// HTTP: GET/SET config
void handleConfig() {
  if (server.hasArg("threshold"))        cfg.threshold        = server.arg("threshold").toInt();
  if (server.hasArg("min_pixels"))       cfg.min_pixels       = server.arg("min_pixels").toInt();
  if (server.hasArg("pan_kp"))           cfg.pan_kp           = server.arg("pan_kp").toFloat();
  if (server.hasArg("tilt_kp"))          cfg.tilt_kp          = server.arg("tilt_kp").toFloat();
  if (server.hasArg("smoothing_frames")) cfg.smoothing_frames = server.arg("smoothing_frames").toInt();
  if (server.hasArg("enabled"))          cfg.enabled          = server.arg("enabled").toInt() != 0;

  if (server.hasArg("pan_center")) {
    cfg.pan_center = server.arg("pan_center").toInt();
    xSemaphoreTake(statMutex, portMAX_DELAY);
    current_pan = cfg.pan_center;
    xSemaphoreGive(statMutex);
  }
  if (server.hasArg("tilt_center")) {
    cfg.tilt_center = server.arg("tilt_center").toInt();
    xSemaphoreTake(statMutex, portMAX_DELAY);
    current_tilt = cfg.tilt_center;
    xSemaphoreGive(statMutex);
  }
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

  // Re-center if either angle just changed
  if (server.hasArg("pan_center") || server.hasArg("tilt_center")) {
    sendServoCommand(cfg.pan_center, cfg.tilt_center);
  }

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{"
      "\"threshold\":%d,"
      "\"min_pixels\":%d,"
      "\"pan_kp\":%.4f,"
      "\"tilt_kp\":%.4f,"
      "\"pan_center\":%d,"
      "\"tilt_center\":%d,"
      "\"smoothing_frames\":%d,"
      "\"enabled\":%s,"
      "\"hmirror\":%s,"
      "\"vflip\":%s"
    "}",
    cfg.threshold, cfg.min_pixels,
    cfg.pan_kp, cfg.tilt_kp,
    cfg.pan_center, cfg.tilt_center,
    cfg.smoothing_frames,
    cfg.enabled ? "true" : "false",
    cfg.hmirror  ? "true" : "false",
    cfg.vflip    ? "true" : "false"
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// =========================================================
// Setup
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== HackPack Turret Tracker — ESP32-CAM ===");

  // UART2 → Nano (TX=14, RX=15)
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("UART2 to Nano ready");

  // Camera
  if (!initCamera()) {
    Serial.println("FATAL: camera failed — rebooting in 5 s");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Camera OK");

  // Capture one frame to learn dimensions and prime the previous-frame buffer
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

  // Mutexes
  camMutex  = xSemaphoreCreateMutex();
  statMutex = xSemaphoreCreateMutex();

  // WiFi
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

  // OTA
  ArduinoOTA.setHostname("turret-cam");
  ArduinoOTA.onStart([]()            { Serial.println("OTA: start");    });
  ArduinoOTA.onEnd([]()              { Serial.println("OTA: end");      });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\n", p * 100 / t);
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();
  Serial.println("OTA ready — hostname: turret-cam");

  // HTTP routes
  server.on("/status",  handleStatus);
  server.on("/config",  handleConfig);
  server.on("/capture", handleCapture);
  server.begin();
  Serial.println("HTTP server started on port 80");

  streamServer.on("/stream", handleStream);
  streamServer.begin();
  Serial.println("Stream server started on port 81");

  // Centre the turret
  sendServoCommand(cfg.pan_center, cfg.tilt_center);

  // Tracking task on core 0 (Arduino loop() runs on core 1)
  xTaskCreatePinnedToCore(
    trackingTask,
    "tracking",
    8192,   // stack bytes
    nullptr,
    1,      // priority
    nullptr,
    0       // core 0
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
