// main.cpp
// HackPack Turret Tracker — ESP32-CAM firmware
//
// Motion-detection tracking with scan-settle-detect state machine.
// Detects motion via block-based grayscale frame differencing and sends
// proportional X/Y error commands to the Arduino Nano over UART
// (GPIO14 TX only — one-way link).
//
// The camera is mounted on the turret, so every servo move shifts the
// entire frame.  The state machine handles this:
//   SCANNING  → send "S\n" so Nano pans slowly; no detection attempted
//   SETTLING  → discard frames while the image stabilises after a move
//   WATCHING  → compare frames; if motion found, send X/Y errors
//
// HTTP endpoints:
//   http://<IP>:81/stream  — MJPEG live stream
//   http://<IP>/capture    — single JPEG snapshot
//   http://<IP>/status     — JSON tracking stats
//   http://<IP>/config     — GET / SET tracking parameters
//   http://<IP>/move?x=N&y=N — manual move (manual_mode only)

#include <Arduino.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "wifi_credentials.h"

Preferences prefs;

// Forward declaration
void saveConfig();

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;

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

// ── Motion detection parameters ──────────────────────────────────────────────
#define FRAME_W     160   // downsampled grayscale width
#define FRAME_H     120   // downsampled grayscale height
#define GRID_W       16   // block grid columns  (FRAME_W / BLOCK_SIZE)
#define GRID_H       12   // block grid rows     (FRAME_H / BLOCK_SIZE)
#define BLOCK_SIZE   10   // pixels per block edge

// ── Detection state machine ──────────────────────────────────────────────────
enum DetectState : uint8_t {
  DETECT_SCANNING,   // no target — requesting Nano to pan
  DETECT_SETTLING,   // waiting for frame to stabilise after turret move
  DETECT_WATCHING,   // comparing frames, looking for motion
};

// =========================================================
// Tunable tracking config (adjustable at runtime via /config)
// =========================================================
struct Config {
  bool enabled      = true;   // master on/off
  bool hmirror      = true;   // flip camera horizontally
  bool vflip        = false;  // flip camera vertically
  int  offset_x     = 0;     // aim point offset from frame centre (pixels)
  int  offset_y     = 0;
  bool manual_mode  = false;  // true = ignore auto-tracking, accept /move cmds
  int  threshold    = 25;     // per-pixel diff to count as "changed" (0-255)
  int  min_blocks   = 3;      // minimum active blocks to confirm motion
  int  settle_frames = 3;     // frames to discard after turret moves
};

Config cfg;

// =========================================================
// State
// =========================================================
TaskHandle_t trackingTaskHandle = nullptr;
uint8_t* rgb_buf   = nullptr;  // RGB888 decode workspace (QVGA)
uint8_t* ref_frame = nullptr;  // reference grayscale 160×120
uint8_t* cur_frame = nullptr;  // current   grayscale 160×120
int      frame_width  = 0;
int      frame_height = 0;

// Stats (written by tracking task, read by HTTP handlers)
volatile unsigned long stat_frames        = 0;
volatile int           stat_motion_blocks = 0;   // active blocks in last frame
volatile int           stat_cx            = -1;   // motion centroid x (pixel)
volatile int           stat_cy            = -1;   // motion centroid y (pixel)
volatile int           stat_ex            =  0;   // last error X (-100..100)
volatile int           stat_ey            =  0;   // last error Y (-100..100)
volatile unsigned long stat_last_cmd_ms   =  0;
volatile unsigned long stat_detect_ms     =  0;   // last detection duration
volatile uint8_t       stat_state         =  0;   // current DetectState

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
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
// Grayscale conversion + 2× downsample (320×240 RGB888 → 160×120 gray)
// =========================================================
void rgb888ToGrayDownsample(const uint8_t* rgb, int srcW, int srcH,
                            uint8_t* gray) {
  for (int dy = 0; dy < FRAME_H; dy++) {
    int sy = dy * 2;
    for (int dx = 0; dx < FRAME_W; dx++) {
      int sx  = dx * 2;
      int idx = (sy * srcW + sx) * 3;
      // Fast luminance: (R*77 + G*150 + B*29) >> 8
      gray[dy * FRAME_W + dx] =
          (rgb[idx] * 77 + rgb[idx + 1] * 150 + rgb[idx + 2] * 29) >> 8;
    }
  }
}

// =========================================================
// Block-based motion detection
// Returns number of active blocks; sets *out_cx, *out_cy to centroid.
// =========================================================
int detectMotion(const uint8_t* ref, const uint8_t* cur,
                 int threshold, int minBlocks,
                 int* out_cx, int* out_cy) {
  int  activeBlocks = 0;
  long sumX = 0, sumY = 0, sumW = 0;

  for (int by = 0; by < GRID_H; by++) {
    for (int bx = 0; bx < GRID_W; bx++) {
      int diffSum = 0;

      for (int y = 0; y < BLOCK_SIZE; y++) {
        int row = by * BLOCK_SIZE + y;
        for (int x = 0; x < BLOCK_SIZE; x++) {
          int col = bx * BLOCK_SIZE + x;
          int idx = row * FRAME_W + col;
          int d   = abs((int)cur[idx] - (int)ref[idx]);
          if (d > threshold) diffSum += d;
        }
      }

      int meanDiff = diffSum / (BLOCK_SIZE * BLOCK_SIZE);
      if (meanDiff > threshold / 2) {
        int blockCX = bx * BLOCK_SIZE + BLOCK_SIZE / 2;
        int blockCY = by * BLOCK_SIZE + BLOCK_SIZE / 2;
        sumX += (long)blockCX * meanDiff;
        sumY += (long)blockCY * meanDiff;
        sumW += meanDiff;
        activeBlocks++;
      }
    }
  }

  if (activeBlocks >= minBlocks && sumW > 0) {
    *out_cx = (int)(sumX / sumW);
    *out_cy = (int)(sumY / sumW);
    return activeBlocks;
  }
  return 0;
}

// =========================================================
// Send tracking error to Nano
// =========================================================
void sendTrackingError(int ex, int ey) {
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "X%dY%d\n", ex, ey);
  Serial2.print(cmd);
  if (ex != 0 || ey != 0) Serial.printf("-> Nano: %s", cmd);

  xSemaphoreTake(statMutex, portMAX_DELAY);
  stat_ex          = ex;
  stat_ey          = ey;
  stat_last_cmd_ms = millis();
  xSemaphoreGive(statMutex);
}

// Send scan command to Nano
void sendScanCommand() {
  Serial2.print("S\n");
  Serial.println("-> Nano: S (scan step)");

  xSemaphoreTake(statMutex, portMAX_DELAY);
  stat_last_cmd_ms = millis();
  xSemaphoreGive(statMutex);
}

// =========================================================
// Tracking task — runs on core 0
// =========================================================
void trackingTask(void* pvParameters) {
  DetectState state      = DETECT_SCANNING;
  int  settleCount       = 0;
  int  lostCount         = 0;
  unsigned long lastScanMs = 0;

  const int SCAN_INTERVAL_MS = 500;   // time between scan steps
  const int LOST_THRESHOLD   = 10;    // frames w/o motion → return to scan
  const int AIM_DEADZONE     = 10;    // error below this = on target, don't resettle

  for (;;) {
    if (!cfg.enabled || cfg.manual_mode) {
      vTaskDelay(pdMS_TO_TICKS(100));
      state = DETECT_SCANNING;
      continue;
    }

    // ── Grab a frame and convert to grayscale 160×120 ─────────────────────
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(200)) != pdTRUE) continue;
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
    if (!rgb_buf || !ref_frame || !cur_frame) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    bool decoded = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb_buf);

    xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    if (!decoded) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    rgb888ToGrayDownsample(rgb_buf, frame_width, frame_height, cur_frame);

    unsigned long t0 = millis();

    // ── State machine ─────────────────────────────────────────────────────
    switch (state) {

      case DETECT_SCANNING: {
        if (millis() - lastScanMs >= (unsigned long)SCAN_INTERVAL_MS) {
          sendScanCommand();
          lastScanMs   = millis();
          settleCount  = 0;
          state        = DETECT_SETTLING;
        }
        break;
      }

      case DETECT_SETTLING: {
        settleCount++;
        if (settleCount >= cfg.settle_frames) {
          // Settled — capture reference frame
          memcpy(ref_frame, cur_frame, FRAME_W * FRAME_H);
          state = DETECT_WATCHING;
        }
        break;
      }

      case DETECT_WATCHING: {
        int cx, cy;
        int blocks = detectMotion(ref_frame, cur_frame,
                                  cfg.threshold, cfg.min_blocks,
                                  &cx, &cy);

        if (blocks > 0) {
          // Convert pixel coords to normalised -100..+100 error
          int ex = ((cx - FRAME_W / 2) * 200) / FRAME_W;
          int ey = ((cy - FRAME_H / 2) * 200) / FRAME_H;
          ex = constrain(ex + cfg.offset_x, -100, 100);
          ey = constrain(ey + cfg.offset_y, -100, 100);

          sendTrackingError(ex, ey);
          lostCount = 0;

          // Update stats
          xSemaphoreTake(statMutex, portMAX_DELAY);
          stat_motion_blocks = blocks;
          stat_cx = cx;
          stat_cy = cy;
          xSemaphoreGive(statMutex);

          if (abs(ex) > AIM_DEADZONE || abs(ey) > AIM_DEADZONE) {
            // Big error — Nano will move, need to resettle
            settleCount = 0;
            state = DETECT_SETTLING;
          }
          // else: small error, Nano won't step → stay in WATCHING
        } else {
          lostCount++;
          if (lostCount > LOST_THRESHOLD) {
            state     = DETECT_SCANNING;
            lostCount = 0;
            Serial.println("Motion lost — resuming scan");
          }
          // Send zero error to keep Nano watchdog alive
          sendTrackingError(0, 0);

          xSemaphoreTake(statMutex, portMAX_DELAY);
          stat_motion_blocks = 0;
          xSemaphoreGive(statMutex);
        }
        break;
      }
    }

    unsigned long detectTime = millis() - t0;

    xSemaphoreTake(statMutex, portMAX_DELAY);
    stat_frames++;
    stat_detect_ms = detectTime;
    stat_state     = (uint8_t)state;
    xSemaphoreGive(statMutex);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =========================================================
// HTTP: MJPEG stream  (port 81, /stream)
// =========================================================
void handleStream() {
  WiFiClient client = streamServer.client();

  client.print("HTTP/1.1 200 OK\r\n"
               "Access-Control-Allow-Origin: *\r\n"
               "Cache-Control: no-cache, no-store, must-revalidate\r\n"
               "Pragma: no-cache\r\n"
               "Expires: -1\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "\r\n");

  while (client.connected()) {
    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(500)) != pdTRUE) continue;
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) { delay(10); continue; }

    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");

    xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    delay(50);
  }
}

// =========================================================
// HTML dashboard — tactical console + live stream
// =========================================================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turret Defense System</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0a;color:#00ff41;font-family:'Courier New',monospace;overflow:hidden;height:100vh}
#wrap{display:flex;height:100vh;gap:0}
#vid{flex:1;background:#000;display:flex;align-items:center;justify-content:center;position:relative;min-width:0}
#vid img{max-width:100%;max-height:100%;object-fit:contain}
#reticle{position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;
  width:80px;height:80px;border:2px solid rgba(255,0,0,0.5);border-radius:50%}
#reticle::before,#reticle::after{content:'';position:absolute;background:rgba(255,0,0,0.4)}
#reticle::before{width:1px;height:100%;left:50%;top:0}
#reticle::after{height:1px;width:100%;top:50%;left:0}
#panel{width:380px;display:flex;flex-direction:column;border-left:1px solid #1a3a1a;background:#0d0d0d}
#hdr{padding:10px 12px;border-bottom:1px solid #1a3a1a;font-size:13px;text-align:center;
  background:#0a140a;letter-spacing:2px;text-transform:uppercase}
#stats{padding:8px 12px;border-bottom:1px solid #1a3a1a;font-size:11px;line-height:1.8;background:#0a100a}
#stats .val{color:#00cc33}
#stats .warn{color:#ff4444}
#stats .lock{color:#ff0;font-weight:bold}
#console{flex:1;overflow-y:auto;padding:8px 12px;font-size:12px;line-height:1.6}
#console::-webkit-scrollbar{width:6px}
#console::-webkit-scrollbar-thumb{background:#1a3a1a}
.log{white-space:pre-wrap;word-break:break-all}
.log.scan{color:#2a7a2a}
.log.alert{color:#ff4444;font-weight:bold}
.log.track{color:#00cc33}
.log.lock{color:#ffcc00;font-weight:bold}
.log.fire{color:#ff0000;font-weight:bold;text-shadow:0 0 6px #f00}
.log.sys{color:#00aaff}
.log.info{color:#888}
</style>
</head>
<body>
<div id="wrap">
  <div id="vid">
    <img id="stream">
    <div id="reticle"></div>
  </div>
  <div id="panel">
    <div id="hdr">Turret Defense System v2.0</div>
    <div id="stats">
      State: <span id="st" class="val">--</span><br>
      Error: X:<span id="ex" class="val">0</span> Y:<span id="ey" class="val">0</span><br>
      Blocks: <span id="bl" class="val">0</span> | FPS: <span id="fps" class="val">--</span> | Detect: <span id="dt" class="val">--</span>ms<br>
      RSSI: <span id="rssi" class="val">--</span>dBm | Heap: <span id="heap" class="val">--</span>
    </div>
    <div id="console"></div>
  </div>
</div>
<script>
const C=document.getElementById('console');
const AIM=15;
let prev={state:'',blocks:0,ex:0,ey:0};
let frames0=0,t0=Date.now(),engageCount=0,firing=false;

function log(msg,cls){
  const d=document.createElement('div');
  d.className='log '+(cls||'');
  const now=new Date();
  const ts=String(now.getHours()).padStart(2,'0')+':'+
           String(now.getMinutes()).padStart(2,'0')+':'+
           String(now.getSeconds()).padStart(2,'0');
  d.textContent='['+ts+'] '+msg;
  C.appendChild(d);
  if(C.children.length>200)C.removeChild(C.firstChild);
  C.scrollTop=C.scrollHeight;
}

function updateReticle(ex,ey,locked){
  const r=document.getElementById('reticle');
  const c=locked?'rgba(255,0,0,0.9)':'rgba(255,0,0,0.4)';
  r.style.borderColor=locked?'#f00':'rgba(255,0,0,0.5)';
  r.style.boxShadow=locked?'0 0 15px #f00, inset 0 0 15px rgba(255,0,0,0.2)':'none';
}

log('TURRET DEFENSE SYSTEM v2.0','sys');
log('Connecting to camera feed...','sys');

const host=location.hostname;
document.getElementById('stream').src='http://'+host+':81/stream';
document.getElementById('stream').onload=function(){log('Camera feed online.','sys');};

function poll(){
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('st').textContent=d.state.toUpperCase();
    document.getElementById('ex').textContent=d.error_x;
    document.getElementById('ey').textContent=d.error_y;
    document.getElementById('bl').textContent=d.motion_blocks;
    document.getElementById('dt').textContent=d.detect_ms;
    document.getElementById('rssi').textContent=d.rssi;
    document.getElementById('heap').textContent=(d.heap/1024).toFixed(0)+'K';

    // FPS calc
    const now=Date.now();
    if(now-t0>2000){
      const fps=((d.frames-frames0)*1000/(now-t0)).toFixed(1);
      document.getElementById('fps').textContent=fps;
      frames0=d.frames;t0=now;
    }

    const st=d.state, bl=d.motion_blocks, ex=d.error_x, ey=d.error_y;
    const locked=Math.abs(ex)<AIM&&Math.abs(ey)<AIM&&bl>0;

    // State transitions
    if(st==='scanning'&&prev.state!=='scanning'){
      if(prev.state==='watching')log('Target lost. Resuming patrol...','info');
      log('SCANNING — Sweeping sector...','scan');
    }
    if(st==='watching'&&prev.state!=='watching'&&prev.state!==''){
      // just settled, watching now
    }
    if(st==='watching'&&bl>0&&prev.blocks===0){
      log('*** MOTION DETECTED ***','alert');
    }
    if(st==='watching'&&bl>0){
      if(locked){
        log('LOCKED ON  X:'+ex+' Y:'+ey,'lock');
        updateReticle(ex,ey,true);
      } else if(Math.abs(ex)>50||Math.abs(ey)>50){
        log('Target X:'+ex+' Y:'+ey+' | Intercepting...','track');
        updateReticle(ex,ey,false);
      } else {
        log('Target X:'+ex+' Y:'+ey+' | Closing in...','track');
        updateReticle(ex,ey,false);
      }
    }
    if(st==='scanning'){
      updateReticle(0,0,false);
    }

    // Detect fire state (locked in watching for consecutive polls)
    if(locked&&!firing){
      firing=true;engageCount++;
      log('>>> ENGAGING TARGET <<< [#'+engageCount+']','fire');
    }
    if(!locked)firing=false;

    // Periodic scan log
    if(st==='scanning'&&d.frames%30===0&&bl===0){
      log('Sweeping sector...','scan');
    }

    prev={state:st,blocks:bl,ex:ex,ey:ey};
  }).catch(()=>{});
}
setInterval(poll,400);
log('Systems online. Awaiting tracking activation.','sys');
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
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

  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);

  xSemaphoreTake(camMutex, pdMS_TO_TICKS(10));
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
}

// State name helper
const char* stateName(uint8_t s) {
  switch (s) {
    case DETECT_SCANNING: return "scanning";
    case DETECT_SETTLING: return "settling";
    case DETECT_WATCHING: return "watching";
    default:              return "unknown";
  }
}

// HTTP: JSON status
void handleStatus() {
  xSemaphoreTake(statMutex, portMAX_DELAY);
  unsigned long frames    = stat_frames;
  int           blocks    = stat_motion_blocks;
  int           cx        = stat_cx;
  int           cy        = stat_cy;
  int           ex        = stat_ex;
  int           ey        = stat_ey;
  unsigned long cmd_ms    = stat_last_cmd_ms;
  unsigned long detect_ms = stat_detect_ms;
  uint8_t       st        = stat_state;
  xSemaphoreGive(statMutex);

  char buf[640];
  snprintf(buf, sizeof(buf),
    "{"
      "\"frames\":%lu,"
      "\"motion_blocks\":%d,"
      "\"centroid_x\":%d,"
      "\"centroid_y\":%d,"
      "\"error_x\":%d,"
      "\"error_y\":%d,"
      "\"state\":\"%s\","
      "\"tracking\":%s,"
      "\"manual_mode\":%s,"
      "\"ms_since_cmd\":%lu,"
      "\"detect_ms\":%lu,"
      "\"ip\":\"%s\","
      "\"rssi\":%d,"
      "\"heap\":%lu"
    "}",
    frames, blocks, cx, cy, ex, ey,
    stateName(st),
    cfg.enabled ? "true" : "false",
    cfg.manual_mode ? "true" : "false",
    millis() - cmd_ms,
    detect_ms,
    WiFi.localIP().toString().c_str(),
    WiFi.RSSI(),
    (unsigned long)ESP.getFreeHeap()
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// HTTP: GET/SET config
void handleConfig() {
  if (server.hasArg("enabled"))       cfg.enabled       = server.arg("enabled").toInt() != 0;
  if (server.hasArg("offset_x"))      cfg.offset_x      = server.arg("offset_x").toInt();
  if (server.hasArg("offset_y"))      cfg.offset_y      = server.arg("offset_y").toInt();
  if (server.hasArg("manual_mode"))   cfg.manual_mode   = server.arg("manual_mode").toInt() != 0;
  if (server.hasArg("threshold"))     cfg.threshold     = constrain(server.arg("threshold").toInt(), 1, 200);
  if (server.hasArg("min_blocks"))    cfg.min_blocks    = constrain(server.arg("min_blocks").toInt(), 1, 100);
  if (server.hasArg("settle_frames")) cfg.settle_frames = constrain(server.arg("settle_frames").toInt(), 1, 20);

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

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{"
      "\"enabled\":%s,"
      "\"manual_mode\":%s,"
      "\"hmirror\":%s,"
      "\"vflip\":%s,"
      "\"offset_x\":%d,"
      "\"offset_y\":%d,"
      "\"threshold\":%d,"
      "\"min_blocks\":%d,"
      "\"settle_frames\":%d,"
      "\"saved\":%s"
    "}",
    cfg.enabled      ? "true" : "false",
    cfg.manual_mode  ? "true" : "false",
    cfg.hmirror      ? "true" : "false",
    cfg.vflip        ? "true" : "false",
    cfg.offset_x, cfg.offset_y,
    cfg.threshold, cfg.min_blocks, cfg.settle_frames,
    saved ? "true" : "false"
  );

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

// HTTP: manual move command (manual_mode only)
void handleMove() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (!cfg.manual_mode) {
    server.send(400, "application/json",
      "{\"error\":\"manual_mode is off — enable it via /config?manual_mode=1\"}");
    return;
  }

  int x = server.hasArg("x") ? server.arg("x").toInt() : 0;
  int y = server.hasArg("y") ? server.arg("y").toInt() : 0;

  int x_steps = abs(x);
  int y_steps = abs(y);
  const char* x_cmd = x > 0 ? "R\n" : "L\n";
  const char* y_cmd = y > 0 ? "D\n" : "U\n";

  for (int i = 0; i < x_steps; i++) Serial2.print(x_cmd);
  for (int i = 0; i < y_steps; i++) Serial2.print(y_cmd);

  Serial.printf("-> Nano: %dx%s %dx%s\n", x_steps, x > 0 ? "R" : "L",
                                            y_steps, y > 0 ? "D" : "U");

  char buf[80];
  snprintf(buf, sizeof(buf), "{\"x\":%d,\"y\":%d}", x, y);
  server.send(200, "application/json", buf);
}

// =========================================================
// NVS persistence
// =========================================================
void saveConfig() {
  prefs.begin("turret", false);
  prefs.putBool("enabled",      cfg.enabled);
  prefs.putBool("hmirror",      cfg.hmirror);
  prefs.putBool("vflip",        cfg.vflip);
  prefs.putInt("offset_x",      cfg.offset_x);
  prefs.putInt("offset_y",      cfg.offset_y);
  prefs.putBool("manual",       cfg.manual_mode);
  prefs.putInt("threshold",     cfg.threshold);
  prefs.putInt("min_blocks",    cfg.min_blocks);
  prefs.putInt("settle_frames", cfg.settle_frames);
  prefs.end();
  Serial.println("Config saved to NVS");
}

void loadConfig() {
  prefs.begin("turret", true);  // read-only
  cfg.enabled       = prefs.getBool("enabled",      cfg.enabled);
  cfg.hmirror       = prefs.getBool("hmirror",      cfg.hmirror);
  cfg.vflip         = prefs.getBool("vflip",        cfg.vflip);
  cfg.offset_x      = prefs.getInt("offset_x",      cfg.offset_x);
  cfg.offset_y      = prefs.getInt("offset_y",      cfg.offset_y);
  cfg.manual_mode   = prefs.getBool("manual",       cfg.manual_mode);
  cfg.threshold     = prefs.getInt("threshold",     cfg.threshold);
  cfg.min_blocks    = prefs.getInt("min_blocks",    cfg.min_blocks);
  cfg.settle_frames = prefs.getInt("settle_frames", cfg.settle_frames);
  prefs.end();
  Serial.printf("Config loaded (threshold=%d, min_blocks=%d, settle=%d)\n",
                cfg.threshold, cfg.min_blocks, cfg.settle_frames);
}

// =========================================================
// Setup
// =========================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== HackPack Turret Tracker — ESP32-CAM ===");
  Serial.println("    Motion-detection mode");

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

    // RGB888 decode buffer (for JPEG → RGB → grayscale pipeline)
    size_t rgb_size = (size_t)fb->width * fb->height * 3;
    rgb_buf = (uint8_t*)ps_malloc(rgb_size);

    esp_camera_fb_return(fb);
    Serial.printf("Frame: %d x %d\n", frame_width, frame_height);

    if (!rgb_buf) {
      Serial.println("FATAL: cannot allocate RGB buffer");
      delay(5000);
      ESP.restart();
    }
  } else {
    Serial.println("FATAL: cannot read first frame — rebooting");
    delay(5000);
    ESP.restart();
  }

  // Grayscale frame buffers (tiny — 19.2 KB each)
  ref_frame = (uint8_t*)ps_malloc(FRAME_W * FRAME_H);
  cur_frame = (uint8_t*)ps_malloc(FRAME_W * FRAME_H);
  if (!ref_frame || !cur_frame) {
    Serial.println("FATAL: cannot allocate grayscale buffers");
    delay(5000);
    ESP.restart();
  }
  memset(ref_frame, 0, FRAME_W * FRAME_H);
  Serial.printf("Motion buffers: %dx%d grayscale (%d bytes each)\n",
                FRAME_W, FRAME_H, FRAME_W * FRAME_H);

  camMutex  = xSemaphoreCreateMutex();
  statMutex = xSemaphoreCreateMutex();

  Serial.printf("Connecting to %s", ssid);
  WiFi.setHostname("turret-cam");
  WiFi.begin(ssid, password);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Dashboard: http://%s/\n",          WiFi.localIP().toString().c_str());
    Serial.printf("  Stream:    http://%s:81/stream\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Status:    http://%s/status\n",    WiFi.localIP().toString().c_str());
    Serial.printf("  Config:    http://%s/config\n",    WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi FAILED — offline mode (tracking still works)");
  }

  ArduinoOTA.setHostname("turret-cam");
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: start — suspending tracking and camera");
    if (trackingTaskHandle) vTaskSuspend(trackingTaskHandle);
    esp_camera_deinit();
  });
  ArduinoOTA.onEnd([]()              { Serial.println("OTA: end");      });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%\n", p * 100 / t);
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();
  Serial.println("OTA ready — hostname: turret-cam");

  server.on("/",        handleRoot);
  server.on("/status",  handleStatus);
  server.on("/config",  handleConfig);
  server.on("/capture", handleCapture);
  server.on("/move",    handleMove);
  server.begin();
  Serial.println("HTTP server started on port 80");

  streamServer.on("/stream", handleStream);
  streamServer.begin();
  Serial.println("Stream server started on port 81");

  xTaskCreatePinnedToCore(
    trackingTask,
    "tracking",
    8192,    // reduced from 16384 — no neural network stack needed
    nullptr,
    1,
    &trackingTaskHandle,
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
