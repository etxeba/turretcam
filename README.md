# HackPack Turret Tracker

A self-contained motion-tracking upgrade for the [CrunchLabs Hack Pack](https://www.crunchlabs.com/) turret. An ESP32-CAM detects motion via frame differencing and sends proportional tracking errors to the Arduino Nano over a one-way UART link. The Nano owns all servo state and translates errors into movement.

**No external computer required** — the turret tracks and fires autonomously once powered on.

---

## Architecture

```
Breakout board (USB-C power)
        | 5V + GND
        v
+-----------------------------------------------+
|                    TURRET                      |
|                                                |
|  +--------------+   GPIO14 TX   +----------+  |
|  |  ESP32-CAM   |--------------+|  Arduino |  |
|  |              |   (one-way)   |  Nano    |  |
|  |  * Camera    |               |          |  |
|  |  * Motion    |               |  * Yaw   |  |
|  |    detection |               |  * Pitch |  |
|  |  * WiFi/OTA  |               |  * Roll  |  |
|  |  * Dashboard |               |  * IR RX |  |
|  +--------------+               +----------+  |
+-----------------------------------------------+
```

The ESP32-CAM runs a **scan-settle-detect** state machine:
1. **SCANNING** — no target; sends `S\n` so the Nano pans slowly
2. **SETTLING** — turret just moved; discards frames while the image stabilises
3. **WATCHING** — compares consecutive frames via block-based grayscale differencing; if motion is found, sends `X<error>Y<error>\n` and re-settles after the Nano moves

This approach is lightweight enough for the ESP32's limited processing power (12-20 FPS vs ~1-2 FPS with the old face detection approach) and handles the camera-on-turret problem by only detecting motion after the frame has settled.

---

## Bill of Materials

| Item | ~Price | Notes |
|------|--------|-------|
| [CrunchLabs Hack Pack turret](https://www.crunchlabs.com/pages/ir-turret-landing) | $66 | Includes Arduino Nano, 3 servos, IR remote, and power PCB |
| [ESP32-CAM (AI Thinker)](https://www.amazon.com/s?k=esp32-cam+ai+thinker) | $10-14 | With OV2640 camera module |
| FTDI USB-to-TTL adapter (FT232RL) | ~$7 | One-time initial flash only; OTA after that |
| Female-to-female jumper wires | ~$5 | 3 wires needed permanently |
| **Total** | **~$95** | |

> **Non-HackPack builds:** This firmware will work with any Arduino Nano + 3-servo pan/tilt/fire rig and any NEC-protocol IR remote. You will need to adjust the servo pins, IR codes, and pitch/roll limits in `turret_nano_firmware.ino` to match your hardware (see notes in the Setup section).

### HackPack hardware notes

The HackPack turret routes power through a breadboard with **two voltage rails**:

| Rail | Voltage | Used for |
|------|---------|----------|
| Logic / 5V | 5 V | Arduino Nano, IR receiver, ESP32-CAM |
| VIN | ~7 V | Yaw and roll (continuous) servos |

> **The ESP32-CAM must connect to the 5V rail -- not VIN.** 7V will damage it. On the HackPack breadboard the 5V rail is the one fed from the Nano's 5V pin, not the battery's raw output. Check the wire color on your build or measure with a multimeter before connecting.

The included USB-C power bank is adequate for the combined load. Keep it charged above 75% -- servos become unreliable as voltage sags. The power bank charges via USB-C and powers the Nano (and through it the ESP32-CAM) via USB-C to the Nano's port.

---

## Wiring

See `docs/WIRING.md` for the full reference with diagrams. Summary:

### Permanent wiring (3 wires, stays on the turret)

| ESP32-CAM | HackPack breadboard | Purpose |
|-----------|---------------------|---------|
| **5V** | **5V rail** (Nano 5V row) | Power -- **must be 5V, not VIN/7V** |
| **GND** | **GND rail** | Common ground |
| **GPIO 14** | **Nano D4** | Tracking error commands (TX only, SoftwareSerial) |

> **Power:** The HackPack breadboard carries two voltage rails. Connect the ESP32-CAM to the **5V rail** -- the row fed from the Nano's 5V pin. The board also has a ~7V VIN rail that powers the continuous servos; connecting the ESP32-CAM there will damage it. Measure with a multimeter if unsure. On other builds use any regulated 5V / 1A supply.

> **No serial conflict:** GPIO 14 connects to D4 (SoftwareSerial), not D0. The Nano's hardware UART (D0/D1) stays free for USB flashing and Serial Monitor at all times -- no wire changes needed when uploading firmware.

### FTDI wiring (one-time flash only)

| FTDI | ESP32-CAM | Notes |
|------|-----------|-------|
| 5V | 5V | Power during flash |
| GND | GND | Ground |
| TX | U0R (RX0) | Cross TX->RX |
| RX | U0T (TX0) | Cross RX->TX |
| -- | GPIO 0 -> GND | Bridge to enter flash mode |

---

## Setup

### 1 -- Install tools

**Nano (Arduino IDE):**

1. Download [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. Install libraries via **Tools -> Manage Libraries**:
   - `IRremote` by Armin Joachimsmeyer -- **version 4.x**
   - `Servo` (usually pre-installed)

**ESP32-CAM (PlatformIO):**

1. Install [PlatformIO CLI](https://platformio.org/install/cli) or the VS Code extension
2. Copy `esp32cam_firmware/include/wifi_credentials.h.example` to `wifi_credentials.h` and fill in your WiFi SSID and password

### 2 -- Flash the Arduino Nano

1. Open `turret_nano_firmware/turret_nano_firmware.ino` in Arduino IDE
2. **If using non-HackPack hardware**, check these values near the top of the file and adjust to match your build:
   ```cpp
   yawServo.attach(10);    // continuous rotation -- base spin
   pitchServo.attach(11);  // positional -- up/down tilt
   rollServo.attach(12);   // continuous rotation -- barrel/fire

   int pitchMax = 150;     // maximum pitch angle (degrees)
   int pitchMin = 33;      // minimum pitch angle (degrees)
   ```
   Also verify the IR codes match your remote -- the defaults are for the NEC remote included with the HackPack. To find codes for a different remote, temporarily add `IrReceiver.printIRResultShort(&Serial);` inside the `IrReceiver.decode()` block, upload, open Serial Monitor at 9600 baud, and press each button.
3. Select board: **Tools -> Board -> Arduino AVR Boards -> Arduino Nano**
4. Select the Nano's USB-C serial port
5. Click **Upload** -- no need to disconnect any wires

### 3 -- Flash the ESP32-CAM (initial, wired)

1. Wire the FTDI adapter to the ESP32-CAM (see wiring table above)
2. **Bridge GPIO 0 to GND** on the ESP32-CAM
3. Build and flash:
   ```bash
   cd esp32cam_firmware
   pio run -t upload
   ```
4. When the console shows `Connecting...` press the **RST** button once
5. After upload: **remove the GPIO 0 bridge**, then press **RST** to boot normally
6. Monitor startup output:
   ```bash
   pio device monitor
   ```
   You should see the IP address and dashboard URL printed at 115200 baud.

All future firmware updates can be done wirelessly via OTA (see below).

### 4 -- Assemble and test

1. Connect the 3 permanent wires (5V, GND, GPIO 14)
2. Power on via USB-C to the breakout board
3. Open the dashboard: `http://<ESP32_IP>/` (or `http://turret-cam.local/`)
4. Wave your hand in front of the camera -- check that motion is detected on the dashboard
5. Activate tracking mode on the Nano (see Modes below) and confirm the turret follows

---

## Dashboard

The ESP32-CAM serves a live tactical dashboard at `http://<ESP32_IP>/` that shows:

- **Left panel:** Live MJPEG camera feed with a crosshair reticle overlay (glows red when locked on target)
- **Right panel:** Real-time tactical console with:
  - Status bar: detection state, X/Y errors, motion blocks, FPS, detection latency, WiFi RSSI, free heap
  - Scrolling log with color-coded messages:
    - `[SCAN] Sweeping sector...` (dim green)
    - `*** MOTION DETECTED ***` (red, bold)
    - `Target X:-45 Y:23 | Intercepting...` (green)
    - `LOCKED ON` (yellow, bold)
    - `>>> ENGAGING TARGET <<< [#3]` (red, glowing)

No USB cable needed -- open the page on any phone or laptop on the same WiFi. The dashboard polls `/status` every 400ms and generates the tactical text from state transitions.

The Nano also outputs the same dramatic text over its USB Serial Monitor at 9600 baud, including a startup banner, weapons check animation, and engagement counter.

---

## Modes

The Nano has three operating modes, toggled with the IR remote. Both sequences share the `5-5-5` prefix -- the 4th button decides which mode activates.

| Sequence | Transition | Effect |
|----------|-----------|--------|
| **5-5-5-5** | MANUAL -> TRACKING | Camera controls the turret |
| **5-5-5-5** | TRACKING -> MANUAL | Returns to manual control |
| **5-5-5-6** | any -> AUTOFIRE | Camera tracks and fires automatically |
| **5-5-5-5** | AUTOFIRE -> MANUAL | Disengages auto-fire |

The turret runs a **weapons check animation** (quick left-right-up-down sweep + barrel spin) on every mode change as physical confirmation.

### MANUAL (default)
- IR remote controls movement, fire, and gestures
- Serial commands accepted over USB: `L`, `R`, `U`, `D`, `F`, `H`, `Y`, `N` (each followed by newline)

### TRACKING
- ESP32-CAM scans the room and tracks detected motion
- Turret sweeps in a raster pattern (alternating yaw with pitch level changes) when no target is present
- When motion is detected, turret follows the motion centroid
- IR remote and serial movement commands are ignored
- Yaw stops automatically if the ESP32 goes silent for >250 ms (watchdog)

### AUTOFIRE
- Same as TRACKING, plus: fires continuously while the target is within the aim threshold
- Stops firing immediately when the lock is lost
- Dwell guard prevents firing as the turret sweeps through the aim point

#### Auto-fire tuning (in `turret_nano_firmware.ino`)

| Constant | Default | Effect |
|----------|---------|--------|
| `AUTOFIRE_AIM_THRESHOLD` | `15` | How tight the aim must be to fire (error within +/-15 of +/-100; lower = harder to trigger) |
| `AUTOFIRE_DWELL_FRAMES` | `8` | Consecutive on-target frames required before firing (~0.5s at 15 FPS) |

---

## IR Remote Reference

| Button | MANUAL mode | TRACKING / AUTOFIRE |
|--------|------------|---------------------|
| Up | Pitch up | -- |
| Down | Pitch down | -- |
| Left | Yaw left | -- |
| Right | Yaw right | -- |
| OK | Fire (single dart) | -- |
| Star | Fire all darts | -- |
| 1 | Shake yes | -- |
| 2 | Shake no | -- |
| 5-5-5-5 | Enter TRACKING | Return to MANUAL |
| 5-5-5-6 | Enter AUTOFIRE | Enter AUTOFIRE |

---

## Motion Detection Algorithm

The ESP32-CAM uses **block-based grayscale frame differencing**:

1. Capture a JPEG frame at QVGA (320x240)
2. Decode to RGB888, then convert and downsample to 160x120 grayscale in a single pass
3. Divide the 160x120 frame into a 16x12 grid of 10x10 pixel blocks (192 blocks total)
4. Compare each block's mean absolute pixel difference against the reference frame
5. A block is "active" if its mean difference exceeds `threshold`
6. Motion is confirmed when the number of active blocks exceeds `min_blocks`
7. Compute the weighted centroid of all active blocks (weighted by difference magnitude)
8. Convert centroid position to normalised error (-100 to +100) relative to frame centre

This runs at **12-20 FPS** using only ~38 KB of PSRAM for the two grayscale frame buffers -- a dramatic improvement over the old face detection approach which needed ~500 KB+ for neural network weights and ran at 1-2 FPS.

---

## ESP32-CAM Configuration

All parameters are tunable at runtime via HTTP -- no reflashing needed.

```bash
# View current config
curl http://<ESP32_IP>/config

# Adjust motion sensitivity (lower threshold = more sensitive)
curl "http://<ESP32_IP>/config?threshold=20"

# Require more active blocks before tracking (reduces false positives)
curl "http://<ESP32_IP>/config?min_blocks=5"

# Adjust settling time after turret moves (more frames = more stable, slower response)
curl "http://<ESP32_IP>/config?settle_frames=4"

# Correct camera-to-barrel misalignment (pixels; tune until turret centres on target)
curl "http://<ESP32_IP>/config?offset_x=-12&offset_y=5"

# Flip image if camera is mounted upside-down or mirrored
curl "http://<ESP32_IP>/config?hmirror=1&vflip=1"

# Disable / enable tracking
curl "http://<ESP32_IP>/config?enabled=0"
curl "http://<ESP32_IP>/config?enabled=1"

# Persist any setting across reboots (add save=1)
curl "http://<ESP32_IP>/config?offset_x=-12&offset_y=5&save=1"
```

### Config parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `threshold` | `25` | Per-pixel difference to count as motion (0-255) |
| `min_blocks` | `3` | Minimum active blocks (out of 192) to confirm motion |
| `settle_frames` | `3` | Frames to discard after turret moves (~200ms at 15 FPS) |
| `enabled` | `1` | Master tracking on/off |
| `hmirror` | `1` | Horizontal mirror |
| `vflip` | `0` | Vertical flip |
| `offset_x` | `0` | Camera-to-barrel X offset in pixels |
| `offset_y` | `0` | Camera-to-barrel Y offset in pixels |
| `manual_mode` | `0` | When `1`, disables auto-tracking and accepts `/move` commands |
| `save` | -- | Pass `1` to persist current config to NVS flash |

### Tuning tips

| Symptom | Fix |
|---------|-----|
| Turret jitters constantly | Increase `threshold` (try 35-50) or `min_blocks` |
| Turret doesn't react | Decrease `threshold` (try 15-20) or `min_blocks` |
| Turret reacts to lighting changes | Increase `threshold` and `min_blocks` |
| Tracking feels laggy | Decrease `settle_frames` to 2 |
| Turret aims slightly off | Tune `offset_x` / `offset_y` via `/config` |
| AUTOFIRE triggers too easily | Increase `AUTOFIRE_AIM_THRESHOLD` or `AUTOFIRE_DWELL_FRAMES` in Nano firmware |

---

## HTTP Endpoints

| Endpoint | Description |
|----------|-------------|
| `http://<IP>/` | **Dashboard** -- live camera feed + tactical console |
| `http://<IP>:81/stream` | Raw MJPEG video stream |
| `http://<IP>/capture` | Single JPEG snapshot |
| `http://<IP>/status` | JSON: tracking stats, errors, detection state |
| `http://<IP>/config` | GET current config / SET parameters |
| `http://<IP>/move?x=N&y=N` | Manual move command (requires `manual_mode=1`) |

### `/status` fields

| Field | Description |
|-------|-------------|
| `frames` | Total frames processed |
| `motion_blocks` | Active blocks in last frame (0-192) |
| `centroid_x` / `centroid_y` | Motion centroid in pixels (160x120 space) |
| `error_x` / `error_y` | Last sent error (+/-100) |
| `state` | Detection state: `scanning`, `settling`, or `watching` |
| `tracking` | Whether tracking is enabled |
| `manual_mode` | Whether manual mode is active |
| `ms_since_cmd` | Milliseconds since last command sent to Nano |
| `detect_ms` | Duration of last detection cycle (ms) |
| `ip` / `rssi` / `heap` | Network and memory info |

---

## Serial Protocol (ESP32-CAM -> Nano)

The ESP32 sends commands over UART at 9600 baud (GPIO 14 TX -> Nano D4 RX):

### Tracking errors

```
X<error_x>Y<error_y>\n
```

- `error_x`, `error_y`: signed integers, range -100 to +100
- Positive `error_x` = target is right of centre -> yaw clockwise
- Positive `error_y` = target is below centre -> pitch down
- `X0Y0\n` = no motion detected (keeps Nano watchdog alive)

### Scan command

```
S\n
```

Sent every ~500ms when no target is present. The Nano executes one slow yaw step in its current scan direction, creating a sweeping patrol pattern. The scan reverses direction every 15 steps and adjusts pitch at each reversal for a raster scan.

### Examples

```
X0Y0\n       no motion -- watchdog keepalive
X-45Y23\n    target left and below centre
X100Y-12\n   target hard right, slightly above centre
S\n          scan step -- Nano pans one increment
```

---

## OTA Updates (after first flash)

Once the ESP32-CAM is running, all future firmware updates are wireless:

```bash
cd esp32cam_firmware
pio run -e esp32cam_ota -t upload
```

If the upload fails: confirm your computer and the ESP32-CAM are on the same WiFi network. The OTA hostname is `turret-cam.local`.

---

## Project Structure

```
turretcam/
+-- README.md
+-- turret.py                        # Interactive Python CLI controller (testing)
+-- esp32cam_firmware/
|   +-- platformio.ini               # PlatformIO build config (USB + OTA envs)
|   +-- include/
|   |   +-- wifi_credentials.h.example
|   |   +-- wifi_credentials.h       # Your WiFi creds (gitignored)
|   +-- src/
|       +-- main.cpp                 # Vision, motion detection, dashboard, OTA
+-- turret_nano_firmware/
|   +-- turret_nano_firmware.ino     # Servo control, IR remote, mode switching
+-- docs/
    +-- WIRING.md                    # Detailed wiring reference
```

---

## License

MIT -- do whatever you want with it.
