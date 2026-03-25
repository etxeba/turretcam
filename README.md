# HackPack Turret Tracker

A self-contained motion-tracking upgrade for the [CrunchLabs Hack Pack](https://www.crunchlabs.com/) turret. An ESP32-CAM handles vision and sends proportional tracking errors to the Arduino Nano over a one-way UART link. The Nano owns all servo state and translates errors into movement.

**No external computer required** — the turret tracks and fires autonomously once powered on.

---

## Architecture

```
Breakout board (USB-C power)
        │ 5V + GND
        ▼
┌───────────────────────────────────────────────┐
│                    TURRET                     │
│                                               │
│  ┌──────────────┐   GPIO14 TX   ┌──────────┐  │
│  │  ESP32-CAM   │──────────────▶│  Arduino │  │
│  │              │   (one-way)   │  Nano    │  │
│  │  • Camera    │               │          │  │
│  │  • Motion    │               │  • Yaw   │  │
│  │    detection │               │  • Pitch │  │
│  │  • WiFi      │               │  • Roll  │  │
│  │  • OTA       │               │  • IR RX │  │
│  └──────────────┘               └──────────┘  │
└───────────────────────────────────────────────┘
```

The ESP32-CAM sends `X<error>Y<error>\n` proportional error commands each frame. The Nano applies them to the yaw (continuous rotation) and pitch (positional) servos. The Nano also handles the IR remote and all three operating modes.

---

## Bill of Materials

| Item | ~Price | Notes |
|------|--------|-------|
| [CrunchLabs Hack Pack turret](https://www.crunchlabs.com/pages/ir-turret-landing) | $66 | Includes Nano, servos, IR remote |
| [ESP32-CAM (AI Thinker)](https://www.amazon.com/s?k=esp32-cam+ai+thinker) | $10–14 | With OV2640 camera module |
| FTDI USB-to-TTL adapter (FT232RL) | ~$7 | One-time initial flash only; OTA after that |
| Female-to-female jumper wires | ~$5 | 3 wires needed permanently |
| **Total** | **~$95** | |

---

## Wiring

See `docs/WIRING.md` for the full reference with diagrams. Summary:

### Permanent wiring (3 wires, stays on the turret)

| ESP32-CAM | Breakout board | Purpose |
|-----------|---------------|---------|
| **5V** | **5V** | Power from breakout board |
| **GND** | **GND** | Common ground |
| **GPIO 14** | **Nano RX (D0)** | Tracking error commands (TX only) |

> The ESP32-CAM is powered from the same breakout board that powers the Nano. Your USB-C supply must support at least **1 A** to cover both boards (~600 mA peak combined).

### FTDI wiring (one-time flash only)

| FTDI | ESP32-CAM | Notes |
|------|-----------|-------|
| 5V | 5V | Power during flash |
| GND | GND | Ground |
| TX | U0R (RX0) | Cross TX→RX |
| RX | U0T (TX0) | Cross RX→TX |
| — | GPIO 0 → GND | Bridge to enter flash mode |

---

## Setup

### 1 — Install Arduino IDE and board support

1. Download [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. Open **File → Preferences** and add to "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools → Board → Boards Manager** → search `esp32` → install **esp32 by Espressif Systems**
4. Install libraries via **Tools → Manage Libraries**:
   - `IRremote` by Armin Joachimsmeyer (Nano)
   - `Servo` (Nano, usually pre-installed)

### 2 — Flash the Arduino Nano

1. Open `turret_nano_firmware/turret_nano_firmware.ino` in Arduino IDE
2. Select board: **Tools → Board → Arduino AVR Boards → Arduino Nano**
3. Select the Nano's USB-C serial port
4. Click **Upload**

No code changes needed — WiFi credentials and servo config live only on the ESP32.

### 3 — Flash the ESP32-CAM (initial, wired)

1. Open `esp32cam_firmware/esp32cam_firmware.ino`
2. Set your WiFi credentials:
   ```cpp
   const char* ssid     = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
3. Wire the FTDI adapter to the ESP32-CAM (see wiring table above)
4. **Bridge GPIO 0 to GND** on the ESP32-CAM
5. Select board: **Tools → Board → esp32 → AI Thinker ESP32-CAM**
6. Select the FTDI's serial port
7. Click **Upload**; when the IDE shows `Connecting...` press the **RST** button once
8. After upload: **remove the GPIO 0 bridge**, then press **RST** to boot normally
9. Open Serial Monitor at **115200 baud** — you should see the IP address printed

All future firmware updates can be done wirelessly via OTA (see below).

### 4 — Assemble and test

1. Connect the 3 permanent wires (5V, GND, GPIO 14)
2. Power on via USB-C to the breakout board
3. Open the live stream: `http://<ESP32_IP>:81/stream`
4. Wave your hand in front of the camera — check that `error_x` / `error_y` respond at `http://<ESP32_IP>/status`
5. Activate tracking mode on the Nano (see Modes below) and confirm the turret follows

---

## Modes

The Nano has three operating modes, toggled with the IR remote. Both sequences share the `5-5-5` prefix — the 4th button decides which mode activates.

| Sequence | Transition | Effect |
|----------|-----------|--------|
| **5 → 5 → 5 → 5** | MANUAL → TRACKING | Camera controls the turret |
| **5 → 5 → 5 → 5** | TRACKING → MANUAL | Returns to manual control |
| **5 → 5 → 5 → 6** | any → AUTOFIRE | Camera tracks and fires automatically |
| **5 → 5 → 5 → 5** | AUTOFIRE → MANUAL | Disengages auto-fire |

The turret homes its servos on every mode change as physical confirmation.

### MANUAL (default)
- IR remote controls movement, fire, and gestures
- Serial commands accepted over USB: `L`, `R`, `U`, `D`, `F`, `H`, `Y`, `N` (each followed by newline)

### TRACKING
- ESP32-CAM sends proportional X/Y error commands; Nano drives servos
- IR remote and serial movement commands are ignored
- Yaw stops automatically if the ESP32 goes silent for >250 ms (watchdog)

### AUTOFIRE
- Same as TRACKING, plus: fires continuously while the target is within the aim threshold
- Stops firing immediately when the lock is lost
- Dwell guard prevents firing as the turret sweeps through the aim point

#### Auto-fire tuning (in `turret_nano_firmware.ino`)

| Constant | Default | Effect |
|----------|---------|--------|
| `AUTOFIRE_AIM_THRESHOLD` | `15` | How tight the aim must be to fire (±% of half-frame; lower = harder to trigger) |
| `AUTOFIRE_DWELL_FRAMES` | `3` | Consecutive on-target frames required before firing (~100 ms at 30 fps) |

---

## IR Remote Reference

| Button | MANUAL mode | TRACKING / AUTOFIRE |
|--------|------------|---------------------|
| ▲ | Pitch up | — |
| ▼ | Pitch down | — |
| ◀ | Yaw left | — |
| ▶ | Yaw right | — |
| OK | Fire (single dart) | — |
| ★ | Fire all darts | — |
| 1 | Shake yes | — |
| 2 | Shake no | — |
| 5-5-5-5 | Enter TRACKING | Return to MANUAL |
| 5-5-5-6 | Enter AUTOFIRE | Enter AUTOFIRE |

---

## Tracking gains (Nano)

The Nano converts the ESP32's ±100 error values into servo commands using integer division — no floating-point library needed.

| Constant | Default | Effect |
|----------|---------|--------|
| `YAW_DIV` | `2` | `error / YAW_DIV` = yaw speed offset (lower = faster yaw tracking) |
| `PITCH_DIV` | `20` | `error / PITCH_DIV` = pitch step in degrees (lower = faster pitch tracking) |

---

## ESP32-CAM configuration

All parameters are tunable at runtime via HTTP — no reflashing needed.

```bash
# View current config
curl http://<ESP32_IP>/config

# Adjust motion sensitivity (lower threshold = more sensitive)
curl "http://<ESP32_IP>/config?threshold=20"

# Require more changed pixels before tracking (reduces false positives)
curl "http://<ESP32_IP>/config?min_pixels=100"

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
| `threshold` | `25` | Pixel-diff value that counts as motion |
| `min_pixels` | `50` | Minimum changed pixels to confirm a target |
| `enabled` | `1` | Master tracking on/off |
| `hmirror` | `0` | Horizontal mirror |
| `vflip` | `0` | Vertical flip |
| `offset_x` | `0` | Camera-to-barrel X offset in pixels |
| `offset_y` | `0` | Camera-to-barrel Y offset in pixels |
| `save` | — | Pass `1` to persist current config to NVS |

### Tuning tips

| Symptom | Fix |
|---------|-----|
| Turret jitters constantly | Increase `threshold` (try 35–50) or `min_pixels` |
| Turret doesn't react | Decrease `threshold` (try 15–20) or `min_pixels` |
| Tracking is sluggish | Lower `YAW_DIV` / `PITCH_DIV` in Nano firmware |
| Tracking overshoots | Raise `YAW_DIV` / `PITCH_DIV` in Nano firmware |
| Turret aims slightly off | Tune `offset_x` / `offset_y` via `/config` |
| False triggers from lighting | Increase `threshold` and `min_pixels` |
| AUTOFIRE triggers too easily | Increase `AUTOFIRE_AIM_THRESHOLD` or `AUTOFIRE_DWELL_FRAMES` |

---

## HTTP endpoints

| Endpoint | Description |
|----------|-------------|
| `http://<IP>:81/stream` | Live MJPEG video stream |
| `http://<IP>/capture` | Single JPEG snapshot |
| `http://<IP>/status` | JSON: tracking stats, errors, settling state, WiFi, heap |
| `http://<IP>/config` | GET current config / SET parameters |

### `/status` fields

| Field | Description |
|-------|-------------|
| `frames` | Total frames processed |
| `motion_pixels` | Changed pixels in last frame |
| `target_x` / `target_y` | Motion centroid in pixels |
| `error_x` / `error_y` | Last sent error (±100) |
| `tracking` | Whether tracking is enabled |
| `ms_since_cmd` | Milliseconds since last command sent to Nano |
| `settle_frames_learned` | Adaptive settling: learned average discard count |
| `baseline_diff` | Adaptive settling: learned quiet-frame pixel diff |
| `ip` / `rssi` / `heap` | Network and memory info |

---

## Serial protocol (ESP32-CAM → Nano)

The ESP32 sends one command per frame over UART at 9600 baud:

```
X<error_x>Y<error_y>\n
```

- `error_x`, `error_y`: signed integers, range −100 to +100
- Positive `error_x` = target is right of centre → yaw clockwise
- Positive `error_y` = target is below centre → pitch down
- `X0Y0\n` = no target detected; Nano watchdog stops yaw

Examples:
```
X0Y0\n       no target
X-45Y23\n    target left and below centre
X100Y-12\n   target hard right, slightly above centre
```

---

## OTA updates (after first flash)

Once the ESP32-CAM is running, all future firmware updates are wireless:

1. In Arduino IDE go to **Tools → Port** — you should see `turret-cam` as a network port
2. Select it and click **Upload** — no wires needed

If the network port doesn't appear: confirm your computer and the ESP32-CAM are on the same WiFi network, then restart Arduino IDE.

---

## Project structure

```
turretcam/
├── README.md
├── esp32cam_firmware/
│   └── esp32cam_firmware.ino   # Vision, motion detection, WiFi, OTA, HTTP
├── turret_nano_firmware/
│   └── turret_nano_firmware.ino # Servo control, IR remote, mode switching
└── docs/
    └── WIRING.md               # Detailed wiring reference with diagrams
```

---

## License

MIT — do whatever you want with it.
