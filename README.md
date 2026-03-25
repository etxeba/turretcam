# Hack Pack Turret Tracker

A self-contained motion-tracking upgrade for the [CrunchLabs Hack Pack](https://www.crunchlabs.com/) turret by Mark Rober. Uses an ESP32-CAM for vision-based motion detection and sends servo commands to the Hack Pack Arduino Nano over UART.

**No external computer required** — the turret tracks autonomously once powered on.

## Architecture

```
┌─────────────────────────────────────────────┐
│                  TURRET                      │
│                                              │
│  ┌──────────────┐     UART      ┌─────────┐ │
│  │  ESP32-CAM   │──────────────▶│  Arduino │ │
│  │              │  GPIO14→D2    │  Nano    │ │
│  │  • Camera    │  GPIO15←D3    │          │ │
│  │  • Motion    │  GND──GND     │  • Servo │ │
│  │    detection │               │  control │ │
│  │  • WiFi      │               └─────────┘ │
│  │  • OTA       │                            │
│  └──────────────┘                            │
│                                              │
│  ┌──────────────────────────────────────┐    │
│  │        USB Power Bank                │    │
│  └──────────────────────────────────────┘    │
└─────────────────────────────────────────────┘
```

## Bill of Materials

| Item | ~Price | Notes |
|------|--------|-------|
| [CrunchLabs Hack Pack turret](https://www.crunchlabs.com/pages/ir-turret-landing) | $66 | Includes Nano + servos |
| [ESP32-CAM](https://www.amazon.com/Hosyond-ESP32-CAM-Bluetooth-Development-Compatible/dp/B09TB1GJ7P) | $14 | With OV2640 camera |
| FTDI USB-to-TTL adapter (FT232RL) | ~$7 | One-time flash, then OTA |
| Female-to-female jumper wires | ~$5 | For wiring |
| ESP32-CAM case/mount | ~$6 | Optional, for clean mounting |
| **Total** | **~$100** | |

## Wiring

### ESP32-CAM ↔ Arduino Nano (permanent, on turret)

| ESP32-CAM | Arduino Nano | Purpose |
|-----------|-------------|---------|
| GPIO 14 | D2 | ESP32 TX → Nano RX (commands) |
| GPIO 15 | D3 | ESP32 RX ← Nano TX (debug) |
| GND | GND | Common ground |

**Note:** Both boards get power from USB independently. The ESP32-CAM from a power bank, the Nano from the same or a shared source.

### FTDI ↔ ESP32-CAM (one-time flash only)

| FTDI Adapter | ESP32-CAM | Notes |
|-------------|-----------|-------|
| 5V | 5V | Power |
| GND | GND | Ground |
| TX | U0R (RX) | Cross TX→RX |
| RX | U0T (TX) | Cross RX→TX |
| — | GPIO 0 → GND | **Bridge for flash mode** |

## Setup Instructions

### Step 1: Install Arduino IDE + ESP32 Support

1. Download [Arduino IDE 2.x](https://www.arduino.cc/en/software)
2. Open **File → Preferences**
3. Add this to "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to **Tools → Board → Boards Manager**, search "esp32", install **"esp32 by Espressif Systems"**

### Step 2: Flash the ESP32-CAM (one-time)

1. Update WiFi credentials in `esp32cam_firmware/esp32cam_firmware.ino`:
   ```cpp
   const char* ssid     = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

2. Wire FTDI adapter to ESP32-CAM (see wiring table above)

3. **Bridge GPIO 0 to GND** on the ESP32-CAM

4. Plug FTDI into your Mac via USB

5. In Arduino IDE:
   - Board: **Tools → Board → esp32 → AI Thinker ESP32-CAM**
   - Port: Select the FTDI's serial port
   - Partition Scheme: **Huge APP (3MB No OTA)** or **Default**

6. Click **Upload**

7. When IDE says "Connecting...", press the **RST** button on ESP32-CAM

8. After upload completes:
   - **Remove** GPIO 0 to GND bridge
   - Press RST to boot normally

9. Open **Serial Monitor** (115200 baud) — you should see the IP address and endpoints

### Step 3: Flash the Nano (via CrunchLabs)

1. Open the CrunchLabs IDE / programming tool

2. Copy the code from `hackpack_nano/hackpack_nano.ino`

3. **Update servo pin numbers** to match your Hack Pack wiring:
   ```cpp
   #define PAN_SERVO_PIN   9   // change if different
   #define TILT_SERVO_PIN  10  // change if different
   ```

4. Upload to the Nano via USB-C

### Step 4: Wire & Test

1. Connect ESP32-CAM to Nano using the permanent wiring (3 wires)
2. Power on both boards
3. Open the stream in a browser: `http://<ESP32_IP>:81/stream`
4. Wave your hand in front of the camera — the turret should follow!
5. Check status: `http://<ESP32_IP>/status`

## Tuning the Tracker

The tracking behavior can be adjusted in real-time via the `/config` endpoint:

```bash
# View current config
curl http://<ESP32_IP>/config

# Adjust motion sensitivity (lower = more sensitive)
curl "http://<ESP32_IP>/config?threshold=20"

# Adjust tracking speed (higher = faster/twitchier)
curl "http://<ESP32_IP>/config?pan_kp=0.1&tilt_kp=0.1"

# Require more motion to trigger (reduces false positives)
curl "http://<ESP32_IP>/config?min_pixels=100"

# Disable/enable tracking
curl "http://<ESP32_IP>/config?enabled=0"
curl "http://<ESP32_IP>/config?enabled=1"

# Re-center the turret
curl "http://<ESP32_IP>/config?pan_center=90&tilt_center=90"
```

### Tuning Tips

| Problem | Fix |
|---------|-----|
| Turret jitters / twitches | Increase `threshold` (try 40-50), increase `smoothing_frames` |
| Turret doesn't react | Decrease `threshold` (try 15-20), decrease `min_pixels` |
| Tracking is too slow | Increase `pan_kp` / `tilt_kp` (try 0.12-0.15) |
| Tracking overshoots | Decrease `pan_kp` / `tilt_kp` (try 0.04-0.06) |
| False triggers from lighting | Increase `threshold` and `min_pixels` |
| Camera image is upside down | Set `hmirror` and `vflip` to 1 in firmware |

## OTA Updates (after first flash)

Once the ESP32-CAM is running with OTA enabled, future code updates are wireless:

1. In Arduino IDE, go to **Tools → Port**
2. You should see `turret-cam` listed as a network port
3. Select it and upload — no wiring needed!

If the network port doesn't appear:
- Make sure your Mac is on the same WiFi network
- Try restarting the Arduino IDE
- Check that the ESP32-CAM is powered on and connected

## Serial Protocol

The ESP32-CAM sends commands to the Nano in this format:

```
P<pan_angle>T<tilt_angle>\n
```

Examples:
- `P90T90\n` — center position
- `P45T120\n` — pan left, tilt up
- `P135T60\n` — pan right, tilt down

Angles are 0-180, matching standard servo range.

## Debugging

### USB Serial (Nano)
- Connect Nano via USB-C, open Serial Monitor at 115200 baud
- Shows incoming commands and servo positions
- Type `T` to toggle tracking mode on/off

### USB Serial (ESP32-CAM, during development with FTDI)
- Shows WiFi status, tracking stats, servo commands sent
- Frame count, motion pixel count, target coordinates

### HTTP Endpoints
| Endpoint | Description |
|----------|-------------|
| `http://<IP>:81/stream` | Live MJPEG video stream |
| `http://<IP>/capture` | Single JPEG snapshot |
| `http://<IP>/status` | JSON with tracking stats, WiFi info, heap |
| `http://<IP>/config` | GET/SET tracking parameters |

## Project Structure

```
hackpack-turret-tracker/
├── README.md
├── esp32cam_firmware/
│   └── esp32cam_firmware.ino    # ESP32-CAM: camera + motion tracking + servo UART
├── hackpack_nano/
│   └── hackpack_nano.ino        # Arduino Nano: serial listener + servo driver
└── docs/
    └── WIRING.md                # Detailed wiring reference
```

## Future Ideas

- **Color tracking**: Track a specific color instead of general motion
- **Face detection**: Use ESP32's built-in Haar cascade (slower, ~1-2 fps)
- **Pi 5 upgrade**: Stream to Pi for YOLO-based detection + face recognition
- **Multiple turrets**: Coordinate multiple turrets on the same network
- **Nerf integration**: Auto-fire when target is centered
- **Mobile app**: Control and monitor via phone

## License

MIT — do whatever you want with it.