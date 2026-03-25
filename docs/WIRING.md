# Wiring Reference — HackPack Turret Tracker

## Overview

Three boards are involved:
- **Arduino Nano** — drives the servos and handles the IR remote
- **ESP32-CAM** (AI Thinker) — handles vision, motion detection, and sends commands
- **FTDI adapter** (FT232RL) — one-time use only for the initial flash; not needed afterwards

> **HackPack users:** the Nano and its 5 V power rail are already on the turret's PCB. The steps below refer to that PCB as the "power board."
> **Other builds:** substitute any regulated 5 V / 1 A supply for the power board references.

---

## 1. Permanent Wiring: ESP32-CAM ↔ Arduino Nano

Connect these wires and leave them on the turret permanently.

| ESP32-CAM pin | Power board / supply | Wire color (suggestion) | Purpose |
|---------------|----------------------|------------------------|---------|
| **5V**        | **5V rail**          | Red                    | Power (shared with Nano)            |
| **GND**       | **GND**              | Black                  | Common ground                        |
| **GPIO 14**   | **Nano RX (D0)**     | Yellow                 | ESP32 TX → Nano RX (servo commands) |

> **Power:** The ESP32-CAM shares the same 5 V supply as the Nano. The supply
> must be able to source at least **600 mA** combined (~500 mA peak for the
> ESP32-CAM + ~50 mA for the Nano + servo load).
>
> **Data flow is one-way:** the ESP32-CAM sends commands to the Nano; the Nano
> sends nothing back.  Only three wires are needed (5V, GND, GPIO 14).
>
> **D0 / Serial conflict:** GPIO 14 connects to the Nano's hardware UART RX
> pin (D0), which is shared with the USB-Serial chip.  **Disconnect the GPIO 14
> wire before uploading new firmware to the Nano or using Serial Monitor**,
> then reconnect it afterwards.

### Diagram

```
5V supply / power board         ESP32-CAM                       Arduino Nano
──────────────────────────────────────────────────────────────────────────────
5V ─────────────────────────▶ 5V
GND ────────────────────────── GND ────────────────────────────── GND
                               GPIO 14 (TX) ──────────────────▶ RX (D0)
```

---

## 2. One-Time Flash Wiring: FTDI Adapter ↔ ESP32-CAM

Use this only when uploading firmware for the first time.
After that, use OTA updates over WiFi.

| FTDI adapter pin | ESP32-CAM pin | Notes |
|-----------------|--------------|-------|
| **5 V**         | **5 V**      | Power the ESP32-CAM from the FTDI |
| **GND**         | **GND**      | Ground |
| **TX**          | **U0R (RX0)**| Cross TX→RX |
| **RX**          | **U0T (TX0)**| Cross RX→TX |
| *(jumper wire)* | **GPIO 0 → GND** | **Short this to enter flash mode** |

### Flash Procedure

1. Wire FTDI to ESP32-CAM as above.
2. **Bridge GPIO 0 to GND** with a jumper wire.
3. Plug FTDI into your computer via USB.
4. In Arduino IDE: select board **AI Thinker ESP32-CAM**, select the FTDI's
   serial port, click **Upload**.
5. When the IDE shows `Connecting...`, press the **RST** button on the
   ESP32-CAM once.
6. Wait for `Done uploading`.
7. **Remove** the GPIO 0 → GND bridge.
8. Press **RST** again to boot normally.
9. Open Serial Monitor at **115200 baud** — you should see the IP address.

---

## 3. Camera Mounting

Mount the ESP32-CAM at the front of the turret so it looks in the same
direction as the barrel.  Adjust `hmirror` / `vflip` at runtime if the image
is backwards or upside-down:

```bash
curl "http://<ESP32_IP>/config?hmirror=1&vflip=1"
```

---

## 4. Power Notes

| Board        | Power source              | Current draw (typical)     |
|--------------|---------------------------|----------------------------|
| ESP32-CAM    | Shared 5 V rail           | ~250 mA idle, ~500 mA peak |
| Arduino Nano | Shared 5 V rail (USB-C)   | ~50 mA + servo load        |
| **Combined** | **Single USB-C supply**   | **~600 mA peak**           |

Make sure your USB-C power supply (or power bank) is rated for at least **1 A**
to handle the combined peak load.  A 5000 mAh bank gives roughly 6–8 hours.

---

## 5. Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Nano receives garbage / nothing | TX wire swapped | Confirm GPIO 14 → Nano RX (D0) |
| Nano upload fails / Serial Monitor garbled | GPIO 14 still connected during upload | Disconnect GPIO 14 wire from D0 before uploading or opening Serial Monitor |
| ESP32-CAM won't enter flash mode | GPIO 0 not grounded | Re-seat jumper before plugging FTDI |
| Servos twitch at startup | Both boards power up at different times | Add a 1-second `delay(1000)` at the start of Nano `setup()` |
| Camera image is mirrored/flipped | Mount orientation | `curl "http://<IP>/config?hmirror=1"` or `?vflip=1` |
| OTA port not visible in IDE | Different WiFi network | Make sure computer and ESP32-CAM are on same SSID |
