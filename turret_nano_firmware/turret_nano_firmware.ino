/*
 * MIT License
 * Copyright (c) 2025 Crunchlabs LLC (IRTurret Control Code)
 * Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)
 */

#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>
#include <SoftwareSerial.h>

// ESP32-CAM TX → Nano D4. RX-only; Nano never transmits back.
SoftwareSerial camSerial(4, -1);

// ── IR button codes (NEC protocol) ───────────────────────────────────────────
#define DECODE_NEC
#define left    0x8
#define right   0x5A
#define up      0x18
#define down    0x52
#define ok      0x1C
#define cmd1    0x45
#define cmd2    0x46
#define cmd5    0x40
#define cmd6    0x43
#define star    0x16

// ── Mode sequences ────────────────────────────────────────────────────────────
// 5-5-5-5 : MANUAL → TRACKING  (or any active mode → MANUAL)
// 5-5-5-6 : any mode → AUTOFIRE
// Both share the 5-5-5 prefix; the 4th button decides which sequence fires.

// ── Servos ───────────────────────────────────────────────────────────────────
Servo yawServo;    // continuous rotation — base spin
Servo pitchServo;  // positional — up/down tilt
Servo rollServo;   // continuous rotation — barrel/fire

int pitchServoVal = 100;

// Yaw: write yawStopSpeed ± offset to set speed; 90 = stopped
int yawStopSpeed  = 90;
int yawMoveSpeed  = 90;   // full-step speed offset used by IR remote
int yawPrecision  = 150;  // ms per IR remote step

// Pitch limits and IR step size
int pitchMoveSpeed = 8;
int pitchMax       = 150;
int pitchMin       = 33;

// Roll
int rollStopSpeed = 90;
int rollMoveSpeed = 90;
int rollPrecision = 158;  // ms for ~60° (one dart)

// ── Tracking gains (integer arithmetic, no float library) ────────────────────
#define YAW_DIV   2    // error / YAW_DIV   = yaw speed offset  (±50 max)
#define PITCH_DIV 20   // error / PITCH_DIV = pitch delta deg    (±5 max)

// Stop yaw if no tracking command received within this window (ms)
#define TRACK_TIMEOUT_MS 250
unsigned long lastTrackMs = 0;

// ── Auto-fire parameters ──────────────────────────────────────────────────────
// Target is considered "aimed" when both X and Y errors are within this value
// (out of ±100). 15 ≈ within 15% of half-frame width/height.
#define AUTOFIRE_AIM_THRESHOLD 15
// Must stay on-target for this many consecutive frames before firing,
// to avoid triggering as the turret sweeps through the aim point.
#define AUTOFIRE_DWELL_FRAMES  3
// While on-target the roll servo spins continuously — no per-shot cooldown.
// The servo stops the moment the aim drifts outside the threshold.

// ── Mode ──────────────────────────────────────────────────────────────────────
enum Mode : uint8_t { MANUAL, TRACKING, AUTOFIRE };
Mode currentMode = MANUAL;

// ── Forward declarations ──────────────────────────────────────────────────────
void shakeHeadYes(int moves = 3);
void shakeHeadNo(int moves = 3);
void handleSerialCommands();

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    camSerial.begin(9600);

    yawServo.attach(10);
    pitchServo.attach(11);
    rollServo.attach(12);

    IrReceiver.begin(9, DISABLE_LED_FEEDBACK);

    homeServos();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.protocol != UNKNOWN) {
            uint8_t irCmd = IrReceiver.decodedIRData.command;

            // ── Mode sequence detector ────────────────────────────────────
            // Tracks consecutive cmd5 presses. On the 4th press, the button
            // identity determines which sequence completed:
            //   cmd5 → 5555 sequence  (MANUAL↔TRACKING toggle)
            //   cmd6 → 5556 sequence  (enter AUTOFIRE)
            // Any other button resets the counter without executing an action.
            static uint8_t fiveCount = 0;

            if (irCmd == cmd5) {
                if (fiveCount < 3) {
                    fiveCount++;
                    // Accumulating prefix — don't execute any action yet
                } else {
                    // 5555 complete
                    fiveCount = 0;
                    if (currentMode == MANUAL) {
                        currentMode = TRACKING;
                        Serial.println("MODE:TRACK");
                    } else {
                        currentMode = MANUAL;
                        Serial.println("MODE:MANUAL");
                    }
                    homeServos();
                }
            } else if (irCmd == cmd6 && fiveCount == 3) {
                // 5556 complete
                fiveCount   = 0;
                currentMode = AUTOFIRE;
                homeServos();
                Serial.println("MODE:AUTOFIRE");
            } else {
                fiveCount = 0;

                // ── Manual-mode IR commands ───────────────────────────────
                if (currentMode == MANUAL) {
                    switch (irCmd) {
                        case up:    upMove(1);            break;
                        case down:  downMove(1);          break;
                        case left:  leftMove(1);          break;
                        case right: rightMove(1);         break;
                        case ok:    fire();               break;
                        case star:  fireAll(); delay(50); break;
                        case cmd1:  shakeHeadYes(3);      break;
                        case cmd2:  shakeHeadNo(3);       break;
                    }
                }
                // In TRACKING / AUTOFIRE all movement/fire IR commands are
                // ignored — the camera has full control.
            }
        }
        IrReceiver.resume();
    }

    delay(5);
    handleSerialCommands();

    // Watchdog: stop movement if ESP32 goes silent (tracking modes only)
    if (currentMode != MANUAL && millis() - lastTrackMs > TRACK_TIMEOUT_MS) {
        yawServo.write(yawStopSpeed);
        rollServo.write(rollStopSpeed); // stop firing if mid-burst
    }
}

// ── Movement functions ────────────────────────────────────────────────────────
void leftMove(int moves) {
    for (int i = 0; i < moves; i++) {
        yawServo.write(yawStopSpeed + yawMoveSpeed);
        delay(yawPrecision);
        yawServo.write(yawStopSpeed);
        delay(5);
    }
}

void rightMove(int moves) {
    for (int i = 0; i < moves; i++) {
        yawServo.write(yawStopSpeed - yawMoveSpeed);
        delay(yawPrecision);
        yawServo.write(yawStopSpeed);
        delay(5);
    }
}

void upMove(int moves) {
    for (int i = 0; i < moves; i++) {
        if ((pitchServoVal + pitchMoveSpeed) < pitchMax) {
            pitchServoVal += pitchMoveSpeed;
            pitchServo.write(pitchServoVal);
            delay(50);
        }
    }
}

void downMove(int moves) {
    for (int i = 0; i < moves; i++) {
        if ((pitchServoVal - pitchMoveSpeed) > pitchMin) {
            pitchServoVal -= pitchMoveSpeed;
            pitchServo.write(pitchServoVal);
            delay(50);
        }
    }
}

void fire() {
    rollServo.write(rollStopSpeed + rollMoveSpeed);
    delay(rollPrecision);
    rollServo.write(rollStopSpeed);
    delay(5);
}

void fireAll() {
    rollServo.write(rollStopSpeed + rollMoveSpeed);
    delay(rollPrecision * 6);
    rollServo.write(rollStopSpeed);
    delay(5);
}

void homeServos() {
    yawServo.write(yawStopSpeed);
    delay(20);
    rollServo.write(rollStopSpeed);
    delay(100);
    pitchServo.write(100);
    delay(100);
    pitchServoVal = 100;
}

void shakeHeadYes(int moves) {
    if ((pitchMax - pitchServoVal) < 15)      pitchServoVal -= 15;
    else if ((pitchServoVal - pitchMin) < 15) pitchServoVal += 15;
    pitchServo.write(pitchServoVal);

    int startAngle = pitchServoVal;
    int nodAngle   = startAngle + 15;

    for (int i = 0; i < moves; i++) {
        for (int a = startAngle; a <= nodAngle; a++) { pitchServo.write(a); delay(7); }
        delay(50);
        for (int a = nodAngle; a >= startAngle; a--) { pitchServo.write(a); delay(7); }
        delay(50);
    }
}

void shakeHeadNo(int moves) {
    for (int i = 0; i < moves; i++) {
        yawServo.write(140); delay(190);
        yawServo.write(yawStopSpeed); delay(50);
        yawServo.write(40);  delay(190);
        yawServo.write(yawStopSpeed); delay(50);
    }
}

// ── Serial command parser ─────────────────────────────────────────────────────
// camSerial (D4) : X<signed_int>Y<signed_int>\n  from ESP32-CAM e.g. "X-45Y23\n"
// Serial    (D0) : single letter + \n            from USB        e.g. "L\n"
//                  L/R = yaw, U/D = pitch, F = fire, H = home, Y/N = gestures
void handleSerialCommands() {

    // ── Camera stream (SoftwareSerial D4) — X/Y tracking data ────────────────
    {
        static char    camBuf[32];
        static uint8_t camPos     = 0;
        static uint8_t dwellCount = 0;

        while (camSerial.available()) {
            char c = camSerial.read();
            if (c == '\n' || c == '\r') {
                if (camPos == 0) continue;
                camBuf[camPos] = '\0';
                camPos = 0;

                if ((currentMode == TRACKING || currentMode == AUTOFIRE) && camBuf[0] == 'X') {
                    int ex = atoi(camBuf + 1);
                    char* p = camBuf + 1;
                    while (*p && *p != 'Y') p++;
                    if (*p == 'Y') {
                        int ey = atoi(p + 1);
                        lastTrackMs = millis();

                        // Yaw: error_x > 0 = target right = clockwise = stop - offset
                        int yawOffset = constrain(ex / YAW_DIV, -90, 90);
                        yawServo.write(yawStopSpeed - yawOffset);

                        // Pitch: error_y > 0 = target below centre = decrease angle
                        int pitchDelta = ey / PITCH_DIV;
                        pitchServoVal = constrain(pitchServoVal - pitchDelta, pitchMin, pitchMax);
                        pitchServo.write(pitchServoVal);

                        // ── Auto-fire logic ───────────────────────────────
                        if (currentMode == AUTOFIRE) {
                            bool onTarget = (ex > -AUTOFIRE_AIM_THRESHOLD && ex < AUTOFIRE_AIM_THRESHOLD)
                                         && (ey > -AUTOFIRE_AIM_THRESHOLD && ey < AUTOFIRE_AIM_THRESHOLD);
                            if (onTarget) {
                                dwellCount++;
                                if (dwellCount >= AUTOFIRE_DWELL_FRAMES) {
                                    rollServo.write(rollStopSpeed + rollMoveSpeed);
                                }
                            } else {
                                dwellCount = 0;
                                rollServo.write(rollStopSpeed);
                            }
                        }
                    }
                }
            } else if (camPos < sizeof(camBuf) - 1) {
                camBuf[camPos++] = c;
            }
        }
    }

    // ── USB serial (hardware Serial D0) — manual letter commands ─────────────
    {
        static char    usbBuf[32];
        static uint8_t usbPos = 0;

        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (usbPos == 0) continue;
                usbBuf[usbPos] = '\0';
                usbPos = 0;

                if (currentMode == MANUAL) {
                    switch (usbBuf[0]) {
                        case 'L': case 'l': leftMove(1);    break;
                        case 'R': case 'r': rightMove(1);   break;
                        case 'U': case 'u': upMove(1);      break;
                        case 'D': case 'd': downMove(1);    break;
                        case 'F': case 'f': fire();         break;
                        case 'H': case 'h': homeServos();   break;
                        case 'Y': case 'y': shakeHeadYes(); break;
                        case 'N': case 'n': shakeHeadNo();  break;
                    }
                }
            } else if (usbPos < sizeof(usbBuf) - 1) {
                usbBuf[usbPos++] = c;
            }
        }
    }
}
