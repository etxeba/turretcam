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

// Stop yaw if no tracking command received within this window (ms)
#define TRACK_TIMEOUT_MS 250
unsigned long lastTrackMs = 0;

// ── Auto-fire parameters ──────────────────────────────────────────────────────
#define AUTOFIRE_AIM_THRESHOLD 15
#define AUTOFIRE_DWELL_FRAMES  8   // ~0.5s at 15 FPS from motion detection

// ── Scan parameters ──────────────────────────────────────────────────────────
#define SCAN_YAW_SPEED    30   // slow speed offset (vs 90 for full)
#define SCAN_YAW_PULSE_MS 100  // ms per scan step
#define SCAN_STEPS_PER_SWEEP 15 // steps before reversing direction
bool scanRight       = true;
int  scanStepCount   = 0;
int  scanPitchLevel  = 0;      // cycles through 3 pitch levels at reversals

// ── Tracking state (for serial output) ───────────────────────────────────────
bool wasTracking     = false;   // true if last cam command was a track (not scan)
int  engageCount     = 0;       // number of fire events (kill counter)

// ── Mode ──────────────────────────────────────────────────────────────────────
enum Mode : uint8_t { MANUAL, TRACKING, AUTOFIRE };
Mode currentMode = MANUAL;

// ── Forward declarations ──────────────────────────────────────────────────────
void shakeHeadYes(int moves = 3);
void shakeHeadNo(int moves = 3);
void handleSerialCommands();
void weaponsCheck();
void printModeBanner(Mode m);

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    camSerial.begin(9600);

    yawServo.attach(10);
    pitchServo.attach(11);
    rollServo.attach(12);

    IrReceiver.begin(9, DISABLE_LED_FEEDBACK);

    homeServos();

    Serial.println(F(""));
    Serial.println(F("==================================="));
    Serial.println(F("  TURRET DEFENSE SYSTEM v2.0"));
    Serial.println(F("  Motion-Tracking Sentry"));
    Serial.println(F("==================================="));
    Serial.println(F("[SYS] Servos........OK"));
    Serial.println(F("[SYS] IR Receiver...OK"));
    Serial.println(F("[SYS] Camera Link...OK"));
    Serial.println(F("[SYS] Mode: MANUAL"));
    Serial.println(F("[SYS] Ready. Use IR 5-5-5-5 for"));
    Serial.println(F("      TRACKING or 5-5-5-6 for"));
    Serial.println(F("      AUTOFIRE."));
    Serial.println(F("==================================="));
    Serial.println(F(""));
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.protocol != UNKNOWN) {
            uint8_t irCmd = IrReceiver.decodedIRData.command;

            // ── Mode sequence detector ────────────────────────────────────
            static uint8_t fiveCount = 0;

            if (irCmd == cmd5) {
                if (fiveCount < 3) {
                    fiveCount++;
                } else {
                    // 5555 complete
                    fiveCount = 0;
                    if (currentMode == MANUAL) {
                        currentMode = TRACKING;
                        printModeBanner(TRACKING);
                        weaponsCheck();
                    } else {
                        currentMode = MANUAL;
                        printModeBanner(MANUAL);
                        homeServos();
                    }
                }
            } else if (irCmd == cmd6 && fiveCount == 3) {
                // 5556 complete
                fiveCount   = 0;
                currentMode = AUTOFIRE;
                printModeBanner(AUTOFIRE);
                weaponsCheck();
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
            }
        }
        IrReceiver.resume();
    }

    delay(5);
    handleSerialCommands();

    // Watchdog: stop movement if ESP32 goes silent (tracking modes only)
    if (currentMode != MANUAL && millis() - lastTrackMs > TRACK_TIMEOUT_MS) {
        yawServo.write(yawStopSpeed);
        rollServo.write(rollStopSpeed);
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

// ── Weapons check animation ─────────────────────────────────────────────────
// Quick servo sweep when entering tracking/autofire — looks cool
void weaponsCheck() {
    Serial.println(F("[SYS] Running systems check..."));

    // Quick yaw left-right
    yawServo.write(yawStopSpeed + 60);
    delay(120);
    yawServo.write(yawStopSpeed - 60);
    delay(240);
    yawServo.write(yawStopSpeed + 60);
    delay(120);
    yawServo.write(yawStopSpeed);
    delay(50);

    // Quick pitch up-down
    pitchServo.write(pitchMax - 10);
    delay(200);
    pitchServo.write(pitchMin + 10);
    delay(200);

    // Barrel spin
    rollServo.write(rollStopSpeed + rollMoveSpeed);
    delay(200);
    rollServo.write(rollStopSpeed);
    delay(50);

    homeServos();

    Serial.println(F("[SYS] All systems nominal."));
    Serial.println(F("[SYS] Sentry active.\n"));

    // Reset scan state
    scanRight      = true;
    scanStepCount  = 0;
    scanPitchLevel = 0;
    wasTracking    = false;
}

// ── Mode banner ──────────────────────────────────────────────────────────────
void printModeBanner(Mode m) {
    Serial.println(F(""));
    Serial.println(F("============================="));
    switch (m) {
        case TRACKING:
            Serial.println(F("  TRACKING MODE ACTIVATED"));
            Serial.println(F("  Turret is autonomous."));
            break;
        case AUTOFIRE:
            Serial.println(F("  !!! AUTOFIRE ENGAGED !!!"));
            Serial.println(F("  Weapons hot. Stand clear."));
            break;
        case MANUAL:
            Serial.println(F("  MANUAL MODE"));
            Serial.println(F("  IR remote control active."));
            break;
    }
    Serial.println(F("============================="));
}

// ── Scan step ────────────────────────────────────────────────────────────────
void doScanStep() {
    // One slow yaw step in current direction
    if (scanRight) {
        yawServo.write(yawStopSpeed - SCAN_YAW_SPEED);
    } else {
        yawServo.write(yawStopSpeed + SCAN_YAW_SPEED);
    }
    delay(SCAN_YAW_PULSE_MS);
    yawServo.write(yawStopSpeed);

    scanStepCount++;

    // Print scan direction with visual indicator
    if (scanStepCount % 3 == 0) {  // don't spam every single step
        if (scanRight) {
            Serial.println(F("[SCAN] >>> Sweeping right..."));
        } else {
            Serial.println(F("[SCAN] <<< Sweeping left..."));
        }
    }

    // Reverse direction at end of sweep, adjust pitch for raster scan
    if (scanStepCount >= SCAN_STEPS_PER_SWEEP) {
        scanStepCount = 0;
        scanRight = !scanRight;

        // Cycle pitch through 3 levels: low, mid, high
        scanPitchLevel = (scanPitchLevel + 1) % 3;
        int pitchAngles[] = {85, 100, 115};
        pitchServoVal = pitchAngles[scanPitchLevel];
        pitchServo.write(pitchServoVal);

        Serial.print(F("[SCAN] --- Reversing. Pitch level "));
        Serial.println(scanPitchLevel);
    }
}

// ── Serial command parser ─────────────────────────────────────────────────────
void handleSerialCommands() {

    // ── Camera stream (SoftwareSerial D4) ────────────────────────────────────
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

                if (currentMode == TRACKING || currentMode == AUTOFIRE) {

                    // ── Scan command ─────────────────────────────────────
                    if (camBuf[0] == 'S') {
                        if (wasTracking) {
                            Serial.println(F("[TRACK] Target lost."));
                            Serial.println(F("[SCAN] Resuming patrol...\n"));
                            wasTracking = false;
                            dwellCount  = 0;
                            rollServo.write(rollStopSpeed); // stop firing
                        }
                        doScanStep();
                        lastTrackMs = millis();
                    }

                    // ── Tracking command ─────────────────────────────────
                    else if (camBuf[0] == 'X') {
                        int ex = atoi(camBuf + 1);
                        char* p = camBuf + 1;
                        while (*p && *p != 'Y') p++;
                        if (*p == 'Y') {
                            int ey = atoi(p + 1);
                            lastTrackMs = millis();

                            // First detection after scanning
                            if (!wasTracking && (ex != 0 || ey != 0)) {
                                wasTracking = true;
                                Serial.println(F(""));
                                Serial.println(F("[ALERT] *** MOTION DETECTED ***"));
                            }

                            // Step-based tracking
                            int xSteps = constrain(abs(ex) / 25, 0, 3);
                            int ySteps = constrain(abs(ey) / 25, 0, 3);

                            if (ex > 0)      rightMove(xSteps);
                            else if (ex < 0) leftMove(xSteps);

                            if (ey > 0)      downMove(ySteps);
                            else if (ey < 0) upMove(ySteps);

                            // Tracking serial output
                            if (ex != 0 || ey != 0) {
                                bool locked = (abs(ex) < AUTOFIRE_AIM_THRESHOLD)
                                           && (abs(ey) < AUTOFIRE_AIM_THRESHOLD);
                                if (locked) {
                                    Serial.println(F("[TRACK] ** LOCKED ON **"));
                                } else {
                                    Serial.print(F("[TRACK] Target X:"));
                                    Serial.print(ex);
                                    Serial.print(F(" Y:"));
                                    Serial.print(ey);
                                    if (abs(ex) > 50 || abs(ey) > 50) {
                                        Serial.println(F(" | Intercepting..."));
                                    } else {
                                        Serial.println(F(" | Closing in..."));
                                    }
                                }
                            }

                            // ── Auto-fire logic ──────────────────────────
                            if (currentMode == AUTOFIRE) {
                                bool onTarget = (abs(ex) < AUTOFIRE_AIM_THRESHOLD)
                                             && (abs(ey) < AUTOFIRE_AIM_THRESHOLD);
                                if (onTarget) {
                                    dwellCount++;
                                    if (dwellCount == AUTOFIRE_DWELL_FRAMES) {
                                        Serial.println(F("[FIRE] >>> ENGAGING TARGET <<<"));
                                        engageCount++;
                                    }
                                    if (dwellCount >= AUTOFIRE_DWELL_FRAMES) {
                                        rollServo.write(rollStopSpeed + rollMoveSpeed);
                                    }
                                } else {
                                    if (dwellCount >= AUTOFIRE_DWELL_FRAMES) {
                                        rollServo.write(rollStopSpeed);
                                        Serial.print(F("[FIRE] Cease fire. Engagements: "));
                                        Serial.println(engageCount);
                                    }
                                    dwellCount = 0;
                                }
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
