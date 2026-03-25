/*
 * MIT License
 * Copyright (c) 2025 Crunchlabs LLC (IRTurret Control Code)
 * Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)
 */

#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>

// ── IR button codes (NEC protocol) ───────────────────────────────────────────
#define DECODE_NEC
#define left    0x8
#define right   0x5A
#define up      0x18
#define down    0x52
#define ok      0x1C
#define cmd1    0x45
#define cmd2    0x46
#define star    0x16

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
// yaw:   offset = error / YAW_DIV   (error range -100..100 → offset -50..50)
// pitch: delta  = error / PITCH_DIV (error 100 → 5° delta)
#define YAW_DIV   2
#define PITCH_DIV 20

// Stop yaw if no tracking command received within this window (ms)
#define TRACK_TIMEOUT_MS 250
unsigned long lastTrackMs = 0;

// ── Forward declarations ──────────────────────────────────────────────────────
void shakeHeadYes(int moves = 3);
void shakeHeadNo(int moves = 3);
void handleSerialCommands();

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);

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
            switch (IrReceiver.decodedIRData.command) {
                case up:    upMove(1);        break;
                case down:  downMove(1);      break;
                case left:  leftMove(1);      break;
                case right: rightMove(1);     break;
                case ok:    fire();           break;
                case star:  fireAll(); delay(50); break;
                case cmd1:  shakeHeadYes(3);  break;
                case cmd2:  shakeHeadNo(3);   break;
            }
        }
        IrReceiver.resume();
    }

    delay(5);
    handleSerialCommands();

    // Watchdog: stop yaw if ESP32 goes silent
    if (millis() - lastTrackMs > TRACK_TIMEOUT_MS) {
        yawServo.write(yawStopSpeed);
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
// Expects: X<signed_int>Y<signed_int>\n   e.g. "X-45Y23\n"
// Gains are applied with integer division to avoid the float library.
void handleSerialCommands() {
    static char    buf[32];
    static uint8_t pos = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            buf[pos] = '\0';
            pos = 0;
            if (buf[0] == 'X') {
                int ex = atoi(buf + 1);
                char* p = buf + 1;
                while (*p && *p != 'Y') p++;
                if (*p == 'Y') {
                    int ey = atoi(p + 1);
                    lastTrackMs = millis();

                    // Yaw: proportional speed (continuous servo)
                    // error_x > 0 = target right = clockwise = yawStopSpeed - offset
                    int yawOffset = constrain(ex / YAW_DIV, -90, 90);
                    yawServo.write(yawStopSpeed - yawOffset);

                    // Pitch: proportional delta (positional servo)
                    // error_y > 0 = target below centre = tilt down = decrease angle
                    int pitchDelta = ey / PITCH_DIV;
                    pitchServoVal = constrain(pitchServoVal - pitchDelta, pitchMin, pitchMax);
                    pitchServo.write(pitchServoVal);
                }
            }
        } else if (pos < sizeof(buf) - 1) {
            buf[pos++] = c;
        }
    }
}
