#ifdef ROLE_MASTER
#include "master.h"
#include "config.h"
#include "system_state.h"
#include <ESP32Servo.h>

static Servo servo;

// Servo oscillation timing
static unsigned long servoTimer   = 0;
static bool          servoAtRest  = true;

static const int  SERVO_REST_DEG  = 0;
static const int  SERVO_MOVE_DEG  = 45;
static const unsigned long SERVO_INTERVAL_MS  = 10000; // wait between sweeps
static const unsigned long SERVO_HOLD_MS      = 500;   // hold at moved position

void masterSetup() {
    servo.attach(SERVO_PIN);
    servo.write(SERVO_REST_DEG);
    servoTimer = millis();

    applyState(SystemState::GREEN);  // hardware init succeeded
}

static void logServo(int deg) {
    int us = servo.readMicroseconds();
    Serial.printf("[SERVO] %3d deg  |  %4d us\n", deg, us);
}

void masterLoop() {
    unsigned long now = millis();

    if (servoAtRest) {
        if (now - servoTimer >= SERVO_INTERVAL_MS) {
            servo.write(SERVO_MOVE_DEG);
            logServo(SERVO_MOVE_DEG);
            servoAtRest = false;
            servoTimer  = now;
        }
    } else {
        if (now - servoTimer >= SERVO_HOLD_MS) {
            servo.write(SERVO_REST_DEG);
            logServo(SERVO_REST_DEG);
            servoAtRest = true;
            servoTimer  = now;
        }
    }
}

#endif
