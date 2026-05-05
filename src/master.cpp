#ifdef ROLE_MASTER
#include "master.h"
#include "config.h"
#include "system_state.h"
#include "lora_packet.h"
#include <SPI.h>
#include <Wire.h>
#include <SX128XLT.h>
#include <ESP32Servo.h>
#include <Adafruit_AHTX0.h>

static SX128XLT      LT;
static Servo         servo;
static Adafruit_AHTX0 aht;
static bool          ahtOk = false;

static unsigned long servoTimer = 0;
static unsigned long txTimer    = 0;
static bool servoAtRest         = true;

static const int  SERVO_REST_DEG         = 0;
static const int  SERVO_MOVE_DEG         = 45;
static const unsigned long SERVO_INTERVAL_MS = 3000;
static const unsigned long SERVO_HOLD_MS     = 1500;
static const unsigned long TX_INTERVAL_MS    = 1000;

static void logServo(int deg) {
    int us = servo.readMicroseconds();
    Serial.printf("[SERVO] %3d deg  |  %4d us\n", deg, us);
}

static void sendTelemetry() {
    ServoPacket pkt;
    pkt.type  = PKT_SERVO;
    pkt.magic = PKT_MAGIC;
    pkt.deg   = (int16_t)(servoAtRest ? SERVO_REST_DEG : SERVO_MOVE_DEG);
    pkt.us    = (uint16_t)servo.readMicroseconds();

    if (ahtOk) {
        sensors_event_t hEvent, tEvent;
        aht.getEvent(&hEvent, &tEvent);
        pkt.tempC10  = (int16_t)(tEvent.temperature       * 10.0f);
        pkt.humPct10 = (uint16_t)(hEvent.relative_humidity * 10.0f);
        Serial.printf("[AHT10] %.1f C  %.1f%%\n",
                      tEvent.temperature, hEvent.relative_humidity);
    } else {
        pkt.tempC10  = 0;
        pkt.humPct10 = 0;
    }

    uint8_t len = LT.transmit((uint8_t*)&pkt, sizeof(pkt), 1000, LORA_TX_POWER, WAIT_TX);
    if (len == 0) Serial.println("[LORA] TX error");
}

void masterSetup() {
    Wire.begin(I2C_SDA, I2C_SCL);
    ahtOk = aht.begin(&Wire);
    Serial.printf("[AHT10] %s\n", ahtOk ? "Ready" : "Not found — check wiring/pins");

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    if (!LT.begin(LORA_NSS, LORA_NRESET, LORA_RFBUSY, LORA_DIO1, DEVICE_SX1280)) {
        Serial.println("[LORA] Init failed");
        applyState(SystemState::RED);
        return;
    }
    LT.setupLoRa(LORA_FREQ_HZ, 0, LORA_SF7, LORA_BW_0800, LORA_CR_4_5);
    Serial.println("[LORA] Ready");

    servo.attach(SERVO_PIN);
    servo.write(SERVO_REST_DEG);
    servoTimer = millis();
    txTimer    = millis();

    applyState(SystemState::GREEN);
}

void masterLoop() {
    unsigned long now = millis();

    if (servoAtRest) {
        if (now - servoTimer >= SERVO_INTERVAL_MS) {
            servo.write(SERVO_MOVE_DEG);
            servoAtRest = false;
            servoTimer  = now;
            logServo(SERVO_MOVE_DEG);
        }
    } else {
        if (now - servoTimer >= SERVO_HOLD_MS) {
            servo.write(SERVO_REST_DEG);
            servoAtRest = true;
            servoTimer  = now;
            logServo(SERVO_REST_DEG);
        }
    }

    if (now - txTimer >= TX_INTERVAL_MS) {
        sendTelemetry();
        txTimer = now;
    }
}

#endif
