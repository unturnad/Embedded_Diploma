#ifdef ROLE_MASTER
#include "master.h"
#include "config.h"
#include "system_state.h"
#include "lora_packet.h"
#include <SPI.h>
#include <Wire.h>
#include <SX128XLT.h>
#include <ESP32Servo.h>

static SX128XLT LT;
static Servo    servo;
static bool     ahtOk = false;

static unsigned long servoTimer  = 0;
static unsigned long txTimer     = 0;
static bool servoAtRest          = true;
static uint8_t     sensorFails    = 0;
static uint8_t     txFails        = 0;
static uint8_t     masterCode     = MASTER_OK;   // current MASTER_* code
static SystemState currentState   = SystemState::YELLOW;

static const int  SERVO_REST_DEG             = 0;
static const int  SERVO_MOVE_DEG             = 45;
static const unsigned long SERVO_INTERVAL_MS = 3000;
static const unsigned long SERVO_HOLD_MS     = 1500;
static const unsigned long TX_INTERVAL_MS    = 1000;

static const uint8_t FAILS_YELLOW = 2;
static const uint8_t FAILS_RED    = 5;

// ─── AHT10 bare I2C ───────────────────────────────────────────────────────────
#define AHT10_ADDR 0x38

static bool aht10Init() {
    // Confirm device ACKs its address (same check as I2C scanner)
    Wire.beginTransmission(AHT10_ADDR);
    if (Wire.endTransmission() != 0) return false;

    // Soft reset
    Wire.beginTransmission(AHT10_ADDR);
    Wire.write(0xBA);
    Wire.endTransmission();
    delay(20);

    // Calibration command (0xBE = broader compatibility than 0xE1)
    Wire.beginTransmission(AHT10_ADDR);
    Wire.write(0xBE);
    Wire.write(0x08);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(300);   // datasheet: wait for calibration to complete

    return true;
}

static bool aht10Read(float &temp, float &hum) {
    // Trigger measurement
    Wire.beginTransmission(AHT10_ADDR);
    Wire.write(0xAC);
    Wire.write(0x33);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    delay(80);

    if (Wire.requestFrom((uint8_t)AHT10_ADDR, (uint8_t)6) != 6) return false;
    uint8_t d[6];
    for (auto &b : d) b = Wire.read();

    if (d[0] & 0x80) return false;   // busy bit still set

    uint32_t rawHum  = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
    uint32_t rawTemp = ((uint32_t)(d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];

    hum  = (rawHum  / 1048576.0f) * 100.0f;
    temp = (rawTemp / 1048576.0f) * 200.0f - 50.0f;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────

static void updateState(bool sensorOk, bool txOk) {
    uint8_t prev = masterCode;

    if (!ahtOk) {
        masterCode = MASTER_NO_SENSOR;
    } else if (!servo.attached()) {
        masterCode = MASTER_NO_SERVO;
    } else {
        sensorFails = sensorOk ? 0 : (uint8_t)(sensorFails + 1);
        txFails     = txOk     ? 0 : (uint8_t)(txFails     + 1);

        if      (sensorFails >= FAILS_RED)    masterCode = MASTER_SENSOR_FAIL;
        else if (txFails     >= FAILS_RED)    masterCode = MASTER_TX_FAIL;
        else if (sensorFails >= FAILS_YELLOW) masterCode = MASTER_SENSOR_WARN;
        else if (txFails     >= FAILS_YELLOW) masterCode = MASTER_TX_WARN;
        else                                  masterCode = MASTER_OK;
    }

    // Derive LED state from code
    currentState = (masterCode == MASTER_OK)                          ? SystemState::GREEN
                 : (masterCode == MASTER_SENSOR_WARN ||
                    masterCode == MASTER_TX_WARN)                     ? SystemState::YELLOW
                 : SystemState::RED;

    if (masterCode != prev) {
        static const char* names[] = {
            "OK", "Sensor warn", "TX warn",
            "No sensor", "No servo", "Sensor fail", "TX fail"
        };
        Serial.printf("[STATE] %s\n", names[masterCode < 7 ? masterCode : 0]);
    }

    applyState(currentState);
}

static void logServo(int deg) {
    int us = servo.readMicroseconds();
    Serial.printf("[SERVO] %3d deg  |  %4d us\n", deg, us);
}

static void sendTelemetry() {
    ServoPacket pkt;
    pkt.type        = PKT_SERVO;
    pkt.magic       = PKT_MAGIC;
    pkt.deg         = (int16_t)(servoAtRest ? SERVO_REST_DEG : SERVO_MOVE_DEG);
    pkt.us          = (uint16_t)servo.readMicroseconds();
    pkt.masterState = masterCode;

    float temp = 0, hum = 0;
    bool sensorOk = ahtOk && aht10Read(temp, hum);

    if (sensorOk) {
        pkt.tempC10  = (int16_t)(temp * 10.0f);
        pkt.humPct10 = (uint16_t)(hum  * 10.0f);
        Serial.printf("[AHT10] %.1f C  %.1f%%\n", temp, hum);
    } else {
        pkt.tempC10  = 0;
        pkt.humPct10 = 0;
        if (ahtOk) Serial.println("[AHT10] Read failed");
    }

    uint8_t len = LT.transmit((uint8_t*)&pkt, sizeof(pkt), 1000, LORA_TX_POWER, WAIT_TX);
    bool txOk = (len > 0);
    if (!txOk) Serial.println("[LORA] TX error");

    updateState(sensorOk, txOk);
}

void masterSetup() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    ahtOk = aht10Init();
    Serial.printf("[AHT10] %s\n", ahtOk ? "Ready" : "Init failed");

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

    // State is GREEN only if AHT10 init succeeded; first TX will confirm fully
    applyState(ahtOk ? SystemState::YELLOW : SystemState::RED);
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
