#ifdef ROLE_MASTER
#include "master.h"
#include "config.h"
#include "system_state.h"
#include "lora_packet.h"
#include <SPI.h>
#include <Wire.h>
#include <SX128XLT.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_S  15   // reset if main loop stalls longer than this

static SX128XLT LT;
static Servo    servo;
static bool     ahtOk       = false;
static bool     i2cBusFault = false;

static unsigned long servoTimer    = 0;
static unsigned long txTimer       = 0;
static unsigned long recoveryTimer = 0;
static bool servoAtRest            = true;
static uint8_t sensorFails         = 0;
static uint8_t txFails             = 0;
static uint8_t masterCode          = MASTER_OK;
static SystemState currentState    = SystemState::YELLOW;

static const int  SERVO_REST_DEG             = 0;
static const int  SERVO_MOVE_DEG             = 45;
static const unsigned long SERVO_INTERVAL_MS = 3000;
static const unsigned long SERVO_HOLD_MS     = 1500;
static const unsigned long TX_INTERVAL_MS    = 1000;
static const unsigned long RECOVERY_INTERVAL_MS = 10000;

static const uint8_t FAILS_YELLOW = 2;
static const uint8_t FAILS_RED    = 5;

// ─── AHT10 bare I2C ───────────────────────────────────────────────────────────
#define AHT10_ADDR 0x38

// 9-clock I2C bus recovery — releases any slave holding SDA low.
static void recoverI2CBus() {
    Serial.println("[I2C] Bus stuck — running 9-clock recovery...");
    Wire.end();
    delay(5);

    pinMode(I2C_SCL, OUTPUT);
    pinMode(I2C_SDA, INPUT);           // pull-up holds SDA; don't fight a stuck slave

    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
        if (digitalRead(I2C_SDA)) {
            Serial.printf("[I2C] SDA released after %d clocks\n", i + 1);
            break;
        }
    }

    // STOP condition: SDA LOW → HIGH while SCL HIGH
    pinMode(I2C_SDA, OUTPUT);
    digitalWrite(I2C_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH); delayMicroseconds(5);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    delay(10);
    Serial.println("[I2C] Bus recovery complete");
}

static bool aht10Init() {
    Wire.beginTransmission(AHT10_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err == 4) { i2cBusFault = true;  return false; }  // bus stuck (not just NACK)
    if (err != 0) { i2cBusFault = false; return false; }  // sensor absent
    i2cBusFault = false;

    Wire.beginTransmission(AHT10_ADDR);
    Wire.write(0xBA);
    Wire.endTransmission();
    delay(20);

    Wire.beginTransmission(AHT10_ADDR);
    Wire.write(0xBE); Wire.write(0x08); Wire.write(0x00);
    Wire.endTransmission();
    delay(300);

    return true;
}

static bool aht10Read(float &temp, float &hum) {
    Wire.beginTransmission(AHT10_ADDR);
    Wire.write(0xAC); Wire.write(0x33); Wire.write(0x00);
    uint8_t err = Wire.endTransmission();
    if (err == 4) { i2cBusFault = true;  return false; }
    if (err != 0) { i2cBusFault = false; return false; }
    delay(80);

    if (Wire.requestFrom((uint8_t)AHT10_ADDR, (uint8_t)6) != 6) return false;
    uint8_t d[6];
    for (auto &b : d) b = Wire.read();
    if (d[0] & 0x80) return false;

    uint32_t rawHum  = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | (d[3] >> 4);
    uint32_t rawTemp = ((uint32_t)(d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | d[5];
    hum  = (rawHum  / 1048576.0f) * 100.0f;
    temp = (rawTemp / 1048576.0f) * 200.0f - 50.0f;
    return true;
}

static bool loraInit() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    if (!LT.begin(LORA_NSS, LORA_NRESET, LORA_RFBUSY, LORA_DIO1, DEVICE_SX1280))
        return false;
    LT.setupLoRa(LORA_FREQ_HZ, 0, LORA_SF7, LORA_BW_0800, LORA_CR_4_5);
    return true;
}

// ─── Recovery ─────────────────────────────────────────────────────────────────
// Retries RED-level subsystems every RECOVERY_INTERVAL_MS without rebooting.

static void tryRecovery() {
    unsigned long now = millis();
    if (now - recoveryTimer < RECOVERY_INTERVAL_MS) return;
    recoveryTimer = now;

    if (masterCode == MASTER_I2C_FAULT) {
        recoverI2CBus();
        i2cBusFault = false;
        ahtOk = aht10Init();
        if (ahtOk) { sensorFails = 0; Serial.println("[RECOVERY] I2C + AHT10 restored"); }
        else          Serial.println("[RECOVERY] I2C recovered but AHT10 still absent");
    } else if (masterCode == MASTER_SENSOR_FAIL || masterCode == MASTER_NO_SENSOR) {
        Serial.println("[RECOVERY] Retrying AHT10...");
        ahtOk = aht10Init();
        if (ahtOk) { sensorFails = 0; Serial.println("[RECOVERY] AHT10 restored"); }
        else          Serial.println("[RECOVERY] AHT10 still unavailable");
    }

    if (masterCode == MASTER_TX_FAIL) {
        Serial.println("[RECOVERY] Retrying LoRa TX...");
        if (loraInit()) { txFails = 0; Serial.println("[RECOVERY] LoRa TX restored"); }
        else              Serial.println("[RECOVERY] LoRa TX still unavailable");
    }
}

// ─── State machine ────────────────────────────────────────────────────────────

static void updateState(bool sensorOk, bool txOk) {
    uint8_t prev = masterCode;

    if (i2cBusFault) {
        masterCode = MASTER_I2C_FAULT;
    } else if (!ahtOk) {
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

    currentState = (masterCode == MASTER_OK)                      ? SystemState::GREEN
                 : (masterCode == MASTER_SENSOR_WARN ||
                    masterCode == MASTER_TX_WARN)                 ? SystemState::YELLOW
                 : SystemState::RED;

    if (masterCode != prev) {
        static const char* names[] = {
            "OK", "Sensor warn", "TX warn",
            "No sensor", "No servo", "Sensor fail", "TX fail", "I2C bus fault"
        };
        Serial.printf("[STATE] %s\n", names[masterCode < 8 ? masterCode : 0]);
    }

    applyState(currentState);

    if (currentState == SystemState::RED) tryRecovery();
}

// ─────────────────────────────────────────────────────────────────────────────

static void logServo(int deg) {
    Serial.printf("[SERVO] %3d deg  |  %4d us\n", deg, servo.readMicroseconds());
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

// ─── Public API ───────────────────────────────────────────────────────────────

void masterSetup() {
    esp_task_wdt_init(WDT_TIMEOUT_S, true);  // panic-reset if loop stalls
    esp_task_wdt_add(NULL);                   // subscribe Arduino loop task

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    ahtOk = aht10Init();
    Serial.printf("[AHT10] %s\n", ahtOk ? "Ready" : "Not found");

    if (!loraInit()) {
        Serial.println("[LORA] Init failed");
        applyState(SystemState::RED);
        return;
    }
    Serial.println("[LORA] Ready");

    servo.attach(SERVO_PIN);
    servo.write(SERVO_REST_DEG);
    servoTimer = millis();
    txTimer    = millis();

    applyState(ahtOk ? SystemState::YELLOW : SystemState::RED);
}

#ifdef I2C_FAULT_DEMO
static void runI2CFaultDemo() {
    static unsigned long demoTimer = 0;
    static bool          faultLive = false;
    unsigned long        now       = millis();

    if (!faultLive && now - demoTimer >= 30000) {
        Wire.end();          // physically stop the I2C bus peripheral
        i2cBusFault = true;
        ahtOk       = false;
        faultLive   = true;
        Serial.println("[DEMO] I2C bus stopped — waiting for auto-recovery...");
    }
    if (faultLive && ahtOk && !i2cBusFault) {
        faultLive = false;
        demoTimer = millis();
    }
}
#endif

void masterLoop() {
    esp_task_wdt_reset();

#ifdef I2C_FAULT_DEMO
    runI2CFaultDemo();
#endif

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
