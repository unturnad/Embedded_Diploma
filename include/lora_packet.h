#pragma once
#include <stdint.h>

#define PKT_SERVO  0x01
#define PKT_MAGIC  0xEB32

// masterState codes — 0 = GREEN, 1-2 = YELLOW, 3+ = RED
#define MASTER_OK           0   // all systems nominal
#define MASTER_SENSOR_WARN  1   // YELLOW: AHT10 intermittent
#define MASTER_TX_WARN      2   // YELLOW: LoRa TX intermittent
#define MASTER_NO_SENSOR    3   // RED: AHT10 not initialized
#define MASTER_NO_SERVO     4   // RED: servo detached
#define MASTER_SENSOR_FAIL  5   // RED: AHT10 completely failed
#define MASTER_TX_FAIL      6   // RED: LoRa TX completely failed

struct __attribute__((packed)) ServoPacket {
    uint8_t  type;         // PKT_SERVO
    uint16_t magic;        // PKT_MAGIC
    int16_t  deg;
    uint16_t us;
    int16_t  tempC10;      // °C × 10
    uint16_t humPct10;     // %  × 10
    uint8_t  masterState;  // one of the MASTER_* codes above
};
