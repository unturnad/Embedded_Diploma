#pragma once
#include <stdint.h>

#define PKT_SERVO  0x01
#define PKT_MAGIC  0xEB32   // filters out noise / false CRC passes at 2.4 GHz

struct __attribute__((packed)) ServoPacket {
    uint8_t  type;       // PKT_SERVO
    uint16_t magic;      // PKT_MAGIC
    int16_t  deg;
    uint16_t us;
    int16_t  tempC10;    // °C × 10  e.g. 235 = 23.5 °C
    uint16_t humPct10;   // %  × 10  e.g. 456 = 45.6 %
};
