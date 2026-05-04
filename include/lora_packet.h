#pragma once
#include <stdint.h>

#define PKT_SERVO  0x01

struct __attribute__((packed)) ServoPacket {
    uint8_t  type;
    int16_t  deg;
    uint16_t us;
};
