#pragma once
#include <Arduino.h>
#include "config.h"

enum class SystemState {
    GREEN,   // all nominal
    YELLOW,  // degraded — sensor anomaly or LoRa intermittent
    RED      // critical — LoRa lost or fatal sensor failure
};

inline void applyState(SystemState s) {
    digitalWrite(LED_GREEN,  s == SystemState::GREEN  ? HIGH : LOW);
    digitalWrite(LED_YELLOW, s == SystemState::YELLOW ? HIGH : LOW);
    digitalWrite(LED_RED,    s == SystemState::RED    ? HIGH : LOW);
}

inline void initStateLEDs() {
    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED,    OUTPUT);
    applyState(SystemState::YELLOW);  // start in YELLOW until first comms
}
