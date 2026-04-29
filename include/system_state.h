#pragma once
#include <Arduino.h>
#include "config.h"

enum class SystemState {
    GREEN,   // all nominal
    YELLOW,  // degraded — sensor anomaly or LoRa intermittent
    RED      // critical — LoRa lost or fatal sensor failure
};

// LEDs are active-low: LOW = on, HIGH = off
inline void applyState(SystemState s) {
    digitalWrite(LED_RED,    s == SystemState::RED    ? LOW : HIGH);
    digitalWrite(LED_YELLOW, s == SystemState::YELLOW ? LOW : HIGH);
    digitalWrite(LED_GREEN,  s == SystemState::GREEN  ? LOW : HIGH);
}

inline void initStateLEDs() {
    pinMode(LED_RED,    OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN,  OUTPUT);
    pinMode(LED_BLUE,   OUTPUT);

    digitalWrite(LED_BLUE, LOW);   // always on — power indicator

    applyState(SystemState::YELLOW);  // start YELLOW until comms established
}
