#pragma once

// ─── LoRa SX1280 (SPI) ─────────────────────────────────────────────────────
// SPI bus: MOSI=23, MISO=19, SCK=18 (ESP32 VSPI defaults)
#define LORA_NSS     5
#define LORA_NRESET  27
#define LORA_RFBUSY  25
#define LORA_DIO1    4    // IRQ

#define LORA_FREQ_HZ   2400000000UL
#define LORA_TX_POWER  10     // dBm
#define LORA_TIMEOUT   5000   // ms — packet timeout → state degrades

// ─── Status LEDs (shared by both roles) ────────────────────────────────────
#define LED_GREEN   32
#define LED_YELLOW  33
#define LED_RED     26

// ─── Master-only pins ───────────────────────────────────────────────────────
#define SERVO_PIN    13
#define TEMP_PIN     14   // DS18B20 OneWire data line

// ─── Slave-only ─────────────────────────────────────────────────────────────
#define SLAVE_WIFI_SSID  "RoboController"
#define SLAVE_WIFI_PASS  "12345678"
#define SLAVE_WIFI_PORT  80
