#pragma once

// ─── LoRa SX1280 (SPI) ─────────────────────────────────────────────────────
// SPI bus: MOSI=23, MISO=19, SCK=18 (ESP32 VSPI defaults)
#define LORA_NSS     15
#define LORA_NRESET  27
#define LORA_RFBUSY  33
#define LORA_DIO1    35   // IRQ (input-only pin — fine for interrupt)

#define LORA_FREQ_HZ   2400000000UL
#define LORA_TX_POWER  10     // dBm
#define LORA_TIMEOUT   5000   // ms — packet timeout → state degrades

// ─── Status LEDs — active-low (anode→3.3V, cathode→GPIO) ──────────────────
// LOW = ON, HIGH = OFF
#define LED_RED     4
#define LED_YELLOW  21
#define LED_GREEN   5
#define LED_BLUE    26   // power indicator, always on

// ─── Master-only pins ───────────────────────────────────────────────────────
#define SERVO_PIN  25
#define TEMP_PIN   14   // DS18B20 OneWire data line (TBD — confirm before wiring)

// ─── Slave-only ─────────────────────────────────────────────────────────────
#define SLAVE_WIFI_SSID  "RoboController"
#define SLAVE_WIFI_PASS  "12345678"
#define SLAVE_WIFI_PORT  80
