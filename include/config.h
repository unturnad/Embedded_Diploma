#pragma once

// ─── LoRa SX1280 (custom SPI bus) ──────────────────────────────────────────
#define LORA_MISO    19
#define LORA_MOSI    23
#define LORA_SCK     18
#define LORA_NSS     12
#define LORA_NRESET  13
#define LORA_RFBUSY  14
#define LORA_DIO1    27   // IRQ

#define LORA_FREQ_HZ   2400000000UL
#define LORA_TX_POWER  10     // dBm
#define LORA_TIMEOUT   5000   // ms — packet timeout → state degrades

// ─── Status LEDs — active-low (anode→3.3V, cathode→GPIO) ──────────────────
// LOW = ON, HIGH = OFF
#define LED_RED     2
#define LED_YELLOW  4
#define LED_GREEN   5
#define LED_BLUE    26   // power indicator, always on

// ─── Master-only pins ───────────────────────────────────────────────────────
#define SERVO_PIN   25
#define I2C_SDA     21   // AHT10 SDA
#define I2C_SCL     22   // AHT10 SCL

// ─── Temperature thresholds ─────────────────────────────────────────────────
#define TEMP_WARN_C   30.0f   // ≥ this → YELLOW "Temperature high"
#define TEMP_CRIT_C   32.0f   // ≥ this → RED    "Temperature critical"

// ─── Slave-only ─────────────────────────────────────────────────────────────
#define SLAVE_WIFI_SSID  "RoboController"
#define SLAVE_WIFI_PASS  "12345678"
#define SLAVE_WIFI_PORT  80
