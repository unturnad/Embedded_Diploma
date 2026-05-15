#ifdef ROLE_SLAVE
#include "slave.h"
#include "config.h"
#include "system_state.h"
#include "lora_packet.h"
#include <SPI.h>
#include <SX128XLT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <esp_task_wdt.h>

static SX128XLT  LT;
static WebServer server(SLAVE_WIFI_PORT);

// ── Shared telemetry ─────────────────────────────────────────────────────────
// Written from Core 0 (LoRa task), read from Core 1 (web + loop).
// All ≤32-bit fields are atomically read/written on Xtensa LX6.
static volatile int16_t       lastDeg      = 0;
static volatile uint16_t      lastUs       = 0;
static volatile int16_t       lastRssi     = 0;
static volatile int8_t        lastSnr      = 0;
static volatile int16_t       lastTempC10    = 0;   // °C × 10
static volatile uint16_t      lastHumPct10   = 0;   // %  × 10
static volatile uint8_t       lastMasterState = MASTER_OK;
static volatile unsigned long lastRxTime      = 0;
static unsigned long          startTime    = 0;   // set at boot, baseline before first packet

// Free-space path loss at 2.4 GHz: d = 10^((TxPower - RSSI - 40.05) / 20) m
static float rssiToDistance(int16_t rssi) {
    return powf(10.0f, ((float)LORA_TX_POWER - (float)rssi - 40.05f) / 20.0f);
}

// Human-readable reason for the current state — set by getOverallState()
static const char* stateReason = "Initialising";

// Maps MASTER_* code → { SystemState, description }
struct MasterInfo { SystemState s; const char* reason; };
static const MasterInfo MASTER_MAP[] = {
    { SystemState::GREEN,  "All systems OK"       },  // MASTER_OK
    { SystemState::YELLOW, "Sensor intermittent"  },  // MASTER_SENSOR_WARN
    { SystemState::YELLOW, "TX intermittent"      },  // MASTER_TX_WARN
    { SystemState::RED,    "Sensor not found"     },  // MASTER_NO_SENSOR
    { SystemState::RED,    "Servo detached"       },  // MASTER_NO_SERVO
    { SystemState::RED,    "Sensor failed"        },  // MASTER_SENSOR_FAIL
    { SystemState::RED,    "TX failed"            },  // MASTER_TX_FAIL
    { SystemState::RED,    "I2C bus fault"        },  // MASTER_I2C_FAULT
};

static SystemState getOverallState() {
    // ── Link health ──────────────────────────────────────────────────────────
    unsigned long ref = (lastRxTime == 0) ? startTime : (unsigned long)lastRxTime;
    unsigned long age = millis() - ref;

    static unsigned long dbgTimer = 0;
    if (millis() - dbgTimer >= 2000) {
        Serial.printf("[STATE] link_age=%lums  master_code=%u\n", age, (unsigned)lastMasterState);
        dbgTimer = millis();
    }

    SystemState linkState;
    const char*  linkReason;
    if      (age < 3000) { linkState = SystemState::GREEN;  linkReason = nullptr;          }
    else if (age < 8000) { linkState = SystemState::YELLOW; linkReason = "Link degraded";  }
    else                 { linkState = SystemState::RED;    linkReason = "Link lost";       }

    // ── Master health ────────────────────────────────────────────────────────
    uint8_t code = (lastMasterState < 8) ? (uint8_t)lastMasterState : MASTER_OK;
    SystemState masterState  = MASTER_MAP[code].s;
    const char* masterReason = MASTER_MAP[code].reason;

    // ── Combine: worst wins; link problem takes display priority ─────────────
    if (linkState == SystemState::RED || masterState == SystemState::RED) {
        stateReason = (linkState == SystemState::RED) ? linkReason : masterReason;
        return SystemState::RED;
    }
    if (linkState == SystemState::YELLOW || masterState == SystemState::YELLOW) {
        stateReason = (linkState == SystemState::YELLOW) ? linkReason : masterReason;
        return SystemState::YELLOW;
    }
    stateReason = masterReason;   // "All systems OK"
    return SystemState::GREEN;
}

// ── LoRa receive task — Core 0 ───────────────────────────────────────────────
// receive() is a busy-wait spin on DIO1; isolating it to Core 0 keeps the
// web server and WiFi stack on Core 1 fully responsive at all times.

static void loraRxTask(void*) {
    // IDLE0 never runs since we busy-wait here — remove from WDT, add this task instead
    esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
    esp_task_wdt_add(NULL);   // WDT now monitors lora_rx; resets every receive() cycle

    for (;;) {
        uint8_t rxBuf[sizeof(ServoPacket)] = {};
        uint8_t rxLen = LT.receive(rxBuf, sizeof(rxBuf), 5000, WAIT_RX);
        esp_task_wdt_reset();   // receive() returned (packet or 5 s timeout) — feed WDT

        if (rxLen >= (uint8_t)sizeof(ServoPacket)) {
            auto* pkt = reinterpret_cast<ServoPacket*>(rxBuf);
            if (pkt->type == PKT_SERVO && pkt->magic == PKT_MAGIC) {
                lastDeg      = pkt->deg;
                lastUs       = pkt->us;
                lastTempC10    = pkt->tempC10;
                lastHumPct10   = pkt->humPct10;
                lastMasterState = pkt->masterState;
                lastRssi       = LT.readPacketRSSI();
                lastSnr      = LT.readPacketSNR();
                lastRxTime   = millis();

                Serial.printf("[LORA] RX  deg=%3d  us=%4u  %.1fC  %.1f%%  "
                              "RSSI=%d dBm  SNR=%d dB  dist~%.1f m\n",
                              (int)lastDeg, (unsigned)lastUs,
                              lastTempC10 / 10.0f, lastHumPct10 / 10.0f,
                              (int)lastRssi, (int)lastSnr,
                              rssiToDistance(lastRssi));
            }
        }
    }
}

// ── Web page ─────────────────────────────────────────────────────────────────

static const char HTML[] = R"html(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>RoboController Telemetry</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:monospace;background:#1a1a2e;color:#e0e0e0;padding:24px}
  h1{color:#00d4ff;margin-bottom:20px;font-size:1.4em}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:480px}
  .card{background:#16213e;border-radius:8px;padding:16px}
  .wide{grid-column:span 2}
  .label{color:#888;font-size:.75em;text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px}
  .value{font-size:1.9em;font-weight:bold}
  .status{display:flex;align-items:center;gap:10px;background:#16213e;
          border-radius:8px;padding:16px;max-width:480px;margin-top:10px}
  .dot{width:14px;height:14px;border-radius:50%;flex-shrink:0}
  .GREEN{background:#00ff88} .YELLOW{background:#ffd700} .RED{background:#ff4444}
</style>
</head>
<body>
<h1>RoboController Telemetry</h1>
<div class="grid">
  <div class="card">
    <div class="label">Servo Angle</div>
    <div class="value" id="deg">--</div>
  </div>
  <div class="card">
    <div class="label">Servo PWM</div>
    <div class="value" id="us">--</div>
  </div>
  <div class="card">
    <div class="label">Temperature</div>
    <div class="value" id="temp">--</div>
  </div>
  <div class="card">
    <div class="label">Humidity</div>
    <div class="value" id="hum">--</div>
  </div>
  <div class="card">
    <div class="label">RSSI</div>
    <div class="value" id="rssi">--</div>
  </div>
  <div class="card">
    <div class="label">SNR</div>
    <div class="value" id="snr">--</div>
  </div>
  <div class="card wide">
    <div class="label">Distance (RSSI estimate)</div>
    <div class="value" id="dist">--</div>
  </div>
</div>
<div class="status">
  <div class="dot" id="dot"></div>
  <span id="state">Waiting for signal...</span>
</div>
<script>
function poll() {
  fetch('/data')
    .then(r => r.json())
    .then(d => {
      document.getElementById('deg').textContent  = d.deg  + ' deg';
      document.getElementById('us').textContent   = d.us   + ' us';
      document.getElementById('temp').textContent = d.temp + ' C';
      document.getElementById('hum').textContent  = d.hum  + ' %';
      document.getElementById('rssi').textContent = d.rssi + ' dBm';
      document.getElementById('snr').textContent  = d.snr  + ' dB';
      document.getElementById('dist').textContent = d.dist + ' m';
      document.getElementById('state').textContent = d.reason;
      document.getElementById('dot').className = 'dot ' + d.state;
    })
    .catch(() => {});
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
)html";

// ── HTTP handlers ─────────────────────────────────────────────────────────────

static void handleRoot() {
    server.send(200, "text/html", HTML);
}

static void handleData() {
    SystemState s = getOverallState();
    applyState(s);

    const char* stateStr;
    switch (s) {
        case SystemState::GREEN:  stateStr = "GREEN";  break;
        case SystemState::YELLOW: stateStr = "YELLOW"; break;
        default:                  stateStr = "RED";    break;
    }

    char distStr[12], tempStr[10], humStr[10];
    snprintf(distStr, sizeof(distStr), "%.1f", rssiToDistance(lastRssi));
    snprintf(tempStr, sizeof(tempStr), "%.1f", lastTempC10  / 10.0f);
    snprintf(humStr,  sizeof(humStr),  "%.1f", lastHumPct10 / 10.0f);

    char buf[260];
    snprintf(buf, sizeof(buf),
        "{\"deg\":%d,\"us\":%u,\"temp\":%s,\"hum\":%s,"
        "\"rssi\":%d,\"snr\":%d,\"dist\":%s,\"state\":\"%s\",\"reason\":\"%s\"}",
        (int)lastDeg, (unsigned)lastUs, tempStr, humStr,
        (int)lastRssi, (int)lastSnr, distStr, stateStr, stateReason);

    server.send(200, "application/json", buf);
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void slaveSetup() {
    esp_task_wdt_init(15, true);  // 15 s timeout, panic-reset on trigger
    esp_task_wdt_add(NULL);        // subscribe Arduino loop task (Core 1)

    startTime = millis();

    WiFi.softAP(SLAVE_WIFI_SSID, SLAVE_WIFI_PASS);
    Serial.printf("[WiFi] AP  SSID: %s  IP: %s\n",
                  SLAVE_WIFI_SSID, WiFi.softAPIP().toString().c_str());

    server.on("/",     handleRoot);
    server.on("/data", handleData);
    server.begin();
    Serial.println("[HTTP] Ready");

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    if (!LT.begin(LORA_NSS, LORA_NRESET, LORA_RFBUSY, LORA_DIO1, DEVICE_SX1280)) {
        Serial.println("[LORA] Init failed");
        applyState(SystemState::RED);
        return;
    }
    LT.setupLoRa(LORA_FREQ_HZ, 0, LORA_SF7, LORA_BW_0800, LORA_CR_4_5);
    Serial.println("[LORA] Ready");

    // LoRa receive (busy-wait on DIO1) runs on Core 0.
    // Core 1 stays free for web server + WiFi stack.
    xTaskCreatePinnedToCore(loraRxTask, "lora_rx", 4096, nullptr, 1, nullptr, 0);

    applyState(SystemState::YELLOW);
}

void slaveLoop() {
    esp_task_wdt_reset();            // feed watchdog
    applyState(getOverallState());   // LEDs update independently of browser
    server.handleClient();
}

#endif
