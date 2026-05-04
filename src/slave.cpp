#ifdef ROLE_SLAVE
#include "slave.h"
#include "config.h"
#include "system_state.h"
#include "lora_packet.h"
#include <SPI.h>
#include <SX128XLT.h>
#include <WiFi.h>
#include <WebServer.h>

static SX128XLT  LT;
static WebServer server(SLAVE_WIFI_PORT);

static int16_t       lastDeg     = 0;
static uint16_t      lastUs      = 0;
static int16_t       lastRssi    = 0;
static int8_t        lastSnr     = 0;
static unsigned long lastRxTime  = 0;

static const char* commsStateStr() {
    unsigned long age = millis() - lastRxTime;
    if (lastRxTime == 0 || age >= 15000) return "RED";
    if (age >= 5000)                     return "YELLOW";
    return "GREEN";
}

// ─── Web page ───────────────────────────────────────────────────────────────

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
    <div class="label">RSSI</div>
    <div class="value" id="rssi">--</div>
  </div>
  <div class="card">
    <div class="label">SNR</div>
    <div class="value" id="snr">--</div>
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
      document.getElementById('rssi').textContent = d.rssi + ' dBm';
      document.getElementById('snr').textContent  = d.snr  + ' dB';
      document.getElementById('state').textContent = d.state;
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

// ─── HTTP handlers ───────────────────────────────────────────────────────────

static void handleRoot() {
    server.send(200, "text/html", HTML);
}

static void handleData() {
    const char* state = commsStateStr();
    applyState(state[0] == 'G' ? SystemState::GREEN
             : state[0] == 'Y' ? SystemState::YELLOW
             : SystemState::RED);

    char buf[128];
    snprintf(buf, sizeof(buf),
        "{\"deg\":%d,\"us\":%u,\"rssi\":%d,\"snr\":%d,\"state\":\"%s\"}",
        lastDeg, lastUs, lastRssi, lastSnr, state);
    server.send(200, "application/json", buf);
}

// ─── Setup / Loop ────────────────────────────────────────────────────────────

void slaveSetup() {
    WiFi.softAP(SLAVE_WIFI_SSID, SLAVE_WIFI_PASS);
    Serial.printf("[WiFi] AP started  SSID: %s  IP: %s\n",
                  SLAVE_WIFI_SSID, WiFi.softAPIP().toString().c_str());

    server.on("/",     handleRoot);
    server.on("/data", handleData);
    server.begin();
    Serial.println("[HTTP] Server ready");

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    if (!LT.begin(LORA_NSS, LORA_NRESET, LORA_RFBUSY, LORA_DIO1, DEVICE_SX1280)) {
        Serial.println("[LORA] Init failed");
        applyState(SystemState::RED);
        return;
    }
    LT.setupLoRa(LORA_FREQ_HZ, 0, LORA_SF7, LORA_BW_0800, LORA_CR_4_5);
    Serial.println("[LORA] Ready");

    applyState(SystemState::YELLOW);
}

void slaveLoop() {
    server.handleClient();

    uint8_t rxBuf[sizeof(ServoPacket)] = {};
    uint8_t rxLen = LT.receive(rxBuf, sizeof(rxBuf), 100, WAIT_RX);

    if (rxLen >= (uint8_t)sizeof(ServoPacket)) {
        ServoPacket* pkt = reinterpret_cast<ServoPacket*>(rxBuf);
        if (pkt->type == PKT_SERVO) {
            lastDeg    = pkt->deg;
            lastUs     = pkt->us;
            lastRssi   = LT.readPacketRSSI();
            lastSnr    = LT.readPacketSNR();
            lastRxTime = millis();
            Serial.printf("[LORA] RX  deg=%3d  us=%4u  RSSI=%d dBm  SNR=%d dB\n",
                          lastDeg, lastUs, lastRssi, lastSnr);
        }
    }
}

#endif
