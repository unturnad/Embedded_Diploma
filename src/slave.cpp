#ifdef ROLE_SLAVE
#include "slave.h"
#include "config.h"
#include "system_state.h"

void slaveSetup() {
    // TODO: init LoRa, WiFi AP, web server
}

void slaveLoop() {
    // TODO: receive telemetry, serve web page, send servo commands
}

#endif
