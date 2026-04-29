#include <Arduino.h>
#include "config.h"
#include "system_state.h"

#if !defined(ROLE_MASTER) && !defined(ROLE_SLAVE)
  #error "No role defined. Use -D ROLE_MASTER or -D ROLE_SLAVE in platformio.ini"
#endif

#ifdef ROLE_MASTER
  #include "master.h"
#endif
#ifdef ROLE_SLAVE
  #include "slave.h"
#endif

void setup() {
    Serial.begin(115200);
    initStateLEDs();

#ifdef ROLE_MASTER
    Serial.println("[MASTER] Booting...");
    masterSetup();
#endif
#ifdef ROLE_SLAVE
    Serial.println("[SLAVE] Booting...");
    slaveSetup();
#endif
}

void loop() {
#ifdef ROLE_MASTER
    masterLoop();
#endif
#ifdef ROLE_SLAVE
    slaveLoop();
#endif
}
