/*
  HD_Amer — Auto‑Arming MSP Transmitter Firmware
  by David Payne (Qromer)
  Target MCU: Waveshare RP2040‑Zero
*/

#include <Arduino.h>
#include "Telemetry.h"
#include "config.h"
#include "led.h"
#include "state_machine.h"
#include "detection.h"
#include "bf_msp.h"
#include "CrossfireELRS.h"
#include "PWMDriver.h"

CrossfireELRS crsf;
PWMDriver pwm;
Telemetry telemetry;

// Shared across cores — volatile for safety
volatile bool systemReady = false;

inline void debugPrint(const String &msg) {
    if (Serial) {
        Serial.println(msg);
    }
}

// -------------------------------------------------------
// Core 0 — RC input + PWM output (time-critical)
// -------------------------------------------------------
void setup() {
    Serial.begin(420000);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 1500)) {
        delay(10);
    }

    debugPrint("Starting up QLiteArmer...");

    // CRSF receiver
    crsf.begin(PIN_CRSF_RX, PIN_CRSF_TX);

    // PWM servo outputs
    pwm.begin(PWM_PINS, 8);

    delay(500);  // let CRSF stabilise first

    systemReady = true;
}

void loop() {
    // CRSF receiver — time sensitive, must run every loop
    crsf.update();

    // PWM outputs from CRSF channels
    for (int i = 0; i < 8; i++) {
        pwm.writeFromCRSF(i, crsf.getChannel(i), crsf.crsfLinkActive);
    }

    bf_msp_parse_incoming();       // always — reads and responds to polls
    bf_msp_firstbeat_update();     // always — no-op after firstbeat completes
    bf_msp_heartbeat_update((currentState == STATE_ARMED)); // always — skips DP heartbeat for V1
    bf_msp_dp_update_osd_nb();     // always — no-op for V1, active for O3/Avatar

    // Add to loop() temporarily for diagnosis
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 2000) {
        lastDebugMs = millis();
        VtxSystemType t = bf_msp_get_vtx_type();
        Serial.print("[DEBUG] VTX type detected: ");
        switch(t) {
            case VTX_UNKNOWN:   Serial.println("UNKNOWN"); break;
            case VTX_DJI_V1:    Serial.println("DJI V1");  break;
            case VTX_DJI_O3:    Serial.println("DJI O3/O4"); break;
            case VTX_WALKSNAIL: Serial.println("Walksnail Avatar"); break;
        }
    }
}

// -------------------------------------------------------
// Core 1 — MSP arming + telemetry (background)
// -------------------------------------------------------
void setup1() {
    // Wait for Core 0 to finish its setup
    while (!systemReady) {
        delay(10);
    }

    // MSP UART to VTX
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(MSP_BAUD);

    // Telemetry (BMP280 + ADC)
    telemetry.begin();

    // MSP subsystem
    bf_msp_init(Serial1, telemetry);

    // LED
    ledInit();

    // VTX detection parser
    detectionInit();

    // Start state machine
    enterState(STATE_BOOT_DETECT);

    debugPrint("Core 1 ready.");
}

void loop1() {
    telemetry.update();

    // Read arm channel — detect no-signal via PWM_NO_SIGNAL_US threshold
    uint16_t armRaw = crsf.getChannel(PWM_ARM_CHANNEL);
    stateMachineUpdate(armRaw, crsf.crsfLinkActive);
}


