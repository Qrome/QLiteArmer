/*
  HD_Amer — Auto‑Arming MSP Transmitter Firmware
  by David Payne (Qromer)
  Target MCU: Waveshare RP2040‑Zero
  PRD Version: v2.3

  ---------------------------------------------------------------------------
  OVERVIEW
  ---------------------------------------------------------------------------
  This firmware automatically arms compatible digital video transmitters (DJI
  O3/O4, Vista/AU, Walksnail) by sending Betaflight‑style MSP packets with the
  `armed` flag set to 1. The module performs system‑specific VTX detection,
  waits a configurable pre‑arm delay, then begins transmitting an MSP heartbeat
  at 5–10 Hz. No RC PWM input is used.

  ---------------------------------------------------------------------------
  LED STATE MACHINE (ONBOARD WS2812 RGB)
  ---------------------------------------------------------------------------
    BLUE  (solid)  — BOOT_DETECT
                     Waiting for VTX UART activity.
                     Hybrid MSP detection:
                       * Detect "$M<" header + plausible length.
                       * Attempt full checksum validation.
                       * If checksum fails but framing is valid, still count.
                     If no valid activity within timeout → ERROR.

    RED   (solid)  — PRE_ARM_DELAY
                     VTX detected. Waiting configurable delay before arming.
                     No armed frames sent during this period.

    GREEN (solid)  — ARMED
                     First MSP_STATUS(armed=1) frame has been SENT.
                     MSP heartbeat continues at 5–10 Hz.

    AMBER (rapid)  — ERROR
                     Any of the following:
                       * No VTX activity within detection timeout.
                       * UART TX failures exceed threshold.
                       * Repeated malformed MSP frames.
                       * Unexpected UART behavior.

  ---------------------------------------------------------------------------
  MSP BEHAVIOR (TRUE BETAFLIGHT STYLE)
  ---------------------------------------------------------------------------
    MSP_API_VERSION   — Example: 1.45.0
    MSP_FC_VARIANT    — "BFLT"
    MSP_FC_NAME       — "HD_ARMER"
    MSP_STATUS        — Betaflight layout:
                          cycleTime, i2cErrorCount, sensor, flags,
                          currentSet, pidProfile, rateProfile
                        Armed flag = bit 0 of `flags`.

    Optional:
      MSP_BATTERY_STATE — Dummy payload (disabled by default)

  ---------------------------------------------------------------------------
  CONFIGURABLE PARAMETERS (config.h)
  ---------------------------------------------------------------------------
    MSP_BAUD                    — default 115200
    VTX_DETECTION_TIMEOUT_MS    — default 30000 ms
    PRE_ARM_DELAY_MS            — default 5000 ms
    HEARTBEAT_INTERVAL_MS       — default 150 ms
    TX_FAIL_THRESHOLD           — default 5
    BAD_CHECKSUM_THRESHOLD      — default 3
    LED_PIN / LED_COUNT         — RP2040‑Zero onboard WS2812

  ---------------------------------------------------------------------------
  FILE ARCHITECTURE
  ---------------------------------------------------------------------------
    HD_Amer.ino        — Entry point, setup(), loop()
    state_machine.*    — State transitions + timing logic
    detection.*        — Hybrid MSP detection (Option C)
    msp.*              — Betaflight‑style MSP serialization
    led.*              — RGB LED control + error flashing
    config.h           — All compile‑time configuration

  ---------------------------------------------------------------------------
  NOTES
  ---------------------------------------------------------------------------
    • Designed for reliability on the bench and in the field.
    • No RC receiver or PWM input is used.
    • UART TX → VTX RX, UART RX ← VTX TX, GND ↔ GND.
    • Behavior matches PRD v2.3 exactly.
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
#include "Telemetry.h"
#include "hardware/adc.h"


CrossfireELRS crsf;
PWMDriver pwm;
Telemetry telemetry;

inline void debugPrint(const String &msg) {
  if (Serial) {
      Serial.println(msg);
  }
}

void setup() {
    Serial.begin(420000);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 1500)) {
        delay(10);
    }

    debugPrint("Starting up QLiteArmer...");

    crsf.begin(PIN_CRSF_RX, PIN_CRSF_TX);
    pwm.begin(PWM_PINS, 8);
    telemetry.begin();

    delay(500);  // let CRSF start first

    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(MSP_BAUD);
    bf_msp_init(Serial1, telemetry);

    ledInit();
    detectionInit();

    enterState(STATE_BOOT_DETECT);
}

void loop() {
  crsf.update();  // updates link state + channels when available
  for (int i = 0; i < 8; i++) {
      pwm.writeFromCRSF(i, crsf.getChannel(i), crsf.crsfLinkActive);
  }
}

void loop1() {
  telemetry.update();
  stateMachineUpdate(crsf.getChannel(PWM_ARM_CHANNEL));
}