#include "config.h"
#include "led.h"
#include "state_machine.h"
#include "detection.h"
#include "bf_msp.h"

SystemState currentState = STATE_BOOT_DETECT;

static uint32_t stateStart    = 0;
static uint32_t lastHeartbeat = 0;

// ---------------------------------------------------------
void enterState(SystemState s) {
    currentState = s;
    stateStart   = millis();

    switch (s) {
        case STATE_BOOT_DETECT:   ledBlue();  break;
        case STATE_PRE_ARM_DELAY: ledRed();   break;
        case STATE_ARMED:         ledGreen(); break;
        case STATE_ERROR:         ledOff();   break;
    }
}

// ---------------------------------------------------------
// armValue   — CRSF raw (172–1811) or 0 if no signal
// linkActive — true when CRSF frames are arriving
// ---------------------------------------------------------
void stateMachineUpdate(uint16_t armValue, bool linkActive) {
    uint32_t now = millis();

    // Convert CRSF raw to microseconds for threshold comparison
    // CRSF 172 = 988µs, CRSF 1811 = 2012µs  (linear)
    uint16_t armUs = 0;
    if (linkActive) {
        armUs = 988 + (uint32_t)(armValue - 172) * (2012 - 988) / (1811 - 172);
    }

    // Determine if a valid PWM signal is present
    bool pwmSignalPresent = linkActive && (armUs > PWM_NO_SIGNAL_US);

    switch (currentState) {

        // -------------------------------------------------
        case STATE_BOOT_DETECT:
            detectionPoll();

            if (vtxDetected) {
                bf_msp_firstbeat_start();
                enterState(STATE_PRE_ARM_DELAY);
            } else if (now - stateStart > VTX_DETECTION_TIMEOUT_MS) {
                enterState(STATE_ERROR);
            }
            break;

        // -------------------------------------------------
        case STATE_PRE_ARM_DELAY: {

            // Send disarmed heartbeat while waiting
            if (now - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
                lastHeartbeat = now;
                bf_msp_heartbeat_update(false);
            }

            if (pwmSignalPresent) {
                // CRSF link is live — use arm channel to control arming
                if (armUs >= PWM_ARM_THRESHOLD) {
                    enterState(STATE_ARMED);
                }
                // else: stay in PRE_ARM_DELAY waiting for arm switch
            } else {
                // No CRSF signal — fall back to timer-based auto-arm
                if (now - stateStart > PRE_ARM_DELAY_MS) {
                    enterState(STATE_ARMED);
                }
            }
        }
        break;

        // -------------------------------------------------
        case STATE_ARMED: {

            // Send armed heartbeat
            if (now - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
                lastHeartbeat = now;
                bf_msp_heartbeat_update(true);
            }

            // If CRSF link is live and arm switch goes low → disarm
            if (pwmSignalPresent && armUs < PWM_ARM_THRESHOLD) {
                enterState(STATE_PRE_ARM_DELAY);
            }
            // If signal was present but is now lost, stay armed
            // (failsafe handled by PWMDriver outputting failsafe values)
        }
        break;

        // -------------------------------------------------
        case STATE_ERROR:
            ledUpdateErrorFlash();
            break;
    }
}