#include "config.h"
#include "led.h"
#include "state_machine.h"
#include "detection.h"
#include "bf_msp.h"

SystemState currentState = STATE_BOOT_DETECT;

static uint32_t stateStart = 0;
static uint32_t lastHeartbeat = 0;

// ---------------------------------------------------------
// PWM helper: returns pulse width in µs, or -1 if no signal
// ---------------------------------------------------------
static int readPwmArmValue() {
    uint32_t pw = pulseIn(PWM_ARM_PIN, HIGH, 25000);   // 25ms timeout
    if (pw < PWM_NO_SIGNAL_US) {
        return -1;   // no PWM present
    }
    return (int)pw;  // valid PWM pulse
}

// ---------------------------------------------------------

void enterState(SystemState s) {
    currentState = s;
    stateStart = millis();

    switch (s) {
        case STATE_BOOT_DETECT:   ledBlue();  break;
        case STATE_PRE_ARM_DELAY: ledRed();   break;
        case STATE_ARMED:         ledGreen(); break;
        case STATE_ERROR:         ledOff();   break;
    }
}

// ---------------------------------------------------------

void stateMachineUpdate() {
    uint32_t now = millis();

    switch (currentState) {

        // -------------------------------------------------
        case STATE_BOOT_DETECT:
            detectionPoll();
            if (vtxDetected) {
                enterState(STATE_PRE_ARM_DELAY);
            } else if (now - stateStart > VTX_DETECTION_TIMEOUT_MS) {
                enterState(STATE_ERROR);
            }
            break;

        // -------------------------------------------------
        case STATE_PRE_ARM_DELAY: {

            // Heartbeat (disarmed)
            if (now - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
                lastHeartbeat = now;
                bf_msp_heartbeat(false);
            }

            // Read PWM
            int pwm = readPwmArmValue();

            if (pwm >= 0) {
                // PWM present → PWM controls arming
                if (pwm >= PWM_ARM_THRESHOLD) {
                    enterState(STATE_ARMED);
                }
                // else stay in PRE_ARM_DELAY
            } else {
                // No PWM → fallback to timer-based auto-arming
                if (now - stateStart > PRE_ARM_DELAY_MS) {
                    enterState(STATE_ARMED);
                }
            }
        }
        break;

        // -------------------------------------------------
        case STATE_ARMED: {

            // Heartbeat (armed)
            if (now - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
                lastHeartbeat = now;
                bf_msp_heartbeat(true);
            }

            // Read PWM
            int pwm = readPwmArmValue();

            // If PWM present AND below threshold → disarm
            if (pwm >= 0 && pwm < PWM_ARM_THRESHOLD) {
                enterState(STATE_PRE_ARM_DELAY);
            }
        }
        break;

        // -------------------------------------------------
        case STATE_ERROR:
            ledUpdateErrorFlash();
            break;
    }
}