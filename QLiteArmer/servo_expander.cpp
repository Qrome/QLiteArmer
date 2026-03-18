#include <Arduino.h>
#include "config.h"
#include "servo_expander.h"

#include "hardware/pwm.h"
#include "hardware/clocks.h"

// -------- Input capture (ELRS PWM on SERVO_IN_PIN) --------

static volatile uint32_t isrLastRiseUs   = 0;
static volatile uint32_t isrLastPeriodUs = 20000;  // default ~50 Hz
static volatile uint32_t isrLastPulseUs  = 1500;   // default mid
static volatile bool     isrHasSample    = false;

void servoInputISR() {
    uint32_t now = micros();
    bool level   = digitalRead(SERVO_IN_PIN);

    if (level) {
        // Rising edge: measure period (rise-to-rise)
        uint32_t period = now - isrLastRiseUs;
        isrLastRiseUs   = now;

        // Basic sanity: ignore insane periods
        if (period >= 2000 && period <= 30000) {
            isrLastPeriodUs = period;
        }
    } else {
        // Falling edge: measure pulse width
        uint32_t pulse = now - isrLastRiseUs;

        // Basic sanity: ignore insane pulses
        if (pulse >= 800 && pulse <= 2200) {
            isrLastPulseUs = pulse;
            isrHasSample   = true;
        }
    }
}

// -------- Output (hardware PWM on SERVO_OUT_PIN) --------

static uint sliceNum;
static uint channel;
static float pwmClkDiv = 64.0f;  // reasonable default divider

void servoExpanderInit() {
    // Input pin: ELRS PWM, use pulldown to avoid floating edges
    pinMode(SERVO_IN_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(SERVO_IN_PIN), servoInputISR, CHANGE);

    // Output pin: hardware PWM
    gpio_set_function(SERVO_OUT_PIN, GPIO_FUNC_PWM);
    sliceNum = pwm_gpio_to_slice_num(SERVO_OUT_PIN);
    channel  = pwm_gpio_to_channel(SERVO_OUT_PIN);

    // Configure PWM clock divider
    pwm_set_clkdiv(sliceNum, pwmClkDiv);

    // Start with a safe default period and pulse
    uint32_t sysClkHz   = clock_get_hz(clk_sys);
    float    ticksPerUs = (sysClkHz / pwmClkDiv) / 1000000.0f;

    uint32_t defaultPeriodUs = 20000; // 50 Hz
    uint32_t wrap            = (uint32_t)(defaultPeriodUs * ticksPerUs);
    if (wrap > 0xFFFF) wrap = 0xFFFF;

    pwm_set_wrap(sliceNum, wrap);

    uint32_t defaultPulseUs = 1500;
    uint32_t level          = (uint32_t)(defaultPulseUs * ticksPerUs);
    if (level > wrap) level = wrap;

    pwm_set_chan_level(sliceNum, channel, level);
    pwm_set_enabled(sliceNum, true);
}

void servoExpanderUpdate() {
    // If we’ve never seen a valid sample yet, do nothing
    if (!isrHasSample) return;

    // Snapshot volatile values
    uint32_t inPulseUs  = isrLastPulseUs;
    uint32_t inPeriodUs = isrLastPeriodUs;

    // Validate again in main context
    if (inPulseUs < 800 || inPulseUs > 2200)   return;
    if (inPeriodUs < 2000 || inPeriodUs > 30000) return;

    // Map input → output pulse width
    uint32_t outPulseUs = map(inPulseUs,
                              SERVO_IN_MIN_US, SERVO_IN_MAX_US,
                              SERVO_OUT_MIN_US, SERVO_OUT_MAX_US);

    // Clamp
    if (outPulseUs < SERVO_OUT_MIN_US) outPulseUs = SERVO_OUT_MIN_US;
    if (outPulseUs > SERVO_OUT_MAX_US) outPulseUs = SERVO_OUT_MAX_US;

    // Convert to PWM ticks
    uint32_t sysClkHz   = clock_get_hz(clk_sys);
    float    ticksPerUs = (sysClkHz / pwmClkDiv) / 1000000.0f;

    uint32_t wrap  = (uint32_t)(inPeriodUs * ticksPerUs);
    if (wrap > 0xFFFF) wrap = 0xFFFF;
    if (wrap < 1000)   wrap = 1000;  // avoid degenerate cases

    uint32_t level = (uint32_t)(outPulseUs * ticksPerUs);
    if (level > wrap) level = wrap;

    pwm_set_wrap(sliceNum, wrap);
    pwm_set_chan_level(sliceNum, channel, level);
}