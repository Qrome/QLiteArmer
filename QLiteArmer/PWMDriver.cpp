#include "PWMDriver.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ======================================================
// Initialization
// ======================================================
void PWMDriver::begin(const uint8_t* pins, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pins[i]);

        uint32_t wrap = 20000; // 20ms = 50Hz
        pwm_set_wrap(slice, wrap);

        float div = (float)clock_get_hz(clk_sys) / (wrap * 50.0f);
        if (div < 1.0f) div = 1.0f;
        if (div > 256.0f) div = 256.0f;

        pwm_set_clkdiv(slice, div);
        pwm_set_enabled(slice, true);

        pwm_set_gpio_level(pins[i], 1500);
        smoothedUs[i] = 1500.0f;
    }
}

// ======================================================
// CRSF → microseconds (per‑channel mapping)
// ======================================================
uint16_t PWMDriver::crsfToUs(uint8_t ch, uint16_t raw) {
    const uint16_t inMin = 172;
    const uint16_t inMax = 1811;

    if (raw < inMin) raw = inMin;
    if (raw > inMax) raw = inMax;

    uint16_t outMin = CH_MAP[ch].minUs;
    uint16_t outMax = CH_MAP[ch].maxUs;

    return outMin + (uint32_t)(raw - inMin) * (outMax - outMin) / (inMax - inMin);
}

// ======================================================
// Exponential smoothing (crisp, receiver‑like)
// ======================================================
uint16_t PWMDriver::applySmoothing(uint8_t ch, uint16_t targetUs) {
    const float alpha = 0.25f;  // higher = crisper, lower = smoother
    smoothedUs[ch] = smoothedUs[ch] + alpha * (targetUs - smoothedUs[ch]);
    return (uint16_t)smoothedUs[ch];
}

// ======================================================
// Write raw microseconds
// ======================================================
void PWMDriver::writeUs(uint8_t ch, uint16_t us) {
    pwm_set_gpio_level(PWM_PINS[ch], us);
}

// ======================================================
// Main CRSF → PWM output path
// ======================================================
void PWMDriver::writeFromCRSF(uint8_t ch, uint16_t raw, bool activeLink) {

    uint16_t us;

    if (!activeLink) {
        // Use failsafe value directly (no smoothing)
        us = CH_MAP[ch].failsafeUs;
    } else {
        // Normal mapped + smoothed output
        us = crsfToUs(ch, raw);
        us = applySmoothing(ch, us);
    }

    writeUs(ch, us);
}
