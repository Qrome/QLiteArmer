#include "PWMDriver.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

void PWMDriver::begin(const uint8_t* pins, uint8_t count) {
    outputCount = pins ? (count > 8 ? 8 : count) : 0;
    uint32_t sliceMask = 0;

    // Paired GPIOs share a slice. Stop all owned slices before configuring
    // either output, and keep the pins low until safe compare values exist.
    for (uint8_t i = 0; i < outputCount; ++i) {
        outputPins[i] = pins[i];
        uint slice = pwm_gpio_to_slice_num(pins[i]);
        pwm_set_enabled(slice, false);
        sliceMask |= 1u << slice;
        gpio_init(pins[i]);
        gpio_put(pins[i], false);
        gpio_set_dir(pins[i], GPIO_OUT);
    }

    pwm_config cfg = pwm_get_default_config();
    // 1 MHz counter: one count per microsecond, 20000 counts per frame.
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 19999);
    for (uint slice = 0; slice < 8; ++slice) {
        if (sliceMask & (1u << slice)) pwm_init(slice, &cfg, false);
    }

    for (uint8_t i = 0; i < outputCount; ++i) {
        smoothedUs[i] = CH_MAP[i].failsafeUs;
        pwm_set_gpio_level(outputPins[i], CH_MAP[i].failsafeUs);
    }
    for (uint8_t i = 0; i < outputCount; ++i) {
        gpio_set_function(outputPins[i], GPIO_FUNC_PWM);
    }
    for (uint slice = 0; slice < 8; ++slice) {
        if (sliceMask & (1u << slice)) pwm_set_enabled(slice, true);
    }
}

uint16_t PWMDriver::crsfToUs(uint8_t ch, uint16_t raw) {
    const uint16_t inMin = 172;
    const uint16_t inMax = 1811;
    if (raw < inMin) raw = inMin;
    if (raw > inMax) raw = inMax;
    int32_t span = (int32_t)CH_MAP[ch].maxUs - CH_MAP[ch].minUs;
    return (uint16_t)(CH_MAP[ch].minUs +
        (int32_t)(raw - inMin) * span / (inMax - inMin));
}

uint16_t PWMDriver::applySmoothing(uint8_t ch, uint16_t targetUs) {
    const float alpha = 0.25f;
    smoothedUs[ch] += alpha * (targetUs - smoothedUs[ch]);
    return (uint16_t)smoothedUs[ch];
}

void PWMDriver::writeUs(uint8_t ch, uint16_t us) {
    if (ch >= outputCount) return;
    pwm_set_gpio_level(outputPins[ch], us);
}

void PWMDriver::writeFromCRSF(uint8_t ch, uint16_t raw, bool activeLink) {
    if (ch >= outputCount) return;
    uint16_t us;
    if (!activeLink) {
        us = CH_MAP[ch].failsafeUs;
        // Prevent the pre-failsafe throttle value returning on reconnection.
        smoothedUs[ch] = us;
    } else {
        us = applySmoothing(ch, crsfToUs(ch, raw));
    }
    writeUs(ch, us);
}
