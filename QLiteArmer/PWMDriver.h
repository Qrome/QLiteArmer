#pragma once
#include <Arduino.h>
#include "config.h"

class PWMDriver {
public:
    void begin(const uint8_t* pins, uint8_t count);
    void writeUs(uint8_t ch, uint16_t us);
    void writeFromCRSF(uint8_t ch, uint16_t raw, bool activeLink);

private:
    uint16_t crsfToUs(uint8_t ch, uint16_t raw);

    // Per‑channel smoothed output (µs)
    float smoothedUs[8] = {
        1500,1500,1500,1500,1500,1500,1500,1500
    };

    uint16_t applySmoothing(uint8_t ch, uint16_t targetUs);
};