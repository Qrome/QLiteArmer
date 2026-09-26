#pragma once
#include <Arduino.h>
#include "config.h"

class PWMDriver {
public:
    // Starts every output at its configured failsafe, including smoothing.
    void begin(const uint8_t* pins, uint8_t count);
    void writeUs(uint8_t ch, uint16_t us);
    // Live control resumes automatically; no throttle-low interlock.
    void writeFromCRSF(uint8_t ch, uint16_t raw, bool activeLink);

private:
    uint8_t outputPins[8] = {0};
    uint8_t outputCount = 0;
    float smoothedUs[8] = {0};
    uint16_t crsfToUs(uint8_t ch, uint16_t raw);
    uint16_t applySmoothing(uint8_t ch, uint16_t targetUs);
};
