#pragma once

enum SystemState {
    STATE_BOOT_DETECT,
    STATE_PRE_ARM_DELAY,
    STATE_ARMED,
    STATE_ERROR
};

extern SystemState currentState;

void enterState(SystemState s);

// armValue    — raw CRSF channel value (172–1811 range)
// linkActive  — true if CRSF link is alive
void stateMachineUpdate(uint16_t armValue, bool linkActive);