#pragma once

enum SystemState {
    STATE_BOOT_DETECT,
    STATE_PRE_ARM_DELAY,
    STATE_ARMED,
    STATE_ERROR
};

extern SystemState currentState;

void enterState(SystemState s);
void stateMachineUpdate(uint16_t armValue);