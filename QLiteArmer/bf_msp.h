#pragma once
#include <Arduino.h>

// Betaflight 4.4.2 MSP emulator

enum BF_MSPCommand : uint16_t {
    BF_MSP_API_VERSION   = 1,
    BF_MSP_FC_VARIANT    = 2,
    BF_MSP_FC_NAME       = 3,
    BF_MSP_STATUS        = 101,
    BF_MSP_STATUS_EX     = 150,
};

// Initialize with the serial port used for MSP (e.g. Serial1)
void bf_msp_init(HardwareSerial &port);

// Send one full BF-style heartbeat (STATUS + STATUS_EX + identity)
void bf_msp_heartbeat(bool armed);