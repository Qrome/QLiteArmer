#pragma once
#include <Arduino.h>


// ======================================================
// Per‑Channel PWM Mapping (microseconds)
// ======================================================
struct ChannelMap {
    uint16_t minUs;       // mapped minimum
    uint16_t maxUs;       // mapped maximum
    uint16_t failsafeUs;  // output when CRSF link is lost
};


static const ChannelMap CH_MAP[8] = {
    {988, 2012, 1500},   // CH1
    {988, 2012, 1500},   // CH2
    {988, 2012, 1000},   // CH3 (Throttle failsafe = 1000)
    {500, 2500, 1500},   // CH4 (Expanded)
    {988, 2012, 1500},   // CH5
    {500, 2500, 1500},   // CH6 (expanded range)
    {988, 2012, 1500},   // CH7
    {988, 2012, 1500}    // CH8
};

// ======================================================
// Pin Configuration
// ======================================================
#define PIN_CRSF_RX     9
#define PIN_CRSF_TX     8

#define PIN_I2C_SDA     4
#define PIN_I2C_SCL     5

#define PIN_VBAT_ADC    26   // ADC0

// ======================================================
// PWM Output Pins (8 channels)
// ======================================================
static const uint8_t PWM_PINS[8] = {
    13, 12, 11, 10, 7, 6, 3, 2
};

// ======================================================
// Battery Divider (30k / 7.5k)
// ======================================================
#define VBAT_R1 30000.0f
#define VBAT_R2  7500.0f

// ======================================================
// Telemetry Rates
// ======================================================
#define TELEMETRY_BATT_HZ   5
#define TELEMETRY_ALT_HZ    5

// PWM-based arming (optional)
static const uint8_t PWM_ARM_CHANNEL     = 4;     // 0 based array 0 - 7
static const uint16_t PWM_ARM_THRESHOLD  = 1700;  // µs required to arm
static const uint16_t PWM_NO_SIGNAL_US   = 900;   // below this = no PWM present

// UART
static const uint32_t MSP_BAUD = 115200;

// Timings (ms)
static const uint32_t VTX_DETECTION_TIMEOUT_MS = 300000;  // 5 minutes
static const uint32_t PRE_ARM_DELAY_MS         = 30000;  // 30 seconds only used if no PWM arming
static const uint32_t HEARTBEAT_INTERVAL_MS    = 200;    // BTFL use 150 (~6.7 Hz)

// Error thresholds
static const uint8_t TX_FAIL_THRESHOLD        = 5;
static const uint8_t BAD_CHECKSUM_THRESHOLD   = 3;

// LED
static const uint8_t LED_PIN   = 16;   // RP2040-Zero onboard WS2812
static const uint8_t LED_COUNT = 1;