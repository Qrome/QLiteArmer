#pragma once
#include <Arduino.h>

// PWM-based arming (optional)
static const uint8_t  PWM_ARM_PIN        = 6;     // GPIO pin for PWM input
static const uint16_t PWM_ARM_THRESHOLD  = 1800;  // µs required to arm
static const uint16_t PWM_NO_SIGNAL_US   = 900;   // below this = no PWM present

// Servo Expander
static const uint8_t  SERVO_IN_PIN        = 7;     // PWM input from RX
static const uint8_t  SERVO_OUT_PIN       = 29;    // PWM output
static const uint16_t SERVO_IN_MIN_US     = 1000;  // expected RX range
static const uint16_t SERVO_IN_MAX_US     = 2000;
static const uint16_t SERVO_OUT_MIN_US    = 500;   // expanded output range LOW
static const uint16_t SERVO_OUT_MAX_US    = 2500;  // expanded high

// UART
static const uint32_t MSP_BAUD = 115200;

// Timings (ms)
static const uint32_t VTX_DETECTION_TIMEOUT_MS = 60000;  // 60 seconds
static const uint32_t PRE_ARM_DELAY_MS         = 30000;  // 30 seconds only used if no PWM arming
static const uint32_t HEARTBEAT_INTERVAL_MS    = 200;    // BTFL use 150 (~6.7 Hz)

// Error thresholds
static const uint8_t TX_FAIL_THRESHOLD        = 5;
static const uint8_t BAD_CHECKSUM_THRESHOLD   = 3;

// LED
static const uint8_t LED_PIN   = 16;   // RP2040-Zero onboard WS2812
static const uint8_t LED_COUNT = 1;