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
    {988, 2012, 1500},   // CH4 
    {988, 2012, 1500},   // CH5
    {988, 2012, 1500},   // CH6     {500, 2500, 1500};  //CH6 (expanded range)(expanded range)
    {988, 2012, 1500},   // CH7
    {988, 2012, 1500}    // CH8
};

#define SHOW_THROTTLE_PERCENT true
static const uint8_t throttleIndex = 2; //0 based index so 2 = CH3
static const char* CraftName = "QLITE";  // Use uppercase and numbers

// ======================================================
// Pin Configuration
// ======================================================
#define PIN_CRSF_RX     9
#define PIN_CRSF_TX     8

#define PIN_GPS_TX      14
#define PIN_GPS_RX      15

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

// Units for altitude + vertical speed
#define OSD_UNITS_METRIC     0
#define OSD_UNITS_IMPERIAL   1

// Set your preferred default here:
#define OSD_UNITS OSD_UNITS_IMPERIAL   // use OSD_UNITS_IMPERIAL or OSD_UNITS_METRIC

// Ground Radar Options
#define USE_RADAR_HOME_INDICATOR true
static const uint16_t RADAR_CELL_FEET    = 200;    // number of feet per cell
static const float RADAR_CELL_RADIUS     = 4.0F;   // number of cells radius around center
static const int RADAR_ROW_CENTER        = 9;      // row center 1080p is 9
static const int RADAR_COL_CENTER        = 25;     // column center 1080p is 25

// Compass Heading Ribbon
#define USE_COMPASS_HEADING true

// PWM-based arming (optional)
static const uint8_t PWM_ARM_CHANNEL     = 4;     // 0 based array 0 - 7
static const uint16_t PWM_ARM_THRESHOLD  = 1700;  // µs required to arm
static const uint16_t PWM_NO_SIGNAL_US   = 900;   // below this = no PWM present

// UART
static const uint32_t MSP_BAUD = 115200;

// Timings (ms)
static const uint32_t VTX_DETECTION_TIMEOUT_MS = 300000;  // 5 minutes
static const uint32_t HEARTBEAT_INTERVAL_MS    = 200;    // BTFL use 150 (~6.7 Hz)

// Error thresholds
static const uint8_t BAD_CHECKSUM_THRESHOLD   = 3;

// LED
static const uint8_t LED_PIN   = 16;   // RP2040-Zero onboard WS2812
static const uint8_t LED_COUNT = 1;

// 16‑direction home arrow glyphs (rows 96–111), CCW, 22.5° per step
static const uint8_t arrowRows[16] = {
    96,  //   0° DOWN
    111, //  22.5° DOWN‑SLIGHT‑LEFT
    110, //  45° DOWN‑LEFT
    109, //  67.5° LEFT‑DOWN
    108, //  90° LEFT
    107, // 112.5° LEFT‑SLIGHT‑UP
    106, // 135° LEFT‑UP
    105, // 157.5° UP‑LEFT
    104, // 180° UP
    103, // 202.5° UP‑SLIGHT‑RIGHT
    102, // 225° UP‑RIGHT
    101, // 247.5° RIGHT‑UP
    100, // 270° RIGHT
    99,  // 292.5° RIGHT‑SLIGHT‑DOWN
    98,  // 315° RIGHT‑DOWN
    97   // 337.5° DOWN‑SLIGHT‑RIGHT
};

// 24 slots base + 12 slot overflow = 36 items total.
// Spacing: 15 degrees per index. 3 small ticks (28) between each cardinal.
static const uint8_t compassList[36] = {
    24, 28, 29, 28, 26, 28, 29, 28, // N to E
    25, 28, 29, 28, 27, 28, 29, 28, // S to W
    24, 28, 29, 28, 26, 28, 29, 28, // N to E (Wrap loop starts)
    25, 28, 29, 28, 27, 28, 29, 28  // S to W
};