#include "bf_msp.h"

static HardwareSerial *bfSerial = nullptr;
static uint32_t lastStatusMS = 0;
static uint32_t lastStatusExMS = 0;
static uint32_t lastAnalogMS = 0;
static uint32_t lastRcMS = 0;

// -----------------------------------------------------------------------------
// MSP v1 sender (8-bit command, no MSB)
// -----------------------------------------------------------------------------
static bool bf_msp_send(uint8_t cmd, const uint8_t *payload, uint8_t length) {
    if (!bfSerial) return false;

    uint8_t checksum = 0;
    checksum ^= length;
    checksum ^= cmd;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= payload[i];
    }

    // Build a small local frame buffer for debug print
    uint8_t frame[6 + 32];  // header + max payload
    uint8_t idx = 0;
    frame[idx++] = '$';
    frame[idx++] = 'M';
    frame[idx++] = '>';
    frame[idx++] = length;
    frame[idx++] = cmd;
    for (uint8_t i = 0; i < length; i++) {
        frame[idx++] = payload ? payload[i] : 0;
    }
    frame[idx++] = checksum;

    // Send to VTX
    bfSerial->write(frame, idx);

    /* Debug to USB
    Serial.print("MSP cmd=");
    Serial.print(cmd);
    Serial.print(" len=");
    Serial.print(length);
    Serial.print(" : ");
    for (uint8_t k = 0; k < idx; k++) {
        Serial.printf("%02X ", frame[k]);
    }
    Serial.println();  
*/
    return true;
}

// -----------------------------------------------------------------------------
// Read battery voltage from GPIO26 (ADC0) using 30k / 7.5k divider
// -----------------------------------------------------------------------------
float readBatteryVoltage() {
    const float ADC_REF = 3.3f;        // RP2040 ADC reference
    const float DIVIDER_RATIO = 5.0f;  // 30k / 7.5k

    uint16_t raw = analogRead(26);     // GPIO26 = ADC0, 0–4095
    float v_adc = raw * (ADC_REF / 4095.0f);
    float v_bat = v_adc * DIVIDER_RATIO;

    return v_bat;
}

// -----------------------------------------------------------------------------
// Identity: Betaflight 4.4.x (DJI-friendly API version)
// -----------------------------------------------------------------------------
static void bf_msp_send_api_version() {
    // DJI AU V1 expects API 1.43 (BF 4.3.x era)
    // uint8_t payload[3] = {1, 43, 0};  // API 1.43
    uint8_t payload[3] = {1, 44, 0};  // API 1.44 (BF 4.4.x)
    bf_msp_send(1, payload, sizeof(payload));   // MSP_API_VERSION = 1
}

static void bf_msp_send_fc_variant() {
    uint8_t payload[4] = {'B','T','F','L'};
    bf_msp_send(2, payload, sizeof(payload));   // MSP_FC_VARIANT = 2
}

static void bf_msp_send_fc_version() {
    uint8_t payload[3] = {4, 4, 2};   // Betaflight 4.4.2
    bf_msp_send(3, payload, sizeof(payload));   // MSP_FC_VERSION = 3
}

static void bf_msp_send_board_info() {
    uint8_t p[7];
    uint8_t i = 0;

    p[i++] = 'B';
    p[i++] = 'T';
    p[i++] = 'F';
    p[i++] = 'L';

    p[i++] = 0;  // hardware revision
    p[i++] = 0;  // board type
    p[i++] = 0;  // target capability flags

    bf_msp_send(4, p, i);   // MSP_BOARD_INFO = 4
}

static void bf_msp_send_build_info() {
    const char *build = "Mar 01 2024";  // any 11 chars
    bf_msp_send(5, (const uint8_t*)build, 11);  // MSP_BUILD_INFO = 5
}

static void bf_msp_send_features() {
    uint32_t features = 0;  // minimal
    bf_msp_send(36, (uint8_t*)&features, 4);  // MSP_FEATURE = 36
}

static void bf_msp_send_fc_name() {
    const char *name = "BETAFLIGHT";
    bf_msp_send(10, (const uint8_t*)name, strlen(name));  // MSP_FC_NAME = 10
}

// -----------------------------------------------------------------------------
// STATUS (MSP v1)
// -----------------------------------------------------------------------------
static void bf_msp_send_status(bool armed) {
    uint8_t p[2+2+2+4+1+1+1];
    uint8_t i = 0;

    auto put16 = [&](uint16_t v) {
        p[i++] = v & 0xFF;
        p[i++] = (v >> 8) & 0xFF;
    };
    auto put32 = [&](uint32_t v) {
        p[i++] = v & 0xFF;
        p[i++] = (v >> 8) & 0xFF;
        p[i++] = (v >> 16) & 0xFF;
        p[i++] = (v >> 24) & 0xFF;
    };

    put16(1000);   // cycleTime
    put16(0);      // i2cErrorCount

    uint16_t sensors = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    put16(sensors);

    uint32_t flags = armed ? 0x00000001 : 0x00000000;
    put32(flags);

    p[i++] = 0;  // currentSet
    p[i++] = 0;  // pidProfile
    p[i++] = 0;  // rateProfile

    bf_msp_send(101, p, i);   // MSP_STATUS = 101
}

// -----------------------------------------------------------------------------
// STATUS_EX (MSP v1)
// -----------------------------------------------------------------------------
static void bf_msp_send_status_ex(bool armed) {
    uint8_t p[26];
    uint8_t i = 0;

    auto put16 = [&](uint16_t v) {
        p[i++] = v & 0xFF;
        p[i++] = (v >> 8) & 0xFF;
    };
    auto put32 = [&](uint32_t v) {
        p[i++] = v & 0xFF;
        p[i++] = (v >> 8) & 0xFF;
        p[i++] = (v >> 16) & 0xFF;
        p[i++] = (v >> 24) & 0xFF;
    };

    put16(1000);   // cycleTime
    put16(0);      // i2cErrorCount

    uint16_t sensors = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    put16(sensors);

    uint32_t flags = armed ? 0x00000001 : 0x00000000;
    put32(flags);

    p[i++] = 0;  // currentSet
    p[i++] = 0;  // pidProfile
    p[i++] = 0;  // rateProfile

    put16(10);   // averageSystemLoadPercent

    uint16_t armingDisableFlags = armed ? 0 : (1 << 0);
    put16(armingDisableFlags);

    p[i++] = armed ? 0 : 1;  // armingDisableFlagsCount

    // Critical for DJI AU V1
    uint16_t armingFlags = armed ? 0x0303 : 0x0002;
    put16(armingFlags);

    uint32_t flightModeFlags = armed ? 0x00000003 : 0x00000002;
    put32(flightModeFlags);

    uint16_t vbatLatest = (uint16_t)(readBatteryVoltage() * 10.0f);
    put16(vbatLatest);

    bf_msp_send(150, p, i);   // MSP_STATUS_EX = 150
}

// -----------------------------------------------------------------------------
// MSP_RC (105) — 16 channels, 1000–2000 range
// -----------------------------------------------------------------------------
static void bf_msp_send_rc() {
    uint8_t p[32];
    uint8_t i = 0;

    auto put16 = [&](uint16_t v) {
        p[i++] = v & 0xFF;
        p[i++] = (v >> 8) & 0xFF;
    };

    // Neutral 1500us for all 16 channels
    for (int ch = 0; ch < 16; ch++) {
        put16(1500);
    }

    bf_msp_send(105, p, i);   // MSP_RC = 105
}

// -----------------------------------------------------------------------------
// MSP_ANALOG — send only main pack voltage (0.1V units)
// -----------------------------------------------------------------------------
// MSP_ANALOG (110), classic Betaflight layout
// uint16_t vbat_x10
// uint16_t mAhDrawn
// uint16_t amperage_x100
// uint16_t rssi

void bf_msp_send_analog() {
    float vbat = readBatteryVoltage();
    uint16_t mspVoltage = (uint16_t)(vbat * 10.0f);  // 12.0V → 120

    uint8_t p[8];
    uint8_t i = 0;

    auto put16 = [&](uint16_t v) {
        p[i++] = v & 0xFF;
        p[i++] = (v >> 8) & 0xFF;
    };

    put16(mspVoltage);  // voltage
    put16(0);           // mAh
    put16(0);           // current
    put16(0);           // rssi

    bf_msp_send(110, p, i);
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void bf_msp_init(HardwareSerial &port) {
    bfSerial = &port;

    bf_msp_send_api_version();
    bf_msp_send_fc_variant();
    bf_msp_send_fc_version();
    bf_msp_send_board_info();
    bf_msp_send_build_info();
    bf_msp_send_features();
    bf_msp_send_fc_name();

    // Optional early telemetry
    bf_msp_send_analog();
}

void bf_msp_heartbeat(bool armed) {
    if (!bfSerial) return;

    uint32_t now = millis();

    // STATUS + STATUS_EX always first
    if (now - lastStatusMS >= 200) {
        lastStatusMS = now;
        bf_msp_send_status(armed);
        bf_msp_send_status_ex(armed);
    }

    // RC next
    if (now - lastRcMS >= 40) {
        lastRcMS = now;
        bf_msp_send_rc();
    }

    // ANALOG last
    if (now - lastAnalogMS >= 100) {
        lastAnalogMS = now;
        bf_msp_send_analog();
    }
}