#include "Telemetry.h"
#include "config.h"      // <--- REQUIRED
#include "hardware/adc.h"

#define CRSF_ADDRESS_TELEMETRY 0xEA
#define CRSF_TYPE_BATTERY   0x08  // ✓ correct
#define CRSF_TYPE_GPS       0x02  // altitude lives here
#define CRSF_TYPE_VARIO     0x07  // vertical speed lives here

// ------------------------------------------------------
// CRC8-D5 for CRSF
// ------------------------------------------------------
static uint8_t crc8_d5(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0xD5;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ------------------------------------------------------
// Telemetry::begin
// ------------------------------------------------------
void Telemetry::begin() {
    adc_init();
    adc_gpio_init(PIN_VBAT_ADC);

    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();

    bmp_ok = bmp.begin(0x76);
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X8,     // temperature oversampling
        Adafruit_BMP280::SAMPLING_X8,     // pressure oversampling
        Adafruit_BMP280::FILTER_X16,      // IIR filter
        Adafruit_BMP280::STANDBY_MS_63    // ~16 Hz update rate
    );

}

// ------------------------------------------------------
// Send CRSF telemetry frame
// ------------------------------------------------------
void Telemetry::sendFrame(uint8_t type, const uint8_t* payload, uint8_t payloadLen) {
    uint8_t buf[32];
    uint8_t idx = 0;

    buf[idx++] = CRSF_ADDRESS_TELEMETRY;   // 0xEA
    buf[idx++] = payloadLen + 2;           // type + payload + CRC
    buf[idx++] = type;

    for (uint8_t i = 0; i < payloadLen; i++)
        buf[idx++] = payload[i];

    buf[idx++] = crc8_d5(&buf[2], payloadLen + 1);

    Serial2.write(buf, idx);
    if (Serial) {
        Serial.print("CRSF: ");
        for (uint8_t i = 0; i < idx; i++) {
            if (buf[i] < 16) Serial.print('0');
            Serial.print(buf[i], HEX);
            Serial.print(' ');
        }
        Serial.println();
    }
}

// ------------------------------------------------------
// Battery reading + smoothing
// ------------------------------------------------------
uint16_t Telemetry::readBatteryMv() {
    adc_select_input(0);
    uint16_t raw = adc_read();

    // Convert ADC reading to voltage at ADC pin
    float vAdc = (raw / 4095.0f) * 3.3f;

    // Apply voltage divider scaling
    float vBat = vAdc * (1.0f + VBAT_R1 / VBAT_R2);

    // Convert to millivolts
    uint16_t mv = (uint16_t)(vBat * 1000.0f);

    return mv;
}

// ------------------------------------------------------
// Altitude reading + smoothing
// ------------------------------------------------------
int32_t Telemetry::readAltitudeCm() {
    if (!bmp_ok) return 0;

    float alt = bmp.readAltitude(1013.25);

    if (!baseSet) {
        baseAlt = alt;
        baseSet = true;
    }

    float rel = (alt - baseAlt) * 100.0f;

    const float alpha = 0.05f;
    if (!altFilterInit) {
        filteredAlt = rel;
        altFilterInit = true;
    } else {
        filteredAlt = filteredAlt + alpha * (rel - filteredAlt);
    }

    return (int32_t)filteredAlt;
}

// ------------------------------------------------------
// Vertical speed (cm/s) from altitude delta
// ------------------------------------------------------
int16_t Telemetry::readVSpeedCms() {
    static int32_t lastAlt = 0;
    static uint32_t lastTime = 0;

    uint32_t now = millis();
    int32_t alt = readAltitudeCm();

    if (lastTime == 0) {
        lastTime = now;
        lastAlt = alt;
        return 0;
    }

    int32_t dAlt = alt - lastAlt;
    uint32_t dt = now - lastTime;

    lastAlt = alt;
    lastTime = now;

    if (dt == 0) return 0;

    float vs = (float)dAlt / (float)dt * 1000.0f;  // cm/s

    const float alpha = 0.1f;
    static float filteredVs = 0;
    filteredVs = filteredVs + alpha * (vs - filteredVs);

    return (int16_t)filteredVs;
}

// ------------------------------------------------------
// Send battery telemetry
// ------------------------------------------------------
void Telemetry::sendBattery() {
    uint16_t mv = readBatteryMv();
    uint16_t cv = mv / 100;  // 0.01V units

    uint8_t p[8];
    p[0] = cv >> 8;      // voltage MSB
    p[1] = cv & 0xFF;    // voltage LSB
    p[2] = 0;            // current MSB
    p[3] = 0;            // current LSB
    p[4] = 0;            // capacity byte 1
    p[5] = 0;            // capacity byte 2
    p[6] = 0;            // capacity byte 3
    p[7] = 0;            // remaining %

    sendFrame(CRSF_TYPE_BATTERY, p, 8);
}

void Telemetry::sendVario() {
    int16_t vs   = readVSpeedCms();          // cm/s
    int32_t cm   = readAltitudeCm();
    int16_t alt  = (int16_t)(cm / 10) + 10000; // dm + 10000 offset

    uint8_t p[4];
    p[0] = vs >> 8;       // vspeed MSB
    p[1] = vs & 0xFF;     // vspeed LSB
    p[2] = alt >> 8;      // altitude MSB
    p[3] = alt & 0xFF;    // altitude LSB

    sendFrame(CRSF_TYPE_VARIO, p, 4);
}

void Telemetry::sendAltitude() {
    int32_t cm = readAltitudeCm();

    // GPS frame altitude: metres + 1000 offset, big-endian uint16
    int16_t metres = (int16_t)(cm / 100);
    uint16_t alt = (uint16_t)(metres + 1000);

    uint8_t p[15] = {0};  // zero out lat, lon, speed, heading, sats

    // Altitude at bytes 12-13, big-endian
    p[12] = alt >> 8;
    p[13] = alt & 0xFF;

    // Set satellites to 6 so EdgeTX considers the GPS "valid"
    p[14] = 6;

    sendFrame(CRSF_TYPE_GPS, p, 15);
}

// ------------------------------------------------------
// Periodic telemetry update
// ------------------------------------------------------
void Telemetry::update() {
    uint32_t now = millis();

    if (now - lastBatt > (1000 / TELEMETRY_BATT_HZ)) {
        lastBatt = now;
        sendBattery();
    }

    if (now - lastAlt > (1000 / TELEMETRY_ALT_HZ)) {
        lastAlt = now;
        sendAltitude();
        sendVario();
    }

}