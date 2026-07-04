#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

class Telemetry {
public:
    // -------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------
    void begin();
    void update();

    // -------------------------------------------------------
    // Sensor Readings
    // -------------------------------------------------------

    // Returns battery voltage in millivolts (e.g. 16800 = 16.8V)
    uint16_t readBatteryMv();

    // Returns altitude in centimeters relative to home point
    int32_t readAltitudeCm();

    // Returns vertical speed in cm/s (positive = climbing)
    int16_t readVSpeedCms();

    // Returns raw pressure in Pascals
    float readPressurePa();

    // Returns temperature in degrees Celsius
    float readTemperatureC();

    // -------------------------------------------------------
    // CRSF Telemetry Transmit Methods
    // -------------------------------------------------------

    // Send battery telemetry frame over CRSF
    void sendBattery();

    // Send barometric altitude frame over CRSF
    void sendAltitude();

    // Send vertical speed (vario) frame over CRSF
    void sendVario();

    // Reset Base Altitude
    void resetBaseAltitude() { baseSet = false; }

    // -------------------------------------------------------
    // Status
    // -------------------------------------------------------

    // Returns true if BMP280 was found and initialized successfully
    bool isBmpOk() const { return bmp_ok; }

    // Returns true if a valid battery voltage reading is available
    bool isBatteryOk() const { return batteryMv > 0; }

private:
    // -------------------------------------------------------
    // CRSF Frame Helper
    // -------------------------------------------------------

    // Builds and sends a CRSF telemetry frame
    // type    — CRSF frame type byte
    // payload — pointer to payload data
    // len     — payload length in bytes
    void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);

    // -------------------------------------------------------
    // BMP280 Sensor
    // -------------------------------------------------------
    Adafruit_BMP280 bmp;
    bool bmp_ok = false;

    // -------------------------------------------------------
    // Altitude Tracking
    // -------------------------------------------------------

    // Home altitude baseline (set on first valid reading)
    bool  baseSet   = false;
    float baseAlt   = 0.0f;

    // Low-pass filtered altitude (cm)
    bool  altFilterInit = false;
    float filteredAlt   = 0.0f;

    // Previous filtered altitude for vspeed calculation
    float prevFilteredAlt     = 0.0f;
    uint32_t prevAltTimestamp = 0;

    // Calculated vertical speed in cm/s
    int16_t vSpeedCms = 0;

    // -------------------------------------------------------
    // Battery Voltage
    // -------------------------------------------------------

    // Latest battery reading in millivolts
    uint16_t batteryMv = 0;

    // ADC smoothing — rolling average
    static const uint8_t ADC_SAMPLES = 8;
    uint32_t adcBuffer[ADC_SAMPLES] = {0};
    uint8_t  adcIndex               = 0;
    bool     adcBufferFull          = false;

    // -------------------------------------------------------
    // Update Timing
    // -------------------------------------------------------

    // Timestamp of last BMP280 read
    uint32_t lastBmpRead = 0;

    // Timestamp of last battery ADC read
    uint32_t lastAdcRead = 0;

    // Timestamp of last CRSF telemetry transmit
    uint32_t lastTelemetrySend = 0;
};