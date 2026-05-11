#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

class Telemetry {
public:
    void begin();
    void update();

    uint16_t readBatteryMv();
    int32_t  readAltitudeCm();
    int16_t  readVSpeedCms();   // NEW

    void sendBattery();
    void sendAltitude();
    void sendVSpeed();          // NEW
    void sendVario();

private:
    void sendFrame(uint8_t type, const uint8_t* payload, uint8_t len);

    // BMP280
    Adafruit_BMP280 bmp;        // NEW
    bool bmp_ok = false;

    // Altitude smoothing
    bool  baseSet = false;
    float baseAlt = 0;
    bool  altFilterInit = false;
    float filteredAlt = 0;

    // VSpeed smoothing
    bool  vspdFilterInit = false;   // NEW
    float filteredVspd = 0;         // NEW

    // Timing
    uint32_t lastBatt = 0;
    uint32_t lastAlt  = 0;
    uint32_t lastVspd = 0;          // NEW
};