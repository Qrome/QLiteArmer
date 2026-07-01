#pragma once

#include <stdint.h>

struct SharedTelemetry {
    float gpsDistHomeM = 0.0f;
    float gpsGroundSpeedCms = 0.0f;
    float gpsCourseDeg = 0.0f;
    float gpsBearingToHomeDeg = 0.0f;
    bool  gpsFix = false;
    uint8_t gpsSats = 0;
    float gpsLatDeg = 0.0f;   // decimal degrees
    float gpsLonDeg = 0.0f;

    float gpsTotalDistM = 0.0f;   // total distance traveled in meters
    bool  gpsTotalActive = false; // tracking enabled once fix+6 sats
    float gpsPrevLatDeg = 0.0f;
    float gpsPrevLonDeg = 0.0f;
    bool  gpsPrevValid = false;

    float baroAltCm = 0.0f;
    float baroVSpeedCms = 0.0f;

    uint16_t batteryMv = 0;

    uint32_t flightStartMs = 0;
    uint32_t flightElapsedMs = 0;
    bool flightTimerRunning = false;

};

extern SharedTelemetry sharedTelem;
