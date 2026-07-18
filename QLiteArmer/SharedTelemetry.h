#pragma once

#include <stdint.h>

struct SharedTelemetry {
    float gpsDistHomeM = 0.0f;
    float gpsGroundSpeedCms = 0.0f;
    float gpsCourseDeg = 0.0f;          // Earth-frame heading (C.O.G.)
    float gpsBearingToHomeDeg = 0.0f;   // Earth-frame bearing to home
    bool  gpsFix = false;
    uint8_t gpsSats = 0;
    float gpsLatDeg = 0.0f;
    float gpsLonDeg = 0.0f;

    float gpsTotalDistM = 0.0f;
    bool  gpsTotalActive = false;
    float gpsPrevLatDeg = 0.0f;
    float gpsPrevLonDeg = 0.0f;
    bool  gpsPrevValid = false;

    float baroAltCm = 0.0f;
    float baroVSpeedCms = 0.0f;

    uint16_t batteryMv = 0;

    uint32_t flightStartMs = 0;
    uint32_t flightElapsedMs = 0;
    bool flightTimerRunning = false;

    // -------------------------------------------------------
    // Unified home-direction pipeline (arrow + radar)
    // -------------------------------------------------------

    float homeRelativeDeg = 0.0f;          // raw relative bearing (pilot frame)
    float homeRelativeSmoothDeg = 0.0f;    // smoothed relative bearing
    float homeDistanceFt = 0.0f;           // distance from home in feet
    float homeRadarRadius = 0.0f;          // scaled radar radius (0–2 cells)
    int   homeRadarRow = 0;                // final radar row
    int   homeRadarCol = 0;                // final radar col
};

extern SharedTelemetry sharedTelem;
