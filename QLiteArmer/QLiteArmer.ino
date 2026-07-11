/*
  HD_Amer — Auto‑Arming MSP Transmitter Firmware
  by David Payne (Qromer)
  Target MCU: Waveshare RP2040‑Zero
*/

#include <Arduino.h>
#include "SharedTelemetry.h"
#include "Telemetry.h"
#include "config.h"
#include "led.h"
#include "state_machine.h"
#include "detection.h"
#include "bf_msp.h"
#include "CrossfireELRS.h"
#include "PWMDriver.h"
#include <SerialPIO.h>
#include "GPS.h"

float distFiltM = 0.0f;
bool distFiltInit = false;
CrossfireELRS crsf;
PWMDriver pwm;
Telemetry telemetry;
SerialPIO SerialGPS(PIN_GPS_TX, PIN_GPS_RX); 
GPS gps;

// Shared across cores — volatile for safety
volatile bool systemReady = false;
volatile bool loopReady = false;
volatile bool gpsReady = false;

inline void debugPrint(const String &msg) {
    if (Serial) {
        Serial.println(msg);
    }
}

void populateSharedTelemetry() {
    gps.update();

    // --- GPS distance smoothing ---
    float rawDist = gps.getDistanceFromHomeM();

    if (!distFiltInit) {
        distFiltM = rawDist;
        distFiltInit = true;
    } else {
        distFiltM = distFiltM * 0.8f + rawDist * 0.2f;
    }

    // Push into shared telemetry
    sharedTelem.gpsDistHomeM = distFiltM;
    sharedTelem.gpsFix = gps.hasFix();
    sharedTelem.gpsSats = gps.getSatCount();
    sharedTelem.gpsGroundSpeedCms = gps.getGroundSpeed();
    sharedTelem.gpsCourseDeg = gps.getCourse() * 0.1f;          // FIXED: real degrees
    sharedTelem.gpsBearingToHomeDeg = gps.getBearingToHomeDeg();
    sharedTelem.gpsLatDeg = gps.getLatitude() * 1e-7f;
    sharedTelem.gpsLonDeg = gps.getLongitude() * 1e-7f;

    sharedTelem.baroAltCm = telemetry.readAltitudeCm();
    sharedTelem.baroVSpeedCms = telemetry.readVSpeedCms();
    sharedTelem.batteryMv = telemetry.readBatteryMv();

    // -------------------------------------------------------
    // Unified Home-Direction Pipeline (arrow + radar)
    // -------------------------------------------------------

    float bearingToHome = sharedTelem.gpsBearingToHomeDeg;   // Earth-frame
    float headingDeg    = sharedTelem.gpsCourseDeg;          // Earth-frame (C.O.G.)
    float distFt        = sharedTelem.gpsDistHomeM * 3.28084f;

    // 1. Pilot-relative bearing
    float rel = bearingToHome - headingDeg;
    if (rel < 0) rel += 360.0f;
    if (rel >= 360.0f) rel -= 360.0f;

    // 2. Smooth the relative bearing
    static float relSmooth = 0.0f;
    relSmooth = relSmooth * 0.90f + rel * 0.10f;

    // 3. Radar radius (RADAR_CELL_FEET ft per cell, max RADAR_CELL_RADIUS cells) see config.h
    float rawRadius = distFt / RADAR_CELL_FEET;
    if (rawRadius > RADAR_CELL_RADIUS) rawRadius = RADAR_CELL_RADIUS;

    static float radiusSmooth = 0.0f;
    radiusSmooth = radiusSmooth * 0.85f + rawRadius * 0.15f;

    // 4. Convert raw relative bearing to radians
    float theta = radians(rel); 

    // 5. Radar offsets around crosshair (9, 25)
    // Calculate the raw mathematical target grid offsets
    float targetRowOffset = -cos(theta) * rawRadius;
    float targetColOffset =  sin(theta) * rawRadius;

    // 6. Responsive Tuning: Use a light filter (0.40f) instead of 0.85f
    // This removes jerky jumps but updates within 2-3 frames
    static float rowOffsetSmooth = 0.0f;
    static float colOffsetSmooth = 0.0f;
    rowOffsetSmooth = rowOffsetSmooth * 0.60f + targetRowOffset * 0.40f;
    colOffsetSmooth = colOffsetSmooth * 0.60f + targetColOffset * 0.40f;

    // Convert directly to final grid coordinates
    int newRow = RADAR_ROW_CENTER + (int)round(rowOffsetSmooth);
    int newCol = RADAR_COL_CENTER + (int)round(colOffsetSmooth);

    // 7. Store unified values in SharedTelemetry
    sharedTelem.homeRelativeDeg       = rel;
    sharedTelem.homeRelativeSmoothDeg = relSmooth;
    sharedTelem.homeDistanceFt        = distFt;
    sharedTelem.homeRadarRadius       = radiusSmooth;
    sharedTelem.homeRadarRow          = newRow;
    sharedTelem.homeRadarCol          = newCol;
}

// -------------------------------------------------------
// Core 0 — RC input + PWM output (time-critical)
// -------------------------------------------------------
void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 1500)) {
        delay(10);
    }

    debugPrint("Starting up QLiteArmer...");

    // CRSF receiver
    crsf.begin(PIN_CRSF_RX, PIN_CRSF_TX);

    // PWM servo outputs
    pwm.begin(PWM_PINS, 8);

    delay(500);  // let CRSF stabilise first

    systemReady = true;
    debugPrint("Core 0 ready.");
}

void loop() {
    while (!loopReady) {
        delay(10);
    }
    // CRSF receiver — time sensitive, must run every loop
    crsf.update();

    // PWM outputs from CRSF channels
    for (int i = 0; i < 8; i++) {
        pwm.writeFromCRSF(i, crsf.getChannel(i), crsf.crsfLinkActive);
    }

    bf_msp_parse_incoming();       // always — reads and responds to polls
    bf_msp_firstbeat_update();     // always — no-op after firstbeat completes
    bf_msp_heartbeat_update((currentState == STATE_ARMED)); // always — skips DP heartbeat for V1
    bf_msp_dp_update_osd_nb();     // always — no-op for V1, active for O3/Avatar

    // Add to loop() temporarily for diagnosis
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 2000) {
        lastDebugMs = millis();
        VtxSystemType t = bf_msp_get_vtx_type();
        Serial.print("[DEBUG] VTX type detected: ");
        switch(t) {
            case VTX_UNKNOWN:   Serial.println("UNKNOWN"); break;
            case VTX_DJI_V1:    Serial.println("DJI V1");  break;
            case VTX_DJI_O3:    Serial.println("DJI O3/O4"); break;
            case VTX_WALKSNAIL: Serial.println("Walksnail Avatar"); break;
        }
    }
}

// -------------------------------------------------------
// Core 1 — MSP arming + telemetry (background)
// -------------------------------------------------------
void setup1() {
    // Wait for Core 0 to finish its setup
    while (!systemReady) {
        delay(100);
    }

    // MSP UART to VTX
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(MSP_BAUD);

    // Telemetry (BMP280 + ADC)
    telemetry.begin();

    // MSP subsystem
    bf_msp_init(Serial1, telemetry, crsf);

    delay(150);
    // LED
    ledInit();

    // GPS on SerialPIO using pins 14 (TX) and 15 (RX)
    static const uint32_t gpsBauds[] = {9600, 38400, 57600, 115200};
    gps.begin(SerialGPS);

    Serial.println("[GPS] Starting autodetect...");
    bool gpsOK = gps.autodetectBaud(gpsBauds, 4, 900);

    if (gpsOK) {
        Serial.println("[GPS] Autodetect successful.");
    } else {
        Serial.println("[GPS] WARNING: No GPS detected.");
    }

    // Start state machine
    enterState(STATE_BOOT_DETECT);

    // VTX detection parser
    detectionInit();

    debugPrint("Core 1 ready.");
    loopReady = true;
}

static uint32_t lastGpsPrint = 0;

void loop1() {
    while (!loopReady) {
        delay(10);
    }
    static SystemState lastState = STATE_BOOT_DETECT;
    telemetry.update();  
    
    populateSharedTelemetry();

    // Start tracking only when GPS fix + 6 sats and armed
    if (!gpsReady && sharedTelem.gpsFix && sharedTelem.gpsSats >= 6) {
        gpsReady = true;
    }


    uint32_t now = millis();

    // Print GPS debug once per second
    if (now - lastGpsPrint > 2000) {
        lastGpsPrint = now;

        Serial.print("[GPS] Fix=");
        Serial.print(gps.hasFix());
        Serial.print(" Sats=");
        Serial.print(gps.getSatCount());
        Serial.print(" Lat=");
        Serial.print(gps.getLatitude());
        Serial.print(" Lon=");
        Serial.print(gps.getLongitude());
        Serial.print(" Alt(cm)=");
        Serial.print(gps.getAltitudeMSL());
        Serial.print(" GS(cm/s)=");
        Serial.print(gps.getGroundSpeed());
        Serial.print(" Course=");
        Serial.println(gps.getCourse());
    }

    uint16_t armRaw = crsf.getChannel(PWM_ARM_CHANNEL);
    stateMachineUpdate(armRaw, crsf.crsfLinkActive);
    // --- Detect ARM transition using state machine ---
    if (currentState == STATE_ARMED && lastState != STATE_ARMED && gpsReady) {
        // Rising edge: system just became ARMED
        gps.forceSetHome();
        telemetry.resetBaseAltitude();
        sharedTelem.gpsTotalDistM = 0.0f;
        sharedTelem.gpsTotalActive = false;
        sharedTelem.gpsPrevValid = false;
    }

    // Only track when ARMED + GPS locked
    if (currentState == STATE_ARMED && gpsReady) {

        if (!sharedTelem.gpsTotalActive) {
            // First valid point after arming
            sharedTelem.gpsPrevLatDeg = sharedTelem.gpsLatDeg;
            sharedTelem.gpsPrevLonDeg = sharedTelem.gpsLonDeg;
            sharedTelem.gpsPrevValid = true;
            sharedTelem.gpsTotalActive = true;
        }

        if (sharedTelem.gpsPrevValid) {
            // Haversine incremental distance
            float lat1 = radians(sharedTelem.gpsPrevLatDeg);
            float lon1 = radians(sharedTelem.gpsPrevLonDeg);
            float lat2 = radians(sharedTelem.gpsLatDeg);
            float lon2 = radians(sharedTelem.gpsLonDeg);

            float dLat = lat2 - lat1;
            float dLon = lon2 - lon1;

            float a = sin(dLat/2)*sin(dLat/2) +
                    cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);

            float c = 2 * atan2(sqrt(a), sqrt(1-a));

            const float R = 6371000.0f; // meters
            float deltaM = R * c;

            // Reject GPS jumps > 100m
            float maxJump = sharedTelem.gpsGroundSpeedCms * 0.05f; // 50ms window
            if (deltaM < maxJump)
                sharedTelem.gpsTotalDistM += deltaM;
        }

        // Update previous
        sharedTelem.gpsPrevLatDeg = sharedTelem.gpsLatDeg;
        sharedTelem.gpsPrevLonDeg = sharedTelem.gpsLonDeg;
        sharedTelem.gpsPrevValid = true;

    } else {
        // Not tracking
        sharedTelem.gpsPrevValid = false;
    }

    lastState = currentState;
}



