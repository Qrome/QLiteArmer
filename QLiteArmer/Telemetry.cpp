#include "Telemetry.h"
#include "config.h"

// -------------------------------------------------------
// CRSF Frame Type Constants
// -------------------------------------------------------
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_BARO_ALTITUDE 0x09
#define CRSF_FRAMETYPE_VARIO 0x07
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8

// -------------------------------------------------------
// Timing Constants (milliseconds)
// -------------------------------------------------------
#define BMP_READ_INTERVAL_MS 50
#define ADC_READ_INTERVAL_MS 20
#define TELEMETRY_SEND_INTERVAL_MS 100

// -------------------------------------------------------
// ADC Configuration — derived from config.h values
//
// Voltage divider ratio: (R1 + R2) / R2
// VBAT_R1 = 30000 ohm (top resistor)
// VBAT_R2 = 7500 ohm (bottom resistor, ADC side)
// Ratio = (30000 + 7500) / 7500 = 5.0
//
// Battery voltage formula:
// batteryMv = (adcRaw / ADC_STEPS_FULL_SCALE) * ADC_VREF_MV *
// VBAT_DIVIDER_RATIO
//
// Example — 4S at 16.4V:
// ADC pin sees: 16400 / 5.0 = 3280 mV
// ADC raw: (3280 / 3300) * 4096 = ~4071
// Recovered: (4071 / 4096) * 3300 * 5.0 = ~16393 mV ✓
// -------------------------------------------------------
#define VBAT_DIVIDER_RATIO ((VBAT_R1 + VBAT_R2) / VBAT_R2)

// RP2040 ADC reference voltage in millivolts
#define ADC_VREF_MV 3300.0f

// RP2040 12-bit ADC: 2^12 = 4096 steps
#define ADC_STEPS_FULL_SCALE 4096.0f

// Low-pass filter coefficient for altitude smoothing
#define ALT_FILTER_ALPHA 0.1f

// -------------------------------------------------------
// CRC8 Helper for CRSF Frames
// -------------------------------------------------------
static uint8_t crsf_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// -------------------------------------------------------
// begin() — Initialize hardware
// NOTE: Serial2 is owned by CrossfireELRS — do NOT call
// Serial2.begin() or setRX/setTX here.
// -------------------------------------------------------
void Telemetry::begin() {
    // --- BMP280 Initialization ---
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();

    if (bmp.begin(0x76)) {
        bmp_ok = true;
    } else if (bmp.begin(0x77)) {
        bmp_ok = true;
    } else {
        bmp_ok = false;
        if (Serial)
            Serial.println("[Telemetry] BMP280 not found at 0x76 or 0x77");
    }

    if (bmp_ok) {
        // Configure BMP280 for good balance of speed and accuracy
        bmp.setSampling(
            Adafruit_BMP280::MODE_NORMAL,
            Adafruit_BMP280::SAMPLING_X2,   // temperature oversampling
            Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling
            Adafruit_BMP280::FILTER_X16,    // IIR filter coefficient
            Adafruit_BMP280::STANDBY_MS_1   // standby time between readings
        );
        if (Serial) Serial.println("[Telemetry] BMP280 initialized.");
    }

    // --- ADC Initialization ---
    // Use PIN_VBAT_ADC from config.h (pin 26 = ADC0 on RP2040)
    pinMode(PIN_VBAT_ADC, INPUT);
    analogReadResolution(12);  // RP2040 supports 12-bit ADC (0–4095)

    // Print the divider ratio so it can be verified on startup
    if (Serial) {
        Serial.print("[Telemetry] VBAT divider ratio: ");
        Serial.println(VBAT_DIVIDER_RATIO, 2);
    }

    // Pre-fill ADC rolling average buffer with the current reading
    // so the first reported voltage is reasonable, not zero
    uint32_t initialAdc = analogRead(PIN_VBAT_ADC);
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        adcBuffer[i] = initialAdc;
    }
    adcBufferFull = true;

    // Calculate initial battery voltage immediately
    float avgAdc = (float)initialAdc;
    batteryMv = (uint16_t)((avgAdc / ADC_STEPS_FULL_SCALE) * ADC_VREF_MV *
                           VBAT_DIVIDER_RATIO);

    if (Serial) {
        Serial.print("[Telemetry] Initial battery voltage: ");
        Serial.print(batteryMv);
        Serial.println(" mV");
    }

    // Initialize all timestamps to now
    lastBmpRead = millis();
    lastAdcRead = millis();
    lastTelemetrySend = millis();

    if (Serial) Serial.println("[Telemetry] Initialized.");
}

// -------------------------------------------------------
// update() — Call this every loop iteration on Core 1
// -------------------------------------------------------
void Telemetry::update() {
    uint32_t now = millis();

    // -------------------------------------------------------
    // ADC Battery Voltage Reading
    // Samples PIN_VBAT_ADC (pin 26) every ADC_READ_INTERVAL_MS
    // Applies rolling average over ADC_SAMPLES readings
    // Converts to millivolts using the voltage divider ratio
    // -------------------------------------------------------
    if (now - lastAdcRead >= ADC_READ_INTERVAL_MS) {
        lastAdcRead = now;

        // Store new ADC sample in rolling circular buffer
        adcBuffer[adcIndex] = analogRead(PIN_VBAT_ADC);
        adcIndex = (adcIndex + 1) % ADC_SAMPLES;
        if (adcIndex == 0) adcBufferFull = true;

        // Compute rolling average over filled portion of buffer
        uint8_t count = adcBufferFull ? ADC_SAMPLES : adcIndex;
        uint32_t sum = 0;
        for (uint8_t i = 0; i < count; i++) {
            sum += adcBuffer[i];
        }
        float avgAdc = (float)sum / (float)count;

        // Convert averaged ADC reading → millivolts
        //
        // V_adc = (avgAdc / 4096) * 3300 mV
        // V_batt = V_adc * divider_ratio
        //
        // Combined:
        // batteryMv = (avgAdc / 4096) * 3300 * ratio
        //
        batteryMv = (uint16_t)((avgAdc / ADC_STEPS_FULL_SCALE) * ADC_VREF_MV *
                               VBAT_DIVIDER_RATIO);
    }

    // -------------------------------------------------------
    // BMP280 Altitude Reading
    // -------------------------------------------------------
    if (bmp_ok && (now - lastBmpRead >= BMP_READ_INTERVAL_MS)) {
        lastBmpRead = now;

        float rawAlt =
            bmp.readAltitude(1013.25f);  // standard sea-level pressure hPa

        // Capture home altitude baseline on first valid reading
        if (!baseSet) {
            if (rawAlt != 0.0f) {
                baseAlt = rawAlt;
                baseSet = true;
            }
        }

        if (baseSet) {
            // Relative altitude above home point in metres
            float relativeAlt = rawAlt - baseAlt;

            if (!altFilterInit) {
                // Seed the filter on first reading
                filteredAlt = relativeAlt;
                prevFilteredAlt = relativeAlt;
                prevAltTimestamp = now;
                altFilterInit = true;
            } else {
                // Apply exponential low-pass filter
                filteredAlt = (ALT_FILTER_ALPHA * relativeAlt) +
                              ((1.0f - ALT_FILTER_ALPHA) * filteredAlt);

                // Calculate vertical speed in cm/s
                uint32_t dt = now - prevAltTimestamp;
                if (dt > 0) {
                    float altChangeCm =
                        (filteredAlt - prevFilteredAlt) * 100.0f;
                    float dtSeconds = (float)dt / 1000.0f;
                    vSpeedCms = (int16_t)(altChangeCm / dtSeconds);
                }

                prevFilteredAlt = filteredAlt;
                prevAltTimestamp = now;
            }
        }
    }

    // -------------------------------------------------------
    // CRSF Telemetry Transmit
    // Sends battery, altitude, and vario frames back to
    // receiver over Serial2 (owned by CrossfireELRS)
    // -------------------------------------------------------
    if (now - lastTelemetrySend >= TELEMETRY_SEND_INTERVAL_MS) {
        lastTelemetrySend = now;
        sendBattery();
        sendAltitude();
        sendVario();
    }
}

// -------------------------------------------------------
// readBatteryMv() — Returns battery voltage in millivolts
// -------------------------------------------------------
uint16_t Telemetry::readBatteryMv() { return batteryMv; }

// -------------------------------------------------------
// readAltitudeCm() — Returns altitude in centimeters
// relative to home point
// -------------------------------------------------------
int32_t Telemetry::readAltitudeCm() {
    if (!bmp_ok || !baseSet || !altFilterInit) return 0;
    return (int32_t)(filteredAlt * 100.0f);
}

// -------------------------------------------------------
// readVSpeedCms() — Returns vertical speed in cm/s
// positive = climbing
// -------------------------------------------------------
int16_t Telemetry::readVSpeedCms() {
    if (!bmp_ok || !altFilterInit) return 0;
    return vSpeedCms;
}

// -------------------------------------------------------
// readPressurePa() — Returns raw pressure in Pascals
// -------------------------------------------------------
float Telemetry::readPressurePa() {
    if (!bmp_ok) return 0.0f;
    return bmp.readPressure();
}

// -------------------------------------------------------
// readTemperatureC() — Returns temperature in Celsius
// -------------------------------------------------------
float Telemetry::readTemperatureC() {
    if (!bmp_ok) return 0.0f;
    return bmp.readTemperature();
}

// -------------------------------------------------------
// sendBattery() — Send CRSF battery sensor frame
//
// CRSF Battery Frame Payload (8 bytes):
// [0-1] Voltage (uint16 big-endian, units: 0.1V)
// [2-3] Current (uint16 big-endian, units: 0.1A)
// [4-6] Capacity (uint24 big-endian, units: mAh)
// [7] Remaining (uint8, units: %)
// -------------------------------------------------------
void Telemetry::sendBattery() {
    uint8_t payload[8];

    // batteryMv is in millivolts; divide by 100 for 0.1V units
    // e.g. 16400 mV → 164 (= 16.4V in 0.1V units)
    uint16_t voltage = batteryMv / 100;
    payload[0] = (voltage >> 8) & 0xFF;  // high byte
    payload[1] = voltage & 0xFF;         // low byte

    // Current: not measured — send 0.0A
    payload[2] = 0x00;
    payload[3] = 0x00;

    // Capacity used: not tracked — send 0 mAh (3 bytes big-endian)
    payload[4] = 0x00;
    payload[5] = 0x00;
    payload[6] = 0x00;

    // Battery remaining: not tracked — send 0%
    payload[7] = 0x00;

    sendFrame(CRSF_FRAMETYPE_BATTERY_SENSOR, payload, sizeof(payload));
}

// -------------------------------------------------------
// sendAltitude() — Send CRSF barometric altitude frame
//
// CRSF Baro Altitude Frame Payload (3 bytes):
//   [0-1] Altitude  (int16 big-endian, units: 0.1m, offset +10000)
//   [2]   VSpeed    (int8, units: 0.1 m/s, signed, clamped -127..+127)
//
// The offset of +10000 means 0m = 10000, -100m = 9000, +100m = 11000
// This allows negative altitudes to be represented as positive integers
// -------------------------------------------------------
void Telemetry::sendAltitude() {
    uint8_t payload[3];

    // Convert altitude from cm to 0.1m units, then apply +10000 offset
    // e.g. 150cm = 1.5m → 15 (0.1m units) + 10000 = 10015
    int32_t altCm = readAltitudeCm();
    float altM = altCm / 100.0f;
    float altTenths = altM * 10.0f;  // convert to 0.1m units
    int16_t altField = (int16_t)(altTenths + 10000.0f);  // apply CRSF offset

    payload[0] = (altField >> 8) & 0xFF;  // high byte
    payload[1] = altField & 0xFF;         // low byte

    // Vertical speed: cm/s → 0.1 m/s units
    // e.g. 150 cm/s = 1.5 m/s → 15 (in 0.1 m/s units)
    // Clamped explicitly to int8 range (-127 to +127) to avoid silent overflow
    int16_t vspeedTenths = (int16_t)(vSpeedCms / 10);
    vspeedTenths = constrain(vspeedTenths, -127, 127);
    payload[2] = (int8_t)vspeedTenths;

    sendFrame(CRSF_FRAMETYPE_BARO_ALTITUDE, payload, sizeof(payload));
}

// -------------------------------------------------------
// sendVario() — Send CRSF vario (vertical speed) frame
//
// CRSF Vario Frame Payload (2 bytes):
//   [0-1] VSpeed (int16 big-endian, units: cm/s, signed)
//
// This is a separate dedicated vario frame — some
// receivers and OSD systems prefer this over the vspeed
// field embedded in the baro altitude frame
// -------------------------------------------------------
void Telemetry::sendVario() {
    uint8_t payload[2];

    // Send vertical speed directly in cm/s — no conversion needed
    int16_t vspeed = readVSpeedCms();
    payload[0] = (vspeed >> 8) & 0xFF;  // high byte
    payload[1] = vspeed & 0xFF;         // low byte

    sendFrame(CRSF_FRAMETYPE_VARIO, payload, sizeof(payload));
}

// -------------------------------------------------------
// sendFrame() — Build and transmit a complete CRSF frame
//
// CRSF Frame Structure:
//   [0]     Destination address  (0xC8 = flight controller)
//   [1]     Frame length         (number of bytes after this field)
//   [2]     Frame type           (e.g. 0x08 = battery)
//   [3..N]  Payload bytes
//   [N+1]   CRC8 checksum       (covers frame type + payload only)
//
// NOTE: Serial2 is owned by CrossfireELRS — we use it
//       here for telemetry TX only, never reconfigure it
// -------------------------------------------------------
void Telemetry::sendFrame(uint8_t frameType, const uint8_t* payload,
                          uint8_t payloadLen) {
    // Total frame length field = type byte (1) + payload + CRC (1)
    uint8_t frameLen = 1 + payloadLen + 1;

    // Build CRC over frame type + payload bytes only
    // (destination address and frame length are NOT included in CRC)
    uint8_t crcBuf[64];
    crcBuf[0] = frameType;
    for (uint8_t i = 0; i < payloadLen; i++) {
        crcBuf[1 + i] = payload[i];
    }
    uint8_t crc = crsf_crc8(crcBuf, 1 + payloadLen);

    // Transmit complete frame over Serial2
    // Serial2 is configured by CrossfireELRS — do NOT reinitialise here
    Serial2.write(CRSF_ADDRESS_FLIGHT_CONTROLLER);  // [0] destination
    Serial2.write(frameLen);                        // [1] frame length
    Serial2.write(frameType);                       // [2] frame type
    for (uint8_t i = 0; i < payloadLen; i++) {
        Serial2.write(payload[i]);  // [3..N] payload
    }
    Serial2.write(crc);  // [N+1] checksum
}

