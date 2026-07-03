/*
bf_msp.cpp — Betaflight MSP Protocol Implementation

Compatible with:
- DJI V1 Air Unit — native MSP voltage/status display only
NO DisplayPort OSD overlay sent to V1
- DJI O3 / O4 — full DisplayPort OSD overlay
- Walksnail Avatar — full DisplayPort OSD overlay
+ heartbeat to suppress disconnect message

MSP v1 little-endian byte order throughout.
FC variant must be "BTFL" for all three systems.

OSD canvas safe area: 50 cols x 18 rows
Fits within both O3/O4 (53x20) and Avatar (50x18).
*/

#include "Telemetry.h"
#include "bf_msp.h"
#include "SharedTelemetry.h"
#include "config.h"


// -------------------------------------------------------
// Module-level references
// -------------------------------------------------------
static HardwareSerial* _serial = nullptr;
static Telemetry* _telemetry = nullptr;
static CrossfireELRS* _elrs = nullptr;

// -------------------------------------------------------
// MSP Command IDs
// -------------------------------------------------------
#define MSP_API_VERSION 1
#define MSP_FC_VARIANT 2
#define MSP_FC_VERSION 3
#define MSP_FC_NAME 10
#define MSP_STATUS 101
#define MSP_STATUS_EX 150
#define MSP_ANALOG 110
#define MSP_ALTITUDE 109
#define MSP_OSD_CONFIG 84
#define MSP_DISPLAYPORT 182
#define MSP_RAW_IMU         102   // 0x5C — accelerometer/gyro raw data
#define MSP_SERVO           94    // 0x5E — servo positions
#define MSP_RC              105   // 0x69 — RC channel values
#define MSP_RC_TUNING       111   // 0x6F — rate/expo settings
#define MSP_PID             112   // 0x70 — PID values
#define MSP_BATTERY_STATE   130   // 0x82 — battery voltage/state (DJI native voltage source)

// -------------------------------------------------------
// MSP DisplayPort Sub-commands
// -------------------------------------------------------
#define MSP_DP_HEARTBEAT 0
#define MSP_DP_RELEASE 1
#define MSP_DP_CLEAR_SCREEN 2
#define MSP_DP_WRITE_STRING 3
#define MSP_DP_DRAW_SCREEN 4

// -------------------------------------------------------
// Betaflight Version Constants
// -------------------------------------------------------
#define BF_VERSION_MAJOR 4
#define BF_VERSION_MINOR 4
#define BF_VERSION_PATCH 0
#define BF_API_VERSION_MAJOR 1
#define BF_API_VERSION_MINOR 45

// -------------------------------------------------------
// Sensor and Arming Flags
// -------------------------------------------------------
#define SENSOR_BARO (1 << 1)
#define ARMING_DISABLE_NO_GYRO (1 << 0)

// -------------------------------------------------------
// Little-endian packing macros (MSP v1 spec)
// -------------------------------------------------------
#define PACK_U16_LE(p, idx, val)                       \
    do {                                               \
        (p)[(idx)++] = (uint8_t)((val) & 0xFF);        \
        (p)[(idx)++] = (uint8_t)(((val) >> 8) & 0xFF); \
    } while (0)

#define PACK_U32_LE(p, idx, val)                        \
    do {                                                \
        (p)[(idx)++] = (uint8_t)((val) & 0xFF);         \
        (p)[(idx)++] = (uint8_t)(((val) >> 8) & 0xFF);  \
        (p)[(idx)++] = (uint8_t)(((val) >> 16) & 0xFF); \
        (p)[(idx)++] = (uint8_t)(((val) >> 24) & 0xFF); \
    } while (0)

// OSD element position encoding for MSP_OSD_CONFIG
#define OSD_POS(visible, row, col)                                   \
    (uint16_t)(((visible) ? (1 << 11) : 0) | (((row) & 0x1F) << 5) | \
               ((col) & 0x1F))

// -------------------------------------------------------
// OSD canvas safe area
// 50 cols x 18 rows — fits both O3/O4 (53x20) and Avatar (50x18)
// -------------------------------------------------------
#define OSD_SAFE_COLS 50
#define OSD_SAFE_ROWS 18

// -------------------------------------------------------
// MSP Request Parser State Machine
// -------------------------------------------------------
typedef enum {
    MSP_PARSE_IDLE,
    MSP_PARSE_HEADER_M,
    MSP_PARSE_DIRECTION,
    MSP_PARSE_LEN,
    MSP_PARSE_CMD,
    MSP_PARSE_PAYLOAD,
    MSP_PARSE_CHECKSUM
} MspParseState;

static MspParseState _parseState = MSP_PARSE_IDLE;
static uint8_t _parseLen = 0;
static uint8_t _parseCmd = 0;
static uint8_t _parseChecksum = 0;
static uint8_t _parseIdx = 0;
static uint8_t _parseBuf[64];

static bool gpsFlashState = false;
static uint32_t gpsFlashMs = 0;

// -------------------------------------------------------
// Internal module state
// -------------------------------------------------------
static bool _isArmed = false;
static bool _initialised = false;

static uint32_t _firstbeatMs = 0;
static uint8_t _firstbeatStep = 0;
static bool _firstbeatPending = false;

static uint32_t _lastHeartbeatMs = 0;
static uint8_t _heartbeatStep = 0;

static uint32_t _lastDpHeartbeatMs = 0;

static uint8_t _osdStep = 0;
static uint32_t _osdStepMs = 0;

static bool _seenApiVersion = false;
static bool _seenFcName = false;
static bool _seenStatusEx = false;
static bool _seenOsdConfig = false;
static bool _seenStatus = false;
static bool _seenAttitude   = false;   // 0x6A
static bool _seenDisplayPort = false;  // 0xB6 (Walksnail only)
static bool _seenV1Only = false;
static uint16_t _statusCount = 0;

// Tracks whether DisplayPort canvas ownership has been claimed
static bool _dpReleased = false;

// VTX type — declared in bf_msp.h, used here as module variable
static VtxSystemType _vtxType = VTX_UNKNOWN;

// Radar smoothing state
static float radarBearingSmooth = 0.0f;
static float radarRadiusSmooth = 0.0f;
static int lastRowH = 0;
static int lastColH = 0;


// -------------------------------------------------------
// Timing constants
// -------------------------------------------------------
#define MSP_FIRSTBEAT_GAP_MS 10
#define MSP_FRAME_GAP_MS 5
#define OSD_FRAME_GAP_MS 2
#define MSP_DP_HEARTBEAT_MS 150

// -------------------------------------------------------
// Forward declarations
// -------------------------------------------------------
static void bf_msp_send(uint8_t cmd, const uint8_t* payload, uint8_t len);
static void bf_msp_send_api_version();
static void bf_msp_send_fc_variant();
static void bf_msp_send_fc_version();
static void bf_msp_send_fc_name();
static void bf_msp_send_status(bool armed);
static void bf_msp_send_status_ex(bool armed);
static void bf_msp_send_analog();
static void bf_msp_send_altitude();
static void bf_msp_send_osd_config();
static void bf_msp_handle_request(uint8_t cmd);
static void bf_msp_detect_vtx(uint8_t cmd);
static void bf_msp_dp_heartbeat();
static void bf_msp_dp_release();
static void bf_msp_dp_clear();
static void bf_msp_dp_draw();
static void bf_msp_dp_write(uint8_t row, uint8_t col, const char* text,
                            uint8_t attr);
static void bf_msp_send_raw_imu();
static void bf_msp_send_servo();
static void bf_msp_send_rc();
static void bf_msp_send_rc_tuning();
static void bf_msp_send_pid();
static void bf_msp_send_battery_state();

// -------------------------------------------------------
// DEBUG ONLY — hex dump an MSP payload before sending
// Remove once native OSD voltage is confirmed working
// -------------------------------------------------------
static void debug_dump_payload(const char* label,
                                uint8_t cmd,
                                const uint8_t* p,
                                uint8_t len) {
    if (!Serial) return;
    Serial.print("[MSP DUMP] ");
    Serial.print(label);
    Serial.print(" cmd=");
    Serial.print(cmd, DEC);
    Serial.print(" len=");
    Serial.print(len, DEC);
    Serial.print(" bytes=");
    for (uint8_t i = 0; i < len; i++) {
        if (p[i] < 0x10) Serial.print("0");
        Serial.print(p[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// -------------------------------------------------------
// MSP_RAW_IMU (102) — 18 bytes little-endian
// Accelerometer (3x int16) + Gyro (3x int16) + Mag (3x int16)
// DJI polls this — return zeroed data, we have no IMU
// -------------------------------------------------------
static void bf_msp_send_raw_imu() {
    uint8_t p[18];
    memset(p, 0, sizeof(p));
    bf_msp_send(MSP_RAW_IMU, p, sizeof(p));
}

// -------------------------------------------------------
// MSP_SERVO (94) — 16 bytes little-endian
// 8x uint16 servo positions (1000-2000us each)
// DJI polls this — return midpoint values
// -------------------------------------------------------
static void bf_msp_send_servo() {
    uint8_t p[16];
    uint8_t idx = 0;
    for (uint8_t i = 0; i < 8; i++) {
        PACK_U16_LE(p, idx, (uint16_t)1500);   // midpoint
    }
    bf_msp_send(MSP_SERVO, p, idx);
}

// -------------------------------------------------------
// MSP_RC (105) — 32 bytes little-endian
// 16x uint16 RC channel values (1000-2000us each)
// DJI polls this — return midpoint values
// -------------------------------------------------------
static void bf_msp_send_rc() {
    uint8_t p[32];
    uint8_t idx = 0;
    for (uint8_t i = 0; i < 16; i++) {
        PACK_U16_LE(p, idx, (uint16_t)1500);   // midpoint
    }
    bf_msp_send(MSP_RC, p, idx);
}

// -------------------------------------------------------
// MSP_RC_TUNING (111) — 14 bytes
// RC rate, expo, and throttle settings
// DJI polls this — return safe defaults
// -------------------------------------------------------
static void bf_msp_send_rc_tuning() {
    uint8_t p[14];
    uint8_t idx = 0;

    p[idx++] = 100;    // RC rate
    p[idx++] = 0;      // RC expo
    p[idx++] = 100;    // roll rate
    p[idx++] = 100;    // pitch rate
    p[idx++] = 100;    // yaw rate
    p[idx++] = 0;      // dynamic throttle PID
    p[idx++] = 0;      // throttle mid
    p[idx++] = 0;      // throttle expo
    PACK_U16_LE(p, idx, (uint16_t)3200);  // TPA breakpoint
    p[idx++] = 0;      // RC yaw expo
    p[idx++] = 0;      // RC yaw rate
    p[idx++] = 0;      // RC pitch rate
    p[idx++] = 0;      // RC pitch expo

    bf_msp_send(MSP_RC_TUNING, p, idx);
}

// -------------------------------------------------------
// MSP_PID (112) — 30 bytes
// 10x PID sets, 3 bytes each (P, I, D)
// DJI polls this — return safe defaults
// -------------------------------------------------------
static void bf_msp_send_pid() {
    uint8_t p[30];
    memset(p, 0, sizeof(p));

    // Roll PID — bytes 0-2
    p[0] = 45;   // P
    p[1] = 40;   // I
    p[2] = 20;   // D

    // Pitch PID — bytes 3-5
    p[3] = 47;
    p[4] = 40;
    p[5] = 22;

    // Yaw PID — bytes 6-8
    p[6] = 45;
    p[7] = 45;
    p[8] = 0;

    bf_msp_send(MSP_PID, p, sizeof(p));
}

// -------------------------------------------------------
// MSP_BATTERY_STATE (130) — 9 bytes
// Polled by DJI O3/O4 for native battery voltage display.
//
// Byte layout:
//   0     uint8   cell count
//   1-2   uint16  battery capacity mAh
//   3     uint8   battery voltage in 0.1V units
//   4-5   uint16  drawn mAh
//   6-7   uint16  current in 0.01A units
//   8     uint8   battery state
//                 0 = OK
//                 1 = WARNING  (below 3.5V per cell)
//                 2 = CRITICAL (below 3.3V per cell)
//                 3 = NOT PRESENT
// -------------------------------------------------------
static void bf_msp_send_battery_state() {
    uint8_t p[9];
    uint8_t idx = 0;

    uint16_t battMv = (_telemetry != nullptr)
                      ? _telemetry->readBatteryMv() : 0;

    // Detect cell count from voltage
    // >12.6V = 4S, >9.0V = 3S, otherwise 2S
    uint8_t cellCount;
    if      (battMv > 12600) cellCount = 4;
    else if (battMv >  9000) cellCount = 3;
    else                     cellCount = 2;

    // Voltage in 0.1V units — 11906mV / 100 = 119 = 11.9V
    uint8_t vbat = (uint8_t)constrain(battMv / 100, 0, 255);

    // Per-cell voltage for state determination
    uint16_t cellMv = (cellCount > 0) ? (battMv / cellCount) : 0;

    uint8_t battState;
    if      (cellMv < 3300) battState = 2;   // CRITICAL
    else if (cellMv < 3500) battState = 1;   // WARNING
    else                    battState = 0;   // OK

    p[idx++] = cellCount;                       // byte 0: cell count
    PACK_U16_LE(p, idx, (uint16_t)0);           // bytes 1-2: capacity mAh unknown
    p[idx++] = vbat;                            // byte 3: voltage in 0.1V units
    PACK_U16_LE(p, idx, (uint16_t)0);           // bytes 4-5: drawn mAh
    PACK_U16_LE(p, idx, (uint16_t)0);           // bytes 6-7: current 0.01A
    p[idx++] = battState;                       // byte 8: battery state

    //debug_dump_payload("BATTERY_STATE", MSP_BATTERY_STATE, p, idx);

    bf_msp_send(MSP_BATTERY_STATE, p, idx);
}

// =======================================================
// PRIVATE — VTX Detection
// Called each time a valid MSP request is received.
// =======================================================
static void bf_msp_detect_vtx(uint8_t cmd) {
    if (_vtxType != VTX_UNKNOWN) return;

    static uint32_t firstMs = 0;
    static bool started = false;

    if (!started) {
        started = true;
        firstMs = millis();
    }

    uint32_t elapsed = millis() - firstMs;

    if (cmd == MSP_STATUS_EX)    _seenStatusEx = true;
    if (cmd == MSP_DISPLAYPORT)  _seenDisplayPort = true;

    // V1-only commands
    if (cmd == 0x6A || cmd == 0x6B || cmd == 0x6C ||
        cmd == 0x6D || cmd == 0x86 || cmd == 0xF7) {
        _seenV1Only = true;
    }

    // Count MSP_STATUS floods
    if (cmd == MSP_STATUS) {
        _statusCount++;
    }

    // 1. Walksnail — flood of STATUS, no STATUS_EX, no V1-only commands
    if (_statusCount > 20 && !_seenStatusEx && !_seenV1Only) {
        _vtxType = VTX_WALKSNAIL;
        Serial.println("[MSP] VTX detected: Walksnail Avatar");
        return;
    }

    // 2. DJI V1 — STATUS_EX + V1-only commands
    if (_seenStatusEx && _seenV1Only) {
        _vtxType = VTX_DJI_V1;
        Serial.println("[MSP] VTX detected: DJI V1");
        return;
    }

    // 3. DJI O3/O4 — STATUS_EX but no V1-only commands
    if (_seenStatusEx && !_seenV1Only && elapsed > 300) {
        _vtxType = VTX_DJI_O3;
        Serial.println("[MSP] VTX detected: DJI O3/O4");
        return;
    }
}


// =======================================================
// PRIVATE — Core MSP Frame Builder
// Builds and sends a complete MSP v1 response frame.
// Format: $ M > [len] [cmd] [payload...] [checksum]
// Checksum = XOR of len, cmd, and all payload bytes.
// =======================================================
static void bf_msp_send(uint8_t cmd, const uint8_t* payload, uint8_t len) {
    if (!_serial) return;

    uint8_t checksum = 0;
    checksum ^= len;
    checksum ^= cmd;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= payload[i];
    }

    _serial->write('$');
    _serial->write('M');
    _serial->write('>');
    _serial->write(len);
    _serial->write(cmd);
    for (uint8_t i = 0; i < len; i++) {
        _serial->write(payload[i]);
    }
    _serial->write(checksum);
}

// =======================================================
// PRIVATE — MSP Response Builders
// =======================================================

static void bf_msp_send_api_version() {
    uint8_t p[3];
    p[0] = 0;
    p[1] = BF_API_VERSION_MAJOR;
    p[2] = BF_API_VERSION_MINOR;
    bf_msp_send(MSP_API_VERSION, p, sizeof(p));
}

static void bf_msp_send_fc_variant() {
    // Must be "BTFL" — all three VTX systems require this exact string
    uint8_t p[4] = {'B', 'T', 'F', 'L'};
    bf_msp_send(MSP_FC_VARIANT, p, sizeof(p));
}

static void bf_msp_send_fc_version() {
    uint8_t p[3];
    p[0] = BF_VERSION_MAJOR;
    p[1] = BF_VERSION_MINOR;
    p[2] = BF_VERSION_PATCH;
    bf_msp_send(MSP_FC_VERSION, p, sizeof(p));
}

static void bf_msp_send_fc_name() {
    const char* name = "BTFL";
    bf_msp_send(MSP_FC_NAME, (const uint8_t*)name, (uint8_t)strlen(name));
}

// -------------------------------------------------------
// MSP_STATUS (101) — 11 bytes little-endian
// -------------------------------------------------------
static void bf_msp_send_status(bool armed) {
    uint8_t p[11];
    uint8_t idx = 0;

    PACK_U16_LE(p, idx, (uint16_t)1000);         // cycle time
    PACK_U16_LE(p, idx, (uint16_t)0);            // i2c errors
    PACK_U16_LE(p, idx, (uint16_t)(SENSOR_BARO | (1 << 5)));  // sensors

    //uint32_t flags = armed ? 1 : 0;
    uint32_t flags = armed ? ((1 << 0) | (1 << 2)) : 0;

    PACK_U32_LE(p, idx, flags);  // arming flags

    p[idx++] = 0;  // current profile

    bf_msp_send(MSP_STATUS, p, idx);
}

// -------------------------------------------------------
// MSP_STATUS_EX (150) — 17 bytes little-endian
//
// Standard BF layout (15 bytes):
//   0-1   uint16  cycle time (us)
//   2-3   uint16  i2c error count
//   4-5   uint16  sensor flags
//   6-9   uint32  flight mode flags (bit 0 = armed)
//   10    uint8   current PID profile index
//   11-12 uint16  average system load %
//   13-14 uint16  arming disable flags
//
// DJI O3/O4 extended (2 extra bytes):
//   15-16 uint16  battery voltage in mV (DJI native voltage source)
//
// Adding the extra uint16 battMv is safe — BF ignores extra bytes
// and DJI O3/O4 reads it for the native voltage overlay.
// -------------------------------------------------------
static void bf_msp_send_status_ex(bool armed) {
    uint8_t p[17];
    uint8_t idx = 0;

    // Bytes 0-1: cycle time
    PACK_U16_LE(p, idx, (uint16_t)1000);

    // Bytes 2-3: i2c error count
    PACK_U16_LE(p, idx, (uint16_t)0);

    // Bytes 4-5: sensor flags
    PACK_U16_LE(p, idx, (uint16_t)SENSOR_BARO);

    // Bytes 6-9: flight mode flags (bit 0 = armed)
    uint32_t flags = armed ? 1 : 0;
    PACK_U32_LE(p, idx, flags);

    // Byte 10: current PID profile index
    p[idx++] = 0;

    // Bytes 11-12: average system load %
    PACK_U16_LE(p, idx, (uint16_t)0);

    // Bytes 13-14: arming disable flags
    uint16_t armFlags = armed ? 0 : (uint16_t)ARMING_DISABLE_NO_GYRO;
    PACK_U16_LE(p, idx, armFlags);

    // Bytes 15-16: battery voltage in mV — DJI O3/O4 native voltage source
    // This is an extended field not in base BF spec but read by DJI firmware
    uint16_t battMv = (_telemetry != nullptr)
                      ? _telemetry->readBatteryMv() : 0;
    PACK_U16_LE(p, idx, battMv);

    //debug_dump_payload("STATUS_EX", MSP_STATUS_EX, p, idx);

    bf_msp_send(MSP_STATUS_EX, p, idx);
}

// -------------------------------------------------------
// MSP_ANALOG (110) — 7 bytes little-endian
//
// Byte layout:
//   0     uint8   battery voltage in 0.1V units
//                 e.g. 11906 mV / 100 = 119 = 11.9V
//   1-2   uint16  mAh drawn
//   3-4   uint16  RSSI 0-1023
//   5-6   uint16  current in 0.01A units
//
// DJI O3/O4 reads native voltage display from this frame.
// -------------------------------------------------------
static void bf_msp_send_analog() {
    uint8_t p[7];
    uint8_t idx = 0;

    uint16_t battMv = (_telemetry != nullptr)
                      ? _telemetry->readBatteryMv() : 0;

    // Convert millivolts to 0.1V units
    // 11906 mV / 100 = 119 = 11.9V
    uint8_t vbat = (uint8_t)constrain(battMv / 100, 0, 255);

    p[idx++] = vbat;                        // battery voltage in 0.1V units
    PACK_U16_LE(p, idx, (uint16_t)0);       // mAh drawn
    PACK_U16_LE(p, idx, (uint16_t)0);       // rssi 0-1023
    PACK_U16_LE(p, idx, (uint16_t)0);       // current in 0.01A units

    //debug_dump_payload("ANALOG", MSP_ANALOG, p, idx);

    bf_msp_send(MSP_ANALOG, p, idx);
}

// -------------------------------------------------------
// MSP_ALTITUDE (109) — 6 bytes little-endian
// Altitude in cm (int32) + vspeed in cm/s (int16)
// -------------------------------------------------------
static void bf_msp_send_altitude() {
    uint8_t p[6];
    uint8_t idx = 0;

    int32_t altCm =
        (_telemetry != nullptr) ? (int32_t)_telemetry->readAltitudeCm() : 0;
    int16_t vspeedCms =
        (_telemetry != nullptr) ? (int16_t)_telemetry->readVSpeedCms() : 0;

    // int32 altitude — little-endian 4 bytes
    p[idx++] = (uint8_t)(altCm & 0xFF);
    p[idx++] = (uint8_t)((altCm >> 8) & 0xFF);
    p[idx++] = (uint8_t)((altCm >> 16) & 0xFF);
    p[idx++] = (uint8_t)((altCm >> 24) & 0xFF);

    // int16 vertical speed — little-endian 2 bytes
    p[idx++] = (uint8_t)(vspeedCms & 0xFF);
    p[idx++] = (uint8_t)((vspeedCms >> 8) & 0xFF);

    bf_msp_send(MSP_ALTITUDE, p, idx);
}

// -------------------------------------------------------
// MSP_OSD_CONFIG (84)
// Tells the VTX which OSD elements are enabled and where
// they are positioned on the canvas.
// Format varies slightly between systems but this layout
// works with both DJI and Walksnail.
// -------------------------------------------------------
static void bf_msp_send_osd_config() {
    uint8_t p[64];
    uint8_t idx = 0;

    p[idx++] = 0x01;  // OSD feature flags — bit 0 = OSD enabled
    p[idx++] = 0x00;  // video system — AUTO
    p[idx++] = 0x00;  // units — metric
    p[idx++] = 0x01;  // warn rssi
    p[idx++] = 0x01;  // warn battery
    p[idx++] = 0x01;  // warn failsafe

    // OSD element positions — each is uint16 little-endian
    // Format: bit11=visible, bits10-5=row, bits4-0=col
    // Using OSD_POS(visible, row, col) macro

    uint16_t positions[] = {
        OSD_POS(1, 1, 10),    // voltage
        OSD_POS(1, 2, 10),   // altitude
        OSD_POS(1, 3, 10),    // vertical speed
        OSD_POS(1, 17, 20),   // arm state
        OSD_POS(1, 0, 10),  // flight mode
        OSD_POS(1, 8, 15)  // crosshair
    };

    uint8_t numElements = sizeof(positions) / sizeof(positions[0]);
    p[idx++] = numElements;

    for (uint8_t i = 0; i < numElements; i++) {
        PACK_U16_LE(p, idx, positions[i]);
    }

    bf_msp_send(MSP_OSD_CONFIG, p, idx);
}

// =======================================================
// PRIVATE — DisplayPort Helpers
// These build MSP_DISPLAYPORT (182) sub-command frames.
// Used only for DJI O3/O4 and Walksnail Avatar.
// DJI V1 does not use DisplayPort.
// =======================================================

// Heartbeat — keeps Avatar OSD alive, suppresses disconnect warning
static void bf_msp_dp_heartbeat() {
    uint8_t p[1];
    p[0] = MSP_DP_HEARTBEAT;
    bf_msp_send(MSP_DISPLAYPORT, p, 1);
}

// Release — claims canvas ownership, required before first write
static void bf_msp_dp_release() {
    uint8_t p[1];
    p[0] = MSP_DP_RELEASE;
    bf_msp_send(MSP_DISPLAYPORT, p, 1);
}

// Clear — erases the entire OSD canvas
static void bf_msp_dp_clear() {
    uint8_t p[1];
    p[0] = MSP_DP_CLEAR_SCREEN;
    bf_msp_send(MSP_DISPLAYPORT, p, 1);
}

// Draw — commits all pending writes and renders them
static void bf_msp_dp_draw() {
    uint8_t p[1];
    p[0] = MSP_DP_DRAW_SCREEN;
    bf_msp_send(MSP_DISPLAYPORT, p, 1);
}

// Write — places a text string at a given row and column
// attr: 0 = normal, 1 = blink
static void bf_msp_dp_write(uint8_t row, uint8_t col, const char* text,
                            uint8_t attr) {
    if (!text) return;

    uint8_t textLen = (uint8_t)strlen(text);
    uint8_t p[64];
    uint8_t idx = 0;

    p[idx++] = MSP_DP_WRITE_STRING;
    p[idx++] = row;
    p[idx++] = col;
    p[idx++] = attr;

    for (uint8_t i = 0; i < textLen && idx < sizeof(p) - 1; i++) {
        p[idx++] = (uint8_t)text[i];
    }

    bf_msp_send(MSP_DISPLAYPORT, p, idx);
}

// =======================================================
// PRIVATE — MSP Request Handler
// Responds to each incoming MSP poll from the Air Unit.
// Unknown commands receive an empty response so the
// Air Unit does not time out waiting for a reply.
// =======================================================
static void bf_msp_handle_request(uint8_t cmd) {
    switch (cmd) {
        case MSP_API_VERSION:    bf_msp_send_api_version();        break;
        case MSP_FC_VARIANT:     bf_msp_send_fc_variant();         break;
        case MSP_FC_VERSION:     bf_msp_send_fc_version();         break;
        case MSP_FC_NAME:        bf_msp_send_fc_name();            break;
        case MSP_STATUS:         bf_msp_send_status(_isArmed);     break;
        case MSP_STATUS_EX:      bf_msp_send_status_ex(_isArmed);  break;
        case MSP_RAW_IMU:        bf_msp_send_raw_imu();            break;
        case MSP_SERVO:          bf_msp_send_servo();              break;
        case MSP_RC:             bf_msp_send_rc();                 break;
        case MSP_ANALOG:         bf_msp_send_analog();             break;
        case MSP_RC_TUNING:      bf_msp_send_rc_tuning();          break;
        case MSP_PID:            bf_msp_send_pid();                break;
        case MSP_ALTITUDE:       bf_msp_send_altitude();           break;
        case MSP_OSD_CONFIG:     bf_msp_send_osd_config();         break;
        case MSP_BATTERY_STATE:  bf_msp_send_battery_state();      break;
        default:
            // Return empty response so DJI does not time out
            bf_msp_send(cmd, nullptr, 0);
            break;
    }
}

// =======================================================
// PUBLIC — Initialisation
// Call once from setup() before any other bf_msp function.
// =======================================================
void bf_msp_init(HardwareSerial& serial, Telemetry& telemetry, CrossfireELRS& elrs) {
    _serial = &serial;
    _telemetry = &telemetry;
    _elrs = &elrs;
    _isArmed = false;
    _initialised = false;
    _firstbeatPending = false;
    _firstbeatStep = 0;
    _heartbeatStep = 0;
    _osdStep = 0;
    _parseState = MSP_PARSE_IDLE;
    _seenApiVersion = false;
    _seenFcName = false;
    _seenStatusEx = false;
    _dpReleased = false;
    _lastDpHeartbeatMs = 0;
    _vtxType = VTX_UNKNOWN;

    if (Serial) Serial.println("[MSP] Initialised.");
}

// =======================================================
// PUBLIC — MSP Request Parser
// Call every loop() — reads incoming bytes from the Air
// Unit and responds to each complete MSP v1 request.
// =======================================================
void bf_msp_parse_incoming() {
    if (!_serial) return;

    while (_serial->available()) {
        uint8_t c = _serial->read();

        switch (_parseState) {
            case MSP_PARSE_IDLE:
                if (c == '$') _parseState = MSP_PARSE_HEADER_M;
                break;

            case MSP_PARSE_HEADER_M:
                _parseState = (c == 'M') ? MSP_PARSE_DIRECTION : MSP_PARSE_IDLE;
                break;

            case MSP_PARSE_DIRECTION:
                // '<' = request to us, '>' = response — ignore responses
                if (c == '<') {
                    _parseState = MSP_PARSE_LEN;
                    _parseChecksum = 0;
                    _parseIdx = 0;
                } else {
                    _parseState = MSP_PARSE_IDLE;
                }
                break;

            case MSP_PARSE_LEN:
                _parseLen = c;
                _parseChecksum ^= c;
                _parseState = MSP_PARSE_CMD;
                break;

            case MSP_PARSE_CMD:
                _parseCmd = c;
                // Track FC_NAME only when the goggles REQUEST it
                if (_parseCmd == MSP_FC_NAME) {
                    _seenFcName = true;
                }
                _parseChecksum ^= c;
                _parseIdx = 0;
                _parseState =
                    (_parseLen > 0) ? MSP_PARSE_PAYLOAD : MSP_PARSE_CHECKSUM;
                break;

            case MSP_PARSE_PAYLOAD:
                _parseBuf[_parseIdx++] = c;
                _parseChecksum ^= c;
                if (_parseIdx >= _parseLen) {
                    _parseState = MSP_PARSE_CHECKSUM;
                }
                break;

            case MSP_PARSE_CHECKSUM:
                if (c == _parseChecksum) {
                    // Respond first, then update detection state
                    //Serial.print("[MSP] POLL CMD: 0x");
                    //Serial.println(_parseCmd, HEX);
                    bf_msp_handle_request(_parseCmd);
                    bf_msp_detect_vtx(_parseCmd);
                    // In bf_msp_parse_incoming() checksum case — temporary poll logger
                    if (!_initialised) {
                        _initialised = true;
                        _firstbeatPending = true;
                        _firstbeatStep = 0;
                        _firstbeatMs = millis();
                        if (Serial)
                            Serial.println("[MSP] First valid frame received.");
                    }
                }
                _parseState = MSP_PARSE_IDLE;
                break;

            default:
                _parseState = MSP_PARSE_IDLE;
                break;
        }
    }
}

// =======================================================
// PUBLIC — bf_msp_firstbeat_start()
// Called by state_machine.cpp when VTX connection confirmed.
// Triggers the initial config burst sequence.
// bf_msp_firstbeat_update() must be called every loop()
// to step through the sequence after this is called.
// =======================================================
void bf_msp_firstbeat_start() {
    if (!_initialised) {
        if (Serial)
            Serial.println("[MSP] firstbeat_start: not initialised yet.");
        return;
    }
    _firstbeatPending = true;
    _firstbeatStep = 0;
    _firstbeatMs = millis();
    if (Serial) Serial.println("[MSP] Firstbeat sequence triggered.");
}

// =======================================================
// PUBLIC — bf_msp_firstbeat_update()
// Call every loop() — non-blocking.
// Steps through the initial config burst one frame at a time.
// Becomes a no-op automatically when sequence is complete.
// =======================================================
void bf_msp_firstbeat_update() {
    if (!_initialised) return;
    if (!_firstbeatPending) return;

    uint32_t now = millis();
    if (now - _firstbeatMs < MSP_FIRSTBEAT_GAP_MS) return;
    _firstbeatMs = now;

    switch (_firstbeatStep) {
        case 0:
            bf_msp_send_api_version();
            break;
        case 1:
            bf_msp_send_fc_variant();
            break;
        case 2:
            bf_msp_send_fc_version();
            break;
        case 3:
            bf_msp_send_fc_name();
            break;
        case 4:
            bf_msp_send_analog();
            break;
        case 5:
            bf_msp_send_osd_config();
            break;
        case 6:
            bf_msp_send_status(_isArmed);
            break;
        case 7:
            bf_msp_send_status_ex(_isArmed);
            break;

        case 8:
            if (_vtxType == VTX_DJI_O3 || _vtxType == VTX_WALKSNAIL) {
                bf_msp_dp_heartbeat();
            }
            break;
        case 9:
            // Release canvas ownership — required by Walksnail Avatar
            if (_vtxType == VTX_DJI_O3 || _vtxType == VTX_WALKSNAIL) {
                bf_msp_dp_release();
            }
            break;
        default:
            _firstbeatPending = false;
            _firstbeatStep = 0;
            if (Serial) Serial.println("[MSP] Firstbeat sequence complete.");
            return;
    }

    _firstbeatStep++;
}

// =======================================================
// PUBLIC — bf_msp_heartbeat_update()
// Call every loop() — sends periodic status, analog, and
// altitude frames to keep the Air Unit happy.
// Also sends DisplayPort heartbeat to Walksnail Avatar
// to suppress the "disconnected" overlay message.
// =======================================================
void bf_msp_heartbeat_update(bool armed) {
    if (!_initialised) return;
    if (_firstbeatPending) return;

    _isArmed = armed;

    // Update flight timer
    if (_isArmed) {
        if (!sharedTelem.flightTimerRunning) {
            sharedTelem.flightTimerRunning = true;
            sharedTelem.flightStartMs = millis();
            sharedTelem.flightElapsedMs = 0;
        } else {
            sharedTelem.flightElapsedMs = millis() - sharedTelem.flightStartMs;
        }
    } else {
        sharedTelem.flightTimerRunning = false;
        // flightElapsedMs stays frozen
    }


    uint32_t now = millis();
    if (now - _lastHeartbeatMs < MSP_FRAME_GAP_MS) return;
    _lastHeartbeatMs = now;

    switch (_heartbeatStep) {
        case 0:
            bf_msp_send_status(_isArmed);
            break;
        case 1:
            bf_msp_send_status_ex(_isArmed);   // <-- REQUIRED FOR DJI NATIVE VOLTAGE
            break;
        case 2:
            bf_msp_send_analog();
            break;
        case 3:
            bf_msp_send_altitude();
            break;
        case 4:
            if (_vtxType == VTX_WALKSNAIL || _vtxType == VTX_DJI_O3) {
                uint32_t nowDp = millis();
                if (nowDp - _lastDpHeartbeatMs >= MSP_DP_HEARTBEAT_MS) {
                    _lastDpHeartbeatMs = nowDp;
                    bf_msp_dp_heartbeat();
                }
            }
            break;
        default:
            _heartbeatStep = 0;
            return;
    }

    _heartbeatStep++;
}

// Convert row number → glyph character
static inline char glyphFromRow(uint8_t row)
{
    return (char)row;
}


static void formatFlightTime(char* buf, uint32_t elapsedMs) {
    uint32_t totalSec = elapsedMs / 1000;
    uint32_t minutes = (totalSec % 3600) / 60;
    uint32_t seconds = totalSec % 60;

    // Format H:MM:SS
    sprintf(buf, "%02u:%02u", minutes, seconds);
}


// =======================================================
// PUBLIC — bf_msp_dp_update_osd_nb()
// Non-blocking DisplayPort OSD writer.
// Call every loop() — steps through clear/write/draw cycle.
//
// Only active for DJI O3/O4 and Walksnail Avatar.
// DJI V1 is skipped — uses its native MSP display instead.
//
// OSD layout — all within 50 col x 18 row safe canvas:
//   Row  0  col  1  — Battery voltage
//   Row  0  col 25  — Altitude
//   Row  1  col  1  — Vertical speed
//   Row 17  col  1  — Arm state  (bottom left)
//   Row 17  col 30  — Flight mode (bottom right)
// =======================================================
void bf_msp_dp_update_osd_nb() {
    if (!_initialised) return;
    if (_firstbeatPending) return;

    // Skip OSD for DJI V1 — uses native display
    if (_vtxType == VTX_DJI_V1) return;

    // Wait for VTX to be identified before sending any OSD
    if (_vtxType == VTX_UNKNOWN) return;

    uint32_t now = millis();
    if (now - _osdStepMs < OSD_FRAME_GAP_MS) return;
    _osdStepMs = now;

    // Telemetry values — captured fresh at the start of each frame cycle
    static float voltV = 0.0f;
    static float altM = 0.0f;       // meters OR feet depending on config
    static float vspeedMs = 0.0f;   // m/s OR ft/s depending on config
    static bool armed = false;
    static float distM = 0.0f;

    char buf[32];

    switch (_osdStep) {

        // -------------------------------------------------------
        // Step 0 — First frame: RELEASE, then refresh telemetry
        // -------------------------------------------------------
        case 0:
            if (!_dpReleased) {
                bf_msp_dp_release();
                bf_msp_dp_clear(); // Clear ONCE right here when the canvas is first claimed
                _dpReleased = true;
                _osdStep++;
                return;
            }

            // Toggle GPS flash state every 500ms
            if (now - gpsFlashMs > 500) {
                gpsFlashMs = now;
                gpsFlashState = !gpsFlashState;
            }

            // Battery voltage
            voltV = (_telemetry != nullptr)
                        ? _telemetry->readBatteryMv() / 1000.0f
                        : 0.0f;

            // Altitude + vertical speed (metric or imperial)
            {
                float altCm = (_telemetry != nullptr)
                                  ? _telemetry->readAltitudeCm()
                                  : 0.0f;
                float vsCms = (_telemetry != nullptr)
                                  ? _telemetry->readVSpeedCms()
                                  : 0.0f;
                distM = sharedTelem.gpsDistHomeM;

#if OSD_UNITS == OSD_UNITS_IMPERIAL
                altM     = altCm * 0.0328084f;     // feet
                vspeedMs = vsCms * 0.0328084f;     // ft/s
#else
                altM     = altCm / 100.0f;         // meters
                vspeedMs = vsCms / 100.0f;         // m/s
#endif
            }

            armed = _isArmed;
            break;

        // -------------------------------------------------------
        // Step 1 — Battery voltage
        // -------------------------------------------------------
        case 1:
            if (_vtxType == VTX_WALKSNAIL) {
                // Walksnail
                snprintf(buf, sizeof(buf), "%c%5.2f%c", 144, voltV, 6);
                bf_msp_dp_write(1, 10, buf, 0);

            } else if (_vtxType == VTX_DJI_V1 || _vtxType == VTX_DJI_O3) {
                // DJI — no battery icon, so use uppercase V
                snprintf(buf, sizeof(buf), "V:%5.2fV", voltV);
                bf_msp_dp_write(1, 10, buf, 0);

            } else {
                // Unknown VTX — safest fallback is plain ASCII
                snprintf(buf, sizeof(buf), "V:%5.2fV", voltV);
                bf_msp_dp_write(1, 10, buf, 0);
            }
            break;

        // -------------------------------------------------------
        // Step 2 — Altitude
        // -------------------------------------------------------
        case 2:
            if (_vtxType == VTX_WALKSNAIL) {
                // Walksnail — safe uppercase characters only
#if OSD_UNITS == OSD_UNITS_IMPERIAL
                snprintf(buf, sizeof(buf), "%4.0f%c%c", altM, 15, 127);   // feet
#else
                snprintf(buf, sizeof(buf), "%5.1f%c%c", altM, 12, 127);   // meters
#endif
                bf_msp_dp_write(1, 34, buf, 0);

            } else if (_vtxType == VTX_DJI_V1 || _vtxType == VTX_DJI_O3) {
                // DJI — same characters, DJI-safe
#if OSD_UNITS == OSD_UNITS_IMPERIAL
                snprintf(buf, sizeof(buf), "A:%4.0fF", altM);   // feet
#else
                snprintf(buf, sizeof(buf), "A:%5.1fM", altM);   // meters
#endif
                bf_msp_dp_write(1, 34, buf, 0);

            } else {
                // Unknown VTX — safest fallback
#if OSD_UNITS == OSD_UNITS_IMPERIAL
                snprintf(buf, sizeof(buf), "A:%4.0fF", altM);
#else
                snprintf(buf, sizeof(buf), "A:%5.1fM", altM);
#endif
                bf_msp_dp_write(1, 34, buf, 0);
            }
            break;


        // -------------------------------------------------------
        // Step 3 — Vertical speed
        // -------------------------------------------------------
        case 3:
            if (_vtxType == VTX_WALKSNAIL) {
                // Walksnail — safe uppercase characters only
#if OSD_UNITS == OSD_UNITS_IMPERIAL
                snprintf(buf, sizeof(buf), "VS:%+5.1f%c", vspeedMs, 153);   // ft/s
#else
                snprintf(buf, sizeof(buf), "VS:%+5.1f%c", vspeedMs, 159);   // m/s
#endif
                bf_msp_dp_write(2, 10, buf, 0);

            } else if (_vtxType == VTX_DJI_V1 || _vtxType == VTX_DJI_O3) {
                // DJI — same characters, DJI-safe
#if OSD_UNITS == OSD_UNITS_IMPERIAL
                snprintf(buf, sizeof(buf), "VS:%+5.1fF/S", vspeedMs);   // ft/s
#else
                snprintf(buf, sizeof(buf), "VS:%+5.1fM/S", vspeedMs);   // m/s
#endif
                bf_msp_dp_write(2, 10, buf, 0);

            } else {
                // Unknown VTX — safest fallback
#if OSD_UNITS == OSD_UNITS_IMPERIAL
                snprintf(buf, sizeof(buf), "VS:%+5.1fF/S", vspeedMs);
#else
                snprintf(buf, sizeof(buf), "VS:%+5.1fM/S", vspeedMs);
#endif
                bf_msp_dp_write(2, 10, buf, 0);
            }
            break;

        // -------------------------------------------------------
        // Step 4 — Link Quality
        // -------------------------------------------------------
        case 4: {
            uint8_t lq = (_elrs != nullptr) ? _elrs->getLinkQuality() : 0;

            if (_vtxType == VTX_WALKSNAIL) {
                snprintf(buf, sizeof(buf), "%3u%%%c", lq, 123);   // Walksnail RSSI icon
                bf_msp_dp_write(0, 23, buf, 0);

            } else {
                snprintf(buf, sizeof(buf), "%3u%%%c", lq, 1);
                bf_msp_dp_write(0, 23, buf, 0);
            }
            break;
        }

        // -------------------------------------------------------
        // Distance From Home
        // -------------------------------------------------------
        case 5: {
            float d = distM;
            char dbuf[16];
        
        #if OSD_UNITS == OSD_UNITS_IMPERIAL
            float ft = d * 3.28084f;
            if (_vtxType == VTX_WALKSNAIL) {
                if (ft < 5280.0f) {
                snprintf(dbuf, sizeof(dbuf), "%3.0f%c%c", ft, 15, 5);
                } else {
                    float mi = ft / 5280.0f;
                    snprintf(dbuf, sizeof(dbuf), "%1.2f%c%c", mi, 126, 5);
                }
            } else { // DJI
                if (ft < 5280.0f) {
                    snprintf(dbuf, sizeof(dbuf), "%3.0f%cF", ft, 5);
                } else {
                    float mi = ft / 5280.0f;
                    snprintf(dbuf, sizeof(dbuf), "%1.2fMI%c", mi, 5);
                }
            }
        #else
            if (_vtxType == VTX_WALKSNAIL) {
                if (d < 1000.0f) {
                    snprintf(dbuf, sizeof(dbuf), "%c%3.0f%c", 5, d, 12);
                } else {
                    float km = d / 1000.0f;
                    snprintf(dbuf, sizeof(dbuf), "%c%1.2f%c", 5, km, 125);
                }
            } else { // DJI
                if (d < 1000.0f) {
                    snprintf(dbuf, sizeof(dbuf), "%c%3.0fM", 5, d);
                } else {
                    float km = d / 1000.0f;
                    snprintf(dbuf, sizeof(dbuf), "%c%1.1fKM", 5, km);
                }
            }
        #endif

            bf_msp_dp_write(0, 35, dbuf, 0);
            break;
        }
        
        // -------------------------------------------------------
        // Ground Radar (Unified Pipeline)
        // -------------------------------------------------------
        case 6: {

            if (!USE_RADAR_HOME_INDICATOR) break;
            if (!sharedTelem.gpsTotalActive) break;

            // Use unified values computed in loop1()
            int rowH = sharedTelem.homeRadarRow;
            int colH = sharedTelem.homeRadarCol;

            snprintf(buf, sizeof(buf), "%c", 9);
            bf_msp_dp_write(rowH, colH, buf, 0);
            break;
        }

        // -------------------------------------------------------
        // Home-direction arrow (Unified Pipeline)
        // -------------------------------------------------------
        case 7: {

            if (!sharedTelem.gpsTotalActive) break;

            // Use unified smoothed relative bearing
            float relSmooth = sharedTelem.homeRelativeSmoothDeg;

            // Betaflight DOWN = 0°, UP = 180°
            float bfBearing = relSmooth + 180.0f;
            if (bfBearing >= 360.0f) bfBearing -= 360.0f;

            // Convert to glyph index (16 directions)
            uint8_t idx = (uint8_t)((bfBearing + 11.25f) / 22.5f) & 0x0F;

            // Fetch glyph
            char arrowGlyph = glyphFromRow(arrowRows[idx]);
            char abuf[2] = { arrowGlyph, 0 };

            // Draw arrow
            bf_msp_dp_write(1, 25, abuf, 0);
            break;
        }

        // -------------------------------------------------------
        // Flight Timer (MM:SS)
        // -------------------------------------------------------
        case 8: {
            char tbuf[12];

            // Format the timer (frozen when disarmed)
            formatFlightTime(tbuf, sharedTelem.flightElapsedMs);

            // Draw timer at row 10, col 20 (adjust as needed)
            bf_msp_dp_write(9, 44, tbuf, 0);

            break;
        }


        // -------------------------------------------------------
        // GPS Number of Satallites
        // -------------------------------------------------------
        case 9: {
            // Number of Satallites
            uint8_t sats = sharedTelem.gpsSats;

            // Flash until GPS fix + 6 sats
            bool gpsLocked = (sharedTelem.gpsFix && sats >= 6);

            if (!gpsLocked && gpsFlashState) {
                // Flash OFF state → draw blanks
                bf_msp_dp_write(0, 4, "     ", 0);
            } else {
                // Solid ON state
                snprintf(buf, sizeof(buf), "%2u%c%c", sats, 30, 31);
                bf_msp_dp_write(0, 3, buf, 0);
            }
            break;
        }
        // -------------------------------------------------------
        // GPS Ground Speed (mph or kph, clamped to 3 digits)
        // -------------------------------------------------------
        case 10: {
            float gsCms = sharedTelem.gpsGroundSpeedCms;

            #if OSD_UNITS == OSD_UNITS_IMPERIAL
                float mph = gsCms * 0.0223694f;   // cm/s → mph
                if (mph > 999.0f) mph = 999.0f;   // clamp to 3 digits

                if (_vtxType == VTX_WALKSNAIL) {
                    // Walksnail-safe
                    snprintf(buf, sizeof(buf), "%3.0f%c%c", mph, 157, 112);
                } else {
                    // DJI-safe: plain ASCII
                    snprintf(buf, sizeof(buf), "%3.0f%c%c", mph, 157, 112);
                }

            #else
                float kph = gsCms * 0.036f;       // cm/s → kph
                if (kph > 999.0f) kph = 999.0f;

                if (_vtxType == VTX_WALKSNAIL) {
                    // Walksnail-safe
                    snprintf(buf, sizeof(buf), "%03.0f%c%c", kph, 158, 112);
                } else {
                    // DJI-safe: plain ASCII
                    snprintf(buf, sizeof(buf), "%03.0f%c%c", kph, 158, 112);
                }
            #endif

            bf_msp_dp_write(2, 35, buf, 0);
            break;
        }
        // -------------------------------------------------------
        // Total Distance Traveled
        // -------------------------------------------------------
        case 11: {
            float totalM = sharedTelem.gpsTotalDistM;

            #if OSD_UNITS == OSD_UNITS_IMPERIAL
                float totalFt = totalM * 3.28084f;
                if (_vtxType == VTX_WALKSNAIL) {
                    if (totalFt < 5280.0f) {
                        // Under 1 mile → feet
                        snprintf(buf, sizeof(buf), "%4.0f%c%c", totalFt, 15, 113);
                    } else {
                        // Miles
                        float mi = totalFt / 5280.0f;
                        snprintf(buf, sizeof(buf), "%1.2f%c%c", mi, 126, 113);
                    }
                } else { // DJI
                    if (totalFt < 5280.0f) {
                        // Under 1 mile → feet
                        snprintf(buf, sizeof(buf), "%4.0fFT%c", totalFt, 113);
                    } else {
                        // Miles
                        float mi = totalFt / 5280.0f;
                        snprintf(buf, sizeof(buf), "%1.2fMI%c", mi, 113);
                    }
                }

            #else
                if (totalM < 1000.0f) {
                    // Under 1 km → meters
                    snprintf(buf, sizeof(buf), "%3.0fM", totalM);
                } else {
                    // Kilometers
                    float km = totalM / 1000.0f;
                    snprintf(buf, sizeof(buf), "%1.2fKM", km);
                }
            #endif

            // Display at (3, 35) — adjust if needed
            bf_msp_dp_write(17, 36, buf, 0);
            break;
        }


        // -------------------------------------------------------
        // Arm state
        // -------------------------------------------------------
        case 12:
            if (armed) {
                bf_msp_dp_write(17, 22, "   ARMED   ", 0);
            } else {
                bf_msp_dp_write(17, 20, "  DISARMED   ", 0);
            }
            break;

        // -------------------------------------------------------
        // Step 7 — Latitude & Longitude
        // -------------------------------------------------------
        case 13: {
            if (sharedTelem.gpsFix) {

                // Format: ±XX.XXXXXX
                float lat = sharedTelem.gpsLatDeg;
                float lon = sharedTelem.gpsLonDeg;

                // Walksnail-safe characters only
                snprintf(buf, sizeof(buf), "%c%+2.6f", 137, lat); 
                bf_msp_dp_write(16, 0, buf, 0);

                snprintf(buf, sizeof(buf), "%c%+3.6f", 152, lon);
                bf_msp_dp_write(17, 0, buf, 0);

            } else {
                // No GPS lock — show placeholders
                bf_msp_dp_write(16, 1, "NO GPS", 0);
                bf_msp_dp_write(17, 1, "-------", 0);
            }
            break;
        }


        // -------------------------------------------------------
        // Flight mode
        // -------------------------------------------------------
        case 14:
            bf_msp_dp_write(0, 10, "QLITE", 0);
            break;

        // -------------------------------------------------------
        // Crosshair
        // -------------------------------------------------------
        case 15:
                if (_vtxType == VTX_WALKSNAIL) {
                    // Walksnail crosshair icon
                    bf_msp_dp_write(9, 25, "s", 0);
                } else if (_vtxType == VTX_DJI_V1 || _vtxType == VTX_DJI_O3) {
                    // DJI has no crosshair icon — use blank
                    bf_msp_dp_write(9, 25, " ", 0);
                } else {
                    // Unknown VTX — safest fallback is no icon
                    bf_msp_dp_write(9, 25, " ", 0);
                }
            break;

        // -------------------------------------------------------
        // Commit frame
        // -------------------------------------------------------
        case 16:
            bf_msp_dp_draw();
            break;

        // -------------------------------------------------------
        // Reset
        // -------------------------------------------------------
        default:
            _osdStep = 0;
            return;
    }

    _osdStep++;
}

// =======================================================
// PUBLIC — bf_msp_set_armed()
// Call whenever armed state changes.
// Updates the internal armed flag used by status frames
// and the OSD arm state display.
// =======================================================
void bf_msp_set_armed(bool armed) { _isArmed = armed; }

// =======================================================
// PUBLIC — bf_msp_get_vtx_type()
// Returns which VTX system has been detected.
// Returns VTX_UNKNOWN if detection is not yet complete.
// =======================================================
VtxSystemType bf_msp_get_vtx_type() { return _vtxType; }