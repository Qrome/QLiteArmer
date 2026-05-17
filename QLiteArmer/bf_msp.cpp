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

// -------------------------------------------------------
// Module-level references
// -------------------------------------------------------
static HardwareSerial* _serial = nullptr;
static Telemetry* _telemetry = nullptr;

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

// Tracks whether DisplayPort canvas ownership has been claimed
static bool _dpReleased = false;

// VTX type — declared in bf_msp.h, used here as module variable
static VtxSystemType _vtxType = VTX_UNKNOWN;

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

// =======================================================
// PRIVATE — VTX Detection
// Called each time a valid MSP request is received.
//
// Observed poll patterns:
// DJI V1: MSP_API_VERSION → MSP_FC_VARIANT → ...
// starts with API_VERSION
// DJI O3/O4: MSP_API_VERSION → MSP_STATUS_EX
// Walksnail: MSP_STATUS (0x65) ONLY — never sends
// MSP_API_VERSION at startup
// =======================================================
static void bf_msp_detect_vtx(uint8_t cmd) {
    if (_vtxType != VTX_UNKNOWN) return;

    if (cmd == MSP_API_VERSION) _seenApiVersion = true;
    if (cmd == MSP_FC_NAME) _seenFcName = true;
    if (cmd == MSP_STATUS_EX) _seenStatusEx = true;

    static uint8_t frameCount = 0;
    frameCount++;

    // Definitive DJI O3/O4 signal — STATUS_EX never sent by V1 or Avatar
    if (_seenStatusEx) {
        _vtxType = VTX_DJI_O3;
        if (Serial) Serial.println("[MSP] VTX detected: DJI O3/O4");
        return;
    }

    // Walksnail Avatar — hammers MSP_STATUS only, never sends API_VERSION
    if (cmd == MSP_STATUS && !_seenApiVersion && frameCount >= 5) {
        _vtxType = VTX_WALKSNAIL;
        if (Serial) Serial.println("[MSP] VTX detected: Walksnail Avatar");
        return;
    }

    // Walksnail secondary signals
    if (cmd == MSP_FC_NAME || cmd == MSP_OSD_CONFIG) {
        _vtxType = VTX_WALKSNAIL;
        if (Serial) Serial.println("[MSP] VTX detected: Walksnail Avatar");
        return;
    }

    // DJI V1 fallback — API_VERSION seen but no STATUS_EX after 20 frames
    if (_seenApiVersion && frameCount > 20 && !_seenStatusEx && !_seenFcName) {
        _vtxType = VTX_DJI_V1;
        if (Serial) Serial.println("[MSP] VTX detected: DJI V1");
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
    const char* name = "QLite";
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
    PACK_U16_LE(p, idx, (uint16_t)SENSOR_BARO);  // sensors

    uint32_t flags = armed ? 1 : 0;
    PACK_U32_LE(p, idx, flags);  // arming flags

    p[idx++] = 0;  // current profile

    bf_msp_send(MSP_STATUS, p, idx);
}

// -------------------------------------------------------
// MSP_STATUS_EX (150) — 16 bytes little-endian
// Polled by DJI O3/O4 — never by Walksnail or V1
// -------------------------------------------------------
static void bf_msp_send_status_ex(bool armed) {
    uint8_t p[16];
    uint8_t idx = 0;

    PACK_U16_LE(p, idx, (uint16_t)1000);
    PACK_U16_LE(p, idx, (uint16_t)0);
    PACK_U16_LE(p, idx, (uint16_t)SENSOR_BARO);

    uint32_t flags = armed ? 1 : 0;
    PACK_U32_LE(p, idx, flags);

    p[idx++] = 0;

    PACK_U16_LE(p, idx, (uint16_t)0);

    uint16_t armFlags = armed ? 0 : (uint16_t)ARMING_DISABLE_NO_GYRO;
    PACK_U16_LE(p, idx, armFlags);

    uint16_t battMv = (_telemetry != nullptr) ? _telemetry->readBatteryMv() : 0;
    p[idx++] = (uint8_t)constrain(battMv / 100, 0, 255);

    bf_msp_send(MSP_STATUS_EX, p, idx);
}

// -------------------------------------------------------
// MSP_ANALOG (110) — 7 bytes little-endian
// Used by DJI V1 for native voltage bar display
// -------------------------------------------------------
static void bf_msp_send_analog() {
    uint8_t p[7];
    uint8_t idx = 0;

    uint16_t battMv = (_telemetry != nullptr) ? _telemetry->readBatteryMv() : 0;
    uint8_t vbat = (uint8_t)constrain(battMv / 100, 0, 255);

    p[idx++] = vbat;                   // battery voltage in 0.1V units
    PACK_U16_LE(p, idx, (uint16_t)0);  // mAh drawn
    PACK_U16_LE(p, idx, (uint16_t)0);  // rssi
    PACK_U16_LE(p, idx, (uint16_t)0);  // current in 0.01A units

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
        OSD_POS(1, 9, 25),   // crosshair
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
        case MSP_API_VERSION:
            bf_msp_send_api_version();
            break;
        case MSP_FC_VARIANT:
            bf_msp_send_fc_variant();
            break;
        case MSP_FC_VERSION:
            bf_msp_send_fc_version();
            break;
        case MSP_FC_NAME:
            bf_msp_send_fc_name();
            break;
        case MSP_STATUS:
            bf_msp_send_status(_isArmed);
            break;
        case MSP_STATUS_EX:
            bf_msp_send_status_ex(_isArmed);
            break;
        case MSP_ANALOG:
            bf_msp_send_analog();
            break;
        case MSP_ALTITUDE:
            bf_msp_send_altitude();
            break;
        case MSP_OSD_CONFIG:
            bf_msp_send_osd_config();
            break;
        default:
            // Send empty response — keeps Air Unit happy
            bf_msp_send(cmd, nullptr, 0);
            break;
    }
}

// =======================================================
// PUBLIC — Initialisation
// Call once from setup() before any other bf_msp function.
// =======================================================
void bf_msp_init(HardwareSerial& serial, Telemetry& telemetry) {
    _serial = &serial;
    _telemetry = &telemetry;
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
                    bf_msp_handle_request(_parseCmd);
                    bf_msp_detect_vtx(_parseCmd);

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
            bf_msp_send_osd_config();
            break;
        case 5:
            bf_msp_send_status(_isArmed);
            break;
        case 6:
            bf_msp_send_status_ex(_isArmed);
            break;
        case 7:
            bf_msp_send_analog();
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

    uint32_t now = millis();
    if (now - _lastHeartbeatMs < MSP_FRAME_GAP_MS) return;
    _lastHeartbeatMs = now;

    switch (_heartbeatStep) {
        case 0:
            bf_msp_send_status(_isArmed);
            break;
        case 1:
            bf_msp_send_analog();
            break;
        case 2:
            bf_msp_send_altitude();
            break;
        case 3:
            // DisplayPort heartbeat for Avatar and O3/O4 only
            // Avatar requires this periodically or shows disconnect warning
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

    char buf[32];

    switch (_osdStep) {

        // -------------------------------------------------------
        // Step 0 — First frame: RELEASE, then refresh telemetry
        // -------------------------------------------------------
        case 0:
            if (!_dpReleased) {
                bf_msp_dp_release();
                _dpReleased = true;
                _osdStep++;
                return;
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

#if OSD_UNITS == OSD_UNITS_IMPERIAL
                altM     = altCm * 0.0328084f;     // feet
                vspeedMs = vsCms * 0.0328084f;     // ft/s
#else
                altM     = altCm / 100.0f;         // meters
                vspeedMs = vsCms / 100.0f;         // m/s
#endif
            }

            armed = _isArmed;
            bf_msp_dp_clear();
            break;

        // -------------------------------------------------------
        // Step 1 — Battery voltage
        // -------------------------------------------------------
        case 1:
            snprintf(buf, sizeof(buf), "V:%5.1f", voltV);
            bf_msp_dp_write(1, 10, buf, 0);
            break;

        // -------------------------------------------------------
        // Step 2 — Altitude
        // -------------------------------------------------------
        case 2:
#if OSD_UNITS == OSD_UNITS_IMPERIAL
            snprintf(buf, sizeof(buf), "A:%5.1f'", altM);     // feet
#else
            snprintf(buf, sizeof(buf), "A:%5.1fM", altM);     // meters
#endif
            bf_msp_dp_write(2, 10, buf, 0);
            break;

        // -------------------------------------------------------
        // Step 3 — Vertical speed
        // -------------------------------------------------------
        case 3:
#if OSD_UNITS == OSD_UNITS_IMPERIAL
            snprintf(buf, sizeof(buf), "VS:%+5.1f'/S", vspeedMs);  // ft/s
#else
            snprintf(buf, sizeof(buf), "VS:%+5.1fM/S", vspeedMs);  // m/s
#endif
            bf_msp_dp_write(3, 10, buf, 0);
            break;

        // -------------------------------------------------------
        // Step 4 — Arm state
        // -------------------------------------------------------
        case 4:
            if (armed) {
                bf_msp_dp_write(17, 20, "*** ARMED ***", 1);
            } else {
                bf_msp_dp_write(17, 20, "  DISARMED   ", 0);
            }
            break;

        // -------------------------------------------------------
        // Step 5 — Flight mode
        // -------------------------------------------------------
        case 5:
            bf_msp_dp_write(0, 10, "QLITE", 0);
            break;

        // -------------------------------------------------------
        // Step 6 — Crosshair
        // -------------------------------------------------------
        case 6:
            bf_msp_dp_write(9, 25, "s", 0);   // Walksnail crosshair icon
            break;

        // -------------------------------------------------------
        // Step 7 — Commit frame
        // -------------------------------------------------------
        case 7:
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