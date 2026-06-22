#include "GPS.h"
#include <string.h>

static float haversineDistM(int32_t lat1e7, int32_t lon1e7,
                            int32_t lat2e7, int32_t lon2e7);

GPS::GPS()
    : _port(nullptr),
      _healthy(false),
      _hasFix(false),
      _fixType(0),
      _satCount(0),
      _lat(0),
      _lon(0),
      _altMSL(0),
      _groundSpeed(0),
      _course(0),
      _lastUpdateMs(0),
      _unixTime(0),
      _nmeaPos(0)
{
    memset(_nmeaBuf, 0, sizeof(_nmeaBuf));

    _homeLat1e7 = 0;
    _homeLon1e7 = 0;
    _homeSet = false;
    _distHomeM = 0.0f;
}

void GPS::begin(HardwareSerial &port) {
    _port = &port;
    resetState();
}

bool GPS::autodetectBaud(const uint32_t *baudList, size_t baudCount,
                         uint32_t perBaudTimeoutMs) {
    if (!_port) {
        Serial.println("[GPS] no Serial1 Port!");
        return false;
    }

    for (size_t i = 0; i < baudCount; i++) {
        uint32_t baud = baudList[i];
        Serial.print("[GPS] Trying baud: ");
        Serial.println(baud);

        _port->begin(baud);
        resetState();

        uint32_t start = millis();
        while (millis() - start < perBaudTimeoutMs) {
            while (_port->available()) {
                char c = _port->read();
                processByte(c);

                if (_healthy) {
                    Serial.print("[GPS] Connected with baud: ");
                    Serial.println(baud);
                    return true;
                }
            }
        }
    }
    return false;
}

void GPS::update() {
    if (!_port) return;
    while (_port->available()) {
        char c = _port->read();
        processByte(c);
    }
}

bool GPS::hasFix() const        { return _hasFix; }
bool GPS::isHealthy() const     { return _healthy; }
uint8_t GPS::getFixType() const { return _fixType; }
uint8_t GPS::getSatCount() const { return _satCount; }
uint32_t GPS::lastUpdateMs() const { return _lastUpdateMs; }

int32_t GPS::getLatitude() const  { return _lat; }
int32_t GPS::getLongitude() const { return _lon; }
int32_t GPS::getAltitudeMSL() const { return _altMSL; }

uint16_t GPS::getGroundSpeed() const { return _groundSpeed; }
uint16_t GPS::getCourse() const      { return _course; }

uint32_t GPS::getUnixTime() const { return _unixTime; }

void GPS::resetState() {
    _healthy = false;
    _hasFix = false;
    _fixType = 0;
    _satCount = 0;
    _lat = 0;
    _lon = 0;
    _altMSL = 0;
    _groundSpeed = 0;
    _course = 0;
    _lastUpdateMs = 0;
    _unixTime = 0;
    _nmeaPos = 0;
    memset(_nmeaBuf, 0, sizeof(_nmeaBuf));
}

void GPS::processByte(char c) {
    bool isNmeaChar =
        (c == '$') ||
        (c == ',') ||
        (c == '.') ||
        (c == '*') ||
        (c == '+') ||
        (c == '-') ||
        (c == ' ') ||
        (c >= '0' && c <= '9') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= 'a' && c <= 'z') ||
        (c == '\r') ||
        (c == '\n');

    if (!isNmeaChar) return;

    if (c == '$') {
        _nmeaPos = 0;
        _nmeaBuf[_nmeaPos++] = c;
        return;
    }

    if (_nmeaPos == 0) return;

    if (c == '\n' || c == '\r') {
        _nmeaBuf[_nmeaPos] = '\0';
        if (_nmeaPos > 6) processSentence(_nmeaBuf);
        _nmeaPos = 0;
        return;
    }

    if (_nmeaPos < NMEA_BUF_LEN - 1) {
        _nmeaBuf[_nmeaPos++] = c;
    }
}

void GPS::processSentence(const char *s) {
    if (s[0] != '$') return;
    if (s[1] != 'G') return;
    if (s[2] != 'P' && s[2] != 'N') return;

    //Serial.print("[NMEA] ");
    //Serial.println(s);

    if (s[3] == 'G' && s[4] == 'G' && s[5] == 'A') parseGGA(s);
    else if (s[3] == 'R' && s[4] == 'M' && s[5] == 'C') parseRMC(s);
}

void GPS::parseGGA(const char *s) {
    const char *p = s;
    int field = 0;
    char latStr[16] = {0};
    char lonStr[16] = {0};
    char ns = 0, ew = 0;
    uint8_t fix = 0;
    uint8_t sats = 0;
    float alt = 0.0f;

    char buf[20];
    int bufPos = 0;

    auto flushField = [&](void) {
        buf[bufPos] = '\0';
        switch (field) {
            case 2: strncpy(latStr, buf, sizeof(latStr) - 1); break;
            case 3: ns = buf[0]; break;
            case 4: strncpy(lonStr, buf, sizeof(lonStr) - 1); break;
            case 5: ew = buf[0]; break;
            case 6: fix = (uint8_t)parseUInt(buf); break;
            case 7: sats = (uint8_t)parseUInt(buf); break;
            case 9: alt = parseFloat(buf); break;
        }
        bufPos = 0;
        field++;
    };

    while (*p) {
        char c = *p++;
        if (c == ',' || c == '*') {
            flushField();
            if (c == '*') break;
        } else if (bufPos < 19) {
            buf[bufPos++] = c;
        }
    }

    int32_t lat1e7 = 0, lon1e7 = 0;
    if (parseLatLon(latStr, ns, lat1e7) && parseLatLon(lonStr, ew, lon1e7)) {
        if (sats > 40) return;
        if (lat1e7 == 0 || lon1e7 == 0) return;

        _lat = lat1e7;
        _lon = lon1e7;
        // Only allow home to auto‑set if fix is valid AND sats >= 6
        if (!_homeSet && _hasFix && _satCount >= 6) {
            _homeLat1e7 = _lat;
            _homeLon1e7 = _lon;
            _homeSet = true;
            _distHomeM = 0.0f;
        }

        // Only compute distance if home is set AND fix is valid AND sats >= 6
        if (_homeSet && _hasFix && _satCount >= 6) {
            _distHomeM = haversineDistM(_homeLat1e7, _homeLon1e7, _lat, _lon);
        } else {
            _distHomeM = 0.0f;
        }

        _altMSL = (int32_t)(alt * 100.0f);
        _fixType = fix;
        _satCount = sats;
        _hasFix = (fix >= 1);
        _healthy = true;
        _lastUpdateMs = millis();
    }
}

void GPS::parseRMC(const char *s) {
    const char *p = s;
    int field = 0;
    char status = 'V';

    char sogStr[16] = {0};
    char cogStr[16] = {0};

    char buf[20];
    int bufPos = 0;

    auto flushField = [&](void) {
        buf[bufPos] = '\0';
        switch (field) {
            case 2: status = buf[0]; break;
            case 7: strncpy(sogStr, buf, sizeof(sogStr) - 1); break;
            case 8: strncpy(cogStr, buf, sizeof(cogStr) - 1); break;
        }
        bufPos = 0;
        field++;
    };

    while (*p) {
        char c = *p++;
        if (c == ',' || c == '*') {
            flushField();
            if (c == '*') break;
        } else if (bufPos < 19) {
            buf[bufPos++] = c;
        }
    }

    float sogKnots = parseFloat(sogStr);
    float cogDeg   = parseFloat(cogStr);

    if (status == 'A') _hasFix = true;

    float sogMs = sogKnots * 0.514444f;
    _groundSpeed = (uint16_t)(sogMs * 100.0f);
    _course = (uint16_t)(cogDeg * 10.0f);
    _healthy = true;
    _lastUpdateMs = millis();
}

bool GPS::parseLatLon(const char *field, char hemi, int32_t &out1e7) {
    if (!field || strlen(field) < 3) return false;

    float v = parseFloat(field);
    if (v == 0.0f) return false;

    int deg = (int)(v / 100.0f);
    float minutes = v - (deg * 100.0f);
    float degFloat = (float)deg + minutes / 60.0f;

    if (hemi == 'S' || hemi == 'W') degFloat = -degFloat;

    out1e7 = (int32_t)(degFloat * 1e7f);
    return true;
}

int32_t GPS::parseInt(const char *s) {
    if (!s || !*s) return 0;
    return (int32_t)strtol(s, nullptr, 10);
}

uint32_t GPS::parseUInt(const char *s) {
    if (!s || !*s) return 0;
    return (uint32_t)strtoul(s, nullptr, 10);
}

uint16_t GPS::parseUInt16(const char *s) {
    return (uint16_t)parseUInt(s);
}

float GPS::parseFloat(const char *s) {
    if (!s || !*s) return 0.0f;
    return (float)atof(s);
}

static float haversineDistM(int32_t lat1e7, int32_t lon1e7,
                            int32_t lat2e7, int32_t lon2e7) {
    const float R = 6371000.0f; // Earth radius in meters

    float lat1 = lat1e7 * 1e-7f * DEG_TO_RAD;
    float lon1 = lon1e7 * 1e-7f * DEG_TO_RAD;
    float lat2 = lat2e7 * 1e-7f * DEG_TO_RAD;
    float lon2 = lon2e7 * 1e-7f * DEG_TO_RAD;

    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;

    float a = sinf(dlat/2)*sinf(dlat/2) +
              cosf(lat1)*cosf(lat2)*sinf(dlon/2)*sinf(dlon/2);

    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    return R * c;
}


void GPS::configureM10_VALSET() {
    if (!_port) return;

    const uint8_t disable_GSV[] = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x91,  0x00,0x00,0x00,0x00};
    const uint8_t disable_GSA[] = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x8F,  0x00,0x00,0x00,0x00};
    const uint8_t disable_VTG[] = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x8C,  0x00,0x00,0x00,0x00};
    const uint8_t disable_GLL[] = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x8B,  0x00,0x00,0x00,0x00};
    const uint8_t disable_GST[] = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x92,  0x00,0x00,0x00,0x00};

    const uint8_t enable_GGA[]  = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x8A,  0x01,0x00,0x00,0x00};
    const uint8_t enable_RMC[]  = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x8E,  0x01,0x00,0x00,0x00};

    const uint8_t rate_1Hz[]    = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x21,  0xE8,0x03,0x00,0x00};

    const uint8_t uart1_nmea[]  = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x11,  0x01,0x00,0x00,0x00};

    const uint8_t save_cfg[]    = {0x01,0x00,0x00,0x00,  0x01,0x00,0x00,0x1F,  0x01,0x00,0x00,0x00};

    sendVALSET(disable_GSV, sizeof(disable_GSV));
    sendVALSET(disable_GSA, sizeof(disable_GSA));
    sendVALSET(disable_VTG, sizeof(disable_VTG));
    sendVALSET(disable_GLL, sizeof(disable_GLL));
    sendVALSET(disable_GST, sizeof(disable_GST));

    sendVALSET(enable_GGA, sizeof(enable_GGA));
    sendVALSET(enable_RMC, sizeof(enable_RMC));

    sendVALSET(rate_1Hz, sizeof(rate_1Hz));
    sendVALSET(uart1_nmea, sizeof(uart1_nmea));

    sendVALSET(save_cfg, sizeof(save_cfg));

    Serial.println("[GPS] M10 configured using CFG‑VALSET");
}

void GPS::sendVALSET(const uint8_t *payload, uint16_t len) {
    if (!_port) return;

    uint8_t header[] = {0xB5, 0x62, 0x06, 0x8A};
    uint16_t msgLen = len;

    _port->write(header, 4);
    _port->write((uint8_t)(msgLen & 0xFF));
    _port->write((uint8_t)(msgLen >> 8));
    _port->write(payload, len);

    uint8_t ckA = 0, ckB = 0;

    ckA += 0x06; ckB += ckA;
    ckA += 0x8A; ckB += ckA;
    ckA += (msgLen & 0xFF); ckB += ckA;
    ckA += (msgLen >> 8);   ckB += ckA;

    for (int i = 0; i < len; i++) {
        ckA += payload[i];
        ckB += ckA;
    }

    _port->write(ckA);
    _port->write(ckB);

    delay(50);
}

float GPS::getDistanceFromHomeM() const {
    return _distHomeM;
}

void GPS::forceSetHome() {
    if (_hasFix && _satCount >= 6) {
        _homeLat1e7 = _lat;
        _homeLon1e7 = _lon;
        _homeSet = true;
        _distHomeM = 0.0f;   // reset distance
    }
}

float GPS::getBearingToHomeDeg() const
{
    if (!_homeSet) return 0.0f;

    // Convert 1e-7 degrees to float degrees
    float lat1 = _lat * 1e-7f;
    float lon1 = _lon * 1e-7f;
    float lat2 = _homeLat1e7 * 1e-7f;
    float lon2 = _homeLon1e7 * 1e-7f;

    // Convert to radians
    float rlat1 = radians(lat1);
    float rlon1 = radians(lon1);
    float rlat2 = radians(lat2);
    float rlon2 = radians(lon2);

    float dLon = rlon2 - rlon1;

    float y = sin(dLon) * cos(rlat2);
    float x = cos(rlat1) * sin(rlat2) -
              sin(rlat1) * cos(rlat2) * cos(dLon);

    float bearing = degrees(atan2(y, x));

    if (bearing < 0) bearing += 360.0f;

    return bearing;
}


