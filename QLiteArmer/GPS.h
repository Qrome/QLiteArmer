#pragma once
#include <Arduino.h>

class GPS {
public:
    GPS();

    void begin(HardwareSerial &port);

    bool autodetectBaud(const uint32_t *baudList, size_t baudCount,
                        uint32_t perBaudTimeoutMs = 250);

    void update();

    bool hasFix() const;
    bool isHealthy() const;
    uint8_t getFixType() const;
    uint8_t getSatCount() const;
    uint32_t lastUpdateMs() const;

    int32_t getLatitude() const;
    int32_t getLongitude() const;
    int32_t getAltitudeMSL() const;

    uint16_t getGroundSpeed() const;
    uint16_t getCourse() const;

    uint32_t getUnixTime() const;

    // M10 configuration (CFG‑VALSET)
    void configureM10_VALSET();

    float getDistanceFromHomeM() const;
    void forceSetHome();
    float getBearingToHomeDeg() const;


private:
    HardwareSerial *_port;
    bool _healthy;
    bool _hasFix;
    uint8_t _fixType;
    uint8_t _satCount;
    int32_t _lat;
    int32_t _lon;
    int32_t _altMSL;
    uint16_t _groundSpeed;
    uint16_t _course;
    uint32_t _lastUpdateMs;
    uint32_t _unixTime;

    // Optional: detect M10 modules
    bool _isM10 = false;

    // NMEA buffer
    static const uint16_t NMEA_BUF_LEN = 256;
    char _nmeaBuf[NMEA_BUF_LEN];
    uint16_t _nmeaPos;

    // Home position
    int32_t _homeLat1e7;
    int32_t _homeLon1e7;
    bool    _homeSet;

    // Distance from home (meters)
    float   _distHomeM;

    void resetState();
    void processByte(char c);
    void processSentence(const char *s);

    void parseGGA(const char *s);
    void parseRMC(const char *s);

    static bool parseLatLon(const char *field, char hemi, int32_t &out1e7);
    static int32_t parseInt(const char *s);
    static uint32_t parseUInt(const char *s);
    static uint16_t parseUInt16(const char *s);
    static float parseFloat(const char *s);

    // Required for M10 VALSET
    void sendVALSET(const uint8_t *payload, uint16_t len);
};