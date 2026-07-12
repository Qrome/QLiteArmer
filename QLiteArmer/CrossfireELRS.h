#pragma once
#include <Arduino.h>

class CrossfireELRS {
public:
    void begin(int rxPin, int txPin);
    bool update();                 // returns true when a valid RC frame is decoded
    uint8_t getLinkQuality() const { return linkQuality; }
    uint16_t getChannel(uint8_t i);
    bool crsfLinkActive = false;     
    uint32_t lastPacketTime = 0;  
    float getChannelPercent(uint8_t i);      // 0.0 - 100.0
    float getChannelPercentBipolar(uint8_t i); // -100.0 - +100.0, centered at 0   

private:
    static const uint16_t CRSF_CHANNEL_MIN = 172;
    static const uint16_t CRSF_CHANNEL_MAX = 1811;
    uint8_t buffer[64];
    uint8_t index = 0;
    uint8_t payloadLen = 0;
    uint16_t channels[16] = {0};
    uint8_t linkQuality = 0;   // 0–100%
    uint8_t crc8(const uint8_t *data, uint8_t len);
    void decodeChannels(const uint8_t *payload);
};