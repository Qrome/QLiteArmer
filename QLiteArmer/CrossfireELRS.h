#pragma once
#include <Arduino.h>

class CrossfireELRS {
public:
    void begin(int rxPin, int txPin);
    bool update(); // True if at least one valid RC frame was decoded.
    uint8_t getLinkQuality() const { return linkQuality; }
    uint16_t getChannel(uint8_t i);
    bool crsfLinkActive = false;
    uint32_t lastPacketTime = 0;
    float getChannelPercent(uint8_t i);
    float getChannelPercentBipolar(uint8_t i);

private:
    static const uint16_t CRSF_CHANNEL_MIN = 172;
    static const uint16_t CRSF_CHANNEL_MAX = 1811;
    static const uint32_t LINK_TIMEOUT_MS = 100;
    uint8_t buffer[64] = {0};
    uint8_t index = 0;
    uint8_t payloadLen = 0;
    uint16_t channels[16] = {0};
    uint8_t linkQuality = 0;
    uint32_t lastByteTime = 0;
    uint8_t crc8(const uint8_t *data, uint8_t len);
    void decodeChannels(const uint8_t *payload);
    void checkTimeout();
};
