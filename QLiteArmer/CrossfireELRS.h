#pragma once
#include <Arduino.h>

class CrossfireELRS {
public:
    void begin(int rxPin, int txPin);
    bool update();                 // returns true when a valid RC frame is decoded
    uint16_t getChannel(uint8_t i);
    bool crsfLinkActive = false;     
    uint32_t lastPacketTime = 0;     

private:
    uint8_t buffer[64];
    uint8_t index = 0;
    uint8_t payloadLen = 0;
    uint16_t channels[16] = {0};

    uint8_t crc8(const uint8_t *data, uint8_t len);
    void decodeChannels(const uint8_t *payload);
};