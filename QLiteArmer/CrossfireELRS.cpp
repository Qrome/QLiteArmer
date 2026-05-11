#include "CrossfireELRS.h"

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_TYPE_RC_CHANNELS 0x16
#define CRSF_RC_PAYLOAD_LEN 22

void CrossfireELRS::begin(int rxPin, int txPin) {
    Serial2.setRX(rxPin);
    Serial2.setTX(txPin);
    Serial2.begin(420000);
}

uint8_t CrossfireELRS::crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    return crc;
}

void CrossfireELRS::decodeChannels(const uint8_t *p) {
    uint32_t bitBuf = 0;
    uint8_t bitCount = 0;
    uint8_t ch = 0;

    for (uint8_t i = 0; i < CRSF_RC_PAYLOAD_LEN; i++) {
        bitBuf |= ((uint32_t)p[i]) << bitCount;
        bitCount += 8;

        while (bitCount >= 11 && ch < 16) {
            channels[ch++] = bitBuf & 0x7FF;
            bitBuf >>= 11;
            bitCount -= 11;
        }
    }
}

bool CrossfireELRS::update() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();

        if (index == 0) {
            if (b == CRSF_SYNC_BYTE) buffer[index++] = b;
            continue;
        }

        if (index == 1) {
            payloadLen = b;
            buffer[index++] = b;
            if (payloadLen > 60) index = 0;
            continue;
        }

        buffer[index++] = b;

        if (index == payloadLen + 2) {
            uint8_t crc = crc8(&buffer[2], payloadLen - 1);
            uint8_t crcFrame = buffer[index - 1];

            if (crc == crcFrame && buffer[2] == CRSF_TYPE_RC_CHANNELS) {
                decodeChannels(&buffer[3]);
                // mark link active
                lastPacketTime = millis();
                crsfLinkActive = true;

                index = 0;
                return true;
            }

            index = 0;
        }
    }
    // link timeout (100ms is standard for ELRS)
    if (millis() - lastPacketTime > 100) {
        crsfLinkActive = false;
    }

    return false;
}

uint16_t CrossfireELRS::getChannel(uint8_t i) {
    return channels[i];
}