#include "CrossfireELRS.h"

#define CRSF_SYNC_BYTE         0xC8
#define CRSF_TYPE_RC_CHANNELS  0x16
#define CRSF_TYPE_LINK_STATS   0x14   // <-- Link Statistics frame for LQ/RSSI
#define CRSF_RC_PAYLOAD_LEN    22


void CrossfireELRS::begin(int rxPin, int txPin) {
    index = 0;
    payloadLen = 0;
    crsfLinkActive = false;
    lastPacketTime = 0;
    lastByteTime = 0;
    linkQuality = 0;
    for (uint8_t i = 0; i < 16; ++i) channels[i] = 0;
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

void CrossfireELRS::checkTimeout() {
    // Only valid channel frames refresh lastPacketTime; statistics do not.
    if ((uint32_t)(millis() - lastPacketTime) >= LINK_TIMEOUT_MS) {
        crsfLinkActive = false;
        linkQuality = 0;
    }
}

bool CrossfireELRS::update() {
    checkTimeout();
    if (index && (uint32_t)(millis() - lastByteTime) >= LINK_TIMEOUT_MS) {
        index = 0; // Discard an abandoned partial frame.
    }

    bool receivedChannels = false;
    // Bound each pass so continuous UART traffic cannot starve PWM updates.
    int bytesRemaining = Serial2.available();
    if (bytesRemaining > 512) bytesRemaining = 512;
    while (bytesRemaining-- > 0 && Serial2.available()) {
        uint8_t b = Serial2.read();
        lastByteTime = millis();

        if (index == 0) {
            if (b == CRSF_SYNC_BYTE) buffer[index++] = b;
            continue;
        }
        if (index == 1) {
            // Length includes type and CRC, excluding sync and length.
            if (b < 2 || b > sizeof(buffer) - 2) {
                index = 0;
                if (b == CRSF_SYNC_BYTE) buffer[index++] = b;
                continue;
            }
            payloadLen = b;
            buffer[index++] = b;
            continue;
        }

        buffer[index++] = b;
        if (index != payloadLen + 2) continue;

        uint8_t crc = crc8(&buffer[2], payloadLen - 1);
        if (crc == buffer[index - 1]) {
            uint8_t type = buffer[2];
            const uint8_t *payload = &buffer[3];
            if (type == CRSF_TYPE_RC_CHANNELS &&
                payloadLen == CRSF_RC_PAYLOAD_LEN + 2) {
                decodeChannels(payload);
                lastPacketTime = millis();
                crsfLinkActive = true;
                receivedChannels = true;
            } else if (type == CRSF_TYPE_LINK_STATS && payloadLen == 12) {
                // Standard link statistics: 10 payload bytes + type + CRC.
                linkQuality = payload[2];
            }
        }
        index = 0;
    }

    // Always run, including when only statistics or malformed frames arrive.
    checkTimeout();
    return receivedChannels;
}

uint16_t CrossfireELRS::getChannel(uint8_t i) {
    return i < 16 ? channels[i] : 0;
}

float CrossfireELRS::getChannelPercent(uint8_t i) {
    if (i >= 16) return 0.0f;

    uint16_t raw = channels[i];
    // Clamp in case a value ever falls outside the spec range
    if (raw < CRSF_CHANNEL_MIN) raw = CRSF_CHANNEL_MIN;
    if (raw > CRSF_CHANNEL_MAX) raw = CRSF_CHANNEL_MAX;

    return (float)(raw - CRSF_CHANNEL_MIN) * 100.0f /
           (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN);
}

float CrossfireELRS::getChannelPercentBipolar(uint8_t i) {
    // Useful for stick axes (roll/pitch/yaw) where you want -100%..+100%
    // centered on the stick's neutral position rather than 0%..100%.
    return (getChannelPercent(i) - 50.0f) * 2.0f;
}