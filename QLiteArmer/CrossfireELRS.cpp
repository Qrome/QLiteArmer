#include "CrossfireELRS.h"

#define CRSF_SYNC_BYTE         0xC8
#define CRSF_TYPE_RC_CHANNELS  0x16
#define CRSF_TYPE_LINK_STATS   0x14   // <-- Link Statistics frame for LQ/RSSI
#define CRSF_RC_PAYLOAD_LEN    22


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
            uint8_t crc       = crc8(&buffer[2], payloadLen - 1);
            uint8_t crcFrame  = buffer[index - 1];

            if (crc == crcFrame) {
                uint8_t type    = buffer[2];
                uint8_t *payload = &buffer[3];

                // -----------------------------------------------
                // RC Channels frame - decode stick/switch channels
                // -----------------------------------------------
                if (type == CRSF_TYPE_RC_CHANNELS) {
                    decodeChannels(payload);
                    lastPacketTime = millis();
                    crsfLinkActive = true;
                    index = 0;
                    return true;
                }

                // -----------------------------------------------
                // Link Statistics frame - read LQ and RSSI
                // payload[2] = Uplink LQ (0-100%)
                // payload[0] = Uplink RSSI Antenna 1 (dBm, negated)
                // -----------------------------------------------
                if (type == CRSF_TYPE_LINK_STATS) {
                    linkQuality  = payload[2];   // LQ: 0-100%
                    //rssi         = -(int8_t)payload[0]; // RSSI in dBm (positive stored, negate for true dBm)
                    index = 0;
                    return false; // don't return true as no new RC data yet
                }
            }

            index = 0;
        }
    }

    // Link timeout check (100ms is standard for ELRS)
    if (millis() - lastPacketTime > 100) {
        crsfLinkActive = false;
    }

    return false;
}

uint16_t CrossfireELRS::getChannel(uint8_t i) {
    return channels[i];
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