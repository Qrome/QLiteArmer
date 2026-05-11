#include "detection.h"
#include "config.h"

bool vtxDetected = false;
uint8_t badChecksumCount = 0;
uint8_t goodFrameCount = 0;

enum ParseState {
    IDLE, HDR_M, HDR_DIR, LEN, CMD_LSB, CMD_MSB, PAYLOAD, CHECKSUM
};

static ParseState st = IDLE;
static uint8_t mspLen = 0;
static uint16_t mspCmd = 0;
static uint8_t mspChecksum = 0;
static uint8_t idx = 0;

void detectionInit() {
    st = IDLE;
}

void resetParser() {
    st = IDLE;
    mspLen = 0;
    mspCmd = 0;
    mspChecksum = 0;
    idx = 0;
}

void processByte(uint8_t b) {
    switch (st) {
        case IDLE:
            if (b == '$') st = HDR_M;
            break;

        case HDR_M:
            if (b == 'M') st = HDR_DIR;
            else resetParser();
            break;

        case HDR_DIR:
            if (b == '<' || b == '>') st = LEN;
            else resetParser();
            break;

        case LEN:
            mspLen = b;
            mspChecksum = b;
            st = CMD_LSB;
            break;

        case CMD_LSB:
            mspCmd = b;
            mspChecksum ^= b;
            st = CMD_MSB;
            break;

        case CMD_MSB:
            mspCmd |= (uint16_t)b << 8;
            mspChecksum ^= b;
            idx = 0;
            st = (mspLen == 0) ? CHECKSUM : PAYLOAD;
            break;

        case PAYLOAD:
            mspChecksum ^= b;
            idx++;
            if (idx >= mspLen) st = CHECKSUM;
            break;

        case CHECKSUM:
            if (b == mspChecksum) goodFrameCount++;
            else badChecksumCount++;

            if (goodFrameCount > 0 || badChecksumCount >= BAD_CHECKSUM_THRESHOLD)
                vtxDetected = true;

            resetParser();
            break;
    }
}

void detectionPoll() {
    Serial.println("Checking Serial1");
    while (Serial1.available()) {
        processByte(Serial1.read());
        Serial.print(".");
    }
    Serial.println("READ!");
}