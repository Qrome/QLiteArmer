#include "detection.h"
#include "config.h"

bool vtxDetected      = false;
uint8_t badChecksumCount = 0;
uint8_t goodFrameCount   = 0;

// MSP v1 parser states
enum ParseState {
    IDLE,
    HDR_M,
    HDR_DIR,
    LEN,
    CMD,       // MSP v1: single byte command
    PAYLOAD,
    CHECKSUM
};

static ParseState st      = IDLE;
static uint8_t mspLen     = 0;
static uint8_t mspCmd     = 0;   // MSP v1 = 8-bit command
static uint8_t mspChecksum = 0;
static uint8_t payloadIdx  = 0;

void detectionInit() {
    st           = IDLE;
    mspLen       = 0;
    mspCmd       = 0;
    mspChecksum  = 0;
    payloadIdx   = 0;
    vtxDetected  = false;
    badChecksumCount = 0;
    goodFrameCount   = 0;
}

static void resetParser() {
    st          = IDLE;
    mspLen      = 0;
    mspCmd      = 0;
    mspChecksum = 0;
    payloadIdx  = 0;
}

// Process one incoming byte through the MSP v1 parser
static void processByte(uint8_t b) {
    switch (st) {

        case IDLE:
            if (b == '$') st = HDR_M;
            break;

        case HDR_M:
            if (b == 'M') st = HDR_DIR;
            else resetParser();
            break;

        case HDR_DIR:
            // Accept both directions: '<' (request) and '>' (response)
            if (b == '<' || b == '>') st = LEN;
            else resetParser();
            break;

        case LEN:
            mspLen      = b;
            mspChecksum = b;   // checksum starts with length
            st          = CMD;
            break;

        case CMD:
            // MSP v1: command is a single uint8_t
            mspCmd       = b;
            mspChecksum ^= b;
            payloadIdx   = 0;
            st = (mspLen == 0) ? CHECKSUM : PAYLOAD;
            break;

        case PAYLOAD:
            mspChecksum ^= b;
            payloadIdx++;
            if (payloadIdx >= mspLen) st = CHECKSUM;
            break;

        case CHECKSUM:
            if (b == mspChecksum) {
                goodFrameCount++;
            } else {
                badChecksumCount++;
            }

            // Detect VTX if:
            //   - at least one valid frame arrived, OR
            //   - enough bad-checksum frames (VTX talking but format drifted)
            if (goodFrameCount > 0 ||
                badChecksumCount >= BAD_CHECKSUM_THRESHOLD) {
                vtxDetected = true;
            }

            resetParser();
            break;
    }
}

// Called from state machine — drain Serial1 RX buffer
void detectionPoll() {
    while (Serial1.available()) {
        processByte((uint8_t)Serial1.read());
    }
}