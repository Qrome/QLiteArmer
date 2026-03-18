#pragma once
#include <Arduino.h>

extern bool vtxDetected;
extern uint8_t badChecksumCount;
extern uint8_t goodFrameCount;

void detectionInit();
void detectionPoll();