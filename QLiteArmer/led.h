#pragma once
#include <NeoPixelBus.h>

extern NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>* rgb;

void ledInit();
void ledBlue();
void ledRed();
void ledGreen();
void ledOff();
void ledYellow();
void ledAmberFlash();
void ledUpdateErrorFlash();
