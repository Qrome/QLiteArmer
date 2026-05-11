#include "led.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel rgb(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

static bool flashState = false;
static uint32_t lastFlash = 0;

void ledInit() {
    rgb.begin();
    rgb.setBrightness(40);
}

void set(uint8_t r, uint8_t g, uint8_t b) {
    rgb.setPixelColor(0, rgb.Color(r,g,b));
    rgb.show();
}

void ledBlue()  { set(0,0,255); }
void ledRed()   { set(255,0,0); }
void ledGreen() { set(0,255,0); }
void ledOff()   { set(0,0,0); }
void ledYellow(){ set(50,50,0); }

void ledAmberFlash() {
    uint32_t now = millis();
    if (now - lastFlash > 80) {  // rapid flash
        lastFlash = now;
        flashState = !flashState;
        if (flashState) set(255,80,0);
        else set(0,0,0);
    }
}

void ledUpdateErrorFlash() {
    ledAmberFlash();
}