#include "led.h"
#include "config.h"
#include <NeoPixelBus.h>

// Use the generic 800 Kbps method (no PIO)
NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> rgb(LED_COUNT, LED_PIN);

static bool flashState = false;
static uint32_t lastFlash = 0;

void ledInit() {
    rgb.Begin();
    //rgb.SetBrightness(40);
    rgb.Show();
}

static inline void set(uint8_t r, uint8_t g, uint8_t b) {
    RgbColor c(r, g, b);
    rgb.SetPixelColor(0, c);
    rgb.Show();
}

void ledBlue()  { set(0,   0,   20); }
void ledRed()   { set(20, 0,   0);   }
void ledGreen() { set(0,   20, 0);   }
void ledOff()   { set(5,   5,   5);   }
void ledYellow(){ set(20, 20, 0);   }

void ledAmberFlash() {
    uint32_t now = millis();
    if (now - lastFlash > 80) {
        lastFlash = now;
        flashState = !flashState;
        if (flashState) set(255, 80, 0);
        else set(0, 0, 0);
    }
}

void ledUpdateErrorFlash() {
    ledAmberFlash();
}
