#include "led.h"
#include "config.h"

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>* rgb = nullptr;

static bool flashState = false;
static uint32_t lastFlash = 0;

void ledInit() {
    // Allocate dynamically AFTER system is stable
    rgb = new NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod>(LED_COUNT, LED_PIN);

    delay(50); // allow DMA/IRQ to settle
    Serial.println("[LED] Calling LED Begin");
    rgb->Begin();
    rgb->Show();
    Serial.println("[LED] Leaving ledInit()");
}

static inline void set(uint8_t r, uint8_t g, uint8_t b) {
    if (!rgb) return;
    rgb->SetPixelColor(0, RgbColor(r, g, b));
    rgb->Show();
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
        if (flashState) set(20, 10, 0);
        else set(0, 0, 0);
    }
}

void ledUpdateErrorFlash() {
    ledAmberFlash();
}
