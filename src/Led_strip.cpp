#include "Led_strip.h"
#include <FastLED.h>
#include <Arduino.h>
#include <Constants.h>

LedStrip::LedStrip() {}

void LedStrip::begin() {
    FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    clear();
}

void LedStrip::playRainbow() {
    // skip if not enough time has passed since last update
    if (micros() - lastUpdateTime < LED_RAINBOW_DELAY_US) {
        return;
    }

    for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(hue + (i * 10), 255, 255);
    }
    hue++;
    FastLED.show();
    
    lastUpdateTime = micros();
}

void LedStrip::playBlinking(CRGB color, int delayMs) {
    fill_solid(leds, NUM_LEDS, color);
    FastLED.show();
    delay(delayMs);
    clear();
    FastLED.show();
    delay(delayMs);
}

void LedStrip::clear() {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
}