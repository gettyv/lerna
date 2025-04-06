#include "Led_strip.h"
#include <FastLED.h>
#include <Arduino.h>
#include <Constants.h>

void LedStrip::begin() {
    FastLED.addLeds<WS2812, GRB>(leds, numLeds);
    clear();
}

void LedStrip::updateRainbow() {
    for (int i = 0; i < numLeds; i++) {
        leds[i] = CHSV(hue + (i * 10), 255, 255);
    }
    hue++;
}

void LedStrip::updateSparkle() {
    int pos = random(numLeds);
    leds[pos] = CRGB::White;
}

void LedStrip::fadeLeds(uint8_t fadeBy) {
   fadeToBlackBy(leds, numLeds, fadeBy);
}

void LedStrip::clear() {
    fill_solid(leds, numLeds, CRGB::Black);
    FastLED.show();
}