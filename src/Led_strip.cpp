#include "Led_strip.h"
#include <FastLED.h>
#include <Arduino.h>
#include <Constants.h>

LedStrip::LedStrip() {}

void LedStrip::begin() {
    FastLED.addLeds<WS2812, LEDS_1_DATA_PIN, GRB>(leds, LEDS_1_NUM);
    clear();
}

void LedStrip::updateRainbow() {
    for (int i = 0; i < LEDS_1_NUM; i++) {
        leds[i] = CHSV(hue + (i * 10), 255, 255);
    }
    hue++;
}

void LedStrip::updateSparkle() {
    int pos = random(LEDS_1_NUM);
    leds[pos] = CRGB::White;
}

void LedStrip::fadeLeds(uint8_t fadeBy) {
   fadeToBlackBy(leds, LEDS_1_NUM, fadeBy);
}

void LedStrip::clear() {
    fill_solid(leds, LEDS_1_NUM, CRGB::Black);
    FastLED.show();
}