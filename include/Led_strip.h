#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <Arduino.h>
#include <FastLED.h>
#include <Constants.h>



class LedStrip {
public:
    LedStrip();
    void begin();
    void playRainbow();
    void playBlinking(CRGB color, int delayMs);
    void clear();

private:
    CRGB leds[NUM_LEDS];
    unsigned long lastUpdateTime = 0;
    uint8_t hue;
};

#endif // LED_STRIP_H