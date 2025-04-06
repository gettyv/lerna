#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <Arduino.h>
#include <FastLED.h>
#include <Constants.h>



class LedStrip {
public:
    LedStrip();
    void begin();
    void updateRainbow();
    void updateSparkle();

    void showLeds() {
        FastLED.show();
    }
    void fadeLeds(uint8_t fadeBy);

    void clear();

private:
    CRGB leds[LEDS_1_NUM];
    uint8_t hue;

};

#endif // LED_STRIP_H