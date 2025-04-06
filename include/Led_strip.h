#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <Arduino.h>
#include <FastLED.h>
#include <Constants.h>



class LedStrip {
public:
    LedStrip(uint16_t numLeds, uint8_t dataPin) : numLeds(numLeds), dataPin(dataPin) {
        leds = new CRGB[numLeds];
        hue = 0;
    };
    void begin();
    void updateRainbow();
    void updateSparkle();

    void showLeds() {
        FastLED.show();
    }
    void fadeLeds(uint8_t fadeBy);

    void clear();

private:
    CRGB* leds;
    uint16_t numLeds;
    uint8_t dataPin;
    uint8_t hue;

};

#endif // LED_STRIP_H