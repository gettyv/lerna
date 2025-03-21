#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class LedStrip {
public:
    LedStrip(uint16_t numPixels, uint8_t pin);
    ~LedStrip();

    void begin();
    void show();
    void setPixelColor(uint16_t n, uint32_t color);
    void clear();
    void setBrightness(uint8_t brightness);

    // Animation functions
    void rainbow(uint8_t wait);
    void theaterChase(uint32_t color, uint8_t wait);
    void colorWipe(uint32_t color, uint8_t wait);
    void pulse(uint32_t color, uint8_t wait);

private:
    Adafruit_NeoPixel strip;
    uint16_t numPixels;
    uint8_t pin;
};

#endif // LED_STRIP_H