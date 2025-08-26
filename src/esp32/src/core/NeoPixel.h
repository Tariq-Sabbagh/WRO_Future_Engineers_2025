#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <Adafruit_NeoPixel.h>

class NeoPixel {
public:
    NeoPixel(int pin, int numPixels = 1);
    void setup();
    void setColor(uint32_t color);
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void clear();
    void testCycle(unsigned long delayMs = 500);

private:
    int _pin;
    int _numPixels;
    Adafruit_NeoPixel _strip;
};

#endif // NEOPIXEL_H