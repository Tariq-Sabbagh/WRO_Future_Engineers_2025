#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <Adafruit_NeoPixel.h>

class NeoPixel {
public:
    NeoPixel(int pin, int numPixels = 2);
    void setup();
    
    // Basic color control
    void setColor(uint32_t color);          // Set both pixels to same color
    void setColor(uint8_t r, uint8_t g, uint8_t b); // Set both pixels to same RGB color
    void setPixelColor(int pixel, uint32_t color);  // Set specific pixel to color
    void setPixelColor(int pixel, uint8_t r, uint8_t g, uint8_t b); // Set specific pixel to RGB
    
    // Predefined colors
    void setRed();
    void setGreen();
    void setBlue();
    void setWhite();
    void setYellow();
    void setCyan();
    void setMagenta();
    void setOrange();
    void setPurple();
    
    // Effects
    void blink(uint32_t color, int times = 3, int delayMs = 250);
    void fadeInOut(uint32_t color, int durationMs = 2000);
    void alternate(uint32_t color1, uint32_t color2, int times = 5, int delayMs = 300);
    void chase(uint32_t color, int times = 3, int delayMs = 100);
    
    // Utility
    void clear();
    void setBrightness(uint8_t brightness);
    void testCycle(unsigned long delayMs = 500);

private:
    int _pin;
    int _numPixels;
    Adafruit_NeoPixel _strip;
};

#endif // NEOPIXEL_H