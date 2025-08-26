#include "NeoPixel.h"
#include <Arduino.h>

NeoPixel::NeoPixel(int pin, int numPixels) : 
    _pin(pin),
    _numPixels(numPixels),
    _strip(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void NeoPixel::setup() {
    Serial.println("Setting up NeoPixel strip...");
    _strip.begin();
    _strip.show(); // Initialize all pixels to off
    _strip.setBrightness(100); // Default to medium brightness
    Serial.println("NeoPixel setup complete");
}

void NeoPixel::setColor(uint32_t color) {
    for(int i = 0; i < _numPixels; i++) {
        _strip.setPixelColor(i, color);
    }
    _strip.show();
}

void NeoPixel::setColor(uint8_t r, uint8_t g, uint8_t b) {
    setColor(_strip.Color(r, g, b));
}

void NeoPixel::setPixelColor(int pixel, uint32_t color) {
    if(pixel >= 0 && pixel < _numPixels) {
        _strip.setPixelColor(pixel, color);
        _strip.show();
    }
}

void NeoPixel::setPixelColor(int pixel, uint8_t r, uint8_t g, uint8_t b) {
    setPixelColor(pixel, _strip.Color(r, g, b));
}

void NeoPixel::setRed() {
    setColor(255, 0, 0);
}

void NeoPixel::setGreen() {
    setColor(0, 255, 0);
}

void NeoPixel::setBlue() {
    setColor(0, 0, 255);
}

void NeoPixel::setWhite() {
    setColor(255, 255, 255);
}

void NeoPixel::setYellow() {
    setColor(255, 255, 0);
}

void NeoPixel::setCyan() {
    setColor(0, 255, 255);
}

void NeoPixel::setMagenta() {
    setColor(255, 0, 255);
}

void NeoPixel::setOrange() {
    setColor(255, 165, 0);
}

void NeoPixel::setPurple() {
    setColor(128, 0, 128);
}

void NeoPixel::blink(uint32_t color, int times, int delayMs) {
    for(int i = 0; i < times; i++) {
        setColor(color);
        delay(delayMs);
        clear();
        if(i < times - 1) delay(delayMs);
    }
}

void NeoPixel::fadeInOut(uint32_t color, int durationMs) {
    int steps = 50;
    int delayTime = durationMs / (steps * 2);
    
    // Fade in
    for(int i = 0; i <= steps; i++) {
        int brightness = map(i, 0, steps, 0, 255);
        _strip.setBrightness(brightness);
        setColor(color);
        delay(delayTime);
    }
    
    // Fade out
    for(int i = steps; i >= 0; i--) {
        int brightness = map(i, 0, steps, 0, 255);
        _strip.setBrightness(brightness);
        setColor(color);
        delay(delayTime);
    }
    
    _strip.setBrightness(100); // Reset to default brightness
}

void NeoPixel::alternate(uint32_t color1, uint32_t color2, int times, int delayMs) {
    for(int i = 0; i < times; i++) {
        setPixelColor(0, color1);
        setPixelColor(1, color2);
        delay(delayMs);
        
        setPixelColor(0, color2);
        setPixelColor(1, color1);
        if(i < times - 1) delay(delayMs);
    }
}

void NeoPixel::chase(uint32_t color, int times, int delayMs) {
    for(int t = 0; t < times; t++) {
        for(int i = 0; i < _numPixels; i++) {
            clear();
            setPixelColor(i, color);
            delay(delayMs);
        }
    }
    clear();
}

void NeoPixel::clear() {
    setColor(0, 0, 0);
}

void NeoPixel::setBrightness(uint8_t brightness) {
    _strip.setBrightness(brightness);
    _strip.show();
}

void NeoPixel::testCycle(unsigned long delayMs) {
    Serial.println("Starting NeoPixel test cycle...");
    
    // Test solid colors
    setRed();
    delay(delayMs);
    setGreen();
    delay(delayMs);
    setBlue();
    delay(delayMs);
    setWhite();
    delay(delayMs);
    
    // Test effects
    blink(_strip.Color(255, 0, 0), 3, 250); // Red blink
    alternate(_strip.Color(255, 0, 0), _strip.Color(0, 0, 255), 5, 300); // Red/Blue alternate
    chase(_strip.Color(0, 255, 0), 3, 100); // Green chase
    fadeInOut(_strip.Color(255, 165, 0), 2000); // Orange fade
    
    clear();
    Serial.println("NeoPixel test complete");
}