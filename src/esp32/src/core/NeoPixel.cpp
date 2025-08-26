#include "core/NeoPixel.h"

NeoPixel::NeoPixel(int pin, int numPixels) : 
    _pin(pin),
    _numPixels(numPixels),
    _strip(numPixels, pin, NEO_GRB + NEO_KHZ800) {}

void NeoPixel::setup() {
    Serial.println("Setting up NeoPixel...");
    _strip.begin();
    _strip.show(); // Initialize all pixels to off
    Serial.println("NeoPixel setup complete");
}

void NeoPixel::setColor(uint32_t color) {
    _strip.fill(color);
    _strip.show();
}

void NeoPixel::setColor(uint8_t r, uint8_t g, uint8_t b) {
    setColor(_strip.Color(r, g, b));
}

void NeoPixel::clear() {
    setColor(0, 0, 0);
}

void NeoPixel::testCycle(unsigned long delayMs) {
    Serial.println("Starting NeoPixel test cycle...");
    
    uint32_t colors[] = {
        _strip.Color(255, 0, 0), // Red
        _strip.Color(0, 255, 0), // Green
        _strip.Color(0, 0, 255), // Blue
        _strip.Color(255, 255, 255) // White
    };

    for (uint32_t color : colors) {
        setColor(color);
        delay(delayMs);
    }
    clear();
    Serial.println("NeoPixel test complete");
}