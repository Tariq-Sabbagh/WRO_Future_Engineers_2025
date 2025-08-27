#include "Button.h"

Button::Button(int pin) {
    _pin = pin;
}

void Button::setup() {
    // We use INPUT_PULLUP because the button connects the pin to GND.
    // This means the pin is HIGH when inactive, and LOW when active (pressed).
    Serial.println("Setting up Button....");
    pinMode(_pin, INPUT_PULLUP);
}

/**
 * @brief Checks if the button is currently being pressed.
 * @return true if button is pressed (pin is LOW), false otherwise.
 */
bool Button::isActive() {
    return digitalRead(_pin) == LOW;
}

/**
 * @brief Checks if the button is currently not being pressed.
 * @return true if button is not pressed (pin is HIGH), false otherwise.
 */
bool Button::isInactive() {
    return digitalRead(_pin) == HIGH;
}


void Button::waitForPressOrRestart(NeoPixel& pixels, unsigned long timeoutMs)

{
    Serial.println("----------------------------------------");
    Serial.print("Press button to continue, restarting after ");
    Serial.print(timeoutMs/1000);
    Serial.print(" seconds : ");
    pixels.setGreen();

    while (this->isInactive())
    {
        if (millis()>= timeoutMs)
        {
            Serial.println("Timeout! Restarting ESP32...");
            ESP.restart();  
        }
        delay(10); 
    }
    pixels.clear();
}

/**
 * @brief Pauses the program until the button is pressed.
 */
void Button::waitForPress(const char *message)
{
    Serial.println("----------------------------------------");
    Serial.print("Press button to continue: ");
    Serial.println(message);

    while (this->isInactive())
    {
        delay(10); // Small delay to prevent busy-waiting 
    }
}