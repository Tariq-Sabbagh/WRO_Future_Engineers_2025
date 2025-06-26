#include "Button.h"

Button::Button(int pin)
{
    _pin = pin;
}

void Button::setup()
{
    // We use INPUT_PULLUP because the button connects the pin to GND.
    // This means the pin is HIGH when inactive, and LOW when active (pressed).
    pinMode(_pin, INPUT_PULLUP);
}

/**
 * @brief Checks if the button is currently being pressed.
 * @return true if button is pressed (pin is LOW), false otherwise.
 */
bool Button::isActive()
{
    return digitalRead(_pin) == LOW;
}

/**
 * @brief Checks if the button is currently not being pressed.
 * @return true if button is not pressed (pin is HIGH), false otherwise.
 */
bool Button::isInactive()
{
    return digitalRead(_pin) == HIGH;
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
