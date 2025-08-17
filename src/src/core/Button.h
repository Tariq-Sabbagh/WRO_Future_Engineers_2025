#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
public:
    Button(int pin);
    void setup();
    bool isActive();   // Returns true if the button is pressed (active)
    bool isInactive(); // Returns true if the button is not pressed (inactive)
    void waitForPress(const char *message = ""); // Blocks code execution until button is pressed

private:
    int _pin;
};

#endif // BUTTON_H