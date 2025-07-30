// SerialCommunicator.h
#pragma once
#include <Arduino.h>

class SerialCommunicator {
public:
    // ترجع true إذا وصل رقم جديد، وتضعه في value
    bool getCommand(int16_t &value);
};
