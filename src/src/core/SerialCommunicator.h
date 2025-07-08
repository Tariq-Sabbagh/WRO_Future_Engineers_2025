#include <Arduino.h>
#include "core/Timer.h"
#pragma once



class SerialCommunicator {
public:
    SerialCommunicator();
    Timer timer;
    bool getManeuverCommand(float &distance, float &angle);
    void clearSerialBuffer();

private:
    static const uint16_t PACKET_SIZE = 4;  // 2 int16_t = 4 bytes
};