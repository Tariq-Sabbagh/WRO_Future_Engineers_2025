#include <Arduino.h>

#pragma once



class SerialCommunicator {
public:
    SerialCommunicator();
    bool getManeuverCommand(float &distance, float &angle);

private:
    static const uint16_t PACKET_SIZE = 4;  // 2 int16_t = 4 bytes
};