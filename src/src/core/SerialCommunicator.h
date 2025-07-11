#pragma once

#include <Arduino.h>
#include "core/Timer.h"

class SerialCommunicator
{
public:
    SerialCommunicator();

    void update(); // Call regularly to process serial input
    void clearSerialBuffer();
    bool getManeuverValues(float &distance, float &angle);
    void resetManeuverValues();
    float getTurn();
    void resetTurn();

private:
    static const uint16_t PACKET_SIZE = 5; // Unified: 1 header + 2 values

    float _distance = -1.0f;
    float _angle = -1.0f;
    float _turn = 0.0f;
    char _lastCommand = 0;

    Timer timer;

    void processPacket(uint8_t *buffer);
};
