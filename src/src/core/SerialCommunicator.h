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
    int getTurn();
    void resetTurn();

private:
    static const uint16_t AVOID_PACKET_SIZE = 5; // 'A' + 2x int16
    static const uint16_t TURN_PACKET_SIZE = 3;  // 'T' + 1x int16

    float _distance = 0.0f;
    float _angle = 0.0f;
    float _turn = 0.0f;
    char _lastCommand = 0; // Stores 'A' or 'T'

    Timer timer;

    void processAvoidCommand(uint8_t *buffer);
    void processTurnCommand(uint8_t *buffer);
};
