#include <Arduino.h>
#include "core/Timer.h"

class SerialCommunicator {
public:
    SerialCommunicator();
    bool getManeuverCommand(float &distance, float &angle, float &diistance_turn);
    void clearSerialBuffer();

private:
    static const uint8_t PACKET_SIZE = 6;  // 3 int16_t = 6 bytes
};