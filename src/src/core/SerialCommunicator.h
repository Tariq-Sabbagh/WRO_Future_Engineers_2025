#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#include <Arduino.h>

// This struct defines the exact 4-byte data packet we expect from the Pi.
// It must match the structure in the Python script.
struct ManeuverData {
    uint16_t distance_cm_x10; // Distance in cm, multiplied by 10
    uint16_t angle_deg_x10;   // Angle in degrees, multiplied by 10
};

class SerialCommunicator {
public:
    SerialCommunicator();
    bool getManeuverCommand(float &distance, float &angle);

private:
    ManeuverData _data;
    const size_t PACKET_SIZE = sizeof(ManeuverData);
};

#endif // SERIAL_COMMUNICATOR_H
