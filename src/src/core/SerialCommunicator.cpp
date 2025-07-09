#include "SerialCommunicator.h"

SerialCommunicator::SerialCommunicator() {}

void SerialCommunicator::clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

bool SerialCommunicator::getManeuverCommand(float &distance, float &angle, float &diistance_turn) {
    if (Serial.available() < PACKET_SIZE) 
        return false;

    // Read raw bytes
    uint8_t buffer[PACKET_SIZE];
    Serial.readBytes(buffer, PACKET_SIZE);

    // Unpack little-endian values
    int16_t dist_int = buffer[0] | (buffer[1] << 8);
    int16_t angle_int = buffer[2] | (buffer[3] << 8);
    int16_t dist_turn_int = buffer[4] | (buffer[5] << 8);

    // Debug output
    Serial.print("Received: ");
    Serial.print(dist_int);
    Serial.print(", ");
    Serial.print(angle_int);
    Serial.print(", ");
    Serial.println(dist_turn_int);

    // Convert to floats
    distance = dist_int / 10.0f;
    angle = angle_int / 10.0f;
    diistance_turn = dist_turn_int / 10.0f;

    return true;
}