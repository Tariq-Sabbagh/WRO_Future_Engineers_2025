#include "SerialCommunicator.h"
#include <Arduino.h>

SerialCommunicator::SerialCommunicator() {}

bool SerialCommunicator::getManeuverCommand(float &distance, float &angle) {
    if (Serial.available() >= PACKET_SIZE) {
        uint8_t buffer[4];
        Serial.readBytes(buffer, PACKET_SIZE);
        
        // Manual unpacking in little-endian format
        int16_t dist_int = buffer[0] | (buffer[1] << 8);
        int16_t angle_int = buffer[2] | (buffer[3] << 8);
        
        // Debug print raw integers
        Serial.print("Raw integers: ");
        Serial.print(dist_int);
        Serial.print(", ");
        Serial.println(angle_int);
        
        distance = dist_int / 10.0f;
        angle = angle_int / 10.0f;
        return true;
    }
    return false;
}