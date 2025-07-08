#include "SerialCommunicator.h"
#include <Arduino.h>

SerialCommunicator::SerialCommunicator() {}

// Clears all data from the serial buffer
void SerialCommunicator::clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

bool SerialCommunicator::getManeuverCommand(float &distance, float &angle) {
    // Read all complete packets (4 bytes each) but only keep the last one
    bool packetAvailable = false;
    uint8_t buffer[4];
    
    while (Serial.available() >= PACKET_SIZE) {
        // Read a full packet into buffer
        Serial.readBytes(buffer, PACKET_SIZE);
        packetAvailable = true; // Mark that we have at least one valid packet
    }
    
    if (packetAvailable) {
        // Unpack the last received packet (little-endian format)
        int16_t dist_int = buffer[0] | (buffer[1] << 8);
        int16_t angle_int = buffer[2] | (buffer[3] << 8);
        
        // Debug print raw integers (only for the last packet)
        Serial.print("Raw integers: ");
        Serial.print(dist_int);
        Serial.print(", ");
        Serial.println(angle_int);
        
        // Convert to floats
        distance = dist_int / 10.0f;
        angle = angle_int / 10.0f;
        return true;
    }
    return false;
}