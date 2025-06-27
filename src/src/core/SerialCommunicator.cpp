#include "SerialCommunicator.h"

SerialCommunicator::SerialCommunicator() {}

/**
 * @brief Checks for and decodes a new maneuver command from the serial port.
 * @param distance Reference to a float to store the decoded distance.
 * @param angle Reference to a float to store the decoded angle.
 * @return true if a new, valid command was received and decoded.
 */
bool SerialCommunicator::getManeuverCommand(float &distance, float &angle) {
    // Check if a full packet of bytes is available to be read.
    if (Serial.available() >= PACKET_SIZE) {
        // Read the bytes directly into the struct. This is fast and efficient.
        Serial.readBytes((char*)&_data, PACKET_SIZE);

        // Decode the integer values back into floats.
        // We divide by 10.0 to restore the decimal point.
        distance = (float)_data.distance_cm_x10 / 10.0f;
        angle = (float)_data.angle_deg_x10 / 10.0f;
        
        return true; // A new command was processed.
    }
    return false; // No new command.
}
