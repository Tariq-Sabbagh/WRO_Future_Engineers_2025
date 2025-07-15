#include "SerialCommunicator.h"

SerialCommunicator::SerialCommunicator() {}

void SerialCommunicator::clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

void SerialCommunicator::update() {
    if (Serial.available() < 1) return;

    // if (Serial.peek() == 'A' || Serial.peek() == 'T') {
    uint8_t buffer[PACKET_SIZE];
    Serial.readBytes(buffer, PACKET_SIZE);
    processPacket(buffer);
    // } else {
        // Unknown header, discard one byte
        // Serial.read();
    // }
}

void SerialCommunicator::processPacket(uint8_t *buffer) {
    char type = buffer[0];
    int16_t val1 = buffer[1] | (buffer[2] << 8);
    int16_t val2 = buffer[3] | (buffer[4] << 8);
    // Serial.println(type);
    if (type == 'A') {
        _distance = val1 / 10.0f;
        _angle    = val2 / 10.0f;
        _lastCommand = 'A';

        // Serial.print("AVOID CMD: dist=");
        // Serial.print(_distance);
        // Serial.print(" angle=");
        // Serial.println(_angle);
    }
    else if (type == 'T') {
        _turn = val1 / 10.0f;  // Only use first int
        _lastCommand = 'T';

        Serial.print("TURN CMD: angle=");
        Serial.println(_turn);
    }
}

bool SerialCommunicator::getManeuverValues(float &distance, float &angle) {
    distance = _distance;
    angle    = _angle;
    return _distance != -1;
}

void SerialCommunicator::resetManeuverValues() {
    _distance = -1;
    _angle = -1;
}

float SerialCommunicator::getTurn() {
    return _turn;
}

void SerialCommunicator::resetTurn() {
    _turn = 0;
}
