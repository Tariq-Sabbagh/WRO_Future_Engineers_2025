#include "SerialCommunicator.h"

SerialCommunicator::SerialCommunicator() {}

void SerialCommunicator::clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
}

void SerialCommunicator::update() {
    if (Serial.available() < 1) return;

    char cmdType = Serial.peek();  // Peek first byte

    if (cmdType == 'A' && Serial.available() >= AVOID_PACKET_SIZE) {
        uint8_t buffer[AVOID_PACKET_SIZE];
        Serial.readBytes(buffer, AVOID_PACKET_SIZE);
        processAvoidCommand(buffer);
    }
    else if (cmdType == 'T' && Serial.available() >= TURN_PACKET_SIZE) {
        uint8_t buffer[TURN_PACKET_SIZE];
        Serial.readBytes(buffer, TURN_PACKET_SIZE);
        processTurnCommand(buffer);
    }
    // Optionally add more command types here
}

void SerialCommunicator::processAvoidCommand(uint8_t *buffer) {
    int16_t dist_int  = buffer[1] | (buffer[2] << 8);
    int16_t angle_int = buffer[3] | (buffer[4] << 8);

    _distance = dist_int / 10.0f;
    _angle    = angle_int / 10.0f;
    _lastCommand = 'A';

    Serial.print("AVOID CMD: dist=");
    Serial.print(_distance);
    Serial.print(" angle=");
    Serial.println(_angle);
}

void SerialCommunicator::processTurnCommand(uint8_t *buffer) {
    int16_t turn_int = buffer[1] | (buffer[2] << 8);

    _turn = turn_int / 10.0f;
    _lastCommand = 'T';

    Serial.print("TURN CMD: angle=");
    Serial.println(_turn);
}

bool SerialCommunicator::getManeuverValues(float &distance, float &angle) {
    distance = _distance;
    angle    = _angle;
    return distance != -1;
}

void SerialCommunicator::resetManeuverValues() {
    _distance = -1;
    _angle = -1;
}

int SerialCommunicator::getTurn() {
    return _turn;
}

void SerialCommunicator::resetTurn() {
}

