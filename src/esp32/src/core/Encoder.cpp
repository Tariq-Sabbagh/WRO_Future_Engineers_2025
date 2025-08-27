#include "Encoder.h"

#define TICKS_PER_REV 4096.0
#define WHEEL_CIRCUMFERENCE_CM 42.7256  // Adjust this for your wheel

Encoder::Encoder()
    : _lastAngle(0), _totalAngle(0), _distanceCm(0) {}

bool Encoder::setup() {
    Wire.begin();
    if (!_as5600.begin()) {
    Serial.println("Encoder not detected!");
    while (1);
  }

  _lastAngle =  _as5600.readAngle();
    _totalAngle = 0;
    _distanceCm = 0;
    return true;
}

void Encoder::reset() {
    _lastAngle = _as5600.readAngle();
    _totalAngle = 0;
    _distanceCm = 0;
}

void Encoder::update() {
    int currentAngle = _as5600.readAngle();
    int delta = currentAngle - _lastAngle;

    // Handle wrap-around
    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;

    _totalAngle += delta;
    _lastAngle = currentAngle;

    _distanceCm = (_totalAngle / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_CM;
}

float Encoder::getDistanceCm() const {
    return _distanceCm;
}
