#include "MotorPID.h"

MotorSpeedPID::MotorSpeedPID(MotorController& motor, Encoder& encoder)
    : _motor(motor), _encoder(encoder), _targetDistance(0), _lastOutput(0) {}

void MotorSpeedPID::setup(float kp, float ki, float kd, float outputMin, float outputMax) {
    _pid.setup(kp, ki, kd);
    _pid.setOutputLimits(outputMin, outputMax);
}

void MotorSpeedPID::setTargetDistance(float distanceCm) {
    _targetDistance = distanceCm;
}

void MotorSpeedPID::reset() {
    _encoder.reset();
    _pid.reset();
    _lastOutput = 0;
}

bool MotorSpeedPID::update() {
    _encoder.update();

    float currentDistance = _encoder.getDistanceCm();
    float output = _pid.compute(_targetDistance, currentDistance);

    _lastOutput = output;

    // Stop motor if close enough
    if (fabs(_targetDistance - currentDistance) < 0.5) { // 0.5 cm tolerance
        _motor.stop();
        return true; // Finished
    }

    // Move motor based on PID output
    _motor.move((int)output);

    return false; // Still moving
}

float MotorSpeedPID::getTargetDistance() const {
    return _targetDistance;
}

float MotorSpeedPID::getCurrentDistance() const {
    return _encoder.getDistanceCm();
}

float MotorSpeedPID::getLastOutput() const {
    return _lastOutput;
}
