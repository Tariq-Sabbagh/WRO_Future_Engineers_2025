#include "MotorController.h"
#include "../config.h"

MotorController::MotorController(int dir1_pin, int dir2_pin, int speed_pin, Encoder* encoder) {
    _dir1_pin = dir1_pin;
    _dir2_pin = dir2_pin;
    _speed_pin = speed_pin;
    _encoder = encoder;
}

void MotorController::setup() {
    Serial.println("Setting up Motors....");
    pinMode(_dir1_pin, OUTPUT);
    pinMode(_dir2_pin, OUTPUT);
    pinMode(_speed_pin, OUTPUT);
    stop();
}

void MotorController::forward(int speed) {
    digitalWrite(_dir1_pin, HIGH);
    digitalWrite(_dir2_pin, LOW);
    analogWrite(_speed_pin, speed);
}

void MotorController::backward(int speed) {
    digitalWrite(_dir1_pin, LOW);
    digitalWrite(_dir2_pin, HIGH);
    analogWrite(_speed_pin, speed);
}

void MotorController::move(int speed) {
    if (speed > 0) forward(speed);
    else backward(-speed);
}

void MotorController::stop() {
    digitalWrite(_dir1_pin, LOW);
    digitalWrite(_dir2_pin, LOW);
    analogWrite(_speed_pin, 0);
}

void MotorController::stopBreak(int direction) {
    if (direction > 0) {
        move(255);
        delay(100);
    } else if (direction < 0) {
        move(-255);
        delay(100);
    }
    digitalWrite(_dir1_pin, LOW);
    digitalWrite(_dir2_pin, LOW);
    analogWrite(_speed_pin, 0);
}

void MotorController::setTargetSpeed(float targetCmPerSec) {
    _targetSpeed = targetCmPerSec;
}

void MotorController::updatePID() {
    unsigned long now = millis();
    float dt = (now - _lastPIDTime) / 1000.0f; // seconds

    if (dt <= 0.0f || dt > 0.5f) {  // skip if too soon or too long gap
        _lastPIDTime = now;
        _prevDistance = _encoder->getDistanceCm();
        return;
    }

    _encoder->update(); // refresh encoder data

    // Calculate actual speed
    float currentDistance = _encoder->getDistanceCm();
    float actualSpeed = (currentDistance - _prevDistance) / dt; // cm/sec
    _prevDistance = currentDistance;

    // PID calculations
    float error = _targetSpeed - actualSpeed;
    _integral += error * dt;
    float derivative = (error - _prevError) / dt;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    // Update PWM (and clamp)
    _currentPWM += output;

// Clamp to allowed range if moving
if (_targetSpeed != 0) {
    if (_currentPWM > 0) {
        if (_currentPWM < 170) _currentPWM = 170;
        if (_currentPWM > 255) _currentPWM = 255;
    } else if (_currentPWM < 0) {
        if (_currentPWM > -170) _currentPWM = -170;
        if (_currentPWM < -255) _currentPWM = -255;
    }
} else {
    // Stop if target speed is zero
    _currentPWM = 0;
}

move(_currentPWM);

    _prevError = error;
    _lastPIDTime = now;
}