#ifndef MOTOR_SPEED_PID_H
#define MOTOR_SPEED_PID_H

#include <Arduino.h>
#include "PIDController.h"
#include "Encoder.h"
#include "MotorController.h"

class MotorSpeedPID {
public:
    MotorSpeedPID(MotorController& motor, Encoder& encoder);

    void setup(float kp, float ki, float kd, float outputMin = -255, float outputMax = 255);
    void setTargetDistance(float distanceCm);
    void reset();

    // Call this in loop()
    bool update();

    float getTargetDistance() const;
    float getCurrentDistance() const;
    float getLastOutput() const;

private:
    MotorController& _motor;
    Encoder& _encoder;
    PIDController _pid;

    float _targetDistance;
    float _lastOutput;
};

#endif
