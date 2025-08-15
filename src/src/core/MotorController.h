#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "Encoder.h" 


class MotorController {
public:
    MotorController(int dir1_pin, int dir2_pin, int speed_pin , Encoder* encoder);
    void setup();
    void forward(int speed);
    void backward(int speed);
    void move(int speed);
    void stop();
    void stopBreak(int direction);

    
    void setTargetSpeed(float target);  // in deg/sec
    void updatePID();                   // call periodically

private:
    int _dir1_pin;
    int _dir2_pin;
    int _speed_pin;

    Encoder* _encoder;

    float _targetSpeed = 0;
    float _kp = 20.0, _ki = 0.5, _kd = 0.5;
    float _prevError = 0, _integral = 0;
    int _currentPWM = 0;
    unsigned long _lastPIDTime = 0;
    float _prevDistance = 0;
};

#endif // MOTOR_CONTROLLER_H