#ifndef STEERING_H
#define STEERING_H

#include <ESP32Servo.h>
#include "config.h"

class Steering {
public:
    Steering(int servo_pin);
    void setup();
    void setAngle(int angle);
    void center();

private:
    Servo _servo;
    int _servo_pin;
};

#endif // STEERING_H