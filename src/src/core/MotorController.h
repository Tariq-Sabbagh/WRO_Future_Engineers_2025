#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class MotorController {
public:
    MotorController(int dir1_pin, int dir2_pin, int speed_pin);
    void setup();
    void forward(int speed);
    void backward(int speed);
    void move(int speed);
    void stop();

private:
    int _dir1_pin;
    int _dir2_pin;
    int _speed_pin;
};

#endif // MOTOR_CONTROLLER_H