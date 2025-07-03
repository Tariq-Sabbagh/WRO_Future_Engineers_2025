#ifndef CAR_H
#define CAR_H

#include "config.h"
#include "MotorController.h"
#include "Steering.h"
#include "DistanceSensors.h"
#include "IMU.h"
#include "Button.h"

class Car {
public:
    Car();
    void setup();
    void loop();

private:
    // Car Components
    MotorController _motors;
    Steering _steering;
    DistanceSensors _distSensors;
    IMU _imu;
    Button _button;

    // State variables from original script
    float _offsetGyro;
    int _turnCounter;
    unsigned long _previousTurnMillis;

    // Private Methods that mirror original functions
    void _moveStraight();
    void _checkForTurns();
    void _turnRight();
    void _turnLeft();
    void _stopAndHalt();
};

#endif // CAR_H
