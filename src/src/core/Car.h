#ifndef CAR_H
#define CAR_H

#include "config.h"
#include "core/MotorController.h"
#include "core/Steering.h"
#include "core/DistanceSensors.h"
#include "core/IMU.h"
#include "core/Button.h"
#include "core/PIDController.h"
#include "core/Timer.h"

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
    Timer _timer;
    PIDController _pid;

    // State variables from original script
    float _offsetGyro;
    int _turnCounter;
    unsigned long _previousTurnMillis;
    float error ;
    String direction="";

    // Private Methods that mirror original functions
    void _moveStraight();
    void _checkForTurns();
    void _turnRight();
    void _turnLeft();
    void _stopAndHalt();
};

#endif // CAR_H
