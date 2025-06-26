#ifndef CAR_H
#define CAR_H

#include "PIDController.h"
#include "MotorController.h"
#include "Steering.h"
#include "DistanceSensors.h"
#include "IMU.h"
#include "Button.h"
#include "core/Timer.h"

class Car
{
public:
    // Hardware
    MotorController _motors;
    Steering _steering;
    DistanceSensors _distSensors;
    IMU _imu;
    Button _button;
    PIDController _pid;

    // State
    int _turnCounter;
    unsigned long _previousTurnMillis;
    String direction;
    float error;
    float angle;

    // Constants
    static const int _totalLabs = 3;
    static const int _segmentsPerLab = 4;
    static const int _all_turn = _totalLabs * _segmentsPerLab;

    // Internal logic
    void _checkForTurns();
    void _moveStraight();
    bool _empty_on_right();
    bool _empty_on_left();
    void _stopAndHalt();

    // New navigation structure
    void _runCourse();
    void _runLab();
    void _goUntilTurn();
    void _decideAndTurn();
    void _turn(float angle);

    Car();
    void setup();
    void loop();
};

#endif
