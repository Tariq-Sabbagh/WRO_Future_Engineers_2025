#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include "config.h"
#include "MotorController.h"
#include "Steering.h"
#include "IMU.h"
#include "core/PIDController.h"
#include "Encoder.h"
#include "SerialCommunicator.h" // Include the new communicator
#include "Button.h"
#include <Wire.h>
#include "Timer.h"
#include "core/DistanceSensors.h"

class ObstacleAvoider
{
public:
    ObstacleAvoider();
    void setup();
    void loop();

private:
    // Components
    MotorController _motors;
    Steering _servo;
    IMU _imu;
    PIDController _pid;
    Encoder _encoder;
    SerialCommunicator _comm; // The new communicator object
    Button _button;
    Timer _timer;
    DistanceSensors _ultra;
    float _steeringAngle;
    float _backwardTarget;

    // State Machine
    enum State
    {
        FORWARD,
        AVOIDING,
        IDLE,
        BACKWARD
    };

    State _currentState;

    // Methods
    void _goForward();
    void _goBackward();
    void _avoidObstacle();
    void _stopAndHalt();
    void _stopUntilTimer();
    void _get_away_walls();
    void _turn();
};

#endif // OBSTACLE_AVOIDER_H
