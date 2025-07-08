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

class ObstacleAvoider {
public:
    ObstacleAvoider();
    void setup();
    void loop();

private:
    // Components
    MotorController _motors;
    Steering _steering;
    IMU _imu;
    PIDController _pid;
    Encoder _encoder;
    SerialCommunicator _comm; // The new communicator object
    Button _button;
    Timer _timer;
    bool reset;


    // State Machine
    enum State { AVOIDING , FORWARD , IDLE};
    State _currentState;

    // Methods
    void _goForward();
    void _goforward();
    void _avoidObstacle();
    void _stopAndHalt();
    void _stopUntilTimer();
};

#endif // OBSTACLE_AVOIDER_H
