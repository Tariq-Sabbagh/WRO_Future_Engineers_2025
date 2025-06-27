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


    // State Machine
    enum State { IDLE, AVOIDING };
    State _currentState;

    // Methods
    void _executeManeuver(float distance, float angle);
    void _turn(float targetAngle);
    void _driveDistance(float targetDistance);
    void _stopAndHalt();
};

#endif // OBSTACLE_AVOIDER_H
