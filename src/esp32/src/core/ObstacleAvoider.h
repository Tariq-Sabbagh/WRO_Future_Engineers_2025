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
#include "MotorPID.h"
#include "core/DistanceSensors.h"
#include "core/TOFSensor.h"
#include <esp_system.h>




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
    TOFSensor _backSensor;
    MotorSpeedPID _motorPID;
    float _steeringAngle;
    float _backwardTarget;
    int _pid_target_parking = -100;
    int _pid_target_parking_In = -90;

    // State Machine
    enum State
    {
        FORWARD,
        AVOIDING,
        IDLE,
        BACKWARD,
        TURN,
        RESET,
        GARAGE
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
    void _resetCar();
    void _garageDoOut();
    void _garageDoIn();
    void _goForwardPID(float targetDistanceCm);
    void _goUntilDistanceUltra(int distance,int motorSpeed); 
    void _goForwardAngle(int motorSpeed,int error);
    void _goBackwardAngle(int motorSpeed,int error);
    void _goUntilDistanceToF(int distance, int motorSpeed);
    
};

#endif // OBSTACLE_AVOIDER_H
