#include "PIDController.h"
#include "MotorController.h"
#include "Steering.h"
#include "DistanceSensors.h"
#include "IMU.h"
#include "Button.h"

class Car {
private:
    MotorController _motors;
    Steering _steering;
    DistanceSensors _distSensors;
    IMU _imu;
    Button _button;
    PIDController _pid;  // PID controller for steering
    
    // Navigation state
    float _offsetGyro;
    int _turnCounter;
    unsigned long _previousTurnMillis;
    String direction;
    float error;
    float angle;


    // Internal methods
    void _checkForTurns();
    void _moveStraight();
    void _turnRight();
    void _turnLeft();
    void _stopAndHalt();

public:
    Car();
    void setup();
    void loop();
};