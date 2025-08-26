#ifndef OpenChallenge_H
#define OpenChallenge_H

#include "config.h"
#include <Wire.h>

#include "MotorController.h"
#include "Steering.h"
#include "IMU.h"
#include "core/PIDController.h"
#include "Encoder.h"
#include "SerialCommunicator.h"
#include "Button.h"
#include "Timer.h"
#include "core/DistanceSensors.h"
#include "core/TOFSensor.h"
#include "MotorPID.h"

class OpenChallenge {
public:
    OpenChallenge();

    void setup();
    void loop();

private:
    
    MotorController      _motors;
    Steering             _steering;
    IMU                  _imu;
    PIDController        _headingPID;     
    Encoder              _encoder;
    SerialCommunicator   _comm;
    Button               _button;
    Timer                _timer;
    DistanceSensors      _ultra;          
    TOFSensor            _backTOF;        
    MotorSpeedPID        _motorPID;       

    
    enum State {
        INIT,
        FORWARD,
        CHECK_TURN,
        TURNING,
        BACKWARD,
        GARAGE,
        IDLE,
        STOPPED
    } _state;

    float _steeringBase   = 0.0f;   
    float _steeringCorr   = 0.0f;
    float _steeringExtra  = 0.0f;   

    float _headingTarget  = 0.0f;   
    unsigned long _lastTurnMs = 0;
    int   _turnCount = 0;

    
    float _turnTargetDeltaDeg = 0.0f;   
    float _backwardTargetCm   = 0.0f;   
    long  _encoderStartTicks  = 0;
    bool turn_right = false;

    
    void _enter(State s);

    
    void _forwardStep();
    
    bool _shouldConsiderTurn() ;
    bool _canTurnLeft()  ;
    bool _canTurnRight() ;

    
    void _beginTurn(float deltaDeg);  
    bool _turnStep();                 

    void _beginBackwardCm(float cm);
    bool _backwardStep(); 

    bool _garageStep();   

    
    void _halt();

 
    void _computeHeadingPID();   
    void _getAwayWalls();        
    void _applySteering();       

    static float _clamp(float v, float lo, float hi);
    // void _handleSerialCommands();
};

#endif // CAR_H
