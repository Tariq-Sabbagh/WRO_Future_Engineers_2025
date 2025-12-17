#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
                                     _steering(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     _timer(),
                                     _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
                                     _comm() // Initialize the communicator
{
    _currentState = FORWARD;
}
float distance, angle,distance_turn,adjacent_side,opposite_side;
void ObstacleAvoider::setup()
{
    Wire.begin();
    _motors.setup();
    _steering.setup();
    _button.setup();
    _encoder.begin();
    while (!Serial)
        ; // Wait for serial monitor to open (remove for production)
    Serial.println("ESP32 Ready");

    if (!_imu.setup())
    {
        Serial.println("FATAL: IMU failed to initialize.");
        _stopAndHalt();
    }
    _button.waitForPress();
    _pid.setup(3.5, 0, 0);
    _pid.setOutputLimits(-90, 90);

    Serial.println("Obstacle Avoider Initialized. Waiting for commands...");
}

void ObstacleAvoider::loop()
{
    _encoder.update();
    _imu.update();
    switch (_currentState)
    {
    case AVOIDING:
        _avoidObstacle();
        break;

    case IDLE:
        _stopUntilTimer();
        break;

    case FORWARD:
        _goForward();
        break;

    case TURN:
        _turn();
        break;
    }
    

}
void ObstacleAvoider::_turn()
{
    Serial.println("turn______________________________");
    float correction = 0;
    float currentHeading = _imu.getHeading();
    float currentDistance = _encoder.getDistanceCm();
    adjacent_side = (cos(angle) * distance) -20;
    if (abs(currentDistance) <= adjacent_side)
        {
            correction = _pid.compute(0, currentHeading);
            _steering.setAngle(-correction);
            _motors.forward(FORWARD_SPEED - 30);
        }
    else 
    {
        correction = _pid.compute(90, currentHeading);
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 30);
        if (_pid.getError() <=2)
        {
            _encoder.reset();
            while(abs(currentDistance) <= opposite_side)
            {
                correction = _pid.compute(0, currentHeading);
                _steering.setAngle(-correction);
                _motors.forward(FORWARD_SPEED - 30); 
            }
            _currentState = FORWARD;
        }
        
        
    }
    

}
void ObstacleAvoider::_stopUntilTimer()
{
    _motors.stop();
    if(_timer.isFinished()) _currentState = FORWARD;
    
}
void ObstacleAvoider::_avoidObstacle()
{ 
    float correction = 0;
    float currentDistance = _encoder.getDistanceCm();
    opposite_side = abs((sin(angle) * distance) - 15);
    Serial.println(opposite_side);
    Serial.println(_ultra.getRightCm() );
    if(_ultra.getRightCm() > opposite_side){
        while (abs(currentDistance) <= distance - 10)
        {
            _imu.update();
            _encoder.update();
            float currentHeading = _imu.getHeading();
            currentDistance = _encoder.getDistanceCm();
            // Serial.println(currentHeading);
            // Serial.println(currentDistance);
            correction = _pid.compute(angle, currentHeading);
            _steering.setAngle(-correction);
            _motors.forward(FORWARD_SPEED - 30);
        }
        while (abs(_imu.getHeading()) >= 5)
        {
            _imu.update();
            _encoder.update();
            // Serial.println(_pid.getError());
            float currentHeading = _imu.getHeading();
            correction = _pid.compute(0, currentHeading);
            _steering.setAngle(-correction);
        }
        
            _comm.clearSerialBuffer();
            _timer.start(2000);
            _currentState = IDLE;
        
    }
    else 
    {
        _currentState =TURN;
    }
}

void ObstacleAvoider::_goForward()
{
    if (_comm.getManeuverCommand(distance, angle))
    {
        Serial.print("Received command -> Distance: ");
        Serial.print(distance);
        Serial.print(" cm, Angle: ");
        Serial.print(angle);
        Serial.println(" degrees.");

        _encoder.reset();

        _currentState = AVOIDING;
    }
    else
    {
        float correction = _pid.compute(0, _imu.getHeading());
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 30);
    }
}

void ObstacleAvoider::_stopAndHalt()
{
    _motors.stop();
    _steering.center();
    Serial.println("Execution Halted.");
    while (true)
        ;
}
