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
float distance, angle,distance_turn;
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
    if(_ultra.getFrontCm()<=30)
    {
        _timer.start(1000);
        float correction_turn = _pid.compute(90, _imu.getHeading());
        _steering.setAngle(-correction_turn);
        _motors.backward(FORWARD_SPEED - 30);
        _timer.isFinished();
        _currentState = FORWARD;
    }

   
    float correction = _pid.compute(0, _imu.getHeading());
    _steering.setAngle(-correction);
    _motors.forward(FORWARD_SPEED - 30);
}
void ObstacleAvoider::_stopUntilTimer()
{
    _motors.stop();
    if(_timer.isFinished()) _currentState = FORWARD;
    
}
void ObstacleAvoider::_avoidObstacle()
{ 
    float correction = 0;
    float currentHeading = _imu.getHeading();
    float currentDistance = _encoder.getDistanceCm();
    if (abs(currentDistance) <= distance)
    {
        correction = _pid.compute(angle, currentHeading);
        _steering.setAngle(-correction);
        _motors.forward(FORWARD_SPEED - 30);
    }
    else if (abs(_imu.getHeading()) >= 5)
    {
        // Serial.println("reset car_______________________________________________________");
        currentHeading = _imu.getHeading();
        correction = _pid.compute(0, currentHeading);
        _steering.setAngle(-correction);
    }
    else
    {
        _comm.clearSerialBuffer();
        _timer.start(2000);
        _currentState = IDLE;
    }
}

void ObstacleAvoider::_goForward()
{
    if (_comm.getManeuverCommand(distance, angle ,distance_turn))
    {
        Serial.print("Received command -> Distance: ");
        Serial.print(distance);
        Serial.print(" cm, Angle: ");
        Serial.print(angle);
        Serial.println(" degrees.");

        _encoder.reset();

        if(angle >= 90)
        {
            _currentState = TURN;
        }
        _currentState = AVOIDING;
    }
    else
    {
        float correction = _pid.compute(0, _imu.getHeading());
        _steering.setAngle(-correction);
        // _motors.forward(FORWARD_SPEED - 30);
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
