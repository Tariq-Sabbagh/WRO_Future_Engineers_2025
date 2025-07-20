#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
                                     _servo(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     _timer(),
                                     _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
                                     _comm() // Initialize the communicator
{
    _currentState = FORWARD;
    _steeringAngle = 0;
}
float distance, angle, _forwardTarget = 0;
int count_turn=0;
void ObstacleAvoider::setup()
{
    Wire.begin();
    _motors.setup();
    _servo.setup();
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

    _comm.clearSerialBuffer();
    Serial.println("Obstacle Avoider Initialized. Waiting for commands...");
}

void ObstacleAvoider::loop()
{
    _encoder.update();
    _imu.update();
    _comm.update();

    if (_comm.getTurn() != 0.f)
    {
        count_turn+=1;
        _forwardTarget += _comm.getTurn();
        _comm.resetTurn();
    }

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
    }
    _get_away_walls();
    if (count_turn >= 12)
    {
        _motors.stop();
        while (true);
        
    }
    _servo.setAngle(_steeringAngle);
}
void ObstacleAvoider::_get_away_walls()
{
    float right = _ultra.getRightCm();
    float left = _ultra.getLeftCm();
    const float k = 1.2;
    if (right <= 40 and right != 0)
    {
        _steeringAngle += k * (40 - right);
    }
    if (left <= 30 and left != 0)
    {
        _steeringAngle -= k * (30 - left);
    }
}
void ObstacleAvoider::_stopUntilTimer()
{
    _motors.stop();
    if (_timer.isFinished())
        _currentState = FORWARD;
}
void ObstacleAvoider::_avoidObstacle()
{
    float correction = 0;
    float currentHeading = _imu.getHeading();
    float currentDistance = _encoder.getDistanceCm();
    if (abs(currentDistance) <= distance)
    {
        correction = _pid.compute(angle, currentHeading);
        _steeringAngle = -correction;
        _motors.forward(FORWARD_SPEED);
    }
    // else if (abs(currentHeading) >= 5)
    // {
    //     // Serial.println("reset car_______________________________________________________");
    //     correction = _pid.compute(0, currentHeading);
    // _steeringAngle = -correction;
    // }
    else
    {
        _comm.resetManeuverValues();
        // _timer.start(200);
        // _currentState = IDLE;
        _currentState = FORWARD;
    }
}

void ObstacleAvoider::_goForward()
{
    if (_comm.getManeuverValues(distance, angle))
    {
        Serial.println("Avoiding Block");

        _encoder.reset();
        _imu.reset();

        distance += 15;
        _currentState = AVOIDING;
    }
    else
    {
        float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
        _steeringAngle = -correction;
        _motors.forward(FORWARD_SPEED);
    }
}

void ObstacleAvoider::_stopAndHalt()
{
    _motors.stop();
    _servo.center();
    Serial.println("Execution Halted.");
    while (true)
        ;
}
