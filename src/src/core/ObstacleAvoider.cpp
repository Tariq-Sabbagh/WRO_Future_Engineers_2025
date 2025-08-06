#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
                                     _servo(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     _timer(),
                                     _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
                                     _backSensor(SHT_LOX, 0x20),
                                     _comm() // Initialize the communicator
{
    _currentState = FORWARD;
    _steeringAngle = 0;
}
float distance, angle, _forwardTarget = 0;
int count_turn = 0;
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

    Serial.println("Starting TOF Sensor...");

    if (!_backSensor.begin())
    {
        Serial.println("Sensor failed to init!");
        while (1)
            ;
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
    _backSensor.update();
    if (_comm.getTurn() != 0.f)
    {
        _currentState = TURN;
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
    case TURN:
        _turn();
        break;
    case RESET:
        _resetCar();
        break;
        // case BACKWARD:
        //     _goBackward();
        //     break;
    }
    _get_away_walls();
    if (count_turn >= 50)
    {
        _motors.stop();
        while (true)
            ;
    }
    // Serial.println(_steeringAngle);
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

void ObstacleAvoider::_resetCar()
{
    float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
    _steeringAngle = correction;
    _motors.move(-FORWARD_SPEED);
    // Serial.println("in reset_________________");
    int distanceTOF = _backSensor.getDistance();

    if (distanceTOF <= 300 )
    {
        count_turn++;
        _currentState = FORWARD;
    }
}
void ObstacleAvoider::_avoidObstacle()
{
    float correction = 0;
    float currentHeading = _imu.getHeading();
    float currentDistance = _encoder.getDistanceCm();
    float dis = distance * sin(angle);
    float left = _ultra.getLeftCm();
    float checkdis = 100 - left ;
    if (dis  > checkdis)
    {
         _currentState = FORWARD;
    }
    else{
    if (abs(currentDistance) <= distance)
    {
        correction = _pid.compute(angle, currentHeading);
        _steeringAngle = -correction;
        _motors.forward(FORWARD_SPEED);
    }
    else
    {
        _comm.resetManeuverValues();
        Serial.println("Done avoiding_________");
        _currentState = FORWARD;
    }
}
}
void ObstacleAvoider::_turn()
{
    float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
    _steeringAngle = -correction;

    if (_ultra.getFrontCm() <= 35 and _pid.geterror() < abs(20))
    {
        Serial.println(_pid.geterror());
        _forwardTarget += _comm.getTurn();
        _comm.resetTurn();
        _currentState = RESET;
    }
}
void ObstacleAvoider::_goBackward()
{
    float currentDistance = abs(_encoder.getDistanceCm());
    if (currentDistance < _backwardTarget)
    {
        _motors.move(-FORWARD_SPEED);
        _steeringAngle = 0;
    }
    else
    {
        // Serial.println("Finished backing up.");
        _encoder.reset();
        _currentState = FORWARD;
    }
}

void ObstacleAvoider::_goForward()
{
    if (_comm.getManeuverValues(distance, angle))
    {
        _encoder.reset();
        _imu.reset();
        _currentState = AVOIDING;
        Serial.println("Started Avoiding_____________");
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
