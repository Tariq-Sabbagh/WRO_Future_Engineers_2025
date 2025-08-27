#include "OpenChallenge.h"

OpenChallenge::OpenChallenge()
    : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN, &_encoder),
      _steering(SERVO_PIN),
      _imu(),
      _headingPID(),
      _encoder(),
      _comm(),
      _button(BUTTON_PIN),
      _timer(),
      _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
      _backTOF(SHT_LOX, 0x20),
      _motorPID(_motors, _encoder),
      _state(INIT),
      _pixels(NEOPIXEL_PIN, 2)

{
}

void OpenChallenge::setup()
{
    Serial.println("_______________________");
    _pixels.setup();
    _pixels.setRed();
    Wire.begin();
    Serial.begin(115200);

    _motors.setup();
    _steering.setup();
    _button.setup();
    // _encoder.begin();

    _motorPID.setup(2.0, 0.5, 0.1, -255, 255);

    while (!_imu.setup())
    {
        Serial.println("FATAL: IMU failed to initialize.");
    }

    while (!_backTOF.begin())
    {
        Serial.println("FATAL: Back TOF failed to init.");
    }
    _button.waitForPressOrRestart(_pixels);

    _headingPID.setup(STEERING_KP, 0.0f, 0.0f);
    _headingPID.setOutputLimits(-90, 90);

    Serial.println("Ready. Press button to start...");

    _imu.update();
    _headingTarget = _imu.getHeading();
    _lastTurnMs = millis();

    _enter(FORWARD);

    _comm.clearSerialBuffer();
    Serial.println("Open Challenge: Initialized.");
}

void OpenChallenge::loop()
{
    _imu.update();
    _comm.update();
    // _handleSerialCommands();

    _steeringBase = 0.0f;
    _steeringCorr = 0.0f;
    _steeringExtra = 0.0f;

    switch (_state)
    {
    case INIT:
        _enter(FORWARD);
        break;

    case FORWARD:
        _forwardStep();
        if (_shouldConsiderTurn())
            if (_turnCount == 0)
                _enter(CHECK_TURN);
            else if (_turnCount > 0)
            {
                if (turn_right)
                {
                    _beginTurn(+90.0f);
                }
                else
                {
                    _beginTurn(-90.0f);
                }
            }
        break;

    case CHECK_TURN:
        if (_canTurnLeft())
        {
            _beginTurn(-90.0f);
            turn_right = false;
        }
        else if (_canTurnRight())
        {
            _beginTurn(+90.0f);
            turn_right = true;
        }
        else
            _enter(FORWARD);
        break;

    case TURNING:
        if (_turnStep())
            _enter(FORWARD);
        break;

    case IDLE:
        _motors.stop();
        break;
    }
    _getAwayWalls();

    _applySteering();

    if (_turnCount >= 12)
    {
        _halt();
    }
}

void OpenChallenge::_enter(State s)
{
    _state = s;
    switch (s)
    {
    case FORWARD:
        _motorPID.reset();
        _headingPID.reset();
        break;
    case TURNING:
        _headingPID.reset();
        break;
    case BACKWARD:
        _motorPID.reset();
        break;
    default:
        break;
    }
}

/* ======= FORWARD ======= */
void OpenChallenge::_forwardStep()
{

    _computeHeadingPID();
    _motors.forward(FORWARD_SPEED);
}

void OpenChallenge::_beginTurn(float deltaDeg)
{
    _turnTargetDeltaDeg = deltaDeg;
    _headingTarget += _turnTargetDeltaDeg;

    _turnCount++;
    _lastTurnMs = millis();
    _enter(TURNING);
}

bool OpenChallenge::_turnStep()
{

    _computeHeadingPID();

    _motors.forward(FORWARD_SPEED);

    float err = _headingTarget - _imu.getHeadingRotating();

    if (fabs(err) < 5.0f && (millis() - _lastTurnMs) > 250)
    {
        return true;
    }
    return false;
}

void OpenChallenge::_halt()
{
    if (millis() - _lastTurnMs >= 1400)
    {
        _motors.stop();
        _steering.center();
        while (true)
        {
            delay(100);
        }
    }
}

void OpenChallenge::_computeHeadingPID()
{
    float corr = _headingPID.compute(_headingTarget, _imu.getHeadingRotating());
    _steeringBase = -corr;
}

void OpenChallenge::_getAwayWalls()
{

    float right = _ultra.getRightCm();
    float left = _ultra.getLeftCm();
    const float k = 2.0f;
    if (right <= _rightWall && right != 0)
    {
        _steeringCorr += k * (_rightWall - right);
    }
    if (left <= _leftWall && left != 0)
    {
        _steeringCorr -= k * (_leftWall - left);
    }
}

void OpenChallenge::_applySteering()
{
    float angle = _steeringBase + _steeringCorr + _steeringExtra;
    angle = _clamp(angle, -90.0f, 90.0f);
    _steering.setAngle(angle);
}

bool OpenChallenge::_shouldConsiderTurn()
{
    if ((millis() - _lastTurnMs) < TURN_COOLDOWN_MS)
        return false;
    float front = _ultra.getFrontCm();
    // Serial.print("Front sens:");
    Serial.println(front);
    return (front > 1 && front <= TURN_TRIGGER_DISTANCE_CM);
}

bool OpenChallenge::_canTurnLeft()
{
    float leftDist = _ultra.getLeftCm();
    Serial.print("left sens:");
    Serial.println(leftDist);
    return (leftDist >= TURN_CLEARANCE_DISTANCE_CM || leftDist == 0);
}

bool OpenChallenge::_canTurnRight()
{
    float rightDist = _ultra.getRightCm();
    Serial.print("right sens:");
    Serial.println(rightDist);
    return (rightDist >= TURN_CLEARANCE_DISTANCE_CM || rightDist == 0);
}

float OpenChallenge::_clamp(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}