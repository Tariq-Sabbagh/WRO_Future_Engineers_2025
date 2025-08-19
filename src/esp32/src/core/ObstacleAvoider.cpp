#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider() : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN, &_encoder),
                                     _servo(SERVO_PIN),
                                     _imu(),
                                     _pid(),
                                     _encoder(),
                                     _button(BUTTON_PIN),
                                     _timer(),
                                     _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
                                     _backSensor(SHT_LOX, 0x20),
                                     _comm(),
                                     _motorPID(_motors, _encoder)
{

    _motorPID.setup(2.0, 0.5, 0.1, -255, 255);

    _currentState = FORWARD;
    _steeringAngle = 0;
}
float distance, angle, _forwardTarget = 0;
int count_turn = 0;
bool turn_right =false;
bool backward =false;
void ObstacleAvoider::setup()
{
    Wire.begin();
    _motors.setup();
    _servo.setup();
    _button.setup();
    _encoder.begin();
    while (!_imu.setup())
    {
        Serial.println("FATAL: IMU failed to initialize.");
    }

    if (!_backSensor.begin())
    {
        Serial.println("TOF Sensor failed to init!");
        while (1)
            ;
    }
    _button.waitForPress();

    _pid.setup(3.5, 0, 0);
    _pid.setOutputLimits(-90, 90);

    _garageDoOut();
    // _garageDoIn();

    _comm.clearSerialBuffer();
    Serial.println("Obstacle Avoider Initialized. Waiting for commands...");
}
void ObstacleAvoider::_garageDoOut()
{

    if (_ultra.getLeftCm() < _ultra.getRightCm())

    {
        _pid_target_parking = -_pid_target_parking * 0.9;
        turn_right = true;

    }

    while (abs(_imu.getHeading()) <= 85)
    {

        _imu.update();
        float correction = _pid.compute(_pid_target_parking, _imu.getHeading());
        _servo.setAngle(-correction);
        _motors.move(190);
    }
    while (_ultra.getFrontCm() >= 35)
    {
        _imu.update();
        float correction = _pid.compute(_pid_target_parking, _imu.getHeading());
        _servo.setAngle(-correction);
    }
    _encoder.reset();
    while (abs(_imu.getHeading()) >= 0 and _encoder.getDistanceCm() < 50)
    {
        _motors.backward(FORWARD_SPEED);
        _encoder.update();
        _imu.update();
        float correction = _pid.compute(0, _imu.getHeading());
        _servo.setAngle(correction);
    }

    _motors.stop();

    //  _goForwardPID(50);
}
void ObstacleAvoider::_garageDoIn()
{

    _motors.move(180);
    while (true )
    {
        if (_ultra.getLeftCm() <15 || _ultra.getRightCm() <15)
        break;
        Serial.println(_ultra.getLeftCm());

          _imu.update();
        float correction = _pid.compute(0, _imu.getHeading());
        _servo.setAngle(-correction);
    }
    

  if (_ultra.getLeftCm() < _ultra.getRightCm())

    {
        _pid_target_parking_In = -_pid_target_parking_In;
    }

    _encoder.reset();
     while ( abs(_encoder.getDistanceCm()) < 10)
    {
       _encoder.update();
        Serial.println(_encoder.getDistanceCm());

        float correction = _pid.compute(0, _imu.getHeading());
        _servo.setAngle(correction);
    }

    while (abs(_imu.getHeading()) <= 89)
    {

        _imu.update();
        float correction = _pid.compute(_pid_target_parking_In, _imu.getHeading());
        _servo.setAngle(-correction);
        _motors.move(180);
    }

    
    while (_ultra.getFrontCm() <= 50)
    {
        Serial.println(_ultra.getFrontCm());
        
        
        _imu.update();
        float correction = _pid.compute(90, _imu.getHeading());
        _servo.setAngle(correction);
        _motors.move(BACKWARD_SPEED);
    }
    _imu.reset();
// Serial.println(abs(_imu.getHeading()));
    while (abs(_imu.getHeading()) < 150)
    {
        // Serial.println(abs(_imu.getHeading()));

        _imu.update();
         float correction = _pid.compute(0, _imu.getHeading());
        _servo.setAngle(-correction);
        _motors.move(BACKWARD_SPEED);
    }
    _servo.center();

    _motors.stopBreak(1);
    while (true);

    

    //  _goForwardPID(50);
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

    if (count_turn >= 12)
    {
        while(true){
        _garageDoIn();}
        
        
    }
    _get_away_walls();
    // Serial.println(_steeringAngle);
    _servo.setAngle(_steeringAngle);
}
void ObstacleAvoider::_get_away_walls()
{
    float right = _ultra.getRightCm();
    float left = _ultra.getLeftCm();
    const float k = 1.2;
    if (right <= 30 and right != 0)
    {
        _steeringAngle += k * (30 - right);
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
    if(!backward)
        {
            _motors.move(-255);
            _timer.wait(500);
            backward=true;
            Serial.println("_______________________________________");
        }
    _motors.move(BACKWARD_SPEED);
    // Serial.println(_backSensor.readDistance());
    int distanceTOF = _backSensor.getDistance();
    // Serial.print("error:");
    // Serial.println(_pid.geterror());
    if (distanceTOF <= 350 and abs(_pid.geterror()) < 15)
    {
        // Serial.println(distanceTOF);
        Serial.println(_pid.geterror());
        count_turn++;
        _motors.move(FORWARD_SPEED);
        _comm.resetManeuverValues();
        _currentState = FORWARD;
        backward = false;
    }
}
void ObstacleAvoider::_avoidObstacle()
{
    float correction = 0;
    float currentHeading = _imu.getHeading();
    float currentDistance = _encoder.getDistanceCm();
    _motors.move(FORWARD_SPEED);
    // if (distance <= 40.0) // too close to move forward
    // {
    //     Serial.println("Too close — backing up");
    //     _encoder.reset();
    //     _backwardTarget = 10.0; // back up 20 cm
    //     _currentState = BACKWARD;
    //     return;
    // }
    // Serial.println(distance);
    // Serial.println(angle);
    if (abs(currentDistance) <= distance)
    {

        correction = _pid.compute(angle, currentHeading);
        _steeringAngle = -correction;
        // Serial.println(_imu.getHeading());
        _motors.forward(FORWARD_SPEED);
    }
    else
    {
        _comm.resetManeuverValues();
        // _timer.start(200);
        // _currentState = IDLE;
        Serial.println("Done avoiding_________");
        _currentState = FORWARD;
    }
}
void ObstacleAvoider::_turn()
{
    float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
    _steeringAngle = -correction;
    // _motors.move(FORWARD_SPEED - 45);
    // Serial.println(_pid.geterror());
    if (_ultra.getFrontCm() <= 25 and _pid.geterror() < abs(20))
    {
        if(turn_right)
        {
            _forwardTarget +=90;
        }
        else 
        {
            _forwardTarget -=90;
        }
        _comm.resetTurn();
        _currentState = RESET;
    }
}
void ObstacleAvoider::_goBackward()
{
    float currentDistance = abs(_encoder.getDistanceCm());
    if (currentDistance < _backwardTarget)
    {
        _motors.move(BACKWARD_SPEED);
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
    if (_comm.getManeuverValues(distance, angle) and abs(_pid.geterror()) < 30)
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

void ObstacleAvoider::_goForwardPID(float targetDistanceCm)
{
    if (_motorPID.getTargetDistance() != targetDistanceCm)
    {
        _motorPID.reset();
        _motorPID.setTargetDistance(targetDistanceCm); // تحديد الهدف الجديد
        Serial.print("Moving forward ");
        Serial.print(targetDistanceCm);
        Serial.println(" cm using PID");
    }

    // تحديث حركة الموتور بناءً على PID
    if (_motorPID.update())
    {
        // وصلنا للمسافة المطلوبة
        Serial.println("Target distance reached!");
        _motorPID.reset(); // إعادة الضبط إذا أردنا هدف جديد لاحقًا
        // هنا ممكن تنتقل لحالة أخرى إذا تستخدم state machine
        //_currentState = TURN; // مثال: الانتقال للحالة TURN
    }

    // التتبع للطباعة/debug
    Serial.print("Current Distance: ");
    Serial.print(_motorPID.getCurrentDistance());
    Serial.print(" cm, PID Output: ");
    Serial.println(_motorPID.getLastOutput());

    // توجيه العجلات حسب الـ steering angle المعتاد
    float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
    _steeringAngle = -correction;
}

void ObstacleAvoider::_stopAndHalt()
{
    _motors.stop();
    _servo.center();
    Serial.println("Execution Halted.");
    while (true)
        ;
}
