#include "OutParking.h"

OutParking::OutParking()
    : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
      _button(BUTTON_PIN),
      _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
      _backSensor(SHT_LOX, 0x20),
      _imu(),
      _pid(),
      _encoder(),
      _servo(SERVO_PIN)
{
}

void OutParking::begin()
{
    // Wire.begin();
    // _motors.setup();
    // _servo.setup();
    // _button.setup();
    // _encoder.begin();

    // while (!Serial)
    //     ; // انتظار فتح المونيتور التسلسلي

    // Serial.println("ESP32 Ready");

    // if (!_imu.setup())
    // {
    //     Serial.println("FATAL: IMU failed to initialize.");
    //     // يمكن إضافة معالجة خطأ هنا إن أردت
    // }

    // Serial.println("Starting TOF Sensor...");

    // if (!_backSensor.begin())
    // {
    //     Serial.println("Sensor failed to init!");
    //     while (1)
    //         ; // التوقف هنا بسبب فشل الاستشعار
    // }

    // _button.waitForPress();
}

void OutParking::Do()

{
    _servo.setup();
    _pid.setup(3.5, 0, 0);
    _pid.setOutputLimits(-90, 90);
    _encoder.update();
 
    if (_ultra.getLeftCm() < _ultra.getRightCm())

    {
        // _servo_angle_parking =-_servo_angle_parking;
        _pid_target_parking =- _pid_target_parking;
    }

    // _servo.setAngle(_servo_angle_parking);
    while (abs(_imu.getHeading()) <= 90)
    {
        // Serial.println(abs(_imu.getHeading()));

        _imu.update();
        float correction = _pid.compute(_pid_target_parking, _imu.getHeading());
        // _steeringAngle = -correction;
        _servo.setAngle(-correction);
        // _servo.setAngle(90);

        _motors.move(FORWARD_SPEED);
    }
    while(_ultra.getFrontCm() >=37)
    {
        _imu.update();
        float correction = _pid.compute(_pid_target_parking, _imu.getHeading());
        // _steeringAngle = -correction;
        _servo.setAngle(-correction);
    }
    while(abs(_imu.getHeading()) >= 0 and _encoder.getDistanceCm() < 50)
    {
        _motors.backward(FORWARD_SPEED+40);
        _encoder.update();
        _imu.update();
        float correction = _pid.compute(0, _imu.getHeading());
        // _steeringAngle = -correction;
        _servo.setAngle(correction);
        // _motors.move(-255);
         
    }
    
    // _servo.setAngle(_min_servo_angle);

    _motors.stop();

    // _servo.center();
    // if(_ultra.getFrontCm()<30){

    // }
}
