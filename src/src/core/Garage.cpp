// #include "Garage.h"

// Garage::Garage()
//     : _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),

//       _button(BUTTON_PIN),
//       _ultra(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
//       _backSensor(SHT_LOX, 0x20),
//       _imu(),
//       _pid(),

//       _encoder(),
//       _servo(SERVO_PIN)
// {
// }

// void Garage::begin()
// {
//     // Wire.begin();
//     // _motors.setup();
//     // _servo.setup();
//     // _button.setup();
//     // _encoder.begin();

//     // while (!Serial)
//     //     ; // انتظار فتح المونيتور التسلسلي

//     // Serial.println("ESP32 Ready");

//     // if (!_imu.setup())
//     // {
//     //     Serial.println("FATAL: IMU failed to initialize.");
//     //     // يمكن إضافة معالجة خطأ هنا إن أردت
//     // }

//     // Serial.println("Starting TOF Sensor...");

//     // if (!_backSensor.begin())
//     // {
//     //     Serial.println("Sensor failed to init!");
//     //     while (1)
//     //         ; // التوقف هنا بسبب فشل الاستشعار
//     // }

//     // _button.waitForPress();
// }

// void Garage::Do()
// {

//     int number_of_turns = 0;

//     delay(500);
//     Serial.println(_backSensor.getDistance());
//     Serial.println(_encoder.getDistanceCm());
//     while (_backSensor.getDistance() > _min_distance)
//     {
//         _motors.backward(FORWARD_SPEED);
//     }
//     _motors.stop(1);
//     // _servo.setAngle(MAX_SERVO_ANGLE);

//     while (number_of_turns < 2)
//     {

//         delay(500);
//         _encoder.reset();
//         _encoder.update();

//         while (_backSensor.getDistance() < _max_distance || abs(_encoder.getDistanceCm()) < 3)
//         {
//             _servo.setAngle(MAX_SERVO_ANGLE);

//             _motors.forward(FORWARD_SPEED);
//             _encoder.update();
//         }
//         _motors.stop(-1);
//         // _button.waitForPress();
//         delay(500);
//         _encoder.reset();
//         _encoder.update();

//         _motors.move(-255);
//         while (abs(_encoder.getDistanceCm()) < 13)
//         {
//             _servo.setAngle(MIN_SERVO_ANGLES);
//             Serial.println(
//                 abs(_encoder.getDistanceCm()));
//             _encoder.update();
//         }
//         _motors.stop(1);
//         number_of_turns++;
//     }

// }
