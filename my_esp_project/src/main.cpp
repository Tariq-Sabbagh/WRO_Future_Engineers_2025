#include <ESP32Servo.h>

const int SERVO_PIN = 32;
const int MOTOR_SPEED_PIN = 5;  // PWM Speed Control
const int MOTOR_DIR1_PIN = 18; // Direction 1
const int MOTOR_DIR2_PIN = 19; // Direction 2

const int SERVO_CENTER_ANGLE = 92;
const int SERVO_MIN_ANGLE = 45;
const int SERVO_MAX_ANGLE = 130;

const int BUTTON_PIN = 15;
Servo myServo;
constexpr uint8_t PACKET_SIZE = sizeof(int16_t) * 2;  // 4 bytes total
bool systemEnabled = false;

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);

  pinMode(MOTOR_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button connected between pin and GND

}

void loop() {
  if (!systemEnabled && digitalRead(BUTTON_PIN) == LOW) {
    delay(100);  // Debounce delay
    if (digitalRead(BUTTON_PIN) == LOW) {
      systemEnabled = true;
      Serial.println("System enabled!");
    }
  }
   if (systemEnabled &&Serial.available() >= PACKET_SIZE) {
    int16_t values[2];  // [0] = speed, [1] = angle
    Serial.readBytes(reinterpret_cast<char*>(values), PACKET_SIZE);

    int speed = values[0];
    int angle = constrain(values[1], SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);  // Servo range
 Serial.print("Received Speed: ");
    Serial.print(speed);
    Serial.print(" | Angle: ");
    Serial.println(angle);


    
    myServo.write(angle);
    if (speed >= 0) {
      digitalWrite(MOTOR_DIR1_PIN, HIGH); // Forward
      digitalWrite(MOTOR_DIR2_PIN, LOW); // Forward
      analogWrite(MOTOR_SPEED_PIN, constrain(speed, 0, 255));
    } else {
       digitalWrite(MOTOR_DIR1_PIN, LOW); // Forward
      digitalWrite(MOTOR_DIR2_PIN, LOW); // Forward
      analogWrite(MOTOR_SPEED_PIN, constrain(speed, 0, 255));
    }
    // delay(100);
  }
}
