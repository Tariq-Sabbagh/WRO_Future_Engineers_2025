#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>
#include <NewPing.h>

#define UltraPinFront 25
#define UltraPinFront 26
#define UltraPinLeft 27

#define MAX_DISTANCE 200  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SpeedPin 5
#define dir1 19
#define dir2 18
#define servoPin 32
#define buttonPin 23



Servo myservo;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t event;
NewPing sonar_left_front(UltraPinLeft, UltraPinLeft, MAX_DISTANCE);
NewPing sonar_front(UltraPinFront, UltraPinFront, MAX_DISTANCE);
NewPing sonar_Right_front(UltraPinFront, UltraPinFront, MAX_DISTANCE);


float distance = 0;
float error = 0;
float offsetGyro = 0;
float TargetOffset = 0;
float servo_angle = 0;
float angle = 0;
float kp = 2.9;
float previousAngle = 0;
int offsetGyroNow;
int turncnt = 0;
int centerServo = 87;
unsigned long previousMillis = -1750;


const long interval = 1000;




void go_Right() {
	if (sonar_front.ping_cm() < 30 && sonar_front.ping_cm() > 1) {
		offsetGyro += 90;
		moveStraight();
		turncnt++;
	}
}

void turn_Right() {
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= 2000 && turn <= 11) {
		if (error < abs(15)) {
		if (sonar_front.ping_cm() <= 60 && sonar_front.ping_cm() > 1) {
			if (sonar_Right_front.ping_cm() >= 120 || sonar_Right_front.ping_cm() == 0) {
			previousMillis = currentMillis;
			Serial.println(error);
			turncnt++;
			offsetGyro += 90;
			}
		}
		}
	}
}
void turn_left() {
	unsigned long currentMillis = millis();
	if (currentMillis - previousMillis >= 2000 && turn <= 11) {
		if (error < abs(15)) {
		if (sonar_front.ping_cm() <= 60 && sonar_front.ping_cm() > 1) {
			if (sonar_left_front.ping_cm() >= 120 || sonar_left_front.ping_cm() == 0) {
			previousMillis = currentMillis;
			turncnt++;
			offsetGyro -= 90;
			}
		}
		}
	}
}
void turn() {
  turn_left();
  turn_Right();
}

void moveStraight() {

	if (offsetGyro == 360) {
		offsetGyro = 0;
	}
	offsetGyroNow = event.orientation.x - offsetGyro;
	if (offsetGyroNow > 180) {
		offsetGyroNow -= 360;
	}
	error = offsetGyroNow - TargetOffset;
	angle = error * kp;
	servo_angle = map(angle, -90, 90, centerServo - 45, centerServo + 45);
	myservo.write(servo_angle);
	motor_Foward();
	turn();
}

void motor_Foward() {
	digitalWrite(dir1, 1);
	digitalWrite(dir2, 0);
	analogWrite(SpeedPin, 150);
}

void motor_Back() {
	digitalWrite(dir1, 0);
	digitalWrite(dir2, 1);
	analogWrite(SpeedPin, 150);
	kp = 0.7;
	kd = 0;
	ki = 
	pidcon pid = new pidCon(kp,kd,ki);
	fload offset = pid.getCorrection(gyro-target);

},

void Stop() {
	digitalWrite(dir1, 0);
	digitalWrite(dir2, 0);
	analogWrite(SpeedPin, 0);
	while (true)
	;
}

void setup() {
	Serial.begin(9600);
	pinMode(dir1, OUTPUT);
	pinMode(dir2, OUTPUT);
	pinMode(SpeedPin, OUTPUT);
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(25);  // standard 50 hz servo
	myservo.attach(, 0, 2000);
	myservo.write(centerServo);


	if (!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while (1)
		;
	}


	offsetGyro = event.orientation.x;

	bno.setExtCrystalUse(true);

	while (!analogRead(buttonPin))
	;
}

void loop() {
	bno.getEvent(&event);
	moveStraight();

	if (turn >= 12) {
		if (millis() - previousMillis >= 1000) {
		Stop();
		}
	}
	// Serial.println(event.orientation.x);


	// Serial.println();

	// Serial.println(error);
	// Serial.println();

	// Serial.println(offsetGyroNow);



	// if(turn >= 12)
	// {
	//   delay(300);
	//   digitalWrite(dir1, 0);
	//   digitalWrite(dir2, 0);
	//   analogWrite(SpeedPin, 0);
	//   while(1);

	// }
}
