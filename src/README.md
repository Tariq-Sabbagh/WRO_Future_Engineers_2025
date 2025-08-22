# Code Logic and Strategy

## Open Challenge

### Core Logic: The `Car` Class

To keep our code organized and easy to manage, we built our entire driving logic inside a single C++ class called `Car`. This class acts as the "brain" of our robot. It brings together all the different hardware components—motors, steering, sensors—and uses a clear algorithm to navigate the challenge track.

Here is the basic structure of our `Car` class:
```cpp
#include "PIDController.h"
#include "MotorController.h"
#include "Steering.h"
#include "DistanceSensors.h"
#include "IMU.h"
#include "Button.h"

class Car {
private:
    // All our hardware objects
    MotorController _motors;
    Steering _steering;
    DistanceSensors _distSensors;
    IMU _imu;
    Button _button;
    PIDController _pid;

    // Variables to track our state on the track
    float _offsetGyro;
    int _turnCounter;
    // ... and other state variables

    // The core actions the car can take
    void _checkForTurns();
    void _moveStraight();
    void _turnRight();
    void _turnLeft();
    void _stopAndHalt();

public:
    // Public functions to run the car
    Car();
    void setup();
    void loop();
};
```
Constructor: Car()
This is where the Car object is first created. Its main job is to initialize all the hardware component objects (like _motors, _steering, etc.) with their correct pin numbers. It also resets all our navigation variables to their starting values, ensuring a clean start for every run.
```cpp
Car::Car() :
    _motors(MOTOR_DIR1_PIN, MOTOR_DIR2_PIN, MOTOR_SPEED_PIN),
    _steering(SERVO_PIN),
    _distSensors(ULTRASONIC_PIN_FRONT, ULTRASONIC_PIN_LEFT, ULTRASONIC_PIN_RIGHT),
    _imu(),
    _button(BUTTON_PIN),
    _pid()
{
    // Reset state variables
    _offsetGyro = 0.0f;
    _turnCounter = 0;
    direction = "straight";
    // ... etc.
}
```
Setup Phase: setup()
The setup() function gets all the hardware ready to go. It configures our PID controller for smooth steering and initializes the IMU sensor. A key feature here is that the code waits for a button press. This allows us to place the robot on the track before it starts moving. Once the button is pressed, it calibrates the gyro by taking an initial reading to know its starting direction.
```cpp
void Car::setup() {
    // Setup all hardware components
    _motors.setup();
    _steering.setup();
    _button.setup();
    
    // Configure PID controller
    _pid.setup(STEERING_KP, 0, 0);
    _pid.setOutputLimits(-90,90);
    
    // Initialize IMU, and stop if it fails
    if (!_imu.setup()) {
        _stopAndHalt();
    }

    // Wait for the start signal
    _button.waitForPress();
    
    // Calibrate gyro to the starting direction
    _imu.update();
    _offsetGyro = _imu.getHeading();
}
```
Main Loop: loop()
This is the heart of our program that runs over and over. The logic is very simple and clear:

Update Sensors: Always get the latest data from the IMU first.

Check for Turns: See if the robot has reached an intersection where it needs to turn.

Drive Straight: Use the PID controller to maintain a perfectly straight line.

Check Finish Condition: After 12 turns, the robot knows the course is complete and stops.

```cpp
void Car::loop() {
    _imu.update(); // 1. Update Sensors
    
    _checkForTurns();   // 2. Check for Turns
    _moveStraight();    // 3. Drive Straight

    // 4. Check if we're done
    if (_turnCounter >= 12) {
        // ... stop the car
        _stopAndHalt();
    }
}
```

Core Functions Explained
_moveStraight()
This function is responsible for keeping the car on course. It constantly compares the car current heading (from the IMU gyro) to its target direction. The difference between these is the "error." This error is fed into a PID controller, which calculates the precise steering correction needed to get back on track. This allows the robot to drive very straight.

```cpp
void Car::_moveStraight() {
    // Calculate heading error relative to our starting direction
    float currentHeading = _imu.getHeading() - _offsetGyro;

    // Normalize the heading to a range of -180 to 180
    if (currentHeading > 180) currentHeading -= 360;
    else if (currentHeading < -180) currentHeading += 360;

    // Use the PID controller to calculate the steering correction
    float steeringCorrection = _pid.compute(0, currentHeading);

    // Apply the correction to the servo and move forward
    _steering.setAngle(-steeringCorrection);
    _motors.forward(FORWARD_SPEED);

}
```

_checkForTurns()
This function decides when and where to turn. Using data from our ultrasonic sensors, it detects when it has reached an open intersection. The logic follows the required pattern for the track. It also includes a "cooldown" to prevent it from making multiple turns right after each other.
```cpp
void Car::_checkForTurns() {
    // Wait for cooldown period to pass
    if (millis() - _previousTurnMillis < TURN_COOLDOWN_MS) {
        return;
    }

    // Only consider turning if we are driving mostly straight
    if (abs(error) < 15) {
        // Logic for deciding whether to turn left or right based on the lap number
        if (_turnCounter < 1) {
            _turnLeft();
        } 
        else if (direction == "right") {
            _turnRight();
        } 
        else if (direction == "left") {
            _turnLeft();
        }
    }
}
```


## Obstacle Challenge

### Obstacle Challenge: Vision Processing (Python):

For the obstacle challenge, our strategy relies on a Python script using the OpenCV library for all vision processing. The script analyzes the camera feed to detect obstacles and turn indicators, then sends commands to the ESP32 microcontroller. The logic is built around a state machine to manage different situations like driving forward, avoiding an obstacle, or turning.

These are the main states our robot can be in:
```cpp
enum State {
    FORWARD,    // Driving straight ahead
    AVOIDING,   // Actively maneuvering around an obstacle
    IDLE,       // Waiting for a command
    BACKWARD,   // Reversing (part of a maneuver)
    TURN,       // Making a 90-degree turn
    RESET,      // Resetting position after a maneuver
    GARAGE      // Parking maneuver
};
```

#### Color Detection using LAB Color Space
To reliably detect colored objects under different lighting conditions, we convert the camera image from the standard RGB format to the LAB color space. The 'L' channel represents lightness, while 'A' and 'B' represent color spectrums. This separation makes it much easier to identify a specific color regardless of shadows or bright lights.

We create a "mask" for each color we're looking for (red, green, blue, orange). This mask is a black and white image where white pixels represent the color we want to find.
```python
def detect_color(self, lab_frame, color_name):
    """Detect specific color in LAB space"""
    profile = self.COLOR_PROFILES[color_name]
    # Create a mask using the lower and upper LAB values for the color
    mask = cv2.inRange(lab_frame, profile['lower'], profile['upper'])
    # Clean up the mask to remove small noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask
```
#### Main Vision Loop: process_frame()
This is the main function that runs for every frame from the camera. Its job is to find obstacles and turns.

Region of Interest (ROI): To improve performance and avoid detecting irrelevant objects in the background, we don't process the entire camera image. Instead, we define a specific rectangular area (an ROI) in the lower part of the frame where we expect to see the track and obstacles. All processing happens only inside this box.

Find Contours: Within the ROI, the code looks for continuous shapes (contours) of our target colors.

Process Obstacles/Turns: Once a colored shape is found, it's passed to another function to decide what to do.
```python
def process_frame(self , mode='rgb'):
    """Process a single frame"""
    frame_rgb = self.capture_frame()
    
    # 1. Define the Region of Interest (ROI)
    obstacle_roi_rgb = self.crop_frame(frame_rgb, 0.1, 0.07, 0.8, 0.7)

    # 2. Find all colored shapes (contours) in the ROI
    contours = self.detect_obstacles(obstacle_roi_rgb)
    dominant_color = self.find_dominant_obstacle(contours)
    
    # 3. Process the most important obstacle found
    if dominant_color:
        self.process_obstacle(
            contours[dominant_color],
            obstacle_roi_rgb,
            dominant_color
        )

    # Also, constantly check for turn indicators on the floor
    self.handle_turn_detection(frame_rgb)

    return frame_rgb
```

#### Handling Obstacles vs. Turns
The system handles obstacles (red/green blocks) differently from turn indicators (blue/orange floor markings).

Obstacle Avoidance (process_obstacle)
When a red or green obstacle is detected, this function calculates a maneuver.

It estimates the distance to the obstacle based on its apparent height in the image.

It calculates the angle needed to steer around it based on its horizontal position in the frame.

Finally, it sends an AVOID command to the ESP32 with the distance to travel and the angle to turn.
```python
def process_obstacle(self, contour, frame_rgb, color_type):
    # ...
    x, y, w, h = cv2.boundingRect(contour)
    # 1. Calculate distance from the obstacle's height
    distance = self.calculate_distance(h)
    # 2. Calculate the angle and path to avoid it
    travel_dist, turn_angle = self.calculate_maneuver(
        frame_rgb.shape, (x, y, w, h), distance, profile['offset_adjust']
    )
    # 3. Send the command to the robot
    if (self.mode is not OperationMode.CAMERA_ONLY) and travel_dist <= 70:
        self.send_command('AVOID', travel_dist, turn_angle)
```

#### Turn Detection (detect_turn)
For turns, we don't need to find a specific shape. We just need to know if there's a patch of blue or orange on the floor.

This function looks at a specific ROI on the floor.

It calculates the percentage of blue or orange pixels within that ROI.

If the percentage crosses a certain threshold (e.g., >10%), it confirms a turn and sends a TURN command to the ESP32 with the correct angle (-90 for left, 90 for right).
```python
def detect_turn(self, frame_rgb):
    # ...
    roi = self.crop_frame(frame_rgb, 0.375, 0.7, 0.25, 0.2)
    # ...
    for color in turn_map:
        mask = self.detect_color(frame_lab, color)
        # 2. Calculate the ratio of colored pixels
        pixel_ratio = np.sum(mask > 0) / mask.size
    
        # 3. If enough color is present, detect the turn
        if pixel_ratio > 0.1:
            turn_detected = turn_map[color]
            return turn_detected
    return None
```

### Obstacle Challenge: Microcontroller Logic (C++)
The ESP32 microcontroller receives the high-level commands from the Python script (like `AVOID` or `TURN`) and translates them into precise physical actions using its `ObstacleAvoider` class. It manages the motors and sensors to execute these maneuvers accurately.

#### Setup Phase: `setup()`
This function initializes all the hardware connected to the ESP32. It starts the motors, servo, sensors (IMU, TOF), and the PID controller. It also waits for a button press to begin the challenge, ensuring the robot is correctly positioned first.
```cpp
void ObstacleAvoider::setup()
{
    // Initialize communication and all hardware components
    Wire.begin();
    _motors.setup();
    _servo.setup();
    _button.setup();
    _encoder.begin();
    // Loop until the IMU is successfully initialized
    while (!_imu.setup())
    {
        Serial.println("FATAL: IMU failed to initialize.");
    }
    // Halt if the rear Time-of-Flight sensor fails
    if (!_backSensor.begin())
    {
        Serial.println("TOF Sensor failed to init!");
        while (1);
    }
    // Wait for the start button to be pressed
    _button.waitForPress();

    // Configure the PID controller for steering
    _pid.setup(3.5, 0, 0);
    _pid.setOutputLimits(-90, 90);

    // Initial maneuver to exit the garage
    _garageDoOut();
   
    // Clear any leftover data in the serial buffer
    _comm.clearSerialBuffer();
    Serial.println("Obstacle Avoider Initialized. Waiting for commands...");
}
```
#### Main Loop and State Machine: `loop()`
The main loop constantly updates sensor readings and checks for new commands from the Python script. It uses a `switch` statement to execute the code corresponding to the robot's current state (`FORWARD`, `AVOIDING`, `TURN`, etc.). This state-based approach makes the logic clean and predictable.
```cpp
void ObstacleAvoider::loop()
{
    // Always update sensor data and check for new commands
    _encoder.update();
    _imu.update();
    _comm.update();
    _backSensor.update();

    // If a turn command is received, switch to the TURN state
    if (_comm.getTurn() != 0.f and _currentState != GARAGE)
    {
        _currentState = TURN;
    }

    // Execute the function for the current state
    switch (_currentState)
    {
    case AVOIDING:
        _avoidObstacle();
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
    case GARAGE:
        _garageDoIn();
        break;
    // ... other states
    }
    // A function to prevent crashing into walls
    _get_away_walls();
    // Apply the final calculated steering angle
    _servo.setAngle(_steeringAngle);
}
```

#### State Explanations
`FORWARD` State:
This is the default state. The robot drives forward while using the PID controller to keep a straight line based on the IMU heading. It continuously listens for an `AVOID` command from the Python script. If a command is received, it switches to the `AVOIDING` state.
```cpp
void ObstacleAvoider::_goForward()
{
    // Check if a new maneuver command has arrived from the vision script
    if (_comm.getManeuverValues(distance, angle) and abs(_pid.geterror()) < 15)
    {
        // If so, reset sensors and switch to AVOIDING state
        _encoder.reset();
        _imu.reset();
        _currentState = AVOIDING;
    }
    else
    {
        // Otherwise, keep driving straight using PID control
        float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
        _steeringAngle = -correction;
        _motors.forward(FORWARD_SPEED);
    }
}
```
`AVOIDING` State:
In this state, the robot executes the avoidance maneuver calculated by the Python script. It steers to the specified `angle` and drives forward for the specified distance (the hypotenuse of the triangle). Once the `distance` is covered, it switches back to the   `FORWARD` state.
```cpp
void ObstacleAvoider::_avoidObstacle()
{
    float currentDistance = _encoder.getDistanceCm();

    // While we haven't traveled the required distance
    if (abs(currentDistance) <= distance)
    {
        // Use the PID controller to steer to the target angle
        float correction = _pid.compute(angle, _imu.getHeading());
        _steeringAngle = -correction;
        _motors.forward(FORWARD_SPEED);
    }
    else
    {
        // Once the maneuver is done, reset values and return to FORWARD state
        _comm.resetManeuverValues();
        _currentState = FORWARD;
    }
}
```
`TURN` State:
When a turn indicator is detected, the robot enters this state. It drives forward until it is close to the wall, then updates its target heading by +/- 90 degrees. After setting the new direction, it switches to the `RESET` state to physically align itself.
```cpp
void ObstacleAvoider::_turn()
{
    // Drive forward until the front wall is close
    if (_ultra.getFrontCm() <= 25 and _pid.geterror() < abs(20))
    {
        // Update the target heading for the new direction
        if(turn_right)
        {
            _forwardTarget += 90;
        }
        else 
        {
            _forwardTarget -= 90;
        }
        _comm.resetTurn();
        // After 11 turns, it's time to park
        if(count_turn >= 11)
        {
            _currentState = GARAGE;
        }
        else {
            // Otherwise, go to the RESET state to realign
            _currentState = RESET;
        }
    }
}
```
`RESET` State:
After a turn, the robot needs to be perfectly positioned for the next straight section. This function reverses the car until the rear Time-of-Flight sensor detects the back wall at a specific distance. This ensures the car starts each section from the same known position, making obstacle calculations more accurate. It then switches back to `FORWARD`.
```cpp
void ObstacleAvoider::_resetCar()
{
    // Use PID to maintain the new target heading while reversing
    float correction = _pid.compute(_forwardTarget, _imu.getHeadingRotating());
    _steeringAngle = correction;
    
    // Drive backwards
    _motors.move(BACKWARD_SPEED);
    
    int distanceTOF = _backSensor.getDistance();
    // Stop reversing when the back wall is at the target distance and the car is straight
    if (distanceTOF <= 350 and abs(_pid.geterror()) < 15)
    {
        count_turn++;
        _motors.move(FORWARD_SPEED); // Give a small forward push
        _currentState = FORWARD; // Return to the default state
    }
}
```
`GARAGE` State:
This is the final state, triggered after 12 turns are completed. It contains the specific sequence of movements required to perform the parallel parking maneuver into the garage area to finish the run.
```cpp
void ObstacleAvoider::_garageDoIn()
{
    // This function contains a hardcoded sequence of turns and movements
    // to navigate the car into the final parking spot.
    // It uses the IMU for turns and sensors for positioning against walls.
    int motorSpeedGarage = 190;
 
    // ... complex sequence of turns, forward, and backward movements ...
    
    // Once parked, stop all motors and end the program.
    _motors.stop();
    while (true);
}
```

## Helper Classes and Abstractions
To keep our main code clean and readable, we created several "helper" classes. Each class is responsible for managing one specific piece of hardware or one specific task. This approach, known as abstraction, makes the code easier to debug and reuse.

`Button` Class
A simple class to manage the physical start button. It handles basic operations like checking if the button is pressed and waiting for a press to start the robot's routine.
```cpp
#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class Button {
public:
    Button(int pin); // Constructor: sets the button's pin number
    void setup(); // Initializes the pin as an input
    bool isActive();   // Returns true if the button is currently pressed
    bool isInactive(); // Returns true if the button is not pressed
    void waitForPress(const char *message = ""); // Pauses the program until the button is pressed

private:
    int _pin; // Stores the GPIO pin number for the button
};

#endif // BUTTON_H
```
`DistanceSensors` Class
This class manages the three ultrasonic sensors (front, left, and right). It uses the NewPing library for measurements and includes a Kalman filter to smooth out the readings and provide more stable distance data.

```cpp
#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <Arduino.h>
#include <NewPing.h>

#define ULTRASONIC_MAX_DISTANCE_CM 400
#define MEDIAN_SAMPLES 5

class DistanceSensors {
public:
    // Constructor: sets the pins for the three ultrasonic sensors
    DistanceSensors(int front_pin, int left_pin, int right_pin);

    float getFrontCm(); // Returns the filtered distance from the front sensor in cm
    float getLeftCm();  // Returns the filtered distance from the left sensor in cm
    float getRightCm(); // Returns the filtered distance from the right sensor in cm

private:
    // NewPing objects for each sensor
    NewPing _sonar_front;
    NewPing _sonar_left;
    NewPing _sonar_right;

    // Variables for the Kalman filter to reduce sensor noise
    float _kalman_front, _kalman_left, _kalman_right;
    float _error_front, _error_left, _error_right;
    const float _kalmanQ, _kalmanR;

    float median(NewPing& sensor, int samples); // Takes a median of readings to reject outliers
    float kalman(float measurement, float& estimate, float& error); // Applies a Kalman filter for smoother data
};

#endif
```

`Encoder` Class
This class interfaces with the AS5600 magnetic encoder. Its main purpose is to track the rotation of the wheels and calculate the total distance the robot has traveled in centimeters.
```cpp
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include "config.h"

class Encoder {
public:
    Encoder(); // Constructor
    bool begin(); // Initializes the AS5600 sensor
    void update(); // Should be called repeatedly to read the sensor and calculate distance
    void reset(); // Resets the total traveled distance to zero
    float getDistanceCm() const; // Returns the total distance traveled since the last reset

private:
    AS5600 _as5600; // The sensor library object
    int _lastAngle; // Stores the previous angle reading to calculate rotation
    float _totalAngle; // Accumulates the total angle of rotation
    float _distanceCm; // Stores the calculated distance in centimeters
};

#endif
```

`IMU` Class
This class manages the BNO055 Inertial Measurement Unit. It handles the initialization and calibration of the sensor and provides easy-to-use functions to get the robot's heading (direction). It can provide a standard heading (-180 to 180) or a continuous heading that keeps track of full rotations (e.g., 360, 450, etc.).
```cpp
#pragma once

#include <Adafruit_BNO055.h>
#include <Wire.h>

class IMU {
public:
    IMU(); // Constructor
    bool setup(); // Initializes and calibrates the BNO055 sensor
    void update(); // Reads the latest data from the sensor
    float getHeading(); // Returns the heading normalized to a -180 to 180 degree range
    float getHeadingRotating(); // Returns a continuous heading that increases/decreases past 360
    void reset(); // Sets the current heading as the new "zero" reference point

private:
    Adafruit_BNO055 _bno; // The sensor library object
    float _heading; // Stores the current calculated heading
    float _offset; // Stores the offset for the zero reference
    float _prevRawHeading; // Tracks the last raw reading to detect full rotations
    int _rotationCount; // Counts the number of full 360-degree rotations
};
```

`MotorController` Class
This class provides control over the main DC drive motor. It handles basic movements like forward, backward, and stop. It also includes a built-in PID controller to maintain a constant target speed, using feedback from the Encoder class.

```cpp
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "Encoder.h" 

class MotorController {
public:
    // Constructor: sets the motor driver pins and links the encoder
    MotorController(int dir1_pin, int dir2_pin, int speed_pin , Encoder* encoder);
    void setup(); // Initializes the motor pins
    void forward(int speed); // Drives the motor forward at a given speed
    void backward(int speed); // Drives the motor backward at a given speed
    void move(int speed); // General move function (positive for forward, negative for backward)
    void stop(); // Lets the motor coast to a stop
    void stopBreak(int direction); // Actively brakes the motor

    void setTargetSpeed(float target); // Sets the desired speed for the PID controller
    void updatePID(); // Updates the motor's PWM based on the PID calculation

private:
    int _dir1_pin, _dir2_pin, _speed_pin; // Motor driver pins
    Encoder* _encoder; // Pointer to the encoder object for speed feedback

    // PID control variables
    float _targetSpeed;
    float _kp, _ki, _kd;
    float _prevError, _integral;
    int _currentPWM;
    unsigned long _lastPIDTime;
    float _prevDistance;
};

#endif // MOTOR_CONTROLLER_H
```

`PIDController` Class
This is a general-purpose PID (Proportional-Integral-Derivative) controller. We use it primarily for steering control, where it takes the target heading and the current heading as input, and calculates the necessary steering correction to minimize the error.

```cpp
#pragma once

class PIDController {
public:
    PIDController(); // Constructor
    void setup(float kp, float ki, float kd); // Initializes the controller with PID gains
    void setTunings(float kp, float ki, float kd); // Allows changing PID gains during runtime
    void setOutputLimits(float min, float max); // Clamps the output value (e.g., servo angle)
    void reset(); // Resets the integral and derivative terms
    float compute(float setpoint, float input); // Calculates the PID output based on a target and current value
    float geterror(); // Returns the last calculated error

private:
    float kp, ki, kd; // The PID gains (Proportional, Integral, Derivative)
    float prevError; // The error from the previous calculation
    float integral; // The accumulated error over time
    float outputMin, outputMax; // The minimum and maximum allowed output
    float error; // The current error
    unsigned long lastTime; // The timestamp of the last calculation
};
```

`SerialCommunicator` Class
This class is responsible for all communication between the ESP32 and the Raspberry Pi (or computer running the Python script). It listens for incoming serial data, parses the commands (like AVOID or TURN), and makes the values available to the main logic.
```cpp
#pragma once

#include <Arduino.h>
#include "core/Timer.h"

class SerialCommunicator
{
public:
    SerialCommunicator(); // Constructor

    void update(); // Must be called repeatedly to check for and process incoming data
    void clearSerialBuffer(); // Clears any pending data from the serial input
    bool getManeuverValues(float &distance, float &angle); // Gets the latest AVOID command values
    void resetManeuverValues(); // Resets the maneuver values after they have been used
    float getTurn(); // Gets the latest TURN command value (angle)
    void resetTurn(); // Resets the turn value after it has been used

private:
    static const uint16_t PACKET_SIZE = 5; // Defines the size of our communication packet

    // Variables to store the parsed command values
    float _distance;
    float _angle;
    float _turn;
    char _lastCommand;

    Timer timer;

    void processPacket(uint8_t *buffer); // Internal function to parse a raw data packet
};
```
`Steering` Class
A straightforward class to control the steering servo motor. It simplifies setting the steering angle and centering the wheels.

```cpp
#ifndef STEERING_H
#define STEERING_H

#include <ESP32Servo.h>
#include "config.h"

class Steering {
public:
    Steering(int servo_pin); // Constructor: sets the servo's pin number
    void setup(); // Attaches the servo object to its pin
    void setAngle(int angle); // Sets the steering servo to a specific angle
    void center(); // A helper function to set the steering to the center position (0 degrees)

private:
    Servo _servo; // The ESP32 servo library object
    int _servo_pin; // Stores the GPIO pin for the servo
};
#endif // STEERING_H
```

`Timer` Class
A simple utility for handling time-based events without using delay(), which would block the entire program. It can be used to check if a certain amount of time has passed.

```cpp
#pragma once

#include <Arduino.h>

class Timer {
public:
    Timer(); // Constructor
    void start(unsigned long duration_ms); // Starts a non-blocking timer for a specific duration
    bool isFinished(); // Returns true if the timer has finished
    void reset(); // Resets the timer
    static void wait(unsigned long duration); // A traditional blocking delay function

private:
    unsigned long _startTime; // The time when the timer was started
    unsigned long _duration; // The duration of the timer
    bool _running; // Flag to indicate if the timer is active
};
```

`TOFSensor` Class
This class manages the VL53L0X Time-of-Flight (TOF) distance sensor mounted on the rear of the robot. TOF sensors are very fast and accurate, making this perfect for the RESET state where the robot needs to position itself precisely against the back wall. It includes a simple averaging filter to stabilize readings.

```cpp
#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#define TOF_FILTER_SIZE 2

class TOFSensor {
public:
    // Constructor: sets the sensor's shutdown pin and I2C address
    TOFSensor(uint8_t xshutPin, uint8_t i2cAddress = 0x30, uint16_t defaultDistance = 8000);

    bool begin(); // Initializes the VL53L0X sensor
    void update(); // Takes a new reading from the sensor and updates the filter
    bool isOutOfRange(); // Checks if the current reading is out of the sensor's range
    uint16_t getDistance(); // Returns the filtered distance in millimeters

private:
    uint8_t _xshutPin; // The pin used to enable/disable the sensor
    uint8_t _i2cAddress; // The I2C address for the sensor
    uint16_t _defaultDistance; // A default value to return if the reading is out of range
    
    // Variables for the averaging filter
    uint16_t _distanceBuffer[TOF_FILTER_SIZE];
    uint16_t _avgDistance;
    uint8_t _bufferIndex;
    bool _bufferFilled;

    Adafruit_VL53L0X _lox; // The sensor library object
    VL53L0X_RangingMeasurementData_t _measure; // A struct to hold the measurement data
};

#endif
```
