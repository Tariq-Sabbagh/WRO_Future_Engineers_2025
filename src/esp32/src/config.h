#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

// -- Motor Controller Pins --
const int MOTOR_SPEED_PIN = 5;  // PWM Speed Control
const int MOTOR_DIR1_PIN = 19; // Direction 1
const int MOTOR_DIR2_PIN = 18; // Direction 2


// -- Servo/Steering Pin --
const int SERVO_PIN = 32;

// -- Ultrasonic Sensor Pins --
const int ULTRASONIC_PIN_FRONT = 25;
const int ULTRASONIC_PIN_RIGHT = 26;
const int ULTRASONIC_PIN_LEFT = 27;

// -- Control Button Pin --
const int BUTTON_PIN = 15;

// =============================================================================
// TUNING PARAMETERS & CONSTANTS
// =============================================================================

// -- Motor Control --
const int MOTOR_PWM_CHANNEL = 1;
const int MOTOR_PWM_FREQ = 30000;
const int MOTOR_PWM_RESOLUTION = 8;
const int FORWARD_SPEED = 170; // Speed value from 0-255
const int BACKWARD_SPEED = -170;

// -- Steering Control --
const int SERVO_CENTER_ANGLE = 90; // The exact angle that makes your servo point straight.
const int SERVO_MIN_PULSE = 0;   // Servo pulse width for 0 degrees
const int SERVO_MAX_PULSE = 2000;  // Servo pulse width for 180 degrees
const float STEERING_KP = 3.5;     // Proportional gain for steering correction. Tune this!

// -- Sensor Settings --
const int ULTRASONIC_MAX_DISTANCE_CM = 200; // Max distance for ultrasonic sensors.

// -- Navigation Logic --
const float TURN_TRIGGER_DISTANCE_CM = 55.0; // How close to a wall before deciding to turn.
const float TURN_CLEARANCE_DISTANCE_CM = 100.0; // How much space must be clear to the side to make a turn.
const unsigned long TURN_COOLDOWN_MS = 1000;  // Minimum time between turns.
const int _all_turn = 45;

const int SHT_LOX = 16;

const int NEOPIXEL_PIN = 17;

#endif // CONFIG_H
