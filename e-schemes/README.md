# WRO Future Engineer Car Project

This project is a smart robotic car designed for the **World Robot Olympiad (WRO) Future Engineer competition**. The car integrates a **Raspberry Pi 5**, **ESP32**, multiple sensors, actuators, a 12V battery pack, and a Pi Camera V3 to enable autonomous navigation, obstacle detection, and precise movement control.

---

## Electronic Section (Power and Sense Management)

This section describes all electronic components, their power management, sensors, actuators, and connections in the car system.

---

## 1. Components Overview

### Raspberry Pi 5
- **Purpose:** Main processor for high-level computations, vision processing, and communication.  
- **Why Used:** Handles complex algorithms like obstacle detection, path planning, and communicates with the ESP32 for motor and sensor control.  
- **Power Source:** 5V powerbank connected via USB to the ESP32 for power and serial communication.

### Raspberry Pi Camera V3
- **Purpose:** Captures video/images for obstacle detection and computer vision tasks.  
- **Why Used:** Enhances autonomous navigation with visual obstacle recognition and path tracking.  
- **Specs:** 12 MP, 4056×3040 still resolution, supports 60 fps video.  
- **Connection:** Directly connected to Raspberry Pi board.  

### ESP32 DevKit
- **Purpose:** Real-time control of motors, servos, and sensors.  
- **Why Used:** Provides low-latency processing for actuator control and sensor reading.  
- **Power Source:** USB from Pi.

### L298N Motor Driver
- **Purpose:** Drives DC motors with speed and directional control via PWM.  
- **Why Used:** Safely interfaces ESP32 with high-current motors.  
- **Power Source:** Directly from 12V battery pack.

### MG90 Servo Motor
- **Purpose:** Controls steering angle for the car.  
- **Why Used:** Provides precise angular motion for navigation.

### LM2596 Step-Down Voltage Regulator
- **Purpose:** Converts 12V battery voltage to lower voltages for microcontrollers and sensors.  
- **Why Used:** Ensures stable voltage and protects electronics.

### Ultrasonic Sensors (Front, Right, Left)
- **Purpose:** Detect obstacles and measure distances.  
- **Why Used:** Enables real-time obstacle avoidance.  
- **Power Supply:** 12V via LM2596 step-down.

### BNO055 IMU Sensor
- **Purpose:** Measures orientation, acceleration, and angular velocity.  
- **Why Used:** Provides accurate feedback for navigation, stabilization, and movement correction.  
- **Power Supply:** 3.3V from ESP32.  
- **Communication:** I2C.

### VL53L0X ToF Sensor
- **Purpose:** Measures precise distances using laser light.  
- **Why Used:** Provides short-range distance measurements for navigation and obstacle detection.  
- **Power Supply:** 3.3V from ESP32.  
- **Communication:** I2C.

### AS5600 Magnetic Encoder
- **Purpose:** Measures motor rotation and angular position with high precision.  
- **Why Used:** Enables precise tracking and control of wheel rotation.  
- **Power Supply:** 3.3V from ESP32.  
- **Communication:** I2C.

### 12V Battery Pack (3 × 18650 Li-ion) with BMS 3S
- **Purpose:** Supplies main power to PCB, motors, and sensors.  
- **Configuration:** 3 × 18650 cells in series (3S) → 12V nominal.  
- **Charging & Protection:** Equipped with **BMS 3S** for safe charging, overcharge/discharge protection, and stable 12V output.  
- **Connection:** Provides 12V to LM2596 and motor driver; integrated into PCB.  

### Control Button
- **Purpose:** Manual operation to start or stop the car.  
- **Why Used:** Provides a simple user interface for testing.

### Power Switch
- **Purpose:** Controls the 12V battery supply to the PCB.  
- **Why Used:** Allows safe powering on/off of all PCB-connected components.

---

## 2. System Architecture

The system architecture consists of two main controllers:

1. **Raspberry Pi 5**
   - Runs computer vision algorithms for obstacle detection using Pi Camera V3.
   - Sends high-level movement commands to ESP32 via USB serial communication.

2. **ESP32 DevKit**
   - Controls DC motors and servo for precise navigation.
   - Reads sensors (ultrasonic, ToF, IMU, AS5600) for feedback and obstacle avoidance.
   - Sends real-time sensor data to Raspberry Pi.

**Power Distribution:**
- 12V battery pack (3 × 18650 Li-ion with BMS 3S) → LM2596 step-down → ESP32 and sensors  
- ESP32 I2C sensors: 3.3V  
- Raspberry Pi 5: 5V powerbank → USB connection  
- Pi Camera V3: Powered through Pi board

---

## 3. Pin Configuration (ESP32)

| Component              | Pin Number / Channel       |
|------------------------|---------------------------|
| Motor Speed (PWM)      | 5 (channel 1, 30kHz, 8-bit) |
| Motor Direction 1      | 19                        |
| Motor Direction 2      | 18                        |
| Servo                  | 32                        |
| Ultrasonic Front       | 25                        |
| Ultrasonic Right       | 26                        |
| Ultrasonic Left        | 27                        |
| Control Button         | 15                        |
| ToF Sensor (SHT_LOX)  | 16                        |
| IMU Sensor (BNO055)   | I2C (SDA/SCL)             |
| AS5600 Encoder         | I2C (SDA/SCL)             |
| Pi Camera V3           | Connected to Raspberry Pi board |

---

## 4. Software Stack

- **Raspberry Pi 5:** Python 3, OpenCV for computer vision, serial communication with ESP32.  
- **ESP32 DevKit:** Arduino C++ firmware for motors, servos, and sensors.  
- **Libraries:** 
  - `Adafruit_BNO055` for IMU  
  - `VL53L0X` library for ToF sensor  
  - `Servo.h` and PWM libraries for motor and servo control

---

## 5. Calibration and Tuning

- **Servo Calibration:** Align center at 90°, set min/max pulse width (0–2000 µs)  
- **Motor PWM Tuning:** Recommended speed 200/255  
- **PID for Steering:** KP = 3.5  
- **Ultrasonic Sensors:** Test max distance = 200 cm, apply averaging filter  
- **IMU & Encoder:** Test rotation and angular feedback accuracy

---

## 6. Testing and Validation

- Obstacle detection range and response time  
- Servo angle precision and turning accuracy  
- Distance tracking using AS5600 encoder  
- Stability and heading using BNO055 IMU  
- Integration testing with Pi Camera obstacle detection

---

## 7. Challenges and Solutions

- **Ultrasonic Noise:** Solved with averaging filter  
- **Servo Jitter:** Adjusted PWM frequency  
- **USB Latency:** Optimized serial communication between Pi and ESP32  
- **Sensor Power Stability:** Ensured regulated 3.3V supply for I2C devices

---

## 8. Future Improvements

- Add AI-based path planning and object recognition  
- Integrate wireless control and telemetry  
- Implement battery monitoring and automated charging  
- Improve sensor fusion algorithms for smoother navigation

---

## 9. Quantity and Pricing (Estimates)

| Component              | Quantity | Estimated Price (USD) | Notes |
|------------------------|----------|---------------------|-------|
| Raspberry Pi 5         | 1        | $80                 | Powered by 5V powerbank |
| Raspberry Pi Camera V3 | 1        | $35                 | Connected to Pi board |
| ESP32 DevKit           | 1        | $12                 | Real-time control |
| L298N Motor Driver     | 1        | $6                  | Dual motor driver |
| MG90 Servo Motor       | 1        | $4                  | Steering control |
| LM2596 Step-Down       | 1        | $2                  | Voltage regulation from 12V |
| BNO055 IMU Sensor      | 1        | $15                 | 3.3V supply |
| Ultrasonic Sensor      | 3        | $3 each ($9 total) | 3.3V supply |
| VL53L0X ToF Sensor     | 1        | $6                  | 3.3V supply |
| AS5600 Encoder         | 1        | $5                  | Magnetic rotary encoder, I2C |
| 12V Battery Pack + BMS | 1        | $2                  | 3 × 18650 Li-ion, BMS 3S, powers PCB & motors |
| Control Button         | 1        | $1                  | Manual control |
| Power Switch           | 1        | $2                  | Controls 12V battery to PCB |
| Misc. Wires & PCB      | -        | $10                 | Connections & mounting |

**Total Estimated Cost:** ~$194  

---

## 10. Project Overview

This car is designed for:

- **Autonomous navigation:** Using ultrasonic, ToF sensors, and Pi Camera V3.  
- **Precise movement control:** Using DC motors, servo steering, IMU, and AS5600 encoder.  
- **Obstacle detection and avoidance:** Real-time detection with sensors and computer vision.  
- **Safe and flexible power management:** 12V battery pack with BMS and switch, 5V powerbank for Pi, 3.3V for I2C sensors.  
- **Future scalability:** Can integrate advanced AI algorithms and visual recognition using Raspberry Pi 5.

---

## 11. Common Problems and Troubleshooting

Even with careful assembly, robotics projects like this car may encounter common issues. Here’s a list of potential problems and solutions:

### 1. Raspberry Pi Not Powering On
- **Cause:** Insufficient power from the 5V powerbank or poor USB connection to ESP32.  
- **Solution:** Ensure the powerbank provides at least 3A output. Use a high-quality USB cable and check connections.

### 2. Pi Camera V3 Not Detected
- **Cause:** Incorrect ribbon cable connection or disabled camera interface.  
- **Solution:** Re-seat the ribbon cable, enable the camera in `raspi-config`, and reboot Pi.

### 3. Motors Not Responding
- **Cause:** Wrong wiring to L298N or incorrect PWM configuration.  
- **Solution:** Double-check motor wiring and verify ESP32 PWM channel and frequency settings.

### 4. Servo Jitter or Incorrect Steering
- **Cause:** PWM frequency mismatch or unstable voltage.  
- **Solution:** Ensure the servo receives stable voltage and adjust PWM frequency. Calibrate center position.

### 5. Ultrasonic Sensor Noise / False Readings
- **Cause:** Interference between sensors or environmental noise.  
- **Solution:** Apply averaging filters in software, separate trigger pins for each sensor, and test in a less reflective environment.

### 6. AS5600 Encoder Incorrect Readings
- **Cause:** Magnetic misalignment or I2C communication error.  
- **Solution:** Ensure magnet is correctly positioned, check I2C connections, and verify library usage.

### 7. IMU (BNO055) Drifting or Instability
- **Cause:** Improper calibration or power fluctuations.  
- **Solution:** Calibrate the IMU properly, ensure stable 3.3V supply, and avoid sudden shocks during operation.

### 8. USB Latency Between Pi and ESP32
- **Cause:** High data rate or inefficient serial communication.  
- **Solution:** Optimize serial protocol, reduce data frequency, and avoid sending unnecessary data.

### 9. VL53L0X ToF Sensor Inaccurate Distance
- **Cause:** Reflective surfaces or ambient light interference.  
- **Solution:** Avoid highly reflective surfaces, calibrate sensor for the specific environment.

### 10. PCB Power Issues
- **Cause:** Overloaded step-down regulator or loose connections.  
- **Solution:** Check current ratings, ensure proper soldering, and verify LM2596 output voltage.
