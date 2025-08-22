<center><h1> Bloody Hilux WRO 2025 </center>


<p align="center">
  <img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/other/readme-images/image.png" alt="Banner">
</p>


[![Facebook](https://img.shields.io/badge/Facebook-%231877F2.svg?style=for-the-badge&logo=Facebook&logoColor=white)](https://www.facebook.com/profile.php?id=61553861744768)



[![Youtube](https://img.shields.io/badge/Youtube-%23FF0000.svg?style=for-the-badge&logo=Youtube&logoColor=white)](https://www.youtube.com/@BloodyHilux)






## Table of Contents

* [The Team](#team)
* [The Challenge](#challenge)
* [The Robot](#robot-image)
* [Performance Video](#video)
* [Mobility Management](#mobility-management)
  * [Powertrain](#powertrain-mechanical)
    * [Drive Motor](#motor-mechanical)
    * [Motor Driver](#motor-driver-mechanical)
  * [Steering](#steering-mechanical)
    * [Servo Motor](#servo-motor)
  * [Chassis](#chassis-mechanical)
* [Power and Sense Management](#power-and-sense-management)
  * [PCB Design](#pcb)
  * [Main Processor](#main-processor)
  * [Microcontroller](#microcontroller)
  * [Li-Po Battery](#li-po-battery)
  * [Power Regulation](#power-regulation)
  * [Sensors](#sensors)
* [Code and Algorithms](#code-and-algorithms)
  * [Vision Processing and Obstacle Detection (Python)](#obstacle-management)
  * [Real-Time Control and State Machine (C++)](#real-time-control)
* [License](#license)


---


## The Team <a class="anchor" id="team"></a>



### [Team Member 1 Tariq Al-Sabbagh Al-Taji]

<p align="center">

  <img src="./team-photos/[member-1-photo].jpg" alt="[Team Member 1 Tariq Al-Sabbagh Al-Taji]" width="80%">

</p>


<b>Role:</b> Programmer

<b>Age:</b> 21



<b>University:</b> Damascus University



<b>Description:</b>  A third-year Information Technology Engineering student at Damascus University. He is a backend developer and a volunteer at the Robotics Club.



---



### [Team Member 2 Tayseer Matar]

<p align="center">

  <img src="./team-photos/member-2-photo.jpg" alt="[Team Member 2 Tayseer Matar]" width="80%">

</p>


<b>Role:</b> Electrical

<b>Age:</b> 21



<b>University:</b> Damascus University



<b>Description:</b> A third-year Information Technology Engineering student at Damascus University. He is a frontend developer and a volunteer at the Robotics Club.



---



### [Team Member 3 Abdulrahman Qassouma]

<p align="center">

  <img src="./team-photos/[member-3-photo].jpg" alt="[Team Member 3 Abdulrahman Qassouma]" width="80%">

</p>

<b>Role:</b> 3D design and the development of mechanical systems

<b>Age:</b> 21



<b>University:</b> Damascus University



<b>Description:</b>  A second-year Mechanical Design Engineering student at Damascus University. He specializes in 3D design and the development of mechanical systems.



---



### [Mohammed Moaaz Sweid]

<p align="center">

  <img src="./team-photos/[coach-photo].jpg" alt="[Mohammed Moaz Sweid]" width="80%">

</p>



<b>Role:</b> Coach



<b>Description:</b> A third-year Information Technology  Engineering student at Damascus University. He is a backend and DevOps developer and serves as a coach at the university's Robotics Club.



---



### Team Photo

<p align="center">

  <img src="./team-photos/[team-photo].jpg" alt="Team" width="80%">

</p>



## The Challenge <a class="anchor" id="challenge"></a>



The **[WRO 2025 Future Engineers - Self-Driving Cars](https://wro-association.org/)** challenge invites teams to design, build, and program a robotic vehicle capable of driving autonomously on a racetrack that changes dynamically for each round. The competition includes two main tasks: completing laps while navigating randomized obstacles and successfully performing a precise parallel parking maneuver.



---



## Photos of our robot Bloody Hilux <a class="anchor" id="robot-image"></a>



<img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/v-photos/front_view.jpg" width="90%" /> | <img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/v-photos/back_view.jpg" width="85%" /> | 


| *Front* | *Back* |

<img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/v-photos/left_view.jpg" width="90%" /> | <img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/v-photos/right_view.jpg" width="85%" /> | 

| *Left* | *Right* |

<img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/v-photos/top_view.jpg" width="90%" /> | <img src="https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/v-photos/bottom_view.jpg" width="85%" /> | 

| *Top* | *Bottom* |



<br>



## Our video of the robot on [Youtube](https://youtu.be/ZicO00x3EqA?feature=shared) <a class="anchor" id="video"></a>

<video src="https://youtu.be/ZicO00x3EqA?feature=shared" width="320" height="240" controls></video>

<br>



# [Mobility Management](https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/models/README.md) <a class="anchor" id="mobility-management"></a>
Our robot's mechanical design is engineered for a precise balance of speed, stability, and cornering efficiency. The core components—powertrain, steering, and chassis—work together to achieve reliable and consistent performance on the track.


## Powertrain <a class="anchor" id="powertrain-mechanical"></a>


### Drive Motor <a class="anchor" id="motor-mechanical"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[drive-motor-photo].jpg" alt="DC Motor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Voltage: 12 V</li>
      <li>Speed: 281 RPM</li>
      <li>Torque: 5 kgf·cm (~0.49 N·m)</li>
    </td>
  </tr>
</table>



**Description:** This DC motor was selected because it provides the best trade-off between torque and speed. It ensures strong acceleration and a reliable top speed on straight tracks. The motor is paired with a 2.2:1 gear ratio to increase wheel speed, a suitable trade-off for the flat competition track.



### Motor Driver <a class="anchor" id="motor-driver-mechanical"></a>
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[motor-driver-photo].png" alt="L298N Motor Driver" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Power supply voltage: 12V</li>
      <li>Output current: Up to 2A per channel</li>
      <li>Control: PWM for speed, Digital pins for direction</li>
    </td>
  </tr>
</table>

**Description:** We are using the L298N motor driver to safely interface the ESP32 with the high-current DC motor. It allows us to control both the speed (via PWM) and the direction of the motor. It is powered directly from the 12V battery pack.



## Steering <a class="anchor" id="steering-mechanical"></a>



**Description:** Our steering system is custom-designed based on Ackermann geometry. This ensures that the inner and outer wheels follow their correct turning radii (inner wheel at 65°, outer at 38°), which minimizes tire drag and makes cornering highly precise and stable. The mechanism is built with custom 3D-printed parts and durable linkages.



### Servo Motor <a class="anchor" id="servo-motor"></a>



<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[servo-motor-photo].jpg" alt="[Servo Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Weight: 9.3 g</li>
      <li>Rotation angle: 0 - 180</li>
    </td>
  </tr>
</table>


**Description:** A single servo motor actuates our Ackermann steering mechanism. It was chosen for its reliability and sufficient torque to handle the steering linkages under load.



## Chassis <a class="anchor" id="chassis-mechanical"></a>



**Description:** The chassis is a hybrid construction using two plexiglass plates as the main structure, connected by custom 3D-printed support components. This design provides an excellent balance of durability and lightness, with a total weight of approximately 1.25 – 1.3 kg. All components are strategically placed to keep the center of gravity low, which is critical for maintaining stability at high speeds and during sharp turns.



![Chassis](./other/readme-images/[chassis-design-image].png "Chassis")



---



# # [Power and Sense Management](https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/e-schemes/README.md) <a class="anchor" id="power-and-sense-management"></a>
Our robot's electronic system is a two-part brain, with a Raspberry Pi 5 handling high-level vision processing and an ESP32 managing real-time hardware control. This distributed architecture ensures that complex calculations do not interfere with the precise, low-latency actions required for navigation.


### PCB Design <a class="anchor" id="pcb"></a>

**Description:** To ensure reliable connections and an organized layout, we integrated all our core electronic components onto a central PCB. This board serves as the main hub for power distribution, taking the 12V input from the battery pack and routing it through the power switch and LM2596 voltage regulator to supply all other components. Using a PCB instead of loose wires significantly improves the robot's durability by preventing accidental disconnections and makes troubleshooting much easier.

### Main Processor <a class="anchor" id="main-processor"></a>
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[rpi5-photo].jpg" alt="Raspberry Pi 5" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Processor: Broadcom BCM2712 2.4GHz quad-core 64-bit Arm Cortex-A76 CPU</li>
      <li>RAM: 8 GB</li>
      <li>Power: 5V/5A DC via USB-C</li>
    </td>
  </tr>
</table>

**Description:** The Raspberry Pi 5 serves as the main brain for all high-level computations. It runs our Python-based computer vision algorithms using OpenCV to process the camera feed, detect obstacles, and identify turn indicators. It then sends high-level commands (e.g., "avoid obstacle at this angle") to the ESP32 via USB serial communication.

### Microcontroller
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[microcontroller-photo].jpg" alt="ESP32 DevKit" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Microcontroller: ESP32</li>
      <li>Frequency: 240 MHz (Dual Core)</li>
      <li>Flash memory: 4 MB</li>
    </td>
  </tr>
</table>

**Description:**We chose the ESP32 for its powerful dual-core processor and reliability in real-time operations. It acts as the hardware controller, executing the commands sent by the Raspberry Pi. Its responsibilities include generating precise PWM signals for the motor and servo, and gathering data from all the low-level sensors (IMU, Encoders, etc.) with minimal latency.



### Li-Po Battery <a class="anchor" id="li-po-battery"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[battery-photo].jpg" alt="12V Battery Pack" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Capacity: 5000 mA</li>
      <li>Voltage: 12V (3S Configuration)</li>
      <li>Protection: BMS 3S for overcharge/discharge protection</li>
    </td>
  </tr>
</table>

**Description:**  The main power for the motors and PCB comes from a 12V battery pack, constructed from three 18650 Li-ion cells in series. A BMS 3S board is integrated to ensure safe charging, prevent over-discharge, and provide a stable 12V output. This power is regulated by an LM2596 step-down converter for the electronics

### Power Regulation
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[lm2596-photo].jpg" alt="LM2596 Step-Down Regulator" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Input Voltage: 12V</li>
      <li>Output Voltage: 5V / 3.3V (Adjustable)</li>
      <li>Max Current: 3A</li>
    </td>
  </tr>
</table>

**Description:** An LM2596 step-down voltage regulator is used to convert the 12V from the main battery pack into a stable 5V or 3.3V supply for the microcontrollers and sensors. This is crucial for protecting the sensitive electronics from the high voltage of the battery.

### Sensors  <a class="anchor" id="microcontroller"></a>

#### Camera
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[camera-photo].jpg" alt="Raspberry Pi Camera V3" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Resolution: 12 Megapixels (4056×3040)</li>
      <li>Frame rate: Up to 60 fps</li>
    </td>
  </tr>
</table>


**Description:**  The Raspberry Pi Camera V3 is our primary sensor for vision processing. Connected directly to the Raspberry Pi 5, it provides a high-resolution video stream that is analyzed in real-time to detect colored obstacles and track lines on the course.


#### IMU Sensor

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[imu-photo].jpg" alt="BNO055 IMU" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Gyroscope range: ±2000 dps</li>
      <li>Accelerometer range: ±4g</li>
      <li>Communication: I2C</li>
    </td>
  </tr>
</table>

**Description:** The BNO055 IMU is crucial for navigation and stability. It provides accurate orientation data (heading), which is fed into our PID steering controller. This allows the robot to drive in a perfectly straight line and execute turns with high precision.


#### Ultrasonic Sensors
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[ultrasonic-photo].jpg" alt="Ultrasonic Sensor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Range: 2cm - 400cm</li>
      <li>Purpose: Wall and obstacle detection</li>
    </td>
  </tr>
</table>

**Description:** Three ultrasonic sensors (front, left, and right) are used for general distance measurement and obstacle avoidance. They provide reliable data for detecting walls during turns and serve as a secondary system to the camera for avoiding unexpected objects.

#### Time-of-Flight (ToF) Sensor

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[tof-photo].jpg" alt="VL53L0X ToF Sensor" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Range: Up to 2m</li>
      <li>Technology: Laser-based</li>
      <li>Communication: I2C</li>
    </td>
  </tr>
</table>

**Description:**  A VL53L0X ToF sensor is mounted on the rear of the robot. It uses a laser to measure distances with high speed and precision. Its primary role is to accurately position the robot against the back wall after a turn, ensuring a consistent starting point for each new section of the track.

#### Magnetic Encoder
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[encoder-photo].jpg" alt="AS5600 Magnetic Encoder" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Resolution: 12-bit (4096 positions per revolution)</li>
      <li>Technology: Hall-effect</li>
      <li>Communication: I2C</li>
    </td>
  </tr>
</table>

**Description:** The AS5600 magnetic encoder is attached to the motor shaft to provide high-precision feedback on wheel rotation. This allows us to accurately track the distance traveled, which is essential for executing precise avoidance maneuvers and for odometry calculations.


# [ Code and Algorithms ](https://github.com/Tariq-Sabbagh/WRO_Future_Engineers_2025/blob/main/src/README.md)<a class="anchor" id="code-and-algorithms"></a>

Our robot's intelligence is built on a distributed software architecture. A Python script on the `Raspberry Pi 5` handles all the complex computer vision tasks, acting as the "eyes" of the robot. It communicates high-level commands to a C++ firmware running on the `ESP32`, which serves as the "nervous system," executing physical movements with real-time precision.

# Vision Processing and Obstacle Detection (Python) <a class="anchor" id="obstacle-management"></a>

The core of our autonomous navigation is the vision script running on the Raspberry Pi. It processes the camera feed in real-time to identify obstacles and turn indicators.

* **Reliable Color Detection:** To detect objects under varying lighting conditions, we convert images from RGB to the LAB color space. This allows us to isolate specific colors accurately, regardless of shadows or bright spots on the track.

* **Focused Processing (ROI):** To improve performance and reduce false detections, we don't process the entire image. Instead, we define a specific Region of Interest (ROI) in the lower part of the frame where we expect to see the track, focusing our processing power only where it matters.

* **Maneuver Calculation:** When an obstacle is detected, the script calculates a precise avoidance maneuver. It estimates the distance to the obstacle based on its apparent height and the required steering angle based on its horizontal position. These two values are then sent as a single command to the ESP32.

```python
def process_obstacle(self, contour, frame_rgb, color_type):
    # Calculate distance from the obstacle's height
    distance = self.calculate_distance(h)
    # Calculate the angle and path to avoid it
    travel_dist, turn_angle = self.calculate_maneuver(...)

    # Send the command to the robot
    self.send_command('AVOID', travel_dist, turn_angle)
```

# Real-Time Control and State Machine (C++) <a class="anchor" id="real-time-control"></a>
The ESP32 firmware is designed for robust, real-time control. It receives commands from the Raspberry Pi and translates them into precise hardware actions using a clean and predictable structure.

* **State Machine:** The core of the firmware is a state machine. The robot operates in distinct states such as FORWARD, AVOIDING, TURN, and RESET. This approach allows us to manage complex sequences of actions in an organized way, making the robot's behavior reliable and easy to debug.
```cpp
void ObstacleAvoider::loop()
{
    // ... update sensors and check for commands ...

    switch (_currentState)
    {
    case AVOIDING: _avoidObstacle(); break;
    case FORWARD:  _goForward();     break;
    case TURN:     _turn();          break;
    case RESET:    _resetCar();      break;
    }
}
```


* **PID Control for Precision:** To drive perfectly straight and execute smooth turns, we use a PID controller. It continuously takes the current heading from the `BNO055 IMU` and compares it to the target heading, calculating the exact steering correction needed. This feedback loop is the key to our robot s stability and accuracy.

* **Code Structure and Helper Classes:** To keep the main logic clean, our C++ code is heavily object-oriented. We developed a suite of helper classes, where each class is responsible for managing a single piece of hardware (e.g., `MotorController`, `IMU`, `Encoder`, `PIDController`). This abstraction allows us to easily manage complex hardware interactions without cluttering the main state machine.


## Copyright <a class="anchor" id="copyright"></a>

```
MIT License

Copyright (c) 2025 Bloody Hilux

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

```
