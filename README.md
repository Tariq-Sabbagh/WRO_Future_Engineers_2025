<center><h1> Bloody hilux WRO 2025 </center>

![Banner](./other/readme-images/image.png)

[![Facebook](https://img.shields.io/badge/Facebook-%231877F2.svg?style=for-the-badge&logo=Facebook&logoColor=white)](https://www.facebook.com/profile.php?id=61553861744768)

[![Youtube](https://img.shields.io/badge/Youtube-%23FF0000.svg?style=for-the-badge&logo=Youtube&logoColor=white)]([www.youtube.com/@BloodyHilux])



## Table of Contents
* [The Team](#team)
* [The Challenge](#challenge)
* [The Robot](#robot-image)
* [Performance Video](#video)
* [Mobility Management](#mobility-management)
  * [Powertrain](#powertrain-mechanical)
  * [Steering](#steering-mechanical)
  * [Chassis](#chassis-mechanical)
* [Power and Sense Management](#power-and-sense-management)
  * [Battery](#li-po-battery)
  * [Microcontroller](#microcontroller)
  * [Sensors](#sensors)
  * [PCB Design](#pcb)
  * [Circuit Diagram](#circuit-diagram)
* [Code for each component](#code-for-each-component)
  * [Drive Motor](#drive-motor-code)
  * [Servo Motor](#servo-motor-code)
  * [Camera](#camera-code)
  * [IMU Sensor](#gyro-sensor-code)
* [Obstacle Management](#obstacle-management)
  * [Qualification Round](#quali-management)
  * [Final Round](#final-management)
* [Robot Construction Guide](#robot-construction-guide)
* [Cost Report](#cost-report)
* [License](#copyright)

---

## The Team <a class="anchor" id="team"></a>

### [Team Member 1 Full Name]
<p align="center">
  <img src="./team-photos/[member-1-photo].jpg" alt="[Team Member 1 Full Name]" width="80%">
</p>

<b>Age:</b> [Age]

<b>High School/University:</b> [School Name]

<b>Description:</b> [Write a brief description of the team member, their interests, and their role on the team.]

---

### [Team Member 2 Full Name]
<p align="center">
  <img src="./team-photos/[member-2-photo].jpg" alt="[Team Member 2 Full Name]" width="80%">
</p>

<b>Age:</b> [Age]

<b>High School/University:</b> [School Name]

<b>Description:</b> [Write a brief description of the team member, their interests, and their role on the team.]

---

### [Team Member 3 Full Name]
<p align="center">
  <img src="./team-photos/[member-3-photo].jpg" alt="[Team Member 3 Full Name]" width="80%">
</p>

<b>Age:</b> [Age]

<b>High School/University:</b> [School Name]

<b>Description:</b> [Write a brief description of the team member, their interests, and their role on the team.]

---

### [Coach Name]
<p align="center">
  <img src="./team-photos/[coach-photo].jpg" alt="[Coach Name]" width="80%">
</p>

<b>Role:</b> Coach

<b>Description:</b> [Write a brief description of the coach.]

---

### Team Photo
<p align="center">
  <img src="./team-photos/[team-photo].jpg" alt="Team" width="80%">
</p>

## The Challenge <a class="anchor" id="challenge"></a>

The **[WRO 2024 Future Engineers - Self-Driving Cars](https://wro-association.org/)** challenge invites teams to design, build, and program a robotic vehicle capable of driving autonomously on a racetrack that changes dynamically for each round. The competition includes two main tasks: completing laps while navigating randomized obstacles and successfully performing a precise parallel parking maneuver.

---

## Photos of our robot [Your Robot's Name] <a class="anchor" id="robot-image"></a>

| <img src="./robot-photos/front.png" width="90%" /> | <img src="./robot-photos/back.png" width="85%" /> | 
| :--: | :--: | 
| *Front* | *Back* |
| <img src="./robot-photos/left.png" width="90%" /> | <img src="./robot-photos/right.png" width="85%" /> | 
| *Left* | *Right* |
| <img src="./robot-photos/top.png" width="90%" /> | <img src="./robot-photos/bottom.png" width="85%" /> | 
| *Top* | *Bottom* |

<br>

## Our video of the robot on [Youtube]([paste-youtube-video-link-here]) <a class="anchor" id="video"></a>

<br>

# Mobility Management <a class="anchor" id="mobility-management"></a>

## Powertrain <a class="anchor" id="powertrain-mechanical"></a>

### Drive Motor <a class="anchor" id="motor-mechanical"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[drive-motor-photo].jpg" alt="[Motor Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Voltage: [Enter Voltage]</li>
      <li>Speed: [Enter Speed]</li>
      <li>Torque: [Enter Torque]</li>
      <li>Weight: [Enter Weight]</li>
    </td>
  </tr>
</table>

**Description:** [Explain why you chose this motor. Mention any custom 3D printed parts like mounts or adapters.]

### Motor Driver <a class="anchor" id="motor-driver-mechanical"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[motor-driver-photo].png" alt="[Motor Driver Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Power supply voltage: [Enter Voltage]</li>
      <li>Output current: [Enter Current]</li>
      <li>Other Features: [Describe other key features]</li>
    </td>
  </tr>
</table>

**Description:** [Explain which motor driver you used and why.]

## Steering <a class="anchor" id="steering-mechanical"></a>

**Description:** [Describe your steering mechanism. Is it Ackermann, a simple parallelogram, or something else? Explain your design choices and mention any custom parts.]

### Servo Motor <a class="anchor" id="servo-motor"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[servo-motor-photo].jpg" alt="[Servo Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Weight: [Enter Weight]</li>
      <li>Stall torque: [Enter Torque]</li>
      <li>Operating speed: [Enter Speed]</li>
      <li>Rotation angle: [Enter Angle]</li>
    </td>
  </tr>
</table>

**Description:** [Explain why you chose this servo motor.]

## Chassis <a class="anchor" id="chassis-mechanical"></a>

**Description:** [Describe your chassis design. What materials did you use (3D prints, LEGO, acrylic)? What are the key design features and how does it integrate all the components? Include 3D model images if possible.]

![Chassis](./other/readme-images/[chassis-design-image].png "Chassis")

---

# Power and Sense Management <a class="anchor" id="power-and-sense-management"></a>

### Li-Po Battery <a class="anchor" id="li-po-battery"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[battery-photo].jpg" alt="[Battery Type]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Capacity: [Enter Capacity, e.g., 450mAh]</li>
      <li>Voltage: [Enter Voltage, e.g., 7.4V/2S]</li>
      <li>Weight: [Enter Weight]</li>
    </td>
  </tr>
</table>

### Microcontroller <a class="anchor" id="microcontroller"></a>

<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[microcontroller-photo].jpg" alt="[Microcontroller Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Microcontroller: [e.g., ESP32]</li>
      <li>Frequency: [Enter Frequency]</li>
      <li>Flash memory: [Enter Memory]</li>
    </td>
  </tr>
</table>

**Description:** [Explain why you chose this microcontroller for your project.]

### Sensors <a class="anchor" id="sensors"></a>

#### Camera
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[camera-photo].jpg" alt="[Camera Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Resolution: [Enter Resolution]</li>
      <li>Frame rate: [Enter Frame Rate]</li>
    </td>
  </tr>
</table>

#### IMU Sensor
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./other/readme-images/[imu-photo].jpg" alt="[IMU Name]" width="100%">
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <li>Gyroscope range: [Enter Range]</li>
      <li>Accelerometer range: [Enter Range]</li>
    </td>
  </tr>
</table>

### PCB Design <a class="anchor" id="pcb"></a>

**Description:** [If you designed a custom PCB, describe it here. If you used a prototype board, explain how the components are connected. Include photos of your board.]

### Circuit Diagram <a class="anchor" id="circuit-diagram"></a>
![Circuit diagram](./electrical-diagram/[your-circuit-diagram].png "Circuit diagram")

---

# Code for each component <a class="anchor" id="code-for-each-component"></a>

## Drive Motor <a class="anchor" id="drive-motor-code"></a>

**Description:** [Briefly explain your logic for controlling the drive motor. What libraries (if any) did you use?]
```ino
// Paste your Arduino code for controlling the drive motor here.
// Example:
void motor_setup() {
  // Setup code here
}

void move_motor(int speed) {
  // Motor movement code here
}
