# Team Trinity – WRO Pakistan 2025
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/Logo%20%2B%20Banner/banner.png" alt="banner">

This repository documents **Roboinity**, Team Trinity’s entry in the **World Robot Olympiad 2025 – Future Engineers (Pakistan)** category.  
Our robot was designed, 3D-printed, and programmed to meet this year’s challenge: building an autonomous vehicle capable of completing laps, avoiding obstacles, and parking with precision.  
We focused on creating a reliable, affordable, and well-documented design so other students can learn from and replicate our work.  

---

## Table of Contents  
- [The Team](#the-team)  
- [The Challenge](#the-challenge)  
- [The Robot](#the-robot--roboinity)  
- [Robot Components & Specifications](#-robot-components--specifications)  
- [Code](#code)  
- [Robot Construction Guide](#robot-construction-guide)  
- [System Diagrams](#system-diagrams)  
- [Media and Resources](#media-and-resources)  
- [Future Improvements](#future-improvements)  

---

## The Team  

**Team Name:** Team Trinity  
**School:** Trinity School  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/Logo%20%2B%20Banner/logo.png" alt="logo">

<table>
  <tr>
    <td align="center">
      <b>Eman Amir</b><br>
      Team Leader, Programmer, Documentation, Social Media  
      <br><i>18 years old. Driven, hardworking, resilient. Leads both the technical side and the project narrative.</i><br><br>
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/eman.png" alt="eman" width="200"/>
    </td>
    <td align="center">
      <b>Fasih ur Rehman</b><br>
      Mechanical Design, Programmer
      <br><i>17 years old. Designer & fabricator, always rethinking solutions to make the chassis stronger and lighter.</i><br><br>
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/fasih.png" alt="fasih" width="200"/>
    </td>
    <td align="center">
      <b>Areena Ihsan</b><br>
      Team Strategist, Social Media  
      <br><i>17 years old. Keeps the team on track and manages media coverage of the journey.</i><br><br>
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/areena.png" alt="areena" width="200"/>
    </td>
  </tr>
  <tr>
    <td align="center">
      <b>Coach: Jahanzab Aslam Siddique</b><br>
      Team Coach  
      <br><i>Mentor and guide, the backbone of the team. Provides technical advice, strategy feedback, and motivation.</i><br><br>
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/sir.png" alt="sir" width="220"/>
    </td>
    <td align="center" colspan="2">
      <b>The Team</b><br><br>
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/team.png" alt="team" width="300"/>
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/selfie.png" alt="selfie" width="300"/>
    </td>
  </tr>
</table>

---

## The Challenge  

The **WRO Future Engineers Challenge 2025** requires teams to:  
- Design a self-driving robot that can navigate a dynamically changing track.  
- Detect and avoid randomized obstacles.  
- Perform a precise parallel parking maneuver.  

**Our strategy:**  
1. Build a lightweight robot to maximize speed and agility.  
2. Use **sensor fusion** (PixyCam + sonar) for reliable obstacle detection.  
3. Apply **PID control** for stable driving and accurate positioning.  

---

## The Robot – Roboinity  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/IMG_3968.png" alt="robot">

**Roboinity** is a fully 3D-printed robot built with affordable, widely available components.  
It integrates **direct-drive motors, servo-based steering, sonar distance sensors, and a Pixy camera** to achieve autonomy.  

**Design priorities:**  
- Lightweight and modular chassis.  
- Redundant microcontroller setup (Arduino Mega + Uno).  
- Stable power regulation with buck converters.  
- Cost-effective sensors that balance performance with budget.  

### Robot Views  

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/front.png" alt="Front view of robot" width="300"/><br>
      <b>Front View</b>
    </td>
    <td align="center">
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/back.png" alt="Back view of robot" width="300"/><br>
      <b>Back View</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/left.png" alt="Left view of robot" width="300"/><br>
      <b>Left View</b>
    </td>
    <td align="center">
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/right.png" alt="Right view of robot" width="300"/><br>
      <b>Right View</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/top.png" alt="Top view of robot" width="300"/><br>
      <b>Top View</b>
    </td>
    <td align="center">
      <img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/v-photos/bottom.png" alt="Bottom view of robot" width="300"/><br>
      <b>Bottom View</b>
    </td>
  </tr>
</table>
---

#  Robot Components & Specifications


---

## 1) Mobility Management  

### Encoder Motor (Direct Drive)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Encoder%20Motor%20(Direct%20Drive).jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose this motor:  
> - **Built-in encoder** for position feedback and closed-loop control  
> - **Direct drive** = fewer mechanical parts, less friction, higher efficiency  
> - **Affordable and reliable** alternative to brushless or geared setups  
> - **Compact** and well-suited for lightweight robot chassis  

This is a 12V DC motor with an integrated encoder, providing ~300 RPM under load and ~0.5 kg·cm torque. The encoder offers 12 counts per revolution, giving the Arduino real-time speed and distance feedback for PID adjustments.  

In our design, this motor directly drives the wheels without complex gearboxes. This reduces mechanical wear, simplifies maintenance, and improves efficiency. The encoder data is used in software to ensure the robot drives straight, maintains consistent speed, and can measure distances accurately during lap navigation.  

<div style="clear: both;"></div><br>

---

### MG90S Servo Motor (Steering)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Servo%20Motor%20(Steering).png__i%3DAA" width="275" align="right" style="margin-left: 20px;" />

Why we chose MG90S:  
> - **Metal gears** provide durability under repeated steering corrections  
> - **Fast response** (0.08 sec/60°) improves maneuverability  
> - **High torque-to-size ratio** (2.2 kg·cm at 6V) suits lightweight steering loads  
> - **Compact form factor** fits into 3D-printed chassis designs  

Operating at 4.8–6V, the MG90S provides up to 120° of rotation, making it ideal for steering mechanisms. We use it to control a 3D-printed steering linkage system that turns both front wheels.  

The servo’s reliability and precision are critical for navigation. It ensures the robot makes sharp, consistent turns around obstacles and accurately aligns for parking maneuvers. Its metal gear construction provides strength and longevity, avoiding failures common in plastic gear servos.  

<div style="clear: both;"></div><br>
---

### Dual Servo Motor (Steering Upgrade)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/dual%20servo%20motor.jpg" width="275" align="right" style="margin-left: 20px;" />

**Why we added dual servos:**  
> - A single servo (MG90S) was sufficient, but under stress it showed **slight steering lag**.  
> - **Dual-servo setup** distributes torque evenly to both wheels.  
> - Reduces **mechanical strain** on a single linkage.  
> - Provides **more stable and accurate steering** during sharp turns.  

**How it works:**  
- Two MG90S (or SG90 for lighter loads) servos are mounted on either side of the steering mechanism.  
- Both are electronically linked and driven by the **same control signal**.  
- This ensures that both front wheels receive identical steering input simultaneously.  

**Technical Details:**  
- **Model:** 2× MG90S Metal Gear Servo  
- **Operating Voltage:** 4.8–6V  
- **Stall Torque (per servo):** 2.2 kg·cm at 6V  
- **Speed:** 0.08 sec/60°  
- **Angle Range:** 120°  
- **Control:** PWM signal from Arduino (both servos share the same pin or use a Y-split cable)  

---

### 3D-Printed Chassis  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Chassis%20(3D-Printed).png" width="275" align="right" style="margin-left: 20px;" />

Why we 3D-printed the chassis:  
> - **Lightweight** (~40% lighter than Lego prototypes) improves acceleration  
> - **Modular compartments** for easy mounting of battery, drivers, and sensors  
> - **Low-cost prototyping** using PLA, easily replaceable if damaged  
> - **Customizable** design allows rapid iteration during testing  

The chassis is fabricated using PLA plastic with a 0.2 mm layer height and 20% infill density. It includes snap-fit joints and modular electronics bays, making it easy to upgrade or replace parts.  

By reducing weight while maintaining strength, the 3D-printed chassis maximizes speed and agility during competition. It also allows for better placement of sensors and wiring, ensuring neat cable management and balanced weight distribution.  

<div style="clear: both;"></div><br>

---

## 2) Power & Sense Management  

### LiPo Battery (2S, 2200 mAh)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Battery.jpeg" width="275" align="right" style="margin-left: 20px;" />

Why we chose this battery:  
> - **High discharge rate** (25–30C) supports motor surges  
> - **Lightweight yet powerful** for robotics applications  
> - **2200 mAh capacity** provides solid runtime without excess weight  
> - **Widely available & rechargeable**  

Our robot is powered by a 2-cell (2S) Lithium Polymer battery, nominally 7.4V and fully charged at 8.4V. With 2200 mAh capacity, it supplies consistent power to motors, servos, and electronics throughout the match.  

The high discharge rate ensures motors receive the necessary current during acceleration or obstacle avoidance. Its compact size keeps the robot lightweight, balancing performance and endurance.  

<div style="clear: both;"></div><br>

---

### Arduino Mega 2560  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Microcontroller%20%E2%80%94%20Arduino%20Mega%202560.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose Arduino Mega:  
> - **Large number of I/O pins** (54 digital, 16 analog) for multiple sensors  
> - **16 MHz clock speed** sufficient for real-time robotics  
> - **Reliable libraries** and broad community support  
> - **Dedicated processing** for driving logic and sensor integration  

The Arduino Mega serves as the central controller of the robot, handling DC motor control, PID loops, servo steering, and ultrasonic sensor processing. Its expanded pin count makes it ideal for a complex build with multiple peripherals.  

By dedicating tasks to the Mega, we avoid pin conflicts and keep performance stable. Its low cost and proven reliability make it an excellent backbone for robotics.  

<div style="clear: both;"></div><br>

---

### Arduino Uno  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Microcontroller%20%E2%80%94%20Arduino%20Uno.webp" width="275" align="right" style="margin-left: 20px;" />

Why we added an Arduino Uno:  
> - **Dedicated handling** of PixyCam image data  
> - **Prevents overload** on the Arduino Mega  
> - **Simple and reliable** for serial communication tasks  
> - **Cost-effective** and widely documented  

The Uno is responsible for managing the Pixy2 camera and feeding processed visual data into the control loop. Offloading this task prevents the Mega from becoming overloaded with vision processing, ensuring smooth integration between sensors and motor control.  

This dual-controller setup makes our system more fault-tolerant and efficient under competition conditions.  

<div style="clear: both;"></div><br>


---

## 3) Sensors  

### Sonar (Ultrasonic) Sensor – HC-SR04 Variant  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Sonar%20(Ultrasonic)%20Sensor.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose HC-SR04 variant:  
> - **Wide detection range** (2–400 cm)  
> - **Good accuracy** (±3 mm) for obstacle avoidance  
> - **Unaffected by lighting** unlike IR sensors or cameras  
> - **Waterproof version** adds durability for competitions  

This ultrasonic sensor works at 40 kHz, emitting sound pulses and measuring the echo time to calculate distance. Its robustness and low cost make it ideal for detecting obstacles and walls in a dynamic competition environment.  

In our robot, these sensors are mounted at the front and sides to ensure collision prevention and track alignment. By fusing multiple readings, the robot can decide whether to turn left, right, or stop before impact.  

<div style="clear: both;"></div><br>

---

### Waterproof Ultrasonic Sensor – JSN-SR04T  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Waterproof%20Sonar%20(Ultrasonic)%20Sensor.webp" width="275" align="right" style="margin-left: 20px;" />

Why we chose JSN-SR04T:  
> - **Waterproof head** tolerates splashes, dust, and debris (IP66)  
> - **Extended range** (25–600 cm) for long-distance detection  
> - **Compact design** for flexible mounting on chassis  
> - **Arduino-compatible** via simple Trigger/Echo interface  

Unlike the HC-SR04, this sensor has a remote, potted transducer that allows outdoor and rugged usage. It has a ~25 cm blind zone, but excels at measuring longer ranges with stability.  

We use this sensor for reliable detection of track boundaries and distant obstacles. It adds resilience to the robot, ensuring functionality even in environments with moisture or uneven flooring.  

<div style="clear: both;"></div><br>

---

### Pixy Camera (Pixy2 + PixyMon)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Pixy%20Camera%20(Pixy2%20%2B%20PixyMon)%20.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose Pixy2:  
> - **Real-time object recognition** without heavy onboard CPU  
> - **High frame rate** (60 fps) for smooth detection  
> - **Multi-protocol support** (UART, I2C, SPI, USB)  
> - **Pre-trained color signatures** simplify programming  

The Pixy2 camera detects objects based on color and shape, outputting coordinates and size directly to the Arduino. With its 1296×976 resolution, it provides fast and efficient visual feedback.  

In our build, the Pixy is trained to recognize red and green objects, which represent obstacles in the WRO challenge. This gives the robot vision capability while keeping the computational load low. Combined with sonar data, it allows smarter navigation decisions and obstacle avoidance.  

<div style="clear: both;"></div><br>

---

## 4) Motor Driver  

### L298N Dual H-Bridge  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/L298N%20Dual%20H-Bridge.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose L298N:  
> - **Dual H-bridge** supports 2 DC motors simultaneously  
> - **PWM speed control** via Arduino  
> - **Integrated heat sink** prevents overheating during long runs  
> - **Cheap and widely available** in robotics kits  

The L298N allows full control of motor speed and direction by using logic signals from the Arduino. It supports 5–35V inputs and can provide ~1A continuous per channel (2A peak), which is sufficient for our DC encoder motors.  

Although newer drivers like TB6612FNG are more efficient, the L298N remains robust, easy to use, and cost-effective. It bridges our Arduino logic with the 12V motors, ensuring reliable propulsion and direction changes.  

<div style="clear: both;"></div><br>

---

## 5) Power Regulation  

### Buck Converter (3A)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Buck%20Converter%20(3A).jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose this converter:  
> - **Stable voltage regulation** for servos (5–6V)  
> - **Prevents overvoltage** damage from LiPo batteries  
> - **Compact module** easy to integrate into wiring  
> - **Reliable under load** with 3A current capacity  

The servo motor requires stable 5–6V. LiPo batteries fluctuate between 8.4V (fully charged) and ~7.0V (discharged). The buck converter ensures the servo always receives the correct voltage, protecting it from premature failure and ensuring consistent steering performance.  

<div style="clear: both;"></div><br>

---

### Buck-Boost Converter (4A)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Buck-Boost%20Converter%20(4A)%20.webp" width="275" align="right" style="margin-left: 20px;" />

Why we chose this converter:  
> - **Maintains stable motor voltage** across full battery cycle  
> - **4A output capacity** suits motor surges  
> - **Buck-boost flexibility** handles both step-up and step-down conversion  
> - **Ensures consistent drive power** for competition reliability  

This converter regulates the voltage supply to the encoder motors. Even as battery voltage drops during a match, the buck-boost module keeps motor output stable. This prevents speed inconsistencies and ensures reliable lap completion under all conditions.  

<div style="clear: both;"></div><br>

---

### Additional Buck Converter (5V Rail)  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Additional%20Buck%20Converter%20(5V%20Rail).jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose this converter:  
> - **Dedicated 5V supply** for Arduino boards  
> - **Protects sensitive logic** from LiPo fluctuations  
> - **Isolates noise** from motors and servos  
> - **Keeps microcontrollers stable** during voltage dips  

This buck converter ensures the Arduino Mega and Uno always receive a clean 5V supply. By isolating control logic from motor noise, it prevents random resets or brownouts, which are common when high-current devices share the same unregulated power.  

This safeguard is essential in competitive robotics, where reliability matters as much as performance.  

<div style="clear: both;"></div><br>


---

## Wiring / Power Notes
- Separate **motor power** and **logic power** rails; share **ground**.  
- Set converters with a multimeter before connecting loads.  
- Add inline fuse on battery positive for safety.

---

## Bill of Materials (Quick View)
- 1× DC Encoder Motor (12V, ~300 RPM, 12 CPR, ~0.5 kg·cm)  
- 1× MG90S Metal Gear Servo  
- 1× PLA 3D-Printed Chassis  
- 1× LiPo 2S 2200 mAh (25–30C)  
- 1× Arduino Mega 2560  
- 1× Arduino Uno  
- 1× Waterproof Ultrasonic Sensor (JSN-SR04T or A02YYUW)  
- 1–2× Waterproof HC-SR04 Ultrasonic Sensors  
- 1× Pixy2 Camera  
- 1× L298N Motor Driver  
- 1× Buck (3A), 1× Buck-Boost (4A), 1× Buck (5V logic)  

---

# Code Explanations and Development Story

Code will be uploaded to the [`/src`](./src) folder.  

**Language:** Arduino C++ 

This section explains **how the code works**, the design trade-offs we made, and the stories behind those choices. It covers both sketches: **CODE 1 (UNO/Slave)** and **CODE 2 (MEGA/Master)**.

---

## System Overview (Why Two Arduinos?)

At first, we tried a single Arduino to do everything: motor control, servo steering, ultrasonic sensing, and PixyCam input. The result was jittery motion and missed readings. Splitting into **two boards** made a huge difference:

- **MEGA (Master):** Reads ultrasonic sensors → packages 6 bytes → sends over I²C.  
- **UNO (Slave):** Receives sensor data → steers servo + drives motor.  

This **decoupled sensing from actuation**, reduced timing conflicts, and made debugging easier.

---

## CODE 1: UNO (Slave) — Drives, Steers, and Listens

### 1) I²C Input

We send 16-bit integers (two bytes each) for each distance. Three sensors × 2 bytes = 6 bytes per frame. Using a fixed-length binary frame avoids parsing issues with ASCII strings.

```cpp
Wire.begin(8);                // UNO as I2C slave at address 8
Wire.onReceive(receiveEvent); // Interrupt callback

void receiveEvent(int howMany) {
  if (howMany < numSensors * 2) return;
  for (int i = 0; i < numSensors; i++) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    distances[i] = (highByte << 8) | lowByte;
  }
}
```
We originally sent ASCII strings like "123,45,67\n". Noise caused lockups and partial reads, so we switched to binary packets.

2) Motor Control
The motor driver uses two pins for direction and one PWM pin for speed.
```cpp
Copy
Edit
const int ENA = 5;
const int IN1 = 7;
const int IN2 = 6;

void forward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

void backward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}
```

At one point, the motor only spun in one direction. We discovered IN1 and IN2 were wired backwards. A quick swap fixed it.

3) Steering Logic
The UNO adjusts steering angles based on distances.
```cpp
Copy
Edit
if (frontDist > 0 && frontDist < 70) {
  if (leftDist > rightDist) {
    steeringServo.write(180); // hard left
    forward(speedT);
    delay(800);
  } else {
    steeringServo.write(110); // hard right
    forward(speedT);
    delay(800);
  }
}

if (leftDist < 30) {
  steeringServo.write(165); // soft left
  forward(speedT);
  delay(60);
}

if (rightDist < 30) {
  steeringServo.write(125); // soft right
  forward(speedT);
  delay(60);
} else {
  steeringServo.write(150); // center
  forward(speedF);
}
```
We taped a ruler in front of the robot and tested turning behavior. Below ~70 cm, the robot needed decisive turns. At ~30 cm on the sides, small corrections kept it centered.

4) Servo Setup
```cpp
Copy
Edit
#include <Servo.h>
Servo steeringServo;

void setup() {
  steeringServo.attach(10);
  steeringServo.write(150); // start centered
}
```
Pin 10 avoided timer conflicts. Centering at boot prevents sudden jerks on power-up.

CODE 2: MEGA (Master) — Reads Sensors and Sends Data

1) Ultrasonic Reads
```cpp
Copy
Edit
int ultrasonicRead(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseInLong(echoPin, HIGH, 25000); // 25 ms timeout

  if (duration == 0 || duration > 25000) return -1;
  return duration / 58; // convert µs to cm
}
```

Timeouts were critical. Without them, a missing echo stalled the entire loop.

2) Debounce Strategy

```cpp
Copy
Edit
int minDistance(int trig, int echo) {
  int d1 = ultrasonicRead(trig, echo);
  delayMicroseconds(500);
  int d2 = ultrasonicRead(trig, echo);

  if (d1 == -1) d1 = MAX_DISTANCE_CM;
  if (d2 == -1) d2 = MAX_DISTANCE_CM;

  return (d1 < d2) ? d1 : d2;
}
```

Two quick reads → keep the smaller. Ultrasonics often return false long distances, so we biased toward the safer short value.

3) I²C Transmission

```cpp
Copy
Edit
Wire.begin(); // MEGA as master

for (int i = 0; i < numSensors; i++) {
  distances[i] = minDistance(trigPins[i], echoPins[i]);
}

Wire.beginTransmission(slaveAddress);
for (int i = 0; i < numSensors; i++) {
  Wire.write(highByte(distances[i]));
  Wire.write(lowByte(distances[i]));
}
Wire.endTransmission();
```

We learned the hard way that writing an int directly can truncate on some boards. Explicitly sending high and low bytes is safer.

---

# Calibration and Field Notes

- Center angle: Wheels aligned manually, servo adjusted until visually straight. Stored as CENTER.
- Speed tiers: One forward speed (speedF) and one turning speed (speedT) worked best.
- Sensor placement: Front sensor tilted slightly downward to avoid ceiling echoes. Side sensors mounted parallel to walls.

## Troubleshooting Notes

| Symptom                 | Cause                          | Fix                                                      |
|--------------------------|--------------------------------|----------------------------------------------------------|
| Servo twitches randomly | Power noise                    | Separate 5V for logic and servo, add capacitors          |
| Robot crashes straight  | Sensor miswired or missing echo| Check wiring and confirm timeout                         |
| Wobbly path along wall  | Overcorrection                 | Add deadband or filter readings                          |
| UNO not reacting        | I²C mismatch                   | Confirm slave address 8 and correct 6-byte I²C frame     |
| Motor spins one way only| IN1/IN2 reversed               | Swap motor wires or invert logic in `forward()`/`backward()` |


##Design Decisions and Lessons

- Two-board architecture: Splitting sensing (Mega) and actuation (UNO) eliminated jitter.
- Binary I²C frames: Faster and more reliable than ASCII.
- Simple steering tiers: Hard turns for obstacles, soft nudges for walls. Reliable under time pressure.
- Future work: Explore PID steering, more efficient drivers (TB6612FNG), and sensor fusion with PixyCam.

--- 

**Key Features:**  
- **PID Control:** Keeps the robot driving straight and stabilizes speed.  
- **PixyCam Color Tracking:** Detects red and green cubes and sends data to Arduino via UART.  
- **Sonar Distance Tracking:** Prevents collisions by measuring proximity to walls and objects.  
- **Sensor Fusion:** Integrates PixyCam and sonar readings for reliable decision-making.  

**Why this approach?**
It allows us to achieve competitive performance using **low-cost, lightweight algorithms** that do not require heavy computation or expensive hardware.  

---

## Robot Construction Guide  

**Step 0:** 3D print all chassis and steering components.  
**Step 1:** Mount servo to the front chassis and connect steering linkage.  
**Step 2:** Install encoder motor in a direct drive configuration.  
**Step 3:** Mount the L298N motor driver and wire to motor.  
**Step 4:** Secure Arduino Mega and Arduino Uno; connect through buck converters.  
**Step 5:** Install LiPo battery and verify power distribution.  
**Step 6:** Mount sonar sensors at the front and sides; wire to Arduino Mega.  
**Step 7:** Attach PixyCam at front with slight upward angle for visibility.  
**Step 8:** Verify connections, secure wiring with zip ties, and upload code.  

---

## System Diagrams  

### Block Diagram – Electronics  




---
## Media and Resources  

- YouTube demo video: *link to be added*  
- CAD models (`/cad`)  
- Wiring diagrams (`/docs`)  
- Photos & videos (`/media`)  

**Social Media:**  
- [YouTube Channel](https://www.youtube.com/channel/UC77UWzOkk5sxfCZbkdylfGg)  
- [Instagram Page](https://www.instagram.com/roboticsattrinity/)  

**Key Resources:**  
- [Arduino Mega](https://store.arduino.cc/products/arduino-mega-2560-rev3)  
- [Arduino Uno](https://store.arduino.cc/products/arduino-uno-rev3)  
- [Pixy Camera](https://pixycam.com/)  
- [L298N Motor Driver](https://www.sparkfun.com/products/14450)  
- [HC-SR04 Ultrasonic Sensor](https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04/)  

---

## Future Improvements  

With additional resources, we would improve Roboinity by:  
- Replacing L298N with **TB6612FNG** or custom PCB for higher efficiency.  
- Using **brushless DC motors** for better performance and efficiency.  
- Integrating a **Raspberry Pi** or **OpenMV camera** for advanced vision processing.  
- Adding **suspension and modular chassis design** for more stability.  
- Using higher-capacity LiPo batteries for longer runtime.  

---



