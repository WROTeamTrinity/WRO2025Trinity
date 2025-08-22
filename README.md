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

- **Eman Amir** – Team Leader, Programmer, Documentation & Media  
  *I am 18 years old. Driven, hardworking, and resilient. Leads both the technical side and the project narrative, making sure Roboinity’s story is documented as well as its mechanics.*
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/eman.png" alt="eman">

- **Fasih ur Rehman** – Mechanical Design  
  *I am 17 years old. The designer and fabricator of the team, always rethinking mechanical solutions to make the chassis stronger and lighter. Keeps the robot alive under pressure.*  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/fasih.png" alt="fasih">

- **Areena Ihsan** – Team Manager & Media  
  *I am 17 years old. Keeps the team on track and ensures deadlines are met. Handles communication, outreach, and manages media coverage of the team’s journey.*  
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/areena.png" alt="areena">

- **Coach: Jahanzab Aslam Siddique** – Team Coach  
  *Mentor and guide, the backbone of the team. Provides technical advice, strategy feedback, and motivation to push through challenges.*
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/sir.png" alt="sir">  

- **The Team**
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/team.png" alt="team">
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/t-photos/selfie.png" alt="selfie">

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
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Encoder%20Motor%20(Direct%20Drive).jpg" alt="Encoder motor used for direct drive with integrated encoder">
- **Type:** DC encoder motor  
- **Voltage:** 12V  
- **Rated Speed:** ~300 RPM (under load)  
- **Encoder Resolution:** 12 counts/rev  
- **Torque:** ~0.5 kg·cm  
- **Role:** Forward/reverse propulsion with position feedback  
- **Why chosen:** Combines actuation + sensing, fewer moving parts than geared setups, affordable vs. BLDC.

---

### Servo Motor (Steering)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Servo%20Motor%20(Steering).png__i%3DAA" alt="MG90S metal gear servo for steering">
- **Model:** MG90S Metal Gear Servo  
- **Operating Voltage:** 4.8–6V  
- **Stall Torque:** 2.2 kg·cm @ 6V  
- **Speed:** 0.08 sec/60°  
- **Angle Range:** 120°  
- **Role:** Steering mechanism  
- **Why chosen:** Compact, reliable, metal gears, sufficient torque for lightweight 3D-printed steering.

---

### Chassis (3D-Printed)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Chassis%20(3D-Printed).png" alt="3D-printed PLA chassis with modular compartments">
- **Material:** PLA plastic  
- **Layer Height:** 0.2 mm  
- **Infill:** 20%  
- **Features:** Snap-fit joints, modular compartments, ~40% lighter than Lego prototypes  
- **Role:** Structural base for motors, electronics, sensors  
- **Why chosen:** Low-cost, easy to print, rapid iteration/customization.

---

## 2) Power & Sense Management

### Battery
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Battery.jpeg" alt="2S 2200 mAh LiPo battery">
- **Type:** LiPo (Lithium Polymer)  
- **Capacity:** 2200 mAh  
- **Cells:** 2S (7.4V nominal, 8.4V max)  
- **Discharge Rate:** 25–30C  
- **Role:** Main power source  
- **Why chosen:** High current output with manageable weight and solid runtime.

---

### Microcontroller — Arduino Mega 2560
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Microcontroller%20%E2%80%94%20Arduino%20Mega%202560.jpg" alt="Arduino Mega 2560 microcontroller">
- **I/O Pins:** 54 digital, 16 analog  
- **Clock:** 16 MHz  
- **Role:** Central control (motors, PID, steering servo, ultrasonic)  
- **Why chosen:** More I/O/memory for multiple peripherals.

---

### Microcontroller — Arduino Uno
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Microcontroller%20%E2%80%94%20Arduino%20Uno.webp" alt="Arduino Uno microcontroller dedicated to PixyCam">
- **I/O Pins:** 14 digital, 6 analog  
- **Clock:** 16 MHz  
- **Role:** Handles PixyCam and sensor fusion  
- **Why chosen:** Offloads image I/O from the Mega for reliability.

---

## 3) Sensors

### Sonar (Ultrasonic) Sensor
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Sonar%20(Ultrasonic)%20Sensor.jpg" alt="Waterproof ultrasonic sensor (HC-SR04 variant)">
- **Model:** Waterproof HC-SR04 variant  
- **Range:** 2–400 cm  
- **Accuracy:** ±3 mm  
- **Frequency:** 40 kHz  
- **Role:** Obstacle/wall distance measurement  
- **Why chosen:** Inexpensive, lighting-agnostic, waterproof for robustness.

---

### Waterproof Sonar (Ultrasonic) Sensor
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Waterproof%20Sonar%20(Ultrasonic)%20Sensor.webp" alt="Waterproof ultrasonic sensor module (JSN-SR04T or equivalent)">
- **Model:** JSN-SR04T (or A02YYUW equivalent)  
- **Operating Voltage:** 5V  
- **Range:** 25–600 cm  
- **Blind Zone:** ~25 cm  
- **Resolution:** ~1 cm  
- **Frequency:** 40 kHz  
- **Interface:** Trigger / Echo (5V TTL)  
- **Ingress Rating:** Waterproof transducer head (IP66-style)  
- **Role:** Distance sensing in wet/splashy environments or near the floor where spills are likely.  
- **Why chosen:** Remote, potted transducer tolerates moisture where standard ultrasonic modules fail; drop-in Arduino compatible using `pulseIn()` or timers.  

---

### Pixy Camera (Pixy2 + PixyMon)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Pixy%20Camera%20(Pixy2%20%2B%20PixyMon)%20.jpg" alt="Pixy2 smart camera for color-based object tracking">
- **Resolution:** 1296×976  
- **Frame Rate:** 60 fps  
- **Interfaces:** UART / I2C / SPI / USB  
- **Role:** Real-time color object detection/tracking (trained for red/green WRO obstacles)  
- **Why chosen:** On-camera processing reduces code and cost vs. Pi/OpenMV.

---

## 4) Motor Driver

### L298N Dual H-Bridge
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/L298N%20Dual%20H-Bridge.jpg" alt="L298N dual H-bridge motor driver board">
- **Operating Voltage:** 5–35V  
- **Current:** 1A continuous, 2A peak per channel  
- **Role:** DC motor speed/direction control  
- **Why chosen:** Cheap, widely available, Arduino-friendly; sufficient for our motor load.

---

## 5) Power Regulation

### Buck Converter (3A)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Buck%20Converter%20(3A).jpg">
- **Role:** Steps LiPo down to stable 5–6V for servo  
- **Why chosen:** Protects servo from LiPo fluctuations.

---

### Buck-Boost Converter (4A)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Buck-Boost%20Converter%20(4A)%20.webp">
- **Role:** Stable voltage for drive motor across battery discharge curve  
- **Why chosen:** Consistent motor performance from full to low battery.

---

### Additional Buck Converter (5V Rail)
<img src="https://github.com/WROTeamTrinity/WRO2025Trinity/blob/main/Others/README/Additional%20Buck%20Converter%20(5V%20Rail).jpg" alt="5V buck converter for logic/microcontrollers">
- **Role:** Clean, regulated 5V for Arduinos  
- **Why chosen:** Prevents brownouts/overvoltage on sensitive logic.

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

## Code  

Code will be uploaded to the [`/src`](./src) folder.  

**Language:** Arduino C++  

**Key Features:**  
- **PID Control:** Keeps the robot driving straight and stabilizes speed.  
- **PixyCam Color Tracking:** Detects red and green cubes and sends data to Arduino via UART.  
- **Sonar Distance Tracking:** Prevents collisions by measuring proximity to walls and objects.  
- **Sensor Fusion:** Integrates PixyCam and sonar readings for reliable decision-making.  

Why this approach?  
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



