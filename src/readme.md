# **CODE 1: UNO (Slave)** – **Controls Motors and Servo**, receives sensor data via I2C

---

### **1. Sensor Input (I2C Communication)**

#### ➤ **Purpose**

Receives distance data (3 sensors: front, left, right) from the master Arduino (Mega) over I2C using the Wire library.

#### ➤ **Setup**

```cpp
Wire.begin(8);  // Set Arduino UNO as I2C slave at address 8
Wire.onReceive(receiveEvent);  // Register function to run when data is received
```

#### ➤ **Function: `receiveEvent()`**

```cpp
void receiveEvent(int howMany) {
  if (howMany < numSensors * 2) return;
  for (int i = 0; i < numSensors; i++) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    distances[i] = (highByte << 8) | lowByte;
  }
}
```

* Each sensor sends 2 bytes (high + low) → 3 sensors = 6 bytes
* Converts 2-byte data into `int` and stores it in `distances[]`

  * `distances[0]` = front
  * `distances[1]` = right
  * `distances[2]` = left

---

### **2. Motor Control (1 DC Motor)**

#### ➤ **Purpose**

Drives a single motor forward or backward using PWM speed and direction pins (via H-bridge or L298N).

#### ➤ **Pins Used**

```cpp
const int ENA = 5;  // PWM pin for speed
const int IN1 = 7;
const int IN2 = 6;
```

#### ➤ **Setup**

```cpp
pinMode(ENA, OUTPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
```

#### ➤ **Functions**

```cpp
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

* **Direction control:** IN1/IN2 determine forward or reverse.
* **Speed control:** ENA uses `analogWrite()` to control speed via PWM.
* **Active braking:** both IN1/IN2 LOW stops the motor.

---

### **3. Main Loop – Autonomous Movement Logic**

#### ➤ **Purpose**

Uses sensor values to decide when to go straight, turn, or avoid obstacles.

#### ➤ **Behavior Flow**

1. Check front distance.

   * If obstacle ahead → turn left or right.
2. Check side distances (left/right).

   * If wall is too close on one side → gently steer away.
3. If no issues → move straight.

#### ➤ **Relevant Logic**

```cpp
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
```

* If front is blocked, robot makes a **hard turn** based on side clearance.

```cpp
if (leftDist < 30) {
  steeringServo.write(165);  // soft left
  forward(speedT);
  delay(60);
} 
```

* If left side is too close → adjust slightly right.

```cpp
if (rightDist < 30) {
  steeringServo.write(125); // soft right
  forward(speedT);
  delay(60);
}
else {
  steeringServo.write(150); // center
  forward(speedF);
}
```

* Otherwise → drive forward centered.

---

### **4. Servo Motor – Steering**

#### ➤ **Purpose**

Controls robot's steering direction.

#### ➤ **Setup**

```cpp
Servo steeringServo;
steeringServo.attach(10);
steeringServo.write(150);  // start centered
```

* Uses `Servo.h` library to control standard servo.
* Pin 10 used for PWM signal to the servo.
* Servo angles adjusted based on movement logic.

---

## Summary for Code 1 (UNO):

| Component        | Summary                                                                                    |
| ---------------- | ------------------------------------------------------------------------------------------ |
| **I2C Receiver** | Receives distances from Mega                                                               |
| **Motor Driver** | Controls speed and direction via PWM & 2 digital pins                                      |
| **Servo Motor**  | Used for steering, turns based on sensor input                                             |
| **Logic**        | If front blocked → turn; if side walls too close → steer slightly away; else → go straight |

---

# **CODE 2: MEGA (Master)** – **Reads Sensors & Sends Data to UNO via I2C**

---

### **1. Ultrasonic Sensor Array**

#### ➤ **Purpose**

Reads distances from 3 HC-SR04 sensors (front, left, right).

#### ➤ **Pin Setup**

```cpp
int trigPins[] = {4, 6, 8};
int echoPins[] = {5, 7, 9};
```

* `trigPins[i]` sends trigger pulse
* `echoPins[i]` listens for return signal

#### ➤ **Setup Function**

```cpp
for (int i = 0; i < numSensors; i++) {
  pinMode(trigPins[i], OUTPUT);
  pinMode(echoPins[i], INPUT);
  digitalWrite(trigPins[i], LOW);
}
```

* Initializes all pins and ensures trigger is low at start.

---

### **2. Ultrasonic Reading Functions**

#### ➤ **Function: `ultrasonicRead()`**

```cpp
int ultrasonicRead(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseInLong(echoPin, HIGH, 25000); // 25ms = ~4 meters

  if (duration == 0 || duration > 25000) return -1;

  return duration / 58; // convert microseconds to cm
}
```

* Sends 10 µs pulse.
* Waits for echo.
* Returns distance in cm.
* Timeout: 25ms to avoid blocking forever.

#### ➤ **Function: `minDistance()`**

```cpp
int minDistance(int trig, int echo) {
  int d1 = ultrasonicRead(trig, echo);
  delayMicroseconds(500);
  int d2 = ultrasonicRead(trig, echo);

  if (d1 == -1) d1 = MAX_DISTANCE_CM;
  if (d2 == -1) d2 = MAX_DISTANCE_CM;

  return (d1 < d2) ? d1 : d2;
}
```

* Reads 2 values per sensor to improve accuracy.
* Returns the smaller of the two (to avoid occasional false long readings).

---

### **3. I2C Master Transmission**

#### ➤ **Setup**

```cpp
Wire.begin();  // Mega as I2C master
```

#### ➤ **Main Loop**

```cpp
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

* Reads all 3 sensors.
* Sends 2 bytes per sensor over I2C to UNO (total 6 bytes).

---

## Summary for Code 2 (MEGA):

| Component              | Summary                                             |
| ---------------------- | --------------------------------------------------- |
| **Ultrasonic Sensors** | 3 sensors (front, left, right) read distance        |
| **Reading Strategy**   | Takes 2 readings, picks the smaller (more reliable) |
| **I2C Transmission**   | Sends 6 bytes (2 per sensor) to UNO slave           |




#### ➤ CAMERA

### 1. Initialization

```cpp
#include <Pixy2.h>
Pixy2 pixy;

void setup() {
  pixy.init();   // Initialize Pixy2 communication
}
```

* `Pixy2 pixy;` creates an object that talks to the camera.
* `pixy.init();` sets up the SPI/I2C/UART connection so Arduino can read detected objects (called *blocks*).

---

### 2. Getting Detected Objects

```cpp
pixy.ccc.getBlocks();   // Ask Pixy2 for latest detected objects (Color Connected Components)
```

* Pixy2 groups same-colored pixels into “**blocks**.”
* Each block has:

  * `m_signature` → which color signature it belongs to (you trained Pixy2 in PixyMon, e.g., Sig 1 = Red, Sig 2 = Green).
  * `m_x, m_y` → center coordinates of the block.
  * `m_width, m_height` → size of the block.

---

### 3. Looping Through Objects

```cpp
if (pixy.ccc.numBlocks) {
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int sig = pixy.ccc.blocks[i].m_signature;
    int x   = pixy.ccc.blocks[i].m_x;
    int y   = pixy.ccc.blocks[i].m_y;
    int w   = pixy.ccc.blocks[i].m_width;
    int h   = pixy.ccc.blocks[i].m_height;
    int area = w * h;
```

* `pixy.ccc.numBlocks` = how many objects are detected right now.
* Each block is read with `.blocks[i]`.
* We extract:

  * **Signature (`sig`)** → is it red (1) or green (2)?
  * **Coordinates (`x,y`)** → where is the object in Pixy’s field of view.
  * **Size (`w*h`)** → helps pick the biggest/closest object.

---

### 4. Filtering by Color

```cpp
if (sig == 1 && area > maxRedArea) {   // Red object
  maxRedArea = area;
  redX = x;
  redY = y;
}
else if (sig == 2 && area > maxGreenArea) {  // Green object
  maxGreenArea = area;
  greenX = x;
  greenY = y;
}
```

* **Signature 1 = Red** → save its biggest detected block.
* **Signature 2 = Green** → save its biggest detected block.
* `redX, redY` and `greenX, greenY` are global variables → later sent to Uno.

This ensures:

* If multiple red/green objects exist, the **largest one (closest)** is chosen.

---

### 5. Sending Pixy Data Over I2C

```cpp
Wire.beginTransmission(slaveAddress);
// Send red coordinates
Wire.write(highByte(redX)); Wire.write(lowByte(redX));
Wire.write(highByte(redY)); Wire.write(lowByte(redY));
// Send green coordinates
Wire.write(highByte(greenX)); Wire.write(lowByte(greenX));
Wire.write(highByte(greenY)); Wire.write(lowByte(greenY));
Wire.endTransmission();
```

* Coordinates are **split into two bytes (high + low)** since Arduino I2C only sends 1 byte at a time.
* Sent to Uno at I2C address `8`.
* On Uno side, these values tell the car whether **red** (turn right) or **green** (turn left) is detected.

---

#### ➤ Putting It All Together

Here’s the **Pixy2 data handling chunk** simplified:

```cpp
// Step 1: Ask Pixy2 for objects
pixy.ccc.getBlocks();

if (pixy.ccc.numBlocks) {
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    int sig = pixy.ccc.blocks[i].m_signature;
    int x   = pixy.ccc.blocks[i].m_x;
    int y   = pixy.ccc.blocks[i].m_y;
    int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;

    if (sig == 1 && area > maxRedArea) { // Red block
      redX = x; redY = y;
    }
    else if (sig == 2 && area > maxGreenArea) { // Green block
      greenX = x; greenY = y;
    }
  }
}

// Step 2: Send best red/green object to Uno
Wire.beginTransmission(slaveAddress);
Wire.write(highByte(redX)); Wire.write(lowByte(redX));
Wire.write(highByte(redY)); Wire.write(lowByte(redY));
Wire.write(highByte(greenX)); Wire.write(lowByte(greenX));
Wire.write(highByte(greenY)); Wire.write(lowByte(greenY));
Wire.endTransmission();
```

---

#### ➤ So the **Pixy2’s role** is:

* Detect **colored objects** (red & green).
* Pick the **closest one** (largest area).
* Send its position `(x,y)` to Uno.

#### ➤ On the **Uno side**:

* If **red seen → turn right**.
* If **green seen → turn left**.
* If **no color seen → rely on distance sensors**.


