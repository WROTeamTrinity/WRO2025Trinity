#include <Wire.h>
#include <Servo.h>

const int numSensors = 3; // 0=front,1=left,2=right
int distances[numSensors];

// Pixy2 data
int redX = -1, redY = -1;
int greenX = -1, greenY = -1;

// Motor pins
const int ENA = 5;  // PWM speed
const int IN1 = 7;
const int IN2 = 6;

// Servo
Servo steeringServo;

const int SERVO_LEFT   = 180;
const int SERVO_CENTER = 150;
const int SERVO_RIGHT  = 110;

// Speeds (0-255)
const int speedF = 250;  // forward speed
const int speedB = 130;  // backward speed
const int speedT = 220;  // turning speed

void setup() {
  Serial.begin(9600);         
  Wire.begin(8);              // I2C slave address
  Wire.onReceive(receiveEvent);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  steeringServo.attach(10);
  stopMotor();
  steeringServo.write(SERVO_CENTER);
}

void loop() {
  // Debug print
  Serial.print("Front: "); Serial.print(distances[0]);
  Serial.print(", Right: "); Serial.print(distances[1]);
  Serial.print(", Left: "); Serial.print(distances[2]);
  Serial.print(" | Red: ");
  if (redX != -1) Serial.print("(" + String(redX) + "," + String(redY) + ")");
  else Serial.print("none");
  Serial.print(" | Green: ");
  if (greenX != -1) Serial.println("(" + String(greenX) + "," + String(greenY) + ")");
  else Serial.println("none");

  int frontDist = distances[0];
  int leftDist  = distances[1];
  int rightDist = distances[2];

  // ============================
  // 1) If Pixy sees red/green
  // ============================
  if (redX != -1) {
    // Red object → Turn RIGHT
    steeringServo.write(SERVO_RIGHT);
    forward(speedT);
    delay(600);
    steeringServo.write(SERVO_CENTER);
  }
  else if (greenX != -1) {
    // Green object → Turn LEFT
    steeringServo.write(SERVO_LEFT);
    forward(speedT);
    delay(600);
    steeringServo.write(SERVO_CENTER);
  }
  else {
    // ============================
    // 2) No Pixy target → Use distance sensors
    // ============================
    if (frontDist > 0 && frontDist < 70) {
      // Corner turning
      if (leftDist > rightDist) {
        steeringServo.write(SERVO_LEFT);
        forward(speedT);
        delay(800);
      } else {
        steeringServo.write(SERVO_RIGHT);
        forward(speedT);
        delay(800);
      }
    } 
    else if (leftDist < 30) {
      steeringServo.write(SERVO_LEFT);
      forward(speedT);
      delay(60);
    }
    else if (rightDist < 30) {
      steeringServo.write(SERVO_RIGHT);
      forward(speedT);
      delay(60);
    }
    else {
      // Free path → go forward
      steeringServo.write(SERVO_CENTER);
      forward(speedF);
    }
  }
}

// ====================
// I2C Receive
// ====================
void receiveEvent(int howMany) {
  if (howMany < (numSensors * 2 + 8)) return;

  // Distances
  for (int i = 0; i < numSensors; i++) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    distances[i] = (highByte << 8) | lowByte;
  }

  // Red X, Y
  int rxHigh = Wire.read(); int rxLow = Wire.read();
  int ryHigh = Wire.read(); int ryLow = Wire.read();
  redX = (rxHigh << 8) | rxLow;
  redY = (ryHigh << 8) | ryLow;
  if (redX == 65535) redX = -1;

  // Green X, Y
  int gxHigh = Wire.read(); int gxLow = Wire.read();
  int gyHigh = Wire.read(); int gyLow = Wire.read();
  greenX = (gxHigh << 8) | gxLow;
  greenY = (gyHigh << 8) | gyLow;
  if (greenX == 65535) greenX = -1;
}

// ====================
// Motor control
// ====================
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
