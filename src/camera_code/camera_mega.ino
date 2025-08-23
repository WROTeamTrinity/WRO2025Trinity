#include <Wire.h>
#include <Pixy2.h>

// ===== Ultrasonic Setup =====
const int numSensors = 3;
int trigPins[numSensors] = {4, 6, 8};
int echoPins[numSensors] = {5, 7, 9};
int distances[numSensors];
const int MAX_DISTANCE_CM = 400;  // max range

// ===== Pixy2 Setup =====
Pixy2 pixy;
int redX = -1, redY = -1;
int greenX = -1, greenY = -1;

const int slaveAddress = 8;  // I2C slave address of Uno

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Mega as Master

  // Ultrasonic setup
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }

  // Pixy2 init
  pixy.init();
}

void loop() {
  // ===== Ultrasonic readings =====
  for (int i = 0; i < numSensors; i++) {
    distances[i] = minDistance(trigPins[i], echoPins[i]);
  }

  // ===== Pixy2 readings (pick closest object) =====
  redX = redY = -1;
  greenX = greenY = -1;
  int maxRedArea = 0;
  int maxGreenArea = 0;

  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int sig = pixy.ccc.blocks[i].m_signature;
      int x   = pixy.ccc.blocks[i].m_x;
      int y   = pixy.ccc.blocks[i].m_y;
      int w   = pixy.ccc.blocks[i].m_width;
      int h   = pixy.ccc.blocks[i].m_height;
      int area = w * h;

      if (sig == 1 && area > maxRedArea) {   // Signature 1 = Red
        maxRedArea = area;
        redX = x;
        redY = y;
      }
      else if (sig == 2 && area > maxGreenArea) {  // Signature 2 = Green
        maxGreenArea = area;
        greenX = x;
        greenY = y;
      }
    }
  }

  // ===== Debug Print =====
  Serial.print("Distances: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(distances[i]);
    if (i < numSensors - 1) Serial.print(", ");
  }
  Serial.print(" | Red: ");
  if (redX != -1) Serial.print("(" + String(redX) + "," + String(redY) + ")");
  else Serial.print("none");
  Serial.print(" | Green: ");
  if (greenX != -1) Serial.print("(" + String(greenX) + "," + String(greenY) + ")");
  else Serial.print("none");
  Serial.println();

  // ===== Send data over I2C =====
  Wire.beginTransmission(slaveAddress);
  // Send distances
  for (int i = 0; i < numSensors; i++) {
    Wire.write(highByte(distances[i]));
    Wire.write(lowByte(distances[i]));
  }
  // Send red coordinates
  Wire.write(highByte(redX)); Wire.write(lowByte(redX));
  Wire.write(highByte(redY)); Wire.write(lowByte(redY));
  // Send green coordinates
  Wire.write(highByte(greenX)); Wire.write(lowByte(greenX));
  Wire.write(highByte(greenY)); Wire.write(lowByte(greenY));
  Wire.endTransmission();

  delay(100);
}

// ===== Ultrasonic helpers =====
int minDistance(int trig, int echo) {
  int d1 = ultrasonicRead(trig, echo);
  delayMicroseconds(500);
  int d2 = ultrasonicRead(trig, echo);

  if (d1 == -1) d1 = MAX_DISTANCE_CM;
  if (d2 == -1) d2 = MAX_DISTANCE_CM;

  return (d1 < d2) ? d1 : d2;
}

int ultrasonicRead(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseInLong(echoPin, HIGH, 25000); // 25ms timeout
  if (duration == 0 || duration > 25000) return -1;

  return duration / 58; // cm
}
