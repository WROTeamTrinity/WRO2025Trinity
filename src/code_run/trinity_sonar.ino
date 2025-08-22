#include <Wire.h>

const int numSensors = 3;
int trigPins[numSensors] = {4, 6, 8};
int echoPins[numSensors] = {5, 7, 9};
int distances[numSensors];
const int MAX_DISTANCE_CM = 400;  // Use this instead of -1 if timeout

const int slaveAddress = 8;  // I2C address of Uno (slave)

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Mega as Master
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }
}

void loop() {
  // Read distances from ultrasonic sensors
  for (int i = 0; i < numSensors; i++) {
    distances[i] = minDistance(trigPins[i], echoPins[i]);
  }

  // Debug print
  Serial.print("Distances sent: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(distances[i]);
    if (i < numSensors - 1) Serial.print(", ");
  }
  Serial.println();

  // Send distances to Uno over I2C (2 bytes per sensor)
  Wire.beginTransmission(slaveAddress);
  for (int i = 0; i < numSensors; i++) {
    Wire.write(highByte(distances[i]));
    Wire.write(lowByte(distances[i]));
  }
  Wire.endTransmission();

  delay(100);  // small delay
}

// Take 2 ultrasonic readings and return the smaller
int minDistance(int trig, int echo) {
  int d1 = ultrasonicRead(trig, echo);
  delayMicroseconds(500);
  int d2 = ultrasonicRead(trig, echo);

  if (d1 == -1) d1 = MAX_DISTANCE_CM;
  if (d2 == -1) d2 = MAX_DISTANCE_CM;

  return (d1 < d2) ? d1 : d2;
}

// Trigger ultrasonic sensor and get distance in cm
int ultrasonicRead(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseInLong(echoPin, HIGH, 25000); // 25ms timeout (~4m)
  if (duration == 0 || duration > 25000) return -1;

  return duration / 58; // convert to cm
}