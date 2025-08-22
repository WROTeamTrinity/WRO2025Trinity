#include <Wire.h>
    #include <Servo.h>

    const int numSensors = 3; // 0=front,1=left,2=right
    int distances[numSensors];

    // Motor\ pins
    const int ENA = 5;  // PWM speed
    const int IN1 = 7;
    const int IN2 = 6;

    // Servo
    Servo steeringServo;

    const int SERVO_LEFT = 100;
    const int SERVO_CENTER = 50;
    const int SERVO_RIGHT = 0;

    // Speeds (0-255)
    const int speedF = 250;  // forward speed
    const int speedB = 130;  // backward speed
  const int speedT = 220;  // turning speed

    int currentSteeringAngle = SERVO_CENTER;  // track current servo angle

    void setup() {
      Serial.begin(9600);         // For debugging Nano receives data or not
      Wire.begin(8);              // I2C slave address
      Wire.onReceive(receiveEvent);

      pinMode(ENA, OUTPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);

      steeringServo.attach(10);
      stopMotor();
      steeringServo.write(150);
    }
void loop() {
  // Debug print distances
  Serial.print("Front: "); Serial.print(distances[0]);
  Serial.print(", Right: "); Serial.print(distances[1]);
  Serial.print(", Left: "); Serial.println(distances[2]);

  int frontDist = distances[0];
  int leftDist  = distances[1];
  int rightDist = distances[2];

  if (frontDist > 0 && frontDist < 70) {
    // --- CORNER TURNING MODE ---
    if (leftDist > rightDist) {
      // Turn left
      steeringServo.write(180);       // Turn left
      forward(speedT);               // Keep motor running
      delay(800);                    // Stay turned for a while
    } 
    else {
      // Turn right
      steeringServo.write(110);      // Turn right
      forward(speedT);               // Keep motor running
      delay(800);                    // Stay turned for a while
    }
  }

  
    if (leftDist < 30) {
      // Turn left
      steeringServo.write(165);       // Turn left
      forward(speedT);               // Keep motor running
      delay(60);                    // Stay turned for a while
    } 
    
    if (rightDist <30) {
      // Turn left
      steeringServo.write(125);       // Turn left
      forward(speedT);               // Keep motor running
      delay(60);                    // Stay turned for a while
    } 
    else{
              // Turn right
      steeringServo.write(150);      // Turn right
      forward(speedF);               // Keep motor running
                    
      
    } }








    void receiveEvent(int howMany) {
      if (howMany < numSensors * 2) return;
      for (int i = 0; i < numSensors; i++) {
        int highByte = Wire.read();
        int lowByte = Wire.read();
        distances[i] = (highByte << 8) | lowByte;
      }
    }

    // Motor control functions
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