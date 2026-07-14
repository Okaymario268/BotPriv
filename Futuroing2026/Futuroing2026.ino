#include <Servo.h>
 
// Pins for Motor
int MotorVelocityPin = 3; 
int MotorPin1 = 6;
int MotorPin2 = 5;

// Motor Values
int CurrentVelocity = 150;

// ServoMotor Values

int CenterServoRotation = 138;
int LeftServoRotation = 125;
int RightServoRotation = 150;



// Set the max range for the ultrasonic
int FrontDistanceRange = 35;
int LeftDistanceRange = 50;
int RightDistanceRange = 50;

// ServoMotor
int ServoPin = 11;
 
// Debug
bool Debug = false;
bool ButtonEnabled = true; // Enable or disable button for testing purpose
bool EnableObstacleChallenge  = true;
 
int MaxYellowToDetect = 12;
 
Servo servo;
 
void MotorBackward() {
  digitalWrite(MotorPin1,LOW);
  digitalWrite(MotorPin2, HIGH);
}

void MotorForward() {
  digitalWrite(MotorPin1,  HIGH);
  digitalWrite(MotorPin2, LOW);
}

void MotorStop() {
  digitalWrite(MotorPin1,LOW);
  digitalWrite(MotorPin2, LOW);
}


void ServoRotation(int Angle) {
  servo.write(Angle);
}

void setup() {
  // Init Motor 
 
  pinMode (MotorPin1, OUTPUT);
  pinMode (MotorPin2, OUTPUT);
  pinMode(MotorVelocityPin, OUTPUT);
  analogWrite(MotorVelocityPin, CurrentVelocity);
  /// Init Servo Motor 
 
  
  Serial.begin(9600);
  servo.attach(ServoPin);
  servo.write(96);
 

}

void loop() {
     
}