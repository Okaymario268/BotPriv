/*
 * Futuroing2026 - Arduino UNO Q (App Lab) - Servo steering calibration
 * -------------------------------------------------------------------
 * Front steering servo: center / left / right angles.
 *
 * Center was measured at -52 deg. Left and right keep the same throw as the
 * original Futuroing2026.ino (center 138 -> left 125 / right 150, i.e. -13 / +12).
 * Tune the three constants below if the wheels don't reach full lock or bind.
 */
#include <Servo.h>

const int ServoPin = 11;       // servo signal pin (verify it's PWM-capable on the UNO Q)

// Steering angles in degrees
const int CenterAngle = -52;   // straight ahead (your measured center)
const int LeftAngle   = -65;   // CenterAngle - 13  -> turn left
const int RightAngle  = -40;   // CenterAngle + 12  -> turn right

Servo servo;

void ServoRotation(int angle) {
  servo.write(angle);
}

void setup() {
  Serial.begin(9600);
  servo.attach(ServoPin);
  ServoRotation(CenterAngle);  // center on startup
}

void loop() {
  // Calibration demo: center -> left -> center -> right, so you can verify each angle.
  ServoRotation(CenterAngle); delay(1000);
  ServoRotation(LeftAngle);   delay(1000);
  ServoRotation(CenterAngle); delay(1000);
  ServoRotation(RightAngle);  delay(1000);
}
