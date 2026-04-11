#include "servo_control.h"

static Servo servo1;
static Servo servo2;

static int servo1Angle = SERVO_DEFAULT_ANGLE;
static int servo2Angle = SERVO_DEFAULT_ANGLE;

static int clampServoAngle(int angle) {
  if (angle < SERVO_MIN_ANGLE) {
    return SERVO_MIN_ANGLE;
  }
  if (angle > SERVO_MAX_ANGLE) {
    return SERVO_MAX_ANGLE;
  }
  return angle;
}

void initServos() {
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  setServo1Angle(SERVO_DEFAULT_ANGLE);
  setServo2Angle(SERVO_DEFAULT_ANGLE);

  Serial.print(F("DEBUG: Servos initialized (S1="));
  Serial.print(servo1Angle);
  Serial.print(F(", S2="));
  Serial.print(servo2Angle);
  Serial.println(F(")"));
}

void setServo1Angle(int angle) {
  servo1Angle = clampServoAngle(angle);
  servo1.write(servo1Angle);

  Serial.print(F("DEBUG: Servo 1 angle -> "));
  Serial.println(servo1Angle);
}

void setServo2Angle(int angle) {
  servo2Angle = clampServoAngle(angle);
  servo2.write(servo2Angle);

  Serial.print(F("DEBUG: Servo 2 angle -> "));
  Serial.println(servo2Angle);
}

void adjustServo1(int delta) {
  setServo1Angle(servo1Angle + delta);
}

void adjustServo2(int delta) {
  setServo2Angle(servo2Angle + delta);
}

int getServo1Angle() {
  return servo1Angle;
}

int getServo2Angle() {
  return servo2Angle;
}
