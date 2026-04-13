#include "servo_control.h"

static Servo servo1;
static Servo servo2;

static int servo1Angle = SERVO_DEFAULT_ANGLE;
static int servo2Angle = SERVO_DEFAULT_ANGLE;

static constexpr int SERVO_SLEW_STEP = 2;
static constexpr int SERVO_SLEW_DELAY_MS = 12;

static int applyNeutralDeadband(int target, int neutral) {
  int delta = target - neutral;
  if (delta < 0) {
    delta = -delta;
  }

  if (delta <= SERVO_NEUTRAL_DEADBAND) {
    return neutral;
  }

  return target;
}

static int clampServoAngle(int angle) {
  if (angle < SERVO_MIN_ANGLE) {
    return SERVO_MIN_ANGLE;
  }
  if (angle > SERVO_MAX_ANGLE) {
    return SERVO_MAX_ANGLE;
  }
  return angle;
}

static void writeServoSmooth(Servo &servo, int *currentAngle, int targetAngle) {
  int current = *currentAngle;
  if (current == targetAngle) {
    servo.write(targetAngle);
    return;
  }

  const int direction = (targetAngle > current) ? 1 : -1;
  while (current != targetAngle) {
    current += direction * SERVO_SLEW_STEP;
    if ((direction > 0 && current > targetAngle) || (direction < 0 && current < targetAngle)) {
      current = targetAngle;
    }
    servo.write(current);
    delay(SERVO_SLEW_DELAY_MS);
  }

  *currentAngle = targetAngle;
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
  const int clamped = clampServoAngle(angle);
  const int calibrated = applyNeutralDeadband(clamped, SERVO1_NEUTRAL_ANGLE);
  writeServoSmooth(servo1, &servo1Angle, calibrated);

  Serial.print(F("DEBUG: Servo 1 angle -> "));
  Serial.println(servo1Angle);
}

void setServo2Angle(int angle) {
  const int clamped = clampServoAngle(angle);
  const int calibrated = applyNeutralDeadband(clamped, SERVO2_NEUTRAL_ANGLE);
  writeServoSmooth(servo2, &servo2Angle, calibrated);

  Serial.print(F("DEBUG: Servo 2 angle -> "));
  Serial.println(servo2Angle);
}

void adjustServo1(int delta) {
  setServo1Angle(servo1Angle + delta);
}

void adjustServo2(int delta) {
  setServo2Angle(servo2Angle + delta);
}

void stopServo1() {
  setServo1Angle(SERVO1_NEUTRAL_ANGLE);
}

void stopServo2() {
  setServo2Angle(SERVO2_NEUTRAL_ANGLE);
}

int getServo1Angle() {
  return servo1Angle;
}

int getServo2Angle() {
  return servo2Angle;
}
