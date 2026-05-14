#include "motor_control.h"

AF_DCMotor motor1(MOTOR1_INDEX);
AF_DCMotor motor2(MOTOR2_INDEX);
AF_DCMotor motor3(MOTOR3_INDEX);
AF_DCMotor motor4(MOTOR4_INDEX);

static uint8_t clampDirection(uint8_t direction) {
  if (direction == DIR_FORWARD || direction == DIR_BACKWARD || direction == DIR_STOP) {
    return direction;
  }
  return DIR_STOP;
}

static void applyMotorCommand(AF_DCMotor &motor, uint8_t direction, uint8_t speed, const __FlashStringHelper *label) {
  const uint8_t safeDirection = clampDirection(direction);

  Serial.print(F("DEBUG: "));
  Serial.print(label);
  Serial.print(F(" -> Dir: "));
  Serial.print(safeDirection);
  Serial.print(F(", Speed: "));
  Serial.println(speed);

  if (safeDirection == DIR_FORWARD) {
    motor.setSpeed(speed);
    motor.run(FORWARD);
    Serial.println(F("  -> FORWARD"));
    return;
  }

  if (safeDirection == DIR_BACKWARD) {
    motor.setSpeed(speed);
    motor.run(BACKWARD);
    Serial.println(F("  -> BACKWARD"));
    return;
  }

  motor.setSpeed(0);
  motor.run(RELEASE);
  Serial.println(F("  -> STOP"));
}

void initMotors() {
  Serial.println(F("DEBUG: Initializing AFMotor channels..."));
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  Serial.println(F("DEBUG: AFMotor channels initialized"));
}

void stopAllMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);

  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  Serial.println(F("DEBUG: All motors stopped"));
}

void setMotor1(uint8_t direction, uint8_t speed) {
  applyMotorCommand(motor1, direction, speed, F("Motor 1"));
}

void setMotor2(uint8_t direction, uint8_t speed) {
  applyMotorCommand(motor2, direction, speed, F("Motor 2"));
}

void setMotor3(uint8_t direction, uint8_t speed) {
  applyMotorCommand(motor3, direction, speed, F("Motor 3"));
}

void setMotor4(uint8_t direction, uint8_t speed) {
  applyMotorCommand(motor4, direction, speed, F("Motor 4"));
}

// Convenience movement functions
void moveForward(uint8_t speed) {
  Serial.println(F("DEBUG: Moving FORWARD"));
  setMotor1(DIR_FORWARD, speed);
  setMotor2(DIR_FORWARD, speed);
  setMotor3(DIR_FORWARD, speed);
  setMotor4(DIR_FORWARD, speed);
}

void moveBackward(uint8_t speed) {
  Serial.println(F("DEBUG: Moving BACKWARD"));
  setMotor1(DIR_BACKWARD, speed);
  setMotor2(DIR_BACKWARD, speed);
  setMotor3(DIR_BACKWARD, speed);
  setMotor4(DIR_BACKWARD, speed);
}

void turnLeft(uint8_t speed) {
  Serial.println(F("DEBUG: Turning LEFT"));
  setMotor1(DIR_BACKWARD, speed);
  setMotor2(DIR_FORWARD, speed);
  setMotor3(DIR_BACKWARD, speed);
  setMotor4(DIR_FORWARD, speed);
}

void turnRight(uint8_t speed) {
  Serial.println(F("DEBUG: Turning RIGHT"));
  setMotor1(DIR_FORWARD, speed);
  setMotor2(DIR_BACKWARD, speed);
  setMotor3(DIR_FORWARD, speed);
  setMotor4(DIR_BACKWARD, speed);
}

void rotateClockwise(uint8_t speed) {
  Serial.println(F("DEBUG: Rotating CLOCKWISE"));
  setMotor1(DIR_FORWARD, speed);
  setMotor2(DIR_BACKWARD, speed);
  setMotor3(DIR_FORWARD, speed);
  setMotor4(DIR_BACKWARD, speed);
}

void rotateCounterClockwise(uint8_t speed) {
  Serial.println(F("DEBUG: Rotating COUNTER-CLOCKWISE"));
  setMotor1(DIR_BACKWARD, speed);
  setMotor2(DIR_FORWARD, speed);
  setMotor3(DIR_BACKWARD, speed);
  setMotor4(DIR_FORWARD, speed);
}
