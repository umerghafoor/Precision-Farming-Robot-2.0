#include "motor_control.h"

void initMotors() {
  Serial.println("DEBUG: Initializing Motor 1 pins...");
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  
  Serial.println("DEBUG: Initializing Motor 2 pins...");
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);
  
  Serial.println("DEBUG: Initializing Motor 3 pins...");
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);
  
  Serial.println("DEBUG: Initializing Motor 4 pins...");
  pinMode(MOTOR4_IN1, OUTPUT);
  pinMode(MOTOR4_IN2, OUTPUT);
  pinMode(MOTOR4_ENB, OUTPUT);
  
  Serial.println("DEBUG: Motor pins initialized");
}

void stopAllMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 0);
  
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  analogWrite(MOTOR2_ENB, 0);
  
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  analogWrite(MOTOR3_ENA, 0);
  
  digitalWrite(MOTOR4_IN1, LOW);
  digitalWrite(MOTOR4_IN2, LOW);
  analogWrite(MOTOR4_ENB, 0);
  
  Serial.println("DEBUG: All motors stopped");
}

void setMotor1(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 1 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
      analogWrite(MOTOR1_ENA, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
      analogWrite(MOTOR1_ENA, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
      analogWrite(MOTOR1_ENA, 0);
      Serial.println("  -> STOP");
      break;
  }
}

void setMotor2(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 2 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR2_IN1, HIGH);
      digitalWrite(MOTOR2_IN2, LOW);
      analogWrite(MOTOR2_ENB, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
      analogWrite(MOTOR2_ENB, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);
      analogWrite(MOTOR2_ENB, 0);
      Serial.println("  -> STOP");
      break;
  }
}

void setMotor3(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 3 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR3_IN1, HIGH);
      digitalWrite(MOTOR3_IN2, LOW);
      analogWrite(MOTOR3_ENA, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, HIGH);
      analogWrite(MOTOR3_ENA, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, LOW);
      analogWrite(MOTOR3_ENA, 0);
      Serial.println("  -> STOP");
      break;
  }
}

void setMotor4(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 4 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR4_IN1, HIGH);
      digitalWrite(MOTOR4_IN2, LOW);
      analogWrite(MOTOR4_ENB, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR4_IN1, LOW);
      digitalWrite(MOTOR4_IN2, HIGH);
      analogWrite(MOTOR4_ENB, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR4_IN1, LOW);
      digitalWrite(MOTOR4_IN2, LOW);
      analogWrite(MOTOR4_ENB, 0);
      Serial.println("  -> STOP");
      break;
  }
}

// Convenience movement functions
void moveForward(uint8_t speed) {
  Serial.println("DEBUG: Moving FORWARD");
  setMotor1(DIR_FORWARD, speed);
  setMotor2(DIR_FORWARD, speed);
  setMotor3(DIR_FORWARD, speed);
  setMotor4(DIR_FORWARD, speed);
}

void moveBackward(uint8_t speed) {
  Serial.println("DEBUG: Moving BACKWARD");
  setMotor1(DIR_BACKWARD, speed);
  setMotor2(DIR_BACKWARD, speed);
  setMotor3(DIR_BACKWARD, speed);
  setMotor4(DIR_BACKWARD, speed);
}

void turnLeft(uint8_t speed) {
  Serial.println("DEBUG: Turning LEFT");
  setMotor1(DIR_BACKWARD, speed);
  setMotor2(DIR_FORWARD, speed);
  setMotor3(DIR_BACKWARD, speed);
  setMotor4(DIR_FORWARD, speed);
}

void turnRight(uint8_t speed) {
  Serial.println("DEBUG: Turning RIGHT");
  setMotor1(DIR_FORWARD, speed);
  setMotor2(DIR_BACKWARD, speed);
  setMotor3(DIR_FORWARD, speed);
  setMotor4(DIR_BACKWARD, speed);
}

void rotateClockwise(uint8_t speed) {
  Serial.println("DEBUG: Rotating CLOCKWISE");
  setMotor1(DIR_FORWARD, speed);
  setMotor2(DIR_BACKWARD, speed);
  setMotor3(DIR_FORWARD, speed);
  setMotor4(DIR_BACKWARD, speed);
}

void rotateCounterClockwise(uint8_t speed) {
  Serial.println("DEBUG: Rotating COUNTER-CLOCKWISE");
  setMotor1(DIR_BACKWARD, speed);
  setMotor2(DIR_FORWARD, speed);
  setMotor3(DIR_BACKWARD, speed);
  setMotor4(DIR_FORWARD, speed);
}
