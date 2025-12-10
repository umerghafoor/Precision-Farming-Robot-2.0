#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "constants.h"

/**
 * Motor Control Module
 * Handles all L298N motor driver operations for 4-wheel drive system
 */

// Motor control structure
struct MotorControl {
  uint8_t direction;
  uint8_t speed;
};

// Initialize all motor pins
void initMotors();

// Stop all motors immediately
void stopAllMotors();

// Individual motor control functions
void setMotor1(uint8_t direction, uint8_t speed);
void setMotor2(uint8_t direction, uint8_t speed);
void setMotor3(uint8_t direction, uint8_t speed);
void setMotor4(uint8_t direction, uint8_t speed);

// Convenience functions for common movements
void moveForward(uint8_t speed);
void moveBackward(uint8_t speed);
void turnLeft(uint8_t speed);
void turnRight(uint8_t speed);
void rotateClockwise(uint8_t speed);
void rotateCounterClockwise(uint8_t speed);

#endif
