#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include <Servo.h>
#include "constants.h"

// Initialize and center both servos.
void initServos();

// Absolute set helpers (angles are constrained to valid servo range).
void setServo1Angle(int angle);
void setServo2Angle(int angle);

// Relative movement helpers.
void adjustServo1(int delta);
void adjustServo2(int delta);

// Stop helpers (especially useful for continuous-rotation servos).
void stopServo1();
void stopServo2();

// Read current cached angles.
int getServo1Angle();
int getServo2Angle();

#endif
