#ifndef CONSTANTS_H
#define CONSTANTS_H

// L298N Motor Driver 1 Pins (Left Side)
#define MOTOR1_IN1 2
#define MOTOR1_IN2 3
#define MOTOR1_ENA 5  // PWM pin for Motor 1 speed

#define MOTOR2_IN1 4
#define MOTOR2_IN2 7
#define MOTOR2_ENB 6  // PWM pin for Motor 2 speed

// L298N Motor Driver 2 Pins (Right Side)
#define MOTOR3_IN1 8
#define MOTOR3_IN2 9
#define MOTOR3_ENA 10  // PWM pin for Motor 3 speed

#define MOTOR4_IN1 11
#define MOTOR4_IN2 12
#define MOTOR4_ENB 3  // PWM pin for Motor 4 speed

// SPI Hardware Pins on Arduino Nano
// MISO = Pin 12
// MOSI = Pin 11
// SCK = Pin 13
// SS = Pin 10

#define LED_PIN 13

// 6D Signal Structure:
// Byte 0: Motor 1 Direction (0=Forward, 1=Backward, 2=Stop)
// Byte 1: Motor 1 Speed (0-255)
// Byte 2: Motor 2 Direction (0=Forward, 1=Backward, 2=Stop)
// Byte 3: Motor 2 Speed (0-255)
// Byte 4: Motor 3 Direction (0=Forward, 1=Backward, 2=Stop)
// Byte 5: Motor 3 Speed (0-255)

#define DIR_FORWARD 0
#define DIR_BACKWARD 1
#define DIR_STOP 2

#define SIGNAL_SIZE 6

#endif