#ifndef CONSTANTS_H
#define CONSTANTS_H

// L298N Motor Driver 1 Pins (Left Side)
#define MOTOR1_IN1 2   // Digital pin 2 -> Motor 1 IN1
#define MOTOR1_IN2 3   // Digital pin 3 -> Motor 1 IN2
#define MOTOR1_ENA 5   // PWM pin 5 -> Motor 1 speed control

#define MOTOR2_IN1 4   // Digital pin 4 -> Motor 2 IN1
#define MOTOR2_IN2 7   // Digital pin 7 -> Motor 2 IN2
#define MOTOR2_ENB 6   // PWM pin 6 -> Motor 2 speed control

// L298N Motor Driver 2 Pins (Right Side)
#define MOTOR3_IN1 8   // Digital pin 8 -> Motor 3 IN1
#define MOTOR3_IN2 9   // Digital pin 9 -> Motor 3 IN2
#define MOTOR3_ENA 10  // PWM pin 10 -> Motor 3 speed control

#define MOTOR4_IN1 11  // Digital pin 11 -> Motor 4 IN1
#define MOTOR4_IN2 12  // Digital pin 12 -> Motor 4 IN2
#define MOTOR4_ENB A0  // Analog pin A0 used as PWM -> Motor 4 speed control

// SPI Hardware Pins on Arduino Nano
// MISO = Pin 12
// MOSI = Pin 11
// SCK = Pin 13
// SS = Pin 10

#define LED_PIN 13     // Onboard LED

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

#define SIGNAL_SIZE 6   // 6-byte signal: [Dir1, Speed1, Dir2, Speed2, Dir3, Speed3]

#endif