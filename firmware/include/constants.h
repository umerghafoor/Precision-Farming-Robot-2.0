#ifndef CONSTANTS_H
#define CONSTANTS_H

// AFMotor channel mapping (Adafruit Motor Shield v1)
#define MOTOR1_INDEX 1
#define MOTOR2_INDEX 2
#define MOTOR3_INDEX 3
#define MOTOR4_INDEX 4

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