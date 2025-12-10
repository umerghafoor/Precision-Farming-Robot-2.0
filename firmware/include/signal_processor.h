#ifndef SIGNAL_PROCESSOR_H
#define SIGNAL_PROCESSOR_H

#include <Arduino.h>
#include "constants.h"
#include "motor_control.h"

/**
 * Signal Processor Module
 * Handles receiving and processing 6D signals from Serial
 */

// 6D Signal buffer
extern byte signalBuffer[SIGNAL_SIZE];
extern byte bufferIndex;
extern bool dataReady;

// Motor control array
extern MotorControl motors[3];

// Process the received 6D signal and apply to motors
void process6DSignal();

// Check for and read incoming serial data
void checkSerialData();

// Handle text commands from serial
void handleTextCommand(String cmd);

#endif
