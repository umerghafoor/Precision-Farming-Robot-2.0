/**
 * Four Wheel Drive Motor Controller
 * Main entry point for USB Serial controlled robot
 * 
 * Modules:
 * - motor_control: Handles Adafruit Motor Shield DC motor operations
 * - servo_control: Handles Servo 1 (D10) and Servo 2 (D9)
 * - signal_processor: Processes 6D signals and text commands
 */

#include <Arduino.h>
#include <avr/io.h>
#include "constants.h"
#include "motor_control.h"
#include "servo_control.h"
#include "signal_processor.h"

static void printResetCause() {
  const uint8_t flags = MCUSR;
  MCUSR = 0;

  Serial.print(F("DEBUG: Reset flags=0x"));
  Serial.println(flags, HEX);

  if (flags == 0) {
    Serial.println(F("DEBUG: Reset cause unknown (power already stable)"));
    return;
  }

  if (flags & _BV(PORF)) Serial.println(F("DEBUG: Power-on reset detected"));
  if (flags & _BV(EXTRF)) Serial.println(F("DEBUG: External reset pin triggered"));
  if (flags & _BV(BORF)) Serial.println(F("DEBUG: Brown-out reset detected (possible power dip)"));
  if (flags & _BV(WDRF)) Serial.println(F("DEBUG: Watchdog reset detected"));
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n========================================");
  Serial.println("Four Wheel Drive - USB Serial Motor Controller");
  Serial.println("========================================\n");
  printResetCause();
  
  // Initialize all motor pins
  initMotors();
  initServos();
  
  // Stop all motors initially
  Serial.println("DEBUG: Stopping all motors...");
  stopAllMotors();
  
  Serial.println("\nDEBUG: USB Serial initialized successfully");
  Serial.println("DEBUG: Waiting for 6D signal data via Serial...\n");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}


void loop() {
  // Check for incoming serial data and process it
  checkSerialData();
  
  delay(10);
}
