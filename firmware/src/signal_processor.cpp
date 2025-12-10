#include "signal_processor.h"

// 6D Signal buffer
byte signalBuffer[SIGNAL_SIZE];
byte bufferIndex = 0;
bool dataReady = false;

// Motor control array
MotorControl motors[3];

void process6DSignal() {
  Serial.println("\n--- Processing 6D Signal ---");
  Serial.print("RAW DATA: ");
  for (int i = 0; i < SIGNAL_SIZE; i++) {
    Serial.print("0x");
    if (signalBuffer[i] < 16) Serial.print("0");
    Serial.print(signalBuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\n");
  
  // Parse 6D signal into motor controls
  // Format: [Dir1, Speed1, Dir2, Speed2, Dir3, Speed3]
  motors[0].direction = signalBuffer[0];
  motors[0].speed = signalBuffer[1];
  
  motors[1].direction = signalBuffer[2];
  motors[1].speed = signalBuffer[3];
  
  motors[2].direction = signalBuffer[4];
  motors[2].speed = signalBuffer[5];
  
  // Apply to L298N motor drivers
  Serial.println("Applying motor controls:");
  setMotor1(motors[0].direction, motors[0].speed);
  setMotor2(motors[1].direction, motors[1].speed);
  setMotor3(motors[2].direction, motors[2].speed);
  
  // Motor 4 mirrors Motor 2 for 4-wheel drive
  setMotor4(motors[1].direction, motors[1].speed);
  
  Serial.println("--- Signal Processing Complete ---\n");
}

void checkSerialData() {
  // Check if we have data available on Serial
  if (Serial.available() >= SIGNAL_SIZE) {
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("DEBUG: Reading 6D signal from Serial...");
    
    // Read 6 bytes from Serial into buffer
    for (int i = 0; i < SIGNAL_SIZE; i++) {
      signalBuffer[i] = Serial.read();
    }
    
    Serial.println("DEBUG: Complete 6D signal received!");
    
    // Process the signal
    process6DSignal();
    
    digitalWrite(LED_PIN, LOW);
  }
  // Check for text commands
  else if (Serial.available() > 0) {
    // Check if it's a text command (starts with a letter)
    char first = Serial.peek();
    
    if (first >= 'A' && first <= 'Z') {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      handleTextCommand(cmd);
    } else {
      // Not enough bytes yet, wait
      delay(10);
    }
  }
}

void handleTextCommand(String cmd) {
  Serial.print("DEBUG: Serial command received: ");
  Serial.println(cmd);
  
  if (cmd == "STOP") {
    Serial.println("DEBUG: Emergency stop commanded!");
    stopAllMotors();
  } else if (cmd == "TEST") {
    Serial.println("DEBUG: Running test sequence...");
    // Test sequence
    setMotor1(DIR_FORWARD, 128);
    delay(2000);
    setMotor1(DIR_STOP, 0);
    delay(500);
    setMotor1(DIR_BACKWARD, 128);
    delay(2000);
    stopAllMotors();
    Serial.println("DEBUG: Test complete");
  } else if (cmd == "STATUS") {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.print("Buffer Index: ");
    Serial.println(bufferIndex);
    Serial.println("Last 6D Signal:");
    for (int i = 0; i < SIGNAL_SIZE; i++) {
      Serial.print("  Byte ");
      Serial.print(i);
      Serial.print(": 0x");
      Serial.println(signalBuffer[i], HEX);
    }
    Serial.println("=====================\n");
  } else if (cmd == "FORWARD") {
    Serial.println("DEBUG: Moving forward");
    moveForward(180);
  } else if (cmd == "BACKWARD") {
    Serial.println("DEBUG: Moving backward");
    moveBackward(180);
  } else if (cmd == "LEFT") {
    Serial.println("DEBUG: Turning left");
    turnLeft(150);
  } else if (cmd == "RIGHT") {
    Serial.println("DEBUG: Turning right");
    turnRight(150);
  } else {
    Serial.println("DEBUG: Unknown command. Available: STOP, TEST, STATUS, FORWARD, BACKWARD, LEFT, RIGHT");
  }
}
