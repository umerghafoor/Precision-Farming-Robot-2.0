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
  if (Serial.available() <= 0) {
    return;
  }

  // Prioritize text commands first so they are not consumed as binary payload.
  char first = Serial.peek();
  if ((first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z')) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handleTextCommand(cmd);
    return;
  }

  // Otherwise process binary 6D control packet when enough bytes are present.
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
  } else {
    // Not enough bytes for a binary packet yet.
    delay(10);
  }
}

void handleTextCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  Serial.print("DEBUG: Serial command received: ");
  Serial.println(cmd);
  
  if (cmd == "STOP" || cmd == "X") {
    Serial.println("DEBUG: Emergency stop commanded!");
    stopAllMotors();
  } else if (cmd == "Q") {
    Serial.println("DEBUG: Servo 1 step left");
    adjustServo1(-SERVO_STEP_ANGLE);
  } else if (cmd == "A") {
    Serial.println("DEBUG: Servo 1 step right");
    adjustServo1(SERVO_STEP_ANGLE);
  } else if (cmd == "W") {
    Serial.println("DEBUG: Servo 2 step left");
    adjustServo2(-SERVO_STEP_ANGLE);
  } else if (cmd == "S") {
    Serial.println("DEBUG: Servo 2 step right");
    adjustServo2(SERVO_STEP_ANGLE);
  } else if (cmd == "Z") {
    Serial.println("DEBUG: Servo 2 step left");
    adjustServo2(-SERVO_STEP_ANGLE);
  } else if (cmd == "C") {
    Serial.println("DEBUG: Servo 2 step right");
    adjustServo2(SERVO_STEP_ANGLE);
  } else if (cmd == "CENTER") {
    Serial.println("DEBUG: Centering both servos");
    setServo1Angle(SERVO_DEFAULT_ANGLE);
    setServo2Angle(SERVO_DEFAULT_ANGLE);
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
  } else if (cmd == "LEFT") {
    Serial.println("DEBUG: Turning left");
    turnLeft(150);
  } else if (cmd == "RIGHT") {
    Serial.println("DEBUG: Turning right");
    turnRight(150);
  } else if (cmd == "FORWARD") {
    Serial.println("DEBUG: Moving forward");
    moveForward(180);
  } else if (cmd == "BACKWARD") {
    Serial.println("DEBUG: Moving backward");
    moveBackward(180);
  } else {
    Serial.println("DEBUG: Unknown command. Available: Q, A, W, S, X/STOP, CENTER, Z, C, TEST, STATUS, FORWARD, BACKWARD, LEFT, RIGHT");
  }
}
