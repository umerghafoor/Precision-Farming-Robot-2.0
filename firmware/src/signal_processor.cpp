#include "signal_processor.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// 6D Signal buffer
byte signalBuffer[SIGNAL_SIZE];
byte bufferIndex = 0;
bool dataReady = false;

// Motor control array
MotorControl motors[3];

static constexpr size_t COMMAND_BUFFER_SIZE = 32;
static char commandBuffer[COMMAND_BUFFER_SIZE];

static bool parseServoAngleCommand(const char *cmd, const char *prefix, int *angleOut) {
  const size_t prefixLen = strlen(prefix);
  if (strncmp(cmd, prefix, prefixLen) != 0) {
    return false;
  }

  const char *value = cmd + prefixLen;
  if (*value == '\0') {
    return false;
  }

  for (size_t i = 0; value[i] != '\0'; i++) {
    char c = value[i];
    if (!isdigit(static_cast<unsigned char>(c)) && !(i == 0 && (c == '+' || c == '-'))) {
      return false;
    }
  }

  *angleOut = atoi(value);
  return true;
}

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
  if (isalpha(static_cast<unsigned char>(first))) {
    const size_t bytesRead = Serial.readBytesUntil('\n', commandBuffer, COMMAND_BUFFER_SIZE - 1);
    commandBuffer[bytesRead] = '\0';
    handleTextCommand(commandBuffer);
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

void handleTextCommand(const char *cmdIn) {
  if (cmdIn == nullptr) {
    return;
  }

  char cmd[COMMAND_BUFFER_SIZE];
  size_t writeIdx = 0;
  for (size_t i = 0; cmdIn[i] != '\0' && writeIdx < COMMAND_BUFFER_SIZE - 1; i++) {
    char c = cmdIn[i];
    if (!isspace(static_cast<unsigned char>(c))) {
      cmd[writeIdx++] = static_cast<char>(toupper(static_cast<unsigned char>(c)));
    }
  }
  cmd[writeIdx] = '\0';

  if (cmd[0] == '\0') {
    return;
  }

  Serial.print("DEBUG: Serial command received: ");
  Serial.println(cmd);

  int requestedAngle = 0;
  if (parseServoAngleCommand(cmd, "S1:", &requestedAngle)) {
    Serial.print("DEBUG: Servo 1 absolute angle request -> ");
    Serial.println(requestedAngle);
    setServo1Angle(requestedAngle);
    return;
  }

  if (parseServoAngleCommand(cmd, "S2:", &requestedAngle)) {
    Serial.print("DEBUG: Servo 2 absolute angle request -> ");
    Serial.println(requestedAngle);
    setServo2Angle(requestedAngle);
    return;
  }

  if (strcmp(cmd, "S1STOP") == 0) {
    Serial.println("DEBUG: Servo 1 stop requested");
    stopServo1();
    return;
  }

  if (strcmp(cmd, "S2STOP") == 0) {
    Serial.println("DEBUG: Servo 2 stop requested");
    stopServo2();
    return;
  }
  
  if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "X") == 0) {
    Serial.println("DEBUG: Emergency stop commanded!");
    stopAllMotors();
  } else if (strcmp(cmd, "Q") == 0) {
    Serial.println("DEBUG: Servo 1 step left");
    adjustServo1(-SERVO_STEP_ANGLE);
  } else if (strcmp(cmd, "A") == 0) {
    Serial.println("DEBUG: Servo 1 step right");
    adjustServo1(SERVO_STEP_ANGLE);
  } else if (strcmp(cmd, "W") == 0) {
    Serial.println("DEBUG: Servo 2 step left");
    adjustServo2(-SERVO_STEP_ANGLE);
  } else if (strcmp(cmd, "S") == 0) {
    Serial.println("DEBUG: Servo 2 step right");
    adjustServo2(SERVO_STEP_ANGLE);
  } else if (strcmp(cmd, "Z") == 0) {
    Serial.println("DEBUG: Servo 2 step left");
    adjustServo2(-SERVO_STEP_ANGLE);
  } else if (strcmp(cmd, "C") == 0) {
    Serial.println("DEBUG: Servo 2 step right");
    adjustServo2(SERVO_STEP_ANGLE);
  } else if (strcmp(cmd, "CENTER") == 0) {
    Serial.println("DEBUG: Centering both servos");
    stopServo1();
    stopServo2();
  } else if (strcmp(cmd, "TEST") == 0) {
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
  } else if (strcmp(cmd, "STATUS") == 0) {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.print("Buffer Index: ");
    Serial.println(bufferIndex);
    Serial.print("Servo 1 angle: ");
    Serial.println(getServo1Angle());
    Serial.print("Servo 2 angle: ");
    Serial.println(getServo2Angle());
    Serial.println("Last 6D Signal:");
    for (int i = 0; i < SIGNAL_SIZE; i++) {
      Serial.print("  Byte ");
      Serial.print(i);
      Serial.print(": 0x");
      Serial.println(signalBuffer[i], HEX);
    }
    Serial.println("=====================\n");
  } else if (strcmp(cmd, "LEFT") == 0) {
    Serial.println("DEBUG: Turning left");
    turnLeft(150);
  } else if (strcmp(cmd, "RIGHT") == 0) {
    Serial.println("DEBUG: Turning right");
    turnRight(150);
  } else if (strcmp(cmd, "FORWARD") == 0) {
    Serial.println("DEBUG: Moving forward");
    moveForward(180);
  } else if (strcmp(cmd, "BACKWARD") == 0) {
    Serial.println("DEBUG: Moving backward");
    moveBackward(180);
  } else {
    Serial.println("DEBUG: Unknown command. Available: Q, A, W, S, S1:<0-180>, S2:<0-180>, S1STOP, S2STOP, X/STOP, CENTER, Z, C, TEST, STATUS, FORWARD, BACKWARD, LEFT, RIGHT");
  }
}
