#include <Arduino.h>
#include "constants.h"

// 6D Signal buffer
byte signalBuffer[SIGNAL_SIZE];
byte bufferIndex = 0;
bool dataReady = false;

// Motor control structure
struct MotorControl {
  uint8_t direction;
  uint8_t speed;
};

MotorControl motors[3]; // 3 motors from 6D signal (can be extended to 4)

void stopAllMotors();
void setMotor1(uint8_t direction, uint8_t speed);
void setMotor2(uint8_t direction, uint8_t speed);
void setMotor3(uint8_t direction, uint8_t speed);
void setMotor4(uint8_t direction, uint8_t speed);
void process6DSignal();

void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("Four Wheel Drive - USB Serial Motor Controller");
  Serial.println("========================================\n");
  
  // Initialize motor pins as outputs
  Serial.println("DEBUG: Initializing Motor 1 pins...");
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);
  
  Serial.println("DEBUG: Initializing Motor 2 pins...");
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);
  
  Serial.println("DEBUG: Initializing Motor 3 pins...");
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);
  
  Serial.println("DEBUG: Initializing Motor 4 pins...");
  pinMode(MOTOR4_IN1, OUTPUT);
  pinMode(MOTOR4_IN2, OUTPUT);
  pinMode(MOTOR4_ENB, OUTPUT);
  
  // Stop all motors initially
  Serial.println("DEBUG: Stopping all motors...");
  stopAllMotors();
  
  Serial.println("\nDEBUG: USB Serial initialized successfully");
  Serial.println("DEBUG: Waiting for 6D signal data via Serial...\n");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}



void stopAllMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  analogWrite(MOTOR1_ENA, 0);
  
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  analogWrite(MOTOR2_ENB, 0);
  
  digitalWrite(MOTOR3_IN1, LOW);
  digitalWrite(MOTOR3_IN2, LOW);
  analogWrite(MOTOR3_ENA, 0);
  
  digitalWrite(MOTOR4_IN1, LOW);
  digitalWrite(MOTOR4_IN2, LOW);
  analogWrite(MOTOR4_ENB, 0);
  
  Serial.println("DEBUG: All motors stopped");
}

void setMotor1(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 1 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
      analogWrite(MOTOR1_ENA, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
      analogWrite(MOTOR1_ENA, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
      analogWrite(MOTOR1_ENA, 0);
      Serial.println("  -> STOP");
      break;
  }
}

void setMotor2(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 2 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR2_IN1, HIGH);
      digitalWrite(MOTOR2_IN2, LOW);
      analogWrite(MOTOR2_ENB, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
      analogWrite(MOTOR2_ENB, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);
      analogWrite(MOTOR2_ENB, 0);
      Serial.println("  -> STOP");
      break;
  }
}

void setMotor3(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 3 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR3_IN1, HIGH);
      digitalWrite(MOTOR3_IN2, LOW);
      analogWrite(MOTOR3_ENA, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, HIGH);
      analogWrite(MOTOR3_ENA, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, LOW);
      analogWrite(MOTOR3_ENA, 0);
      Serial.println("  -> STOP");
      break;
  }
}

void setMotor4(uint8_t direction, uint8_t speed) {
  Serial.print("DEBUG: Motor 4 -> Dir: ");
  Serial.print(direction);
  Serial.print(", Speed: ");
  Serial.println(speed);
  
  switch(direction) {
    case DIR_FORWARD:
      digitalWrite(MOTOR4_IN1, HIGH);
      digitalWrite(MOTOR4_IN2, LOW);
      analogWrite(MOTOR4_ENB, speed);
      Serial.println("  -> FORWARD");
      break;
    case DIR_BACKWARD:
      digitalWrite(MOTOR4_IN1, LOW);
      digitalWrite(MOTOR4_IN2, HIGH);
      analogWrite(MOTOR4_ENB, speed);
      Serial.println("  -> BACKWARD");
      break;
    case DIR_STOP:
    default:
      digitalWrite(MOTOR4_IN1, LOW);
      digitalWrite(MOTOR4_IN2, LOW);
      analogWrite(MOTOR4_ENB, 0);
      Serial.println("  -> STOP");
      break;
  }
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
  
  // For 4-wheel drive, you might want to mirror motors:
  // Motor 4 mirrors Motor 2 or use similar control
  setMotor4(motors[1].direction, motors[1].speed); // Example: Motor 4 = Motor 2
  
  Serial.println("--- Signal Processing Complete ---\n");
}

void loop() {
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
      } else {
        Serial.println("DEBUG: Unknown command. Available: STOP, TEST, STATUS");
      }
    } else {
      // Not enough bytes yet, wait
      delay(10);
    }
  }
  
  delay(10);
}
