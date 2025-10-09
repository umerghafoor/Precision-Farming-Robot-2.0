/*
 * Precision Farming Robot 2.0 - Main Controller
 * 
 * This is the main control program for the precision farming robot.
 * It integrates sensors, navigation, and motor control.
 * 
 * Author: Umer Ghafoor
 * License: MIT
 */

#include <Wire.h>
#include <DHT.h>

// Pin Definitions
#define DHT_PIN 7
#define DHT_TYPE DHT22
#define SOIL_MOISTURE_PIN A0
#define MOTOR_LEFT_FWD 9
#define MOTOR_LEFT_BWD 8
#define MOTOR_RIGHT_FWD 11
#define MOTOR_RIGHT_BWD 10
#define TRIG_PIN 6
#define ECHO_PIN 5
#define LED_STATUS 13

// Constants
#define MOISTURE_THRESHOLD 500
#define OBSTACLE_DISTANCE 30
#define MOTOR_SPEED 200

// Objects
DHT dht(DHT_PIN, DHT_TYPE);

// Global Variables
float temperature = 0.0;
float humidity = 0.0;
int soilMoisture = 0;
int distance = 0;
bool obstacleDetected = false;
char command = 'S'; // S=Stop, F=Forward, B=Backward, L=Left, R=Right

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  Serial.println("Precision Farming Robot 2.0");
  Serial.println("Initializing...");
  
  // Initialize Pins
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  // Initialize Sensors
  dht.begin();
  
  // Startup Indication
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_STATUS, HIGH);
    delay(200);
    digitalWrite(LED_STATUS, LOW);
    delay(200);
  }
  
  Serial.println("Initialization Complete!");
  Serial.println("Commands: F=Forward, B=Backward, L=Left, R=Right, S=Stop");
}

void loop() {
  // Read Sensors
  readSensors();
  
  // Check for obstacles
  checkObstacles();
  
  // Read Serial Commands
  if (Serial.available() > 0) {
    command = Serial.read();
  }
  
  // Execute Movement
  if (!obstacleDetected) {
    executeMovement(command);
  } else {
    stopMotors();
    Serial.println("Obstacle detected! Stopping.");
  }
  
  // Log data every 5 seconds
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 5000) {
    logData();
    lastLog = millis();
  }
  
  // Status LED blink
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    lastBlink = millis();
  }
  
  delay(50);
}

void readSensors() {
  // Read DHT22 sensor
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
  // Read soil moisture
  soilMoisture = analogRead(SOIL_MOISTURE_PIN);
  
  // Handle sensor errors
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Warning: DHT sensor read failed!");
    temperature = 0.0;
    humidity = 0.0;
  }
}

void checkObstacles() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  distance = duration * 0.034 / 2; // Convert to cm
  
  // Check if obstacle is too close
  obstacleDetected = (distance > 0 && distance < OBSTACLE_DISTANCE);
}

void executeMovement(char cmd) {
  switch(cmd) {
    case 'F':
    case 'f':
      moveForward();
      break;
    case 'B':
    case 'b':
      moveBackward();
      break;
    case 'L':
    case 'l':
      turnLeft();
      break;
    case 'R':
    case 'r':
      turnRight();
      break;
    case 'S':
    case 's':
    default:
      stopMotors();
      break;
  }
}

void moveForward() {
  analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void moveBackward() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, MOTOR_SPEED);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, MOTOR_SPEED);
}

void turnLeft() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, MOTOR_SPEED);
  analogWrite(MOTOR_RIGHT_FWD, MOTOR_SPEED);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void turnRight() {
  analogWrite(MOTOR_LEFT_FWD, MOTOR_SPEED);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, MOTOR_SPEED);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void logData() {
  Serial.println("=== Sensor Data ===");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Soil Moisture: ");
  Serial.println(soilMoisture);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.println("==================");
}
