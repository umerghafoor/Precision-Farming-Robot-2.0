/*
 * Motor Test Example
 * 
 * Tests motor movement patterns
 */

#define MOTOR_LEFT_FWD 9
#define MOTOR_LEFT_BWD 8
#define MOTOR_RIGHT_FWD 11
#define MOTOR_RIGHT_BWD 10
#define MOTOR_SPEED 200

void setup() {
  Serial.begin(115200);
  Serial.println("Motor Test Example");
  
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);
  
  Serial.println("Testing motors in 3 seconds...");
  delay(3000);
}

void loop() {
  // Test Forward
  Serial.println("Moving Forward");
  moveForward();
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Test Backward
  Serial.println("Moving Backward");
  moveBackward();
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Test Left Turn
  Serial.println("Turning Left");
  turnLeft();
  delay(1000);
  stopMotors();
  delay(1000);
  
  // Test Right Turn
  Serial.println("Turning Right");
  turnRight();
  delay(1000);
  stopMotors();
  delay(1000);
  
  Serial.println("Test complete. Repeating...\n");
  delay(2000);
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
