/*
 * Motor Controller Module
 * 
 * Controls DC motors for robot movement
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

class MotorController {
  private:
    int leftFwd, leftBwd, rightFwd, rightBwd;
    int currentSpeed;
    int maxSpeed;
    
  public:
    MotorController(int lf, int lb, int rf, int rb, int maxSpd = 255) {
      leftFwd = lf;
      leftBwd = lb;
      rightFwd = rf;
      rightBwd = rb;
      maxSpeed = maxSpd;
      currentSpeed = 200;
      
      pinMode(leftFwd, OUTPUT);
      pinMode(leftBwd, OUTPUT);
      pinMode(rightFwd, OUTPUT);
      pinMode(rightBwd, OUTPUT);
    }
    
    void setSpeed(int speed) {
      currentSpeed = constrain(speed, 0, maxSpeed);
    }
    
    int getSpeed() {
      return currentSpeed;
    }
    
    void forward() {
      analogWrite(leftFwd, currentSpeed);
      analogWrite(leftBwd, 0);
      analogWrite(rightFwd, currentSpeed);
      analogWrite(rightBwd, 0);
    }
    
    void backward() {
      analogWrite(leftFwd, 0);
      analogWrite(leftBwd, currentSpeed);
      analogWrite(rightFwd, 0);
      analogWrite(rightBwd, currentSpeed);
    }
    
    void turnLeft() {
      analogWrite(leftFwd, 0);
      analogWrite(leftBwd, currentSpeed);
      analogWrite(rightFwd, currentSpeed);
      analogWrite(rightBwd, 0);
    }
    
    void turnRight() {
      analogWrite(leftFwd, currentSpeed);
      analogWrite(leftBwd, 0);
      analogWrite(rightFwd, 0);
      analogWrite(rightBwd, currentSpeed);
    }
    
    void rotateLeft(int speed = -1) {
      int rotSpeed = (speed == -1) ? currentSpeed / 2 : speed;
      analogWrite(leftFwd, 0);
      analogWrite(leftBwd, rotSpeed);
      analogWrite(rightFwd, rotSpeed);
      analogWrite(rightBwd, 0);
    }
    
    void rotateRight(int speed = -1) {
      int rotSpeed = (speed == -1) ? currentSpeed / 2 : speed;
      analogWrite(leftFwd, rotSpeed);
      analogWrite(leftBwd, 0);
      analogWrite(rightFwd, 0);
      analogWrite(rightBwd, rotSpeed);
    }
    
    void stop() {
      analogWrite(leftFwd, 0);
      analogWrite(leftBwd, 0);
      analogWrite(rightFwd, 0);
      analogWrite(rightBwd, 0);
    }
    
    // Move forward for a specific duration
    void moveForward(int duration) {
      forward();
      delay(duration);
      stop();
    }
    
    // Move backward for a specific duration
    void moveBackward(int duration) {
      backward();
      delay(duration);
      stop();
    }
    
    // Turn left for a specific duration
    void turnLeftFor(int duration) {
      turnLeft();
      delay(duration);
      stop();
    }
    
    // Turn right for a specific duration
    void turnRightFor(int duration) {
      turnRight();
      delay(duration);
      stop();
    }
};

#endif
