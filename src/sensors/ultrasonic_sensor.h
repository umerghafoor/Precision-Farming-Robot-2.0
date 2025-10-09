/*
 * Ultrasonic Distance Sensor Module
 * 
 * HC-SR04 ultrasonic sensor for obstacle detection
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
  private:
    int trigPin;
    int echoPin;
    long duration;
    int distance;
    int maxDistance;
    
  public:
    UltrasonicSensor(int trig, int echo, int maxDist = 400) {
      trigPin = trig;
      echoPin = echo;
      maxDistance = maxDist;
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }
    
    // Measure distance in centimeters
    int measureDistance() {
      // Clear trigger pin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      
      // Send ultrasonic pulse
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      
      // Read echo
      duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
      
      // Calculate distance
      distance = duration * 0.034 / 2;
      
      // Validate reading
      if (distance > maxDistance || distance <= 0) {
        distance = maxDistance;
      }
      
      return distance;
    }
    
    // Get last measured distance
    int getDistance() {
      return distance;
    }
    
    // Check if obstacle is within threshold
    bool obstacleDetected(int threshold = 30) {
      measureDistance();
      return (distance > 0 && distance < threshold);
    }
    
    // Get average of multiple readings
    int getAverageDistance(int samples = 3) {
      long sum = 0;
      for (int i = 0; i < samples; i++) {
        sum += measureDistance();
        delay(50);
      }
      return sum / samples;
    }
};

#endif
