/*
 * Soil Moisture Sensor Module
 * 
 * Functions for reading and processing soil moisture data
 */

#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include <Arduino.h>

class SoilSensor {
  private:
    int pin;
    int dryValue;    // Analog value when dry (typically ~1023)
    int wetValue;    // Analog value when wet (typically ~300-400)
    
  public:
    SoilSensor(int sensorPin, int dry = 1023, int wet = 350) {
      pin = sensorPin;
      dryValue = dry;
      wetValue = wet;
      pinMode(pin, INPUT);
    }
    
    // Read raw analog value
    int readRaw() {
      return analogRead(pin);
    }
    
    // Read moisture as percentage (0-100%)
    int readMoisture() {
      int raw = readRaw();
      int moisture = map(raw, dryValue, wetValue, 0, 100);
      moisture = constrain(moisture, 0, 100);
      return moisture;
    }
    
    // Check if soil needs watering
    bool needsWatering(int threshold = 30) {
      return readMoisture() < threshold;
    }
    
    // Get moisture level description
    String getMoistureLevel() {
      int moisture = readMoisture();
      if (moisture < 20) return "Very Dry";
      else if (moisture < 40) return "Dry";
      else if (moisture < 60) return "Moderate";
      else if (moisture < 80) return "Moist";
      else return "Very Moist";
    }
    
    // Calibrate sensor
    void calibrate(int dry, int wet) {
      dryValue = dry;
      wetValue = wet;
    }
};

#endif
