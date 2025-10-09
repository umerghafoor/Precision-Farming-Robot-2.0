/*
 * Temperature and Humidity Sensor Module
 * 
 * Wrapper for DHT22 sensor with additional features
 */

#ifndef TEMP_HUMIDITY_SENSOR_H
#define TEMP_HUMIDITY_SENSOR_H

#include <Arduino.h>
#include <DHT.h>

class TempHumiditySensor {
  private:
    DHT* dht;
    float lastTemp;
    float lastHumidity;
    unsigned long lastRead;
    int readInterval;
    
  public:
    TempHumiditySensor(int pin, uint8_t type = DHT22, int interval = 2000) {
      dht = new DHT(pin, type);
      readInterval = interval;
      lastTemp = 0.0;
      lastHumidity = 0.0;
      lastRead = 0;
    }
    
    void begin() {
      dht->begin();
    }
    
    // Read temperature in Celsius
    float readTemperature() {
      if (millis() - lastRead >= readInterval) {
        float temp = dht->readTemperature();
        if (!isnan(temp)) {
          lastTemp = temp;
        }
        lastRead = millis();
      }
      return lastTemp;
    }
    
    // Read temperature in Fahrenheit
    float readTemperatureF() {
      return readTemperature() * 9.0/5.0 + 32.0;
    }
    
    // Read humidity percentage
    float readHumidity() {
      if (millis() - lastRead >= readInterval) {
        float hum = dht->readHumidity();
        if (!isnan(hum)) {
          lastHumidity = hum;
        }
        lastRead = millis();
      }
      return lastHumidity;
    }
    
    // Calculate heat index
    float getHeatIndex(bool fahrenheit = false) {
      float temp = fahrenheit ? readTemperatureF() : readTemperature();
      return dht->computeHeatIndex(temp, lastHumidity, fahrenheit);
    }
    
    // Get comfort level based on temperature and humidity
    String getComfortLevel() {
      float temp = readTemperature();
      float hum = readHumidity();
      
      if (temp < 10) return "Too Cold";
      else if (temp > 35) return "Too Hot";
      else if (hum < 30) return "Too Dry";
      else if (hum > 70) return "Too Humid";
      else if (temp >= 18 && temp <= 24 && hum >= 40 && hum <= 60) return "Optimal";
      else return "Acceptable";
    }
    
    // Check if readings are valid
    bool isValid() {
      return !isnan(lastTemp) && !isnan(lastHumidity);
    }
};

#endif
