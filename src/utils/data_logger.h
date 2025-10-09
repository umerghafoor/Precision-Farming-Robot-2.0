/*
 * Data Logger Module
 * 
 * Logs sensor data to SD card
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include <SD.h>

class DataLogger {
  private:
    int csPin;
    String filename;
    bool sdReady;
    
  public:
    DataLogger(int chipSelect, String fname = "data.csv") {
      csPin = chipSelect;
      filename = fname;
      sdReady = false;
    }
    
    bool begin() {
      if (!SD.begin(csPin)) {
        Serial.println("SD card initialization failed!");
        sdReady = false;
        return false;
      }
      Serial.println("SD card initialized.");
      sdReady = true;
      
      // Create header if file doesn't exist
      if (!SD.exists(filename.c_str())) {
        writeHeader();
      }
      
      return true;
    }
    
    void writeHeader() {
      if (!sdReady) return;
      
      File dataFile = SD.open(filename.c_str(), FILE_WRITE);
      if (dataFile) {
        dataFile.println("Timestamp,Temperature,Humidity,SoilMoisture,Distance");
        dataFile.close();
        Serial.println("Data header written.");
      }
    }
    
    void logData(unsigned long timestamp, float temp, float humidity, int moisture, int distance) {
      if (!sdReady) {
        Serial.println("SD card not ready!");
        return;
      }
      
      File dataFile = SD.open(filename.c_str(), FILE_WRITE);
      if (dataFile) {
        dataFile.print(timestamp);
        dataFile.print(",");
        dataFile.print(temp);
        dataFile.print(",");
        dataFile.print(humidity);
        dataFile.print(",");
        dataFile.print(moisture);
        dataFile.print(",");
        dataFile.println(distance);
        dataFile.close();
        
        // Also print to serial
        Serial.print("Logged: ");
        Serial.print(timestamp);
        Serial.print(" | T:");
        Serial.print(temp);
        Serial.print(" | H:");
        Serial.print(humidity);
        Serial.print(" | M:");
        Serial.print(moisture);
        Serial.print(" | D:");
        Serial.println(distance);
      } else {
        Serial.println("Error opening data file!");
      }
    }
    
    void logString(String data) {
      if (!sdReady) return;
      
      File dataFile = SD.open(filename.c_str(), FILE_WRITE);
      if (dataFile) {
        dataFile.println(data);
        dataFile.close();
      }
    }
    
    bool isReady() {
      return sdReady;
    }
};

#endif
