/*
 * Configuration File for Precision Farming Robot
 * 
 * Adjust these settings based on your hardware setup
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============ PIN CONFIGURATION ============

// DHT22 Temperature/Humidity Sensor
#define DHT_PIN 7
#define DHT_TYPE DHT22

// Soil Moisture Sensor
#define SOIL_MOISTURE_PIN A0

// Motor Pins (L298N Motor Driver)
#define MOTOR_LEFT_FWD 9
#define MOTOR_LEFT_BWD 8
#define MOTOR_RIGHT_FWD 11
#define MOTOR_RIGHT_BWD 10

// Ultrasonic Sensor (HC-SR04)
#define TRIG_PIN 6
#define ECHO_PIN 5

// Status LED
#define LED_STATUS 13

// Optional: SD Card Module
#define SD_CS_PIN 4

// Optional: GPS Module (if using SoftwareSerial)
#define GPS_RX_PIN 2
#define GPS_TX_PIN 3

// Optional: Bluetooth Module (HC-05)
#define BT_RX_PIN 2
#define BT_TX_PIN 3


// ============ SENSOR CALIBRATION ============

// Soil Moisture Calibration
#define SOIL_DRY_VALUE 1023      // Reading in air (dry)
#define SOIL_WET_VALUE 350       // Reading in water (wet)
#define MOISTURE_THRESHOLD 500   // Water if reading > this value

// Ultrasonic Sensor
#define OBSTACLE_DISTANCE 30     // Stop distance in cm
#define MAX_DISTANCE 400         // Maximum sensing range in cm


// ============ MOTOR CONFIGURATION ============

#define MOTOR_SPEED 200          // Default motor speed (0-255)
#define MOTOR_MAX_SPEED 255      // Maximum motor speed
#define TURN_SPEED 180           // Speed for turning
#define ROTATION_TIME 500        // Time for 90-degree turn (ms)


// ============ TIMING CONFIGURATION ============

#define SENSOR_READ_INTERVAL 2000    // Read sensors every 2 seconds
#define DATA_LOG_INTERVAL 5000       // Log data every 5 seconds
#define STATUS_LED_INTERVAL 1000     // Blink LED every 1 second


// ============ SERIAL CONFIGURATION ============

#define SERIAL_BAUD 115200       // Serial communication baud rate
#define DEBUG_MODE true          // Enable debug messages


// ============ FEATURE ENABLES ============

#define ENABLE_SD_LOGGING false  // Enable SD card logging
#define ENABLE_GPS false         // Enable GPS module
#define ENABLE_BLUETOOTH false   // Enable Bluetooth module
#define ENABLE_AUTO_NAVIGATION false  // Enable autonomous navigation


// ============ NAVIGATION PARAMETERS ============

// Autonomous navigation settings (if enabled)
#define AUTO_FORWARD_TIME 2000   // Move forward duration (ms)
#define AUTO_TURN_TIME 500       // Turn duration (ms)
#define OBSTACLE_AVOIDANCE true  // Enable obstacle avoidance


// ============ CROP THRESHOLDS ============

// Optimal ranges for different crops
// Adjust based on your specific crop requirements

// Temperature ranges (°C)
#define TEMP_MIN 18.0
#define TEMP_MAX 28.0
#define TEMP_CRITICAL_LOW 10.0
#define TEMP_CRITICAL_HIGH 35.0

// Humidity ranges (%)
#define HUMIDITY_MIN 40.0
#define HUMIDITY_MAX 70.0
#define HUMIDITY_CRITICAL_LOW 30.0
#define HUMIDITY_CRITICAL_HIGH 80.0

// Soil moisture (%)
#define SOIL_MOISTURE_MIN 30.0
#define SOIL_MOISTURE_MAX 70.0


// ============ ALERT CONFIGURATION ============

#define ENABLE_ALERTS true       // Enable warning messages
#define ALERT_BUZZER_PIN 12      // Optional: Buzzer pin for alerts


// ============ POWER MANAGEMENT ============

#define BATTERY_VOLTAGE_PIN A1   // Optional: Battery voltage monitoring
#define LOW_BATTERY_VOLTAGE 10.5 // Low battery threshold (V)
#define CRITICAL_BATTERY_VOLTAGE 10.0  // Critical battery voltage (V)


#endif // CONFIG_H
