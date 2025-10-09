# API Reference

Complete reference for all modules and functions in the Precision Farming Robot 2.0.

## Core Modules

### main.ino

Main control program for the robot.

#### Global Variables
- `float temperature` - Current temperature reading (°C)
- `float humidity` - Current humidity reading (%)
- `int soilMoisture` - Soil moisture analog value
- `int distance` - Obstacle distance (cm)
- `bool obstacleDetected` - True if obstacle within threshold
- `char command` - Current movement command

#### Functions

##### `void setup()`
Initializes all pins, sensors, and serial communication.

##### `void loop()`
Main control loop. Handles sensor reading, obstacle detection, movement, and data logging.

##### `void readSensors()`
Reads all sensor values and updates global variables.

##### `void checkObstacles()`
Uses ultrasonic sensor to detect obstacles and updates `obstacleDetected` flag.

##### `void executeMovement(char cmd)`
Executes movement command.
- **Parameters**: 
  - `cmd` - Command character (F/B/L/R/S)

##### `void moveForward()`
Moves robot forward at current speed.

##### `void moveBackward()`
Moves robot backward at current speed.

##### `void turnLeft()`
Turns robot left (tank turn).

##### `void turnRight()`
Turns robot right (tank turn).

##### `void stopMotors()`
Stops all motors.

##### `void logData()`
Prints sensor data to serial monitor.

---

## Sensor Modules

### SoilSensor

Manages soil moisture sensor readings.

#### Constructor
```cpp
SoilSensor(int sensorPin, int dry = 1023, int wet = 350)
```
- **Parameters**:
  - `sensorPin` - Analog pin connected to sensor
  - `dry` - Calibration value for dry soil (default: 1023)
  - `wet` - Calibration value for wet soil (default: 350)

#### Methods

##### `int readRaw()`
Returns raw analog value from sensor.
- **Returns**: Analog value (0-1023)

##### `int readMoisture()`
Returns moisture percentage.
- **Returns**: Moisture level (0-100%)

##### `bool needsWatering(int threshold = 30)`
Checks if soil needs watering.
- **Parameters**:
  - `threshold` - Moisture percentage threshold (default: 30%)
- **Returns**: True if moisture < threshold

##### `String getMoistureLevel()`
Returns descriptive moisture level.
- **Returns**: "Very Dry", "Dry", "Moderate", "Moist", or "Very Moist"

##### `void calibrate(int dry, int wet)`
Updates calibration values.
- **Parameters**:
  - `dry` - New dry calibration value
  - `wet` - New wet calibration value

#### Example
```cpp
#include "sensors/soil_sensor.h"

SoilSensor soil(A0);

void setup() {
  Serial.begin(115200);
}

void loop() {
  int moisture = soil.readMoisture();
  Serial.print("Moisture: ");
  Serial.print(moisture);
  Serial.println("%");
  
  if (soil.needsWatering()) {
    Serial.println("Needs watering!");
  }
  
  delay(2000);
}
```

---

### TempHumiditySensor

Wrapper for DHT22 sensor with enhanced features.

#### Constructor
```cpp
TempHumiditySensor(int pin, uint8_t type = DHT22, int interval = 2000)
```
- **Parameters**:
  - `pin` - Digital pin connected to sensor
  - `type` - Sensor type (DHT22 or DHT11)
  - `interval` - Minimum read interval in ms (default: 2000)

#### Methods

##### `void begin()`
Initializes the sensor. Call in `setup()`.

##### `float readTemperature()`
Returns temperature in Celsius.
- **Returns**: Temperature (°C)

##### `float readTemperatureF()`
Returns temperature in Fahrenheit.
- **Returns**: Temperature (°F)

##### `float readHumidity()`
Returns humidity percentage.
- **Returns**: Humidity (0-100%)

##### `float getHeatIndex(bool fahrenheit = false)`
Calculates heat index.
- **Parameters**:
  - `fahrenheit` - Use Fahrenheit if true
- **Returns**: Heat index value

##### `String getComfortLevel()`
Returns comfort assessment.
- **Returns**: "Too Cold", "Too Hot", "Too Dry", "Too Humid", "Optimal", or "Acceptable"

##### `bool isValid()`
Checks if last readings were valid.
- **Returns**: True if readings are valid

#### Example
```cpp
#include "sensors/temp_humidity_sensor.h"

TempHumiditySensor dht(7);

void setup() {
  Serial.begin(115200);
  dht.begin();
}

void loop() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");
  
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
  Serial.print("Comfort: ");
  Serial.println(dht.getComfortLevel());
  
  delay(2000);
}
```

---

### UltrasonicSensor

HC-SR04 ultrasonic distance sensor control.

#### Constructor
```cpp
UltrasonicSensor(int trig, int echo, int maxDist = 400)
```
- **Parameters**:
  - `trig` - Trigger pin
  - `echo` - Echo pin
  - `maxDist` - Maximum distance in cm (default: 400)

#### Methods

##### `int measureDistance()`
Measures and returns distance.
- **Returns**: Distance in cm

##### `int getDistance()`
Returns last measured distance.
- **Returns**: Distance in cm

##### `bool obstacleDetected(int threshold = 30)`
Checks for obstacle within threshold.
- **Parameters**:
  - `threshold` - Distance threshold in cm (default: 30)
- **Returns**: True if obstacle detected

##### `int getAverageDistance(int samples = 3)`
Returns average of multiple readings.
- **Parameters**:
  - `samples` - Number of samples (default: 3)
- **Returns**: Average distance in cm

#### Example
```cpp
#include "sensors/ultrasonic_sensor.h"

UltrasonicSensor ultrasonic(6, 5);

void setup() {
  Serial.begin(115200);
}

void loop() {
  int distance = ultrasonic.measureDistance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  if (ultrasonic.obstacleDetected(30)) {
    Serial.println("Obstacle ahead!");
  }
  
  delay(500);
}
```

---

## Navigation Modules

### MotorController

Controls DC motors for robot movement.

#### Constructor
```cpp
MotorController(int lf, int lb, int rf, int rb, int maxSpd = 255)
```
- **Parameters**:
  - `lf` - Left forward pin
  - `lb` - Left backward pin
  - `rf` - Right forward pin
  - `rb` - Right backward pin
  - `maxSpd` - Maximum speed (default: 255)

#### Methods

##### `void setSpeed(int speed)`
Sets motor speed.
- **Parameters**:
  - `speed` - Speed value (0-255)

##### `int getSpeed()`
Returns current speed setting.
- **Returns**: Current speed (0-255)

##### `void forward()`
Moves robot forward.

##### `void backward()`
Moves robot backward.

##### `void turnLeft()`
Turns robot left (tank turn).

##### `void turnRight()`
Turns robot right (tank turn).

##### `void rotateLeft(int speed = -1)`
Rotates robot left in place.
- **Parameters**:
  - `speed` - Rotation speed, -1 for half current speed

##### `void rotateRight(int speed = -1)`
Rotates robot right in place.
- **Parameters**:
  - `speed` - Rotation speed, -1 for half current speed

##### `void stop()`
Stops all motors.

##### `void moveForward(int duration)`
Moves forward for specified time.
- **Parameters**:
  - `duration` - Time in milliseconds

##### `void moveBackward(int duration)`
Moves backward for specified time.
- **Parameters**:
  - `duration` - Time in milliseconds

##### `void turnLeftFor(int duration)`
Turns left for specified time.
- **Parameters**:
  - `duration` - Time in milliseconds

##### `void turnRightFor(int duration)`
Turns right for specified time.
- **Parameters**:
  - `duration` - Time in milliseconds

#### Example
```cpp
#include "navigation/motor_controller.h"

MotorController motors(9, 8, 11, 10);

void setup() {
  Serial.begin(115200);
  motors.setSpeed(200);
}

void loop() {
  motors.moveForward(2000);  // Forward 2 seconds
  delay(500);
  motors.turnRightFor(500);  // Turn right 0.5 seconds
  delay(500);
}
```

---

## Utility Modules

### DataLogger

Logs sensor data to SD card.

#### Constructor
```cpp
DataLogger(int chipSelect, String fname = "data.csv")
```
- **Parameters**:
  - `chipSelect` - CS pin for SD card
  - `fname` - Filename (default: "data.csv")

#### Methods

##### `bool begin()`
Initializes SD card.
- **Returns**: True if successful

##### `void writeHeader()`
Writes CSV header to file.

##### `void logData(unsigned long timestamp, float temp, float humidity, int moisture, int distance)`
Logs sensor data.
- **Parameters**:
  - `timestamp` - Time in milliseconds
  - `temp` - Temperature value
  - `humidity` - Humidity value
  - `moisture` - Soil moisture value
  - `distance` - Distance value

##### `void logString(String data)`
Logs custom string.
- **Parameters**:
  - `data` - String to log

##### `bool isReady()`
Checks if SD card is ready.
- **Returns**: True if ready

#### Example
```cpp
#include "utils/data_logger.h"

DataLogger logger(4);  // CS pin 4

void setup() {
  Serial.begin(115200);
  if (logger.begin()) {
    Serial.println("Logger ready");
  }
}

void loop() {
  logger.logData(millis(), 25.3, 65.2, 450, 35);
  delay(5000);
}
```

---

## Constants and Definitions

### Pin Assignments (main.ino)
```cpp
#define DHT_PIN 7
#define SOIL_MOISTURE_PIN A0
#define MOTOR_LEFT_FWD 9
#define MOTOR_LEFT_BWD 8
#define MOTOR_RIGHT_FWD 11
#define MOTOR_RIGHT_BWD 10
#define TRIG_PIN 6
#define ECHO_PIN 5
#define LED_STATUS 13
```

### Configuration Constants
```cpp
#define MOISTURE_THRESHOLD 500
#define OBSTACLE_DISTANCE 30
#define MOTOR_SPEED 200
```

---

## Serial Commands

When connected via USB or Bluetooth, send these commands:

| Command | Action |
|---------|--------|
| `F` | Move forward |
| `B` | Move backward |
| `L` | Turn left |
| `R` | Turn right |
| `S` | Stop |

Commands are case-insensitive.

---

## Data Format

### CSV Log Format
```
Timestamp,Temperature,Humidity,SoilMoisture,Distance
1000,25.3,65.2,450,35
```

### Serial Output Format
```
=== Sensor Data ===
Temperature: 25.3 °C
Humidity: 65.2 %
Soil Moisture: 450
Distance: 35 cm
==================
```

---

## Error Handling

### Sensor Read Failures
- DHT22 returns NaN on failure
- Code sets values to 0.0 and prints warning
- System continues operation with last valid readings

### SD Card Errors
- Logger prints error messages
- Data also logged to serial as backup
- System continues without SD logging

### Obstacle Detection
- Robot stops automatically when obstacle detected
- Resumes on new command when clear
- Distance 0 or > max indicates no valid reading

---

## Integration Example

Complete example using all modules:

```cpp
#include "sensors/soil_sensor.h"
#include "sensors/temp_humidity_sensor.h"
#include "sensors/ultrasonic_sensor.h"
#include "navigation/motor_controller.h"
#include "utils/data_logger.h"

SoilSensor soil(A0);
TempHumiditySensor dht(7);
UltrasonicSensor ultrasonic(6, 5);
MotorController motors(9, 8, 11, 10);
DataLogger logger(4);

void setup() {
  Serial.begin(115200);
  dht.begin();
  logger.begin();
  motors.setSpeed(200);
}

void loop() {
  // Read sensors
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  int moisture = soil.readMoisture();
  int distance = ultrasonic.measureDistance();
  
  // Log data
  logger.logData(millis(), temp, humidity, moisture, distance);
  
  // Navigate
  if (!ultrasonic.obstacleDetected(30)) {
    motors.forward();
  } else {
    motors.stop();
    motors.turnRightFor(500);
  }
  
  delay(100);
}
```
