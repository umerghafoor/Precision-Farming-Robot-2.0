/*
 * Simple Sensor Test Example
 * 
 * Tests all sensors independently
 */

#include <DHT.h>

#define DHT_PIN 7
#define DHT_TYPE DHT22
#define SOIL_PIN A0
#define TRIG_PIN 6
#define ECHO_PIN 5

DHT dht(DHT_PIN, DHT_TYPE);

void setup() {
  Serial.begin(115200);
  Serial.println("Sensor Test Example");
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  dht.begin();
  
  Serial.println("Testing sensors...");
}

void loop() {
  // Test DHT22
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  Serial.println("\n=== Sensor Readings ===");
  
  if (!isnan(temp) && !isnan(humidity)) {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("DHT22: Failed to read");
  }
  
  // Test Soil Moisture
  int moisture = analogRead(SOIL_PIN);
  Serial.print("Soil Moisture: ");
  Serial.print(moisture);
  Serial.print(" (");
  Serial.print(map(moisture, 1023, 350, 0, 100));
  Serial.println("%)");
  
  // Test Ultrasonic
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  Serial.println("======================\n");
  
  delay(2000);
}
