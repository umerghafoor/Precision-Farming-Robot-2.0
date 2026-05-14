#include <Arduino.h>
#include <Wire.h>

void scan() {
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 128; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.print(F("FOUND 0x"));
            if (addr < 16) Serial.print('0');
            Serial.println(addr, HEX);
            found++;
        }
    }
    if (found == 0) Serial.println(F("NO DEVICES FOUND"));
    else { Serial.print(found); Serial.println(F(" device(s)")); }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(500);
    Serial.println(F("=== I2C SCAN ==="));
    scan();
}

void loop() {
    delay(3000);
    Serial.println(F("--- rescan ---"));
    scan();
}
