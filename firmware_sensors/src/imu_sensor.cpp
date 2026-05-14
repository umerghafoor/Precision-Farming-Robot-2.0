#include "imu_sensor.h"
#include <Arduino.h>
#include <MPU6050.h>

static MPU6050 mpu;

ImuInitResult initIMU() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        return ImuInitResult::NOT_FOUND;
    }
    Serial.println(F("IMU: MPU-6050 found at 0x68"));
    return ImuInitResult::OK;
}

bool readIMU(ImuData &out) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Scale to match serial format: accel in mg (x1000), gyro in dps (x10)
    out.ax = static_cast<int16_t>(ax / 16.384f);   // ±2g range → mg
    out.ay = static_cast<int16_t>(ay / 16.384f);
    out.az = static_cast<int16_t>(az / 16.384f);
    out.gx = static_cast<int16_t>(gx / 13.1f);     // ±250dps range → dps x10
    out.gy = static_cast<int16_t>(gy / 13.1f);
    out.gz = static_cast<int16_t>(gz / 13.1f);
    return true;
}

void printIMU(const ImuData &d) {
    Serial.print(F("A:"));
    Serial.print(d.ax); Serial.print(',');
    Serial.print(d.ay); Serial.print(',');
    Serial.print(d.az);
    Serial.print(F(" G:"));
    Serial.print(d.gx); Serial.print(',');
    Serial.print(d.gy); Serial.print(',');
    Serial.println(d.gz);
}
