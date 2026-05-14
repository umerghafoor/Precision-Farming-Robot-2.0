#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

struct ImuData {
    int16_t ax, ay, az;   // Accelerometer (raw counts)
    int16_t gx, gy, gz;   // Gyroscope     (raw counts)
    int16_t mx, my, mz;   // Magnetometer  (raw counts)
};

bool initIMU();
bool readIMU(ImuData &out);
void printIMU(const ImuData &d);

#endif
