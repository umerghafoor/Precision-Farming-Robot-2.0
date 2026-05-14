#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

struct ImuData {
    int16_t ax, ay, az;   // Accelerometer (raw counts, mg x1000)
    int16_t gx, gy, gz;   // Gyroscope     (raw counts, dps x10)
};

enum class ImuInitResult : uint8_t {
    OK        = 0,
    NOT_FOUND = 1,
};

ImuInitResult initIMU();
bool readIMU(ImuData &out);
void printIMU(const ImuData &d);

#endif
