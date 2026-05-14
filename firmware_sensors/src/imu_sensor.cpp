#include "imu_sensor.h"
#include <Arduino.h>
#include <MPU9250.h>

static MPU9250 mpu;

ImuInitResult initIMU() {
    Wire.begin();
    MPU9250Setting setting;
    setting.accel_fs_sel     = ACCEL_FS_SEL::A4G;
    setting.gyro_fs_sel      = GYRO_FS_SEL::G500DPS;
    setting.mag_output_bits  = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice     = 0x03;
    setting.gyro_dlpf_cfg    = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice    = 0x01;
    setting.accel_dlpf_cfg   = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {
        return ImuInitResult::NOT_FOUND;
    }

    // Calibration: update() must succeed quickly after setup
    const uint32_t calStart = millis();
    bool accelGyroDone = false;
    bool magDone       = false;
    mpu.calibrateAccelGyro();
    accelGyroDone = true;
    mpu.calibrateMag();
    magDone = true;
    (void)calStart;

    if (!accelGyroDone || !magDone) {
        return ImuInitResult::CALIB_FAILED;
    }

    return ImuInitResult::OK;
}

bool readIMU(ImuData &out) {
    if (!mpu.update()) {
        return false;
    }
    out.ax = static_cast<int16_t>(mpu.getAccX() * 1000);
    out.ay = static_cast<int16_t>(mpu.getAccY() * 1000);
    out.az = static_cast<int16_t>(mpu.getAccZ() * 1000);
    out.gx = static_cast<int16_t>(mpu.getGyroX() * 10);
    out.gy = static_cast<int16_t>(mpu.getGyroY() * 10);
    out.gz = static_cast<int16_t>(mpu.getGyroZ() * 10);
    out.mx = static_cast<int16_t>(mpu.getMagX() * 10);
    out.my = static_cast<int16_t>(mpu.getMagY() * 10);
    out.mz = static_cast<int16_t>(mpu.getMagZ() * 10);
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
    Serial.print(d.gz);
    Serial.print(F(" M:"));
    Serial.print(d.mx); Serial.print(',');
    Serial.print(d.my); Serial.print(',');
    Serial.println(d.mz);
}
