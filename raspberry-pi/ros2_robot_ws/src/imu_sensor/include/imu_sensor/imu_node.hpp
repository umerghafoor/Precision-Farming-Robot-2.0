#ifndef IMU_SENSOR_IMU_NODE_HPP
#define IMU_SENSOR_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUNode : public rclcpp::Node {
public:
  IMUNode();

private:
  // I2C configuration for MPU6050
  static const int I2C_BUS = 1;              // I2C bus 1 on Raspberry Pi
  static const int MPU6050_ADDRESS = 0x68;   // Default I2C address of MPU6050

  // Timer for periodic sensor reading
  rclcpp::TimerBase::SharedPtr sensor_timer_;
  
  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Methods
  void initializeSensor();
  void sensorCallback();
  void readIMUData();
  
  // Data storage
  float accel_x_, accel_y_, accel_z_;
  float gyro_x_, gyro_y_, gyro_z_;
  float temperature_;
};

#endif  // IMU_SENSOR_IMU_NODE_HPP
