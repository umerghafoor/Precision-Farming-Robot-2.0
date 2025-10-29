#include "imu_sensor/imu_node.hpp"

// Define static constants declared in header
const int IMUNode::I2C_BUS;
const int IMUNode::MPU6050_ADDRESS;

IMUNode::IMUNode() : Node("imu_node") {
  // Declare parameters
  this->declare_parameter("update_rate", 50.0);  // Hz
  this->declare_parameter("i2c_bus", I2C_BUS);
  this->declare_parameter("i2c_address", MPU6050_ADDRESS);
  
  // Initialize IMU sensor
  initializeSensor();
  
  // Create publisher
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  
  // Create timer for periodic sensor reading
  double update_rate = this->get_parameter("update_rate").as_double();
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate));
  sensor_timer_ = this->create_wall_timer(period, std::bind(&IMUNode::sensorCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "IMU Node initialized with update rate: %.1f Hz", update_rate);
}

void IMUNode::initializeSensor() {
  RCLCPP_INFO(this->get_logger(), "Initializing MPU6050 IMU sensor...");
  
  // In a real implementation, you would:
  // 1. Open I2C device (e.g., /dev/i2c-1)
  // 2. Set slave address to MPU6050_ADDRESS
  // 3. Write configuration registers
  // 4. Enable accelerometer and gyroscope
  
  // Example I2C register values for MPU6050:
  // Power Management 1 register (0x6B): Write 0x00 to wake up
  // Accelerometer config register (0x1C): Set full scale range
  // Gyroscope config register (0x1B): Set full scale range
  
  RCLCPP_INFO(this->get_logger(), "MPU6050 sensor initialized successfully");
}

void IMUNode::sensorCallback() {
  readIMUData();
  
  // Create and publish IMU message
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = this->now();
  imu_msg.header.frame_id = "imu_link";
  
  // Set accelerometer data (m/s^2)
  imu_msg.linear_acceleration.x = accel_x_;
  imu_msg.linear_acceleration.y = accel_y_;
  imu_msg.linear_acceleration.z = accel_z_;
  
  // Set accelerometer covariance
  imu_msg.linear_acceleration_covariance[0] = 0.01;  // x variance
  imu_msg.linear_acceleration_covariance[4] = 0.01;  // y variance
  imu_msg.linear_acceleration_covariance[8] = 0.01;  // z variance
  
  // Set angular velocity data (rad/s)
  imu_msg.angular_velocity.x = gyro_x_;
  imu_msg.angular_velocity.y = gyro_y_;
  imu_msg.angular_velocity.z = gyro_z_;
  
  // Set angular velocity covariance
  imu_msg.angular_velocity_covariance[0] = 0.001;   // x variance
  imu_msg.angular_velocity_covariance[4] = 0.001;   // y variance
  imu_msg.angular_velocity_covariance[8] = 0.001;   // z variance
  
  imu_pub_->publish(imu_msg);
  
  RCLCPP_DEBUG(this->get_logger(), 
               "IMU - Accel: [%.2f, %.2f, %.2f] m/s^2, Gyro: [%.2f, %.2f, %.2f] rad/s",
               accel_x_, accel_y_, accel_z_, gyro_x_, gyro_y_, gyro_z_);
}

void IMUNode::readIMUData() {
  // In a real implementation, you would:
  // 1. Read raw data from I2C device
  // 2. Convert raw values to physical units
  // 3. Apply calibration and filtering
  
  // Example values (replace with actual I2C reads)
  // For now, using simulated data
  
  // Accelerometer: ±2g full scale (16384 LSB/g)
  // Raw values to m/s^2 conversion: raw_value / 16384 * 9.81
  accel_x_ = 0.0f;   // gravity component
  accel_y_ = 0.0f;
  accel_z_ = 9.81f;  // 1g in z-axis when level
  
  // Gyroscope: ±250 deg/s full scale (131 LSB/deg/s)
  // Raw values to rad/s conversion: (raw_value / 131) * (PI / 180)
  gyro_x_ = 0.0f;
  gyro_y_ = 0.0f;
  gyro_z_ = 0.0f;
  
  // Temperature is available but not published here
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
