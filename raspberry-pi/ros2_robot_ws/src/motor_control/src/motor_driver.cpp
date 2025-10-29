#include "motor_control/motor_driver.hpp"
#include <algorithm>
#include <cmath>

MotorDriver::MotorDriver() : Node("motor_driver") {
  // Initialize GPIO
  initializeGPIO();
  
  // Declare and get parameters
  this->declare_parameter("wheel_base", 0.2);  // Distance between left and right wheels in meters
  this->declare_parameter("wheel_radius", 0.05);  // Wheel radius in meters
  this->declare_parameter("max_speed", 1.0);  // Maximum linear speed in m/s
  
  // Subscribe to velocity commands
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Motor Driver Node initialized");
}

void MotorDriver::initializeGPIO() {
  RCLCPP_INFO(this->get_logger(), "Initializing GPIO pins...");
  setupGPIOPins();
}

void MotorDriver::setupGPIOPins() {
  // In a real implementation, you would use a GPIO library like WiringPi or gpiozero
  // For now, we'll just log the setup
  RCLCPP_INFO(this->get_logger(), "GPIO setup complete");
  
  // Note: On actual hardware, you would initialize GPIO pins here
  // Example using system calls (not recommended for production):
  // system("echo 17 > /sys/class/gpio/export");
  // system("echo out > /sys/class/gpio/gpio17/direction");
}

void MotorDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Extract linear and angular velocities
  double linear_x = msg->linear.x;      // Forward/backward movement
  double angular_z = msg->angular.z;    // Rotation around z-axis
  
  // Get wheel base parameter
  double wheel_base = this->get_parameter("wheel_base").as_double();
  
  // Differential drive kinematics
  // For a 4-wheel differential robot:
  // v_left = linear_x - (angular_z * wheel_base / 2)
  // v_right = linear_x + (angular_z * wheel_base / 2)
  
  double left_speed = linear_x - (angular_z * wheel_base / 2.0);
  double right_speed = linear_x + (angular_z * wheel_base / 2.0);
  
  // Clamp speeds to [-1, 1] range
  left_speed = std::max(-1.0, std::min(1.0, left_speed));
  right_speed = std::max(-1.0, std::min(1.0, right_speed));
  
  RCLCPP_DEBUG(this->get_logger(), 
               "cmd_vel - linear_x: %.2f, angular_z: %.2f, left_speed: %.2f, right_speed: %.2f",
               linear_x, angular_z, left_speed, right_speed);
  
  // Set motor speeds
  // Motor 1 & 3 are left side
  setMotorSpeed(1, left_speed);
  setMotorSpeed(3, left_speed);
  
  // Motor 2 & 4 are right side
  setMotorSpeed(2, right_speed);
  setMotorSpeed(4, right_speed);
}

void MotorDriver::setMotorSpeed(int motor_num, float speed) {
  // Speed should be in range [-1, 1]
  speed = std::max(-1.0f, std::min(1.0f, speed));
  
  // Determine direction
  bool forward = (speed >= 0);
  setMotorDirection(motor_num, forward);
  
  // Convert speed to PWM value [0-255]
  int pwm_value = static_cast<int>(std::abs(speed) * 255.0f);
  
  // Select PWM pin based on motor number
  int pwm_pin = 0;
  switch (motor_num) {
    case 1: pwm_pin = MOTOR1_PWM_PIN; break;
    case 2: pwm_pin = MOTOR2_PWM_PIN; break;
    case 3: pwm_pin = MOTOR3_PWM_PIN; break;
    case 4: pwm_pin = MOTOR4_PWM_PIN; break;
    default: return;
  }
  
  pwmWrite(pwm_pin, pwm_value);
}

void MotorDriver::setMotorDirection(int motor_num, bool forward) {
  // Set motor direction pins based on forward/backward direction
  // forward = true -> pin1 HIGH, pin2 LOW
  // forward = false -> pin1 LOW, pin2 HIGH
  
  int pin1 = 0, pin2 = 0;
  switch (motor_num) {
    case 1:
      pin1 = MOTOR1_PIN1;
      pin2 = MOTOR1_PIN2;
      break;
    case 2:
      pin1 = MOTOR2_PIN1;
      pin2 = MOTOR2_PIN2;
      break;
    case 3:
      pin1 = MOTOR3_PIN1;
      pin2 = MOTOR3_PIN2;
      break;
    case 4:
      pin1 = MOTOR4_PIN1;
      pin2 = MOTOR4_PIN2;
      break;
    default:
      return;
  }
  
  // In real implementation, set GPIO pins here
  RCLCPP_DEBUG(this->get_logger(), "Motor %d direction: %s", motor_num, forward ? "Forward" : "Backward");
}

void MotorDriver::pwmWrite(int pin, int value) {
  // In real implementation, write PWM value to pin
  // This would use a GPIO PWM library or /sys/class/pwm interface
  RCLCPP_DEBUG(this->get_logger(), "PWM Pin %d: %d", pin, value);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>());
  rclcpp::shutdown();
  return 0;
}
