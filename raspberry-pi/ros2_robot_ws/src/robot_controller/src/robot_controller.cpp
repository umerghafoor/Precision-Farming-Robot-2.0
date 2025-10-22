#include "robot_controller/robot_controller.hpp"

RobotController::RobotController() : Node("robot_controller") {
  // Declare parameters
  this->declare_parameter("control_loop_rate", 20.0);  // Hz
  this->declare_parameter("emergency_stop_enabled", true);
  this->declare_parameter("max_linear_velocity", 1.0);  // m/s
  this->declare_parameter("max_angular_velocity", 2.0); // rad/s
  this->declare_parameter("min_battery_voltage", 7.0);  // 7V minimum for safety

  // Create subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&RobotController::cmdVelCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&RobotController::imuCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&RobotController::odometryCallback, this, std::placeholders::_1));

  // Create publishers
  motor_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_filtered", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

  // Create control loop timer
  double control_rate = this->get_parameter("control_loop_rate").as_double();
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate));
  control_timer_ = this->create_wall_timer(period, std::bind(&RobotController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Robot Controller initialized with control loop rate: %.1f Hz",
              control_rate);
  publishStatus("INITIALIZED");
}

void RobotController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (is_emergency_stop_) {
    RCLCPP_WARN(this->get_logger(), "Emergency stop active. Ignoring velocity command.");
    return;
  }

  // Get velocity limits from parameters
  double max_linear = this->get_parameter("max_linear_velocity").as_double();
  double max_angular = this->get_parameter("max_angular_velocity").as_double();

  // Clamp velocities to safe ranges
  geometry_msgs::msg::Twist clamped_cmd = *msg;

  if (std::abs(clamped_cmd.linear.x) > max_linear) {
    clamped_cmd.linear.x = (clamped_cmd.linear.x > 0) ? max_linear : -max_linear;
  }
  if (std::abs(clamped_cmd.angular.z) > max_angular) {
    clamped_cmd.angular.z = (clamped_cmd.angular.z > 0) ? max_angular : -max_angular;
  }

  current_cmd_vel_ = clamped_cmd;

  RCLCPP_DEBUG(this->get_logger(),
               "Velocity command - linear_x: %.2f m/s, angular_z: %.2f rad/s",
               current_cmd_vel_.linear.x, current_cmd_vel_.angular.z);
}

void RobotController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Process IMU data for stability monitoring
  double accel_magnitude = std::sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x +
                                     msg->linear_acceleration.y * msg->linear_acceleration.y +
                                     msg->linear_acceleration.z * msg->linear_acceleration.z);

  RCLCPP_DEBUG(this->get_logger(), "IMU acceleration magnitude: %.2f m/s^2", accel_magnitude);
}

void RobotController::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Process odometry for position tracking
  RCLCPP_DEBUG(this->get_logger(), "Odometry - Position: [%.3f, %.3f]", 
               msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void RobotController::controlLoop() {
  // Check battery voltage
  double min_voltage = this->get_parameter("min_battery_voltage").as_double();
  if (battery_voltage_ > 0.0 && battery_voltage_ < min_voltage) {
    RCLCPP_ERROR(this->get_logger(), "Low battery voltage: %.2f V (minimum: %.2f V)",
                 battery_voltage_, min_voltage);
    emergencyStop();
    return;
  }

  // Publish motor commands
  if (!is_emergency_stop_) {
    motor_cmd_pub_->publish(current_cmd_vel_);
  } else {
    // Publish zero velocities during emergency stop
    geometry_msgs::msg::Twist stop_cmd;
    motor_cmd_pub_->publish(stop_cmd);
  }
}

void RobotController::publishStatus(const std::string& status) {
  auto msg = std_msgs::msg::String();
  msg.data = status;
  status_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
}

void RobotController::emergencyStop() {
  is_emergency_stop_ = true;
  geometry_msgs::msg::Twist stop_cmd;
  motor_cmd_pub_->publish(stop_cmd);
  publishStatus("EMERGENCY_STOP");
  RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED!");
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
