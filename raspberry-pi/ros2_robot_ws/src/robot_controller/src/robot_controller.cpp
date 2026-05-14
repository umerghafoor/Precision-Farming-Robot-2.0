#include "robot_controller/robot_controller.hpp"

#include <cmath>
#include <chrono>
#include <functional>

RobotController::RobotController() : Node("robot_controller") {
  this->declare_parameter("control_loop_rate",     20.0);
  this->declare_parameter("emergency_stop_enabled", true);
  this->declare_parameter("max_linear_velocity",   1.0);
  this->declare_parameter("max_angular_velocity",  2.0);
  this->declare_parameter("min_battery_voltage",   7.0);
  this->declare_parameter("command_timeout",       1.0);
  this->declare_parameter("wheel_separation",      0.2);

  wheel_separation_ = this->get_parameter("wheel_separation").as_double();

  // ── Subscribers ────────────────────────────────────────────────────────────
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&RobotController::cmdVelCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    std::bind(&RobotController::imuCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&RobotController::odometryCallback, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/battery_voltage", 10,
    std::bind(&RobotController::batteryCallback, this, std::placeholders::_1));

  // ── Publishers ─────────────────────────────────────────────────────────────
  // Safety-checked commands go to /cmd_vel_safe; spi_controller_bridge listens there.
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);
  status_pub_      = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

  // ── Control loop ───────────────────────────────────────────────────────────
  const double rate = this->get_parameter("control_loop_rate").as_double();
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
  control_timer_ = this->create_wall_timer(
    period, std::bind(&RobotController::controlLoop, this));

  last_cmd_time_ = this->now();

  RCLCPP_INFO(get_logger(),
    "RobotController ready — safety layer between /cmd_vel and /cmd_vel_safe"
    "  rate=%.0f Hz", rate);
  publishStatus("INITIALIZED");
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

void RobotController::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (is_emergency_stop_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Emergency stop active — ignoring /cmd_vel");
    return;
  }

  const double max_lin = this->get_parameter("max_linear_velocity").as_double();
  const double max_ang = this->get_parameter("max_angular_velocity").as_double();

  current_cmd_vel_ = *msg;
  current_cmd_vel_.linear.x  = std::clamp(current_cmd_vel_.linear.x,  -max_lin, max_lin);
  current_cmd_vel_.angular.z = std::clamp(current_cmd_vel_.angular.z, -max_ang, max_ang);
  last_cmd_time_ = this->now();
}

void RobotController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double mag = std::sqrt(
    msg->linear_acceleration.x * msg->linear_acceleration.x +
    msg->linear_acceleration.y * msg->linear_acceleration.y +
    msg->linear_acceleration.z * msg->linear_acceleration.z);
  RCLCPP_DEBUG(get_logger(), "IMU accel magnitude: %.2f m/s²", mag);
}

void RobotController::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_DEBUG(get_logger(), "Odom pos: [%.3f, %.3f]",
    msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void RobotController::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  battery_voltage_ = msg->data;
  RCLCPP_DEBUG(get_logger(), "Battery: %.2f V", battery_voltage_);
}

// ── Control loop ──────────────────────────────────────────────────────────────

void RobotController::controlLoop() {
  // Battery check
  const double min_v = this->get_parameter("min_battery_voltage").as_double();
  if (battery_voltage_ > 0.0 && battery_voltage_ < min_v) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "Low battery: %.2f V (min %.2f V)", battery_voltage_, min_v);
    emergencyStop();
    return;
  }

  // Command timeout — zero velocity if nothing received recently
  const double timeout = this->get_parameter("command_timeout").as_double();
  if ((this->now() - last_cmd_time_).seconds() > timeout) {
    if (current_cmd_vel_.linear.x != 0.0 || current_cmd_vel_.angular.z != 0.0) {
      RCLCPP_WARN(get_logger(), "Command timeout — stopping robot");
      current_cmd_vel_ = geometry_msgs::msg::Twist{};
    }
  }

  if (is_emergency_stop_) {
    // Keep publishing zero so the bridge sees it
    cmd_vel_out_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  cmd_vel_out_pub_->publish(current_cmd_vel_);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

void RobotController::publishStatus(const std::string & status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Status: %s", status.c_str());
}

void RobotController::emergencyStop() {
  if (!this->get_parameter("emergency_stop_enabled").as_bool()) {
    RCLCPP_WARN(get_logger(), "E-stop triggered but disabled by parameter");
    return;
  }
  if (!is_emergency_stop_) {
    is_emergency_stop_ = true;
    current_cmd_vel_   = geometry_msgs::msg::Twist{};
    publishStatus("EMERGENCY_STOP");
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP ACTIVATED");
  }
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
