#ifndef ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>

class RobotController : public rclcpp::Node {
public:
  RobotController();

private:
  // ── Subscribers ────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr      imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr     battery_sub_;

  // ── Publishers ─────────────────────────────────────────────────────────────
  // Publishes the safety-checked command downstream to spi_controller_bridge
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_out_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         status_pub_;

  // ── Timer ──────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ── State ──────────────────────────────────────────────────────────────────
  geometry_msgs::msg::Twist current_cmd_vel_;
  double   battery_voltage_{0.0};
  bool     is_emergency_stop_{false};
  rclcpp::Time last_cmd_time_;
  double   wheel_separation_{0.2};

  // ── Callbacks ──────────────────────────────────────────────────────────────
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void controlLoop();

  // ── Helpers ────────────────────────────────────────────────────────────────
  void publishStatus(const std::string & status);
  void emergencyStop();
};

#endif  // ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
