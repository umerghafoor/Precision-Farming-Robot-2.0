#ifndef MOTOR_CONTROL_SPI_CONTROLLER_BRIDGE_HPP
#define MOTOR_CONTROL_SPI_CONTROLLER_BRIDGE_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include <array>
#include <string>

class SPIControllerBridge : public rclcpp::Node {
public:
  SPIControllerBridge();
  ~SPIControllerBridge() override;

private:
  static constexpr uint8_t DIR_FORWARD = 0;
  static constexpr uint8_t DIR_BACKWARD = 1;
  static constexpr uint8_t DIR_STOP = 2;

  struct MotorCommand {
    uint8_t direction;
    uint8_t speed;
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void servo1Callback(const std_msgs::msg::Int16::SharedPtr msg);
  void servo2Callback(const std_msgs::msg::Int16::SharedPtr msg);
  void transmitTimerCallback();

  void openAndConfigureSPI();
  bool transmitPacket(const std::array<uint8_t, 8> & packet);
  std::array<uint8_t, 8> buildPacket() const;
  MotorCommand toMotorCommand(double normalized_speed) const;
  uint8_t clampServoAngle(int angle) const;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr servo2_sub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;

  // SPI settings
  std::string spi_device_;
  int spi_fd_;
  uint8_t spi_mode_;
  uint8_t spi_bits_per_word_;
  uint32_t spi_speed_hz_;

  // Robot control settings
  double wheel_base_;
  double max_linear_velocity_;
  double cmd_timeout_sec_;

  // Command state
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time latest_cmd_time_;
  uint8_t servo1_angle_;
  uint8_t servo2_angle_;
};

#endif  // MOTOR_CONTROL_SPI_CONTROLLER_BRIDGE_HPP
