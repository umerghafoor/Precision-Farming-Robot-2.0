#ifndef MOTOR_CONTROL_SPI_CONTROLLER_BRIDGE_HPP
#define MOTOR_CONTROL_SPI_CONTROLLER_BRIDGE_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

#include <array>
#include <mutex>
#include <string>

class SPIControllerBridge : public rclcpp::Node {
public:
  SPIControllerBridge();
  ~SPIControllerBridge() override;

private:
  static constexpr uint8_t DIR_FORWARD  = 0;
  static constexpr uint8_t DIR_BACKWARD = 1;
  static constexpr uint8_t DIR_STOP     = 2;

  struct MotorCommand {
    uint8_t direction;
    uint8_t speed;
  };

  // ── ROS callbacks ──────────────────────────────────────────────────────────
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void servo1Callback(const std_msgs::msg::Int16::SharedPtr msg);
  void servo2Callback(const std_msgs::msg::Int16::SharedPtr msg);
  void transmitTimerCallback();

  // ── Packet helpers ─────────────────────────────────────────────────────────
  // Firmware SIGNAL_SIZE = 6: [Dir1,Spd1,Dir2,Spd2,Dir3,Spd3]
  std::array<uint8_t, 6> buildPacket() const;
  MotorCommand toMotorCommand(double normalized_speed) const;
  uint8_t clampServoAngle(int angle) const;
  bool transmitPacket(const std::array<uint8_t, 6> & packet);

  // Servo angles sent as text "S1:<deg>\n" only when the value changes
  void sendServoCommand(int servo_id, uint8_t angle);
  uint8_t prev_servo1_angle_{255};  // 255 = never sent
  uint8_t prev_servo2_angle_{255};

  // ── USB Serial transport ───────────────────────────────────────────────────
  std::string detectSerialPort(const std::string & hint);
  bool openSerial(const std::string & port);
  void closeSerial();

  std::string serial_port_;
  int         serial_fd_{-1};
  std::mutex  write_mutex_;

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr      servo1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr      servo2_sub_;
  rclcpp::TimerBase::SharedPtr                               tx_timer_;

  // ── Robot parameters ───────────────────────────────────────────────────────
  double wheel_base_;
  double max_linear_velocity_;
  double cmd_timeout_sec_;

  // ── Command state ──────────────────────────────────────────────────────────
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time              latest_cmd_time_;
  uint8_t servo1_angle_{90};
  uint8_t servo2_angle_{90};
};

#endif  // MOTOR_CONTROL_SPI_CONTROLLER_BRIDGE_HPP
