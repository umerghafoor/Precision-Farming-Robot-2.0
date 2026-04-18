#ifndef ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>

// Direction constants (from Arduino firmware)
#define DIR_FORWARD 0
#define DIR_BACKWARD 1
#define DIR_STOP 2

// Forward declarations
class SerialPort;

class RobotController : public rclcpp::Node {
public:
  RobotController();
  ~RobotController();

private:
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Serial communication
  std::unique_ptr<SerialPort> serial_port_;
  bool serial_connected_ = false;

  // Control state
  geometry_msgs::msg::Twist current_cmd_vel_;
  double battery_voltage_ = 0.0;
  bool is_emergency_stop_ = false;
  rclcpp::Time last_cmd_time_;

  // Motor parameters
  double max_linear_velocity_ = 1.0;
  double max_angular_velocity_ = 2.0;
  double wheel_separation_ = 0.2;  // meters, distance between wheels

  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void controlLoop();

  // Helper methods
  void publishStatus(const std::string& status);
  void emergencyStop();
  bool initSerialConnection();
  bool send6DSignal(uint8_t dir1, uint8_t speed1, uint8_t dir2, uint8_t speed2, 
                    uint8_t dir3, uint8_t speed3);
  void twistToMotorCommands(const geometry_msgs::msg::Twist& twist,
                           uint8_t& motor1_dir, uint8_t& motor1_speed,
                           uint8_t& motor2_dir, uint8_t& motor2_speed);
};

#endif  // ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
