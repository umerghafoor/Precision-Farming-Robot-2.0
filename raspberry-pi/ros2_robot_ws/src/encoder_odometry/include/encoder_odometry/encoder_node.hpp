#ifndef ENCODER_ODOMETRY_ENCODER_NODE_HPP
#define ENCODER_ODOMETRY_ENCODER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

class EncoderNode : public rclcpp::Node {
public:
  EncoderNode();

private:
  // GPIO pins for encoder inputs (interrupt pins)
  static const int ENCODER1_A_PIN = 4;   // Motor 1 (Front Left)
  static const int ENCODER1_B_PIN = 14;
  static const int ENCODER2_A_PIN = 15;  // Motor 2 (Front Right)
  static const int ENCODER2_B_PIN = 18;
  static const int ENCODER3_A_PIN = 2;   // Motor 3 (Rear Left)
  static const int ENCODER3_B_PIN = 3;
  static const int ENCODER4_A_PIN = 7;   // Motor 4 (Rear Right)
  static const int ENCODER4_B_PIN = 8;

  // Encoder parameters
  static const int COUNTS_PER_REVOLUTION = 20;  // Adjust based on your encoder

  // Odometry tracking
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;  // Robot pose
  rclcpp::Time last_update_time_;

  // Encoder counters
  int32_t encoder_count_[4] = {0, 0, 0, 0};
  int32_t last_encoder_count_[4] = {0, 0, 0, 0};

  // Timer for odometry calculation
  rclcpp::TimerBase::SharedPtr odom_timer_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Methods
  void initializeEncoders();
  void odometryCallback();
  void calculateOdometry();
  void updateEncoderCounts();
  void publishOdometry();
};

#endif  // ENCODER_ODOMETRY_ENCODER_NODE_HPP
