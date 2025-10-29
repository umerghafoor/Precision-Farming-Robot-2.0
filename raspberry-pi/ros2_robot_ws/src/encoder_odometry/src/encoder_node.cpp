#include "encoder_odometry/encoder_node.hpp"
#include <cmath>

const int EncoderNode::COUNTS_PER_REVOLUTION;

EncoderNode::EncoderNode() : Node("encoder_node") {
  // Declare parameters
  this->declare_parameter("wheel_radius", 0.05);      // meters
  this->declare_parameter("wheel_base", 0.2);         // distance between wheels
  this->declare_parameter("counts_per_rev", COUNTS_PER_REVOLUTION);
  this->declare_parameter("update_rate", 20.0);       // Hz

  // Initialize encoders
  initializeEncoders();

  // Create odometry publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // Initialize time
  last_update_time_ = this->now();

  // Create timer for odometry calculation
  double update_rate = this->get_parameter("update_rate").as_double();
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate));
  odom_timer_ = this->create_wall_timer(period, std::bind(&EncoderNode::odometryCallback, this));

  RCLCPP_INFO(this->get_logger(), "Encoder Node initialized with update rate: %.1f Hz", update_rate);
}

void EncoderNode::initializeEncoders() {
  RCLCPP_INFO(this->get_logger(), "Initializing encoder pins...");

  // In a real implementation, you would:
  // 1. Export GPIO pins
  // 2. Set pins to input mode
  // 3. Attach interrupt handlers for each encoder channel
  // 4. Set up counter registers

  // Example using wiringPi or gpiozero libraries
  // pinMode(ENCODER1_A_PIN, INPUT);
  // pinMode(ENCODER1_B_PIN, INPUT);
  // wiringPiISR(ENCODER1_A_PIN, INT_EDGE_RISING, &encoder1_isr);

  RCLCPP_INFO(this->get_logger(), "Encoder pins initialized");
}

void EncoderNode::odometryCallback() {
  updateEncoderCounts();
  calculateOdometry();
  publishOdometry();
}

void EncoderNode::updateEncoderCounts() {
  // In a real implementation, you would read encoder counts from GPIO
  // For now, using simulated values
  
  // Example: Read from /sys/class/gpio or use a library
  // FILE* fp = fopen("/sys/class/gpio/gpio4/value", "r");
  // fscanf(fp, "%d", &encoder_count_[0]);
  
  // Simulated: encoder counts remain at last value
}

void EncoderNode::calculateOdometry() {
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();

  if (dt <= 0.0) return;

  // Get robot parameters
  double wheel_radius = this->get_parameter("wheel_radius").as_double();
  double wheel_base = this->get_parameter("wheel_base").as_double();
  int counts_per_rev = this->get_parameter("counts_per_rev").as_int();

  // Calculate wheel distances traveled
  double wheel_circumference = 2.0 * M_PI * wheel_radius;
  double distance_per_count = wheel_circumference / counts_per_rev;

  // Calculate distance traveled by each side
  double left_distance = 0.0, right_distance = 0.0;

  // Left motors (1 and 3)
  int32_t left_delta = (encoder_count_[0] - last_encoder_count_[0] +
                        encoder_count_[2] - last_encoder_count_[2]) / 2;
  left_distance = left_delta * distance_per_count;

  // Right motors (2 and 4)
  int32_t right_delta = (encoder_count_[1] - last_encoder_count_[1] +
                         encoder_count_[3] - last_encoder_count_[3]) / 2;
  right_distance = right_delta * distance_per_count;

  // Average distance traveled
  double distance = (left_distance + right_distance) / 2.0;

  // Distance difference for rotation
  double delta_theta = (right_distance - left_distance) / wheel_base;

  // Update position using differential drive kinematics
  double avg_theta = theta_ + delta_theta / 2.0;

  x_ += distance * cos(avg_theta);
  y_ += distance * sin(avg_theta);
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

  // Update time and encoder counts
  last_update_time_ = current_time;
  for (int i = 0; i < 4; i++) {
    last_encoder_count_[i] = encoder_count_[i];
  }

  RCLCPP_DEBUG(this->get_logger(), "Odometry - x: %.3f, y: %.3f, theta: %.3f",
               x_, y_, theta_);
}

void EncoderNode::publishOdometry() {
  // Create odometry message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // Set position
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  // Convert theta to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  // Set covariance
  odom_msg.pose.covariance[0] = 0.1;     // x variance
  odom_msg.pose.covariance[7] = 0.1;     // y variance
  odom_msg.pose.covariance[35] = 0.1;    // theta variance

  // Set twist (velocities) - would need to calculate from encoder deltas
  odom_msg.twist.twist.linear.x = 0.0;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = 0.0;

  odom_pub_->publish(odom_msg);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderNode>());
  rclcpp::shutdown();
  return 0;
}
