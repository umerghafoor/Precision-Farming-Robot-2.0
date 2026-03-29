#ifndef CAMERA_SENSOR_CAMERA_NODE_HPP
#define CAMERA_SENSOR_CAMERA_NODE_HPP

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node {
public:
  CameraNode();
  ~CameraNode() override;

private:
  void initializeCamera();
  void closeCamera();
  void publishFrame();

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  int camera_fd_;
  size_t frame_bytes_;
  std::vector<uint8_t> frame_buffer_;

  std::string video_device_;
  int image_width_;
  int image_height_;
  double publish_rate_;
  std::string frame_id_;
  std::string camera_topic_;

  std::chrono::steady_clock::time_point last_retry_time_;
};

#endif  // CAMERA_SENSOR_CAMERA_NODE_HPP