#ifndef CAMERA_SENSOR_CAMERA_NODE_HPP
#define CAMERA_SENSOR_CAMERA_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <libcamera/libcamera.h>

class CameraNode : public rclcpp::Node {
public:
  CameraNode();
  ~CameraNode() override;

private:
  void initializeCamera();
  void closeCamera();
  void publishFrame();
  void requestComplete(libcamera::Request *request);

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  std::unique_ptr<libcamera::CameraManager> camera_manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::CameraConfiguration> config_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;

  int image_width_;
  int image_height_;
  double publish_rate_;
  std::string frame_id_;
  std::string camera_topic_;

  std::chrono::steady_clock::time_point last_retry_time_;
  libcamera::PixelFormat pixel_format_;
  size_t frame_size_;
  bool camera_initialized_;
};

#endif  // CAMERA_SENSOR_CAMERA_NODE_HPP