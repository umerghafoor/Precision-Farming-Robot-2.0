#include "camera_sensor/camera_node.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

CameraNode::CameraNode() : Node("camera_node"), camera_fd_(-1), frame_bytes_(0U) {
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("image_width", 640);
  this->declare_parameter("image_height", 480);
  this->declare_parameter("publish_rate", 30.0);
  this->declare_parameter("frame_id", "camera_link");
  this->declare_parameter("camera_topic", "/camera/raw");

  video_device_ = this->get_parameter("video_device").as_string();
  image_width_ = this->get_parameter("image_width").as_int();
  image_height_ = this->get_parameter("image_height").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();
  camera_topic_ = this->get_parameter("camera_topic").as_string();

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_topic_, 10);
  initializeCamera();

  // Keep timer period valid even if publish_rate is set to 0 accidentally.
  const double safe_rate = std::max(1.0, publish_rate_);
  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / safe_rate));
  publish_timer_ = this->create_wall_timer(period, std::bind(&CameraNode::publishFrame, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Camera node initialized (device=%s, %dx%d, topic=%s, rate=%.1f Hz)",
    video_device_.c_str(), image_width_, image_height_, camera_topic_.c_str(), safe_rate);
}

CameraNode::~CameraNode() {
  closeCamera();
}

void CameraNode::initializeCamera() {
  closeCamera();

  camera_fd_ = open(video_device_.c_str(), O_RDWR);
  if (camera_fd_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not open %s: %s",
      video_device_.c_str(),
      std::strerror(errno));
    return;
  }

  v4l2_format fmt;
  std::memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = static_cast<uint32_t>(image_width_);
  fmt.fmt.pix.height = static_cast<uint32_t>(image_height_);
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (ioctl(camera_fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "VIDIOC_S_FMT failed for %s: %s",
      video_device_.c_str(),
      std::strerror(errno));
    closeCamera();
    return;
  }

  image_width_ = static_cast<int>(fmt.fmt.pix.width);
  image_height_ = static_cast<int>(fmt.fmt.pix.height);

  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Camera %s does not support YUYV output after format set.",
      video_device_.c_str());
    closeCamera();
    return;
  }

  frame_bytes_ = static_cast<size_t>(image_width_) * static_cast<size_t>(image_height_) * 2U;
  frame_buffer_.resize(frame_bytes_);

  RCLCPP_INFO(
    this->get_logger(),
    "Camera opened: %s (%dx%d, encoding=yuv422_yuy2)",
    video_device_.c_str(),
    image_width_,
    image_height_);
}

void CameraNode::closeCamera() {
  if (camera_fd_ >= 0) {
    close(camera_fd_);
    camera_fd_ = -1;
  }
}

void CameraNode::publishFrame() {
  if (camera_fd_ < 0) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_retry_time_ > std::chrono::seconds(1)) {
      RCLCPP_WARN(this->get_logger(), "Camera not available, retrying...");
      initializeCamera();
      last_retry_time_ = now;
    }
    return;
  }

  if (frame_buffer_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Frame buffer not initialized");
    return;
  }

  const ssize_t bytes_read = read(camera_fd_, frame_buffer_.data(), frame_bytes_);
  if (bytes_read < 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Camera read failed: %s", std::strerror(errno));
    closeCamera();
    return;
  }

  if (static_cast<size_t>(bytes_read) != frame_bytes_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to grab frame");
    return;
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.height = static_cast<uint32_t>(image_height_);
  msg.width = static_cast<uint32_t>(image_width_);
  msg.encoding = "yuv422_yuy2";
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image_width_ * 2);
  msg.data.assign(frame_buffer_.begin(), frame_buffer_.end());

  image_pub_->publish(msg);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}