#include "camera_sensor/camera_node.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <sys/mman.h>

using namespace libcamera;

CameraNode::CameraNode()
  : Node("camera_node"),
    camera_initialized_(false),
    image_width_(640),
    image_height_(480),
    publish_rate_(30.0),
    frame_id_("camera_link"),
    camera_topic_("/camera/raw"),
    frame_size_(0) {
  // Declare parameters
  this->declare_parameter("image_width", 640);
  this->declare_parameter("image_height", 480);
  this->declare_parameter("publish_rate", 30.0);
  this->declare_parameter("frame_id", "camera_link");
  this->declare_parameter("camera_topic", "/camera/raw");

  // Get parameters
  image_width_ = this->get_parameter("image_width").as_int();
  image_height_ = this->get_parameter("image_height").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();
  camera_topic_ = this->get_parameter("camera_topic").as_string();

  // Create publisher
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_topic_, 10);

  // Initialize camera
  initializeCamera();

  // Create timer for publishing
  const double safe_rate = std::max(1.0, publish_rate_);
  const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / safe_rate));
  publish_timer_ = this->create_wall_timer(period, std::bind(&CameraNode::publishFrame, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Camera node initialized (%dx%d, topic=%s, rate=%.1f Hz)",
    image_width_, image_height_, camera_topic_.c_str(), safe_rate);
}

CameraNode::~CameraNode() {
  closeCamera();
}

void CameraNode::initializeCamera() {
  try {
    // Create and start camera manager
    camera_manager_ = std::make_unique<CameraManager>();
    camera_manager_->start();

    // Get list of cameras
    auto cameras = camera_manager_->cameras();
    if (cameras.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No cameras found");
      camera_manager_.reset();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Found %zu camera(s)", cameras.size());

    // Use first available camera
    camera_ = camera_manager_->get(cameras[0]);
    if (!camera_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get camera");
      camera_manager_.reset();
      return;
    }

    // Acquire camera
    if (camera_->acquire()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to acquire camera");
      camera_.reset();
      camera_manager_.reset();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Camera acquired: %s", camera_->id().c_str());

    // Generate default configuration
    config_ = camera_->generateConfiguration({StreamRole::VideoRecording});
    if (!config_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to generate camera configuration");
      camera_->release();
      camera_.reset();
      camera_manager_.reset();
      return;
    }

    // Set resolution
    StreamConfiguration &streamConfig = config_->at(0);
    streamConfig.size.width = image_width_;
    streamConfig.size.height = image_height_;

    // Try to set pixel format to handle different camera types
    // Prefer YUV formats for faster processing
    streamConfig.pixelFormat = formats::YUV420;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Requesting format: %s, resolution: %dx%d",
      streamConfig.pixelFormat.toString().c_str(),
      streamConfig.size.width,
      streamConfig.size.height);

    // Validate configuration
    if (config_->validate() != CameraConfiguration::Valid) {
      RCLCPP_WARN(this->get_logger(), "Configuration not fully valid, adjusting...");
      if (config_->validate() == CameraConfiguration::Invalid) {
        RCLCPP_ERROR(this->get_logger(), "Camera configuration invalid");
        camera_->release();
        camera_.reset();
        camera_manager_.reset();
        return;
      }
    }

    // Display adjusted configuration
    const StreamConfiguration &finalConfig = config_->at(0);
    RCLCPP_INFO(
      this->get_logger(),
      "Camera configured: %s, %ux%u, stride=%u",
      finalConfig.pixelFormat.toString().c_str(),
      finalConfig.size.width,
      finalConfig.size.height,
      finalConfig.stride);

    // Update our dimensions to match actual configuration
    image_width_ = finalConfig.size.width;
    image_height_ = finalConfig.size.height;
    pixel_format_ = finalConfig.pixelFormat;

    // Apply configuration
    if (camera_->configure(config_.get())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to configure camera");
      camera_->release();
      camera_.reset();
      config_.reset();
      camera_manager_.reset();
      return;
    }

    // Allocate frame buffers
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    for (StreamConfiguration &cfg : *config_) {
      if (allocator_->allocate(cfg.stream())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to allocate buffers");
        camera_->release();
        camera_.reset();
        config_.reset();
        allocator_.reset();
        camera_manager_.reset();
        return;
      }

      size_t buffer_count = allocator_->buffers(cfg.stream()).size();
      RCLCPP_DEBUG(this->get_logger(), "Allocated %zu buffers", buffer_count);
    }

    // Create requests
    requests_.clear();
    Stream *stream = config_->at(0).stream();
    auto buffers = allocator_->buffers(stream);

    for (auto &buffer : buffers) {
      auto request = camera_->createRequest();
      if (!request) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create request");
        camera_->release();
        camera_.reset();
        config_.reset();
        allocator_.reset();
        camera_manager_.reset();
        return;
      }

      if (request->addBuffer(stream, buffer.get())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add buffer to request");
        camera_->release();
        camera_.reset();
        config_.reset();
        allocator_.reset();
        camera_manager_.reset();
        return;
      }

      requests_.push_back(std::move(request));
    }

    // Start camera
    if (camera_->start()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start camera");
      camera_->release();
      camera_.reset();
      config_.reset();
      allocator_.reset();
      camera_manager_.reset();
      return;
    }

    // Queue initial requests
    for (auto &request : requests_) {
      if (camera_->queueRequest(request.get())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to queue request");
        camera_->stopAndRelease();
        camera_.reset();
        config_.reset();
        allocator_.reset();
        camera_manager_.reset();
        return;
      }
    }

    // Calculate frame size for YUV420 (12 bits per pixel)
    frame_size_ = (image_width_ * image_height_ * 12) / 8;

    camera_initialized_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Camera opened: libcamera (%dx%d, encoding=yuv420, frame_size=%zu bytes)",
      image_width_,
      image_height_,
      frame_size_);

  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Camera initialization failed: %s", e.what());
    closeCamera();
  }
}

void CameraNode::closeCamera() {
  camera_initialized_ = false;

  if (camera_ && camera_->isActive()) {
    try {
      camera_->stopAndRelease();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Error stopping camera: %s", e.what());
    }
  }

  requests_.clear();
  allocator_.reset();
  config_.reset();
  camera_.reset();

  if (camera_manager_) {
    try {
      camera_manager_->stop();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Error stopping camera manager: %s", e.what());
    }
  }
  camera_manager_.reset();
}

void CameraNode::publishFrame() {
  if (!camera_initialized_) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_retry_time_ > std::chrono::seconds(1)) {
      RCLCPP_WARN(this->get_logger(), "Camera not available, retrying...");
      initializeCamera();
      last_retry_time_ = now;
    }
    return;
  }

  try {
    // Wait for completed request (with timeout)
    auto completed = camera_->requestCompleted(std::chrono::milliseconds(100));
    if (!completed) {
      return;  // No frame ready yet, try again next cycle
    }

    auto metadata = completed->metadata();
    Stream *stream = config_->at(0).stream();
    FrameBuffer *buffer = completed->buffers().at(stream);

    if (!buffer) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No buffer in completed request");
      return;
    }

    // Get frame data
    const FrameBuffer::Plane &plane = buffer->planes()[0];
    auto dmabuf = std::get_if<FrameBuffer::Private::dmabuf>(&plane.fd.storage);
    if (!dmabuf) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to get DMA buffer");
      return;
    }

    int dmafd = dmabuf->fd;
    const auto map_length = plane.length + plane.offset;
    auto mapped = mmap(nullptr, map_length, PROT_READ, MAP_SHARED, dmafd, 0);

    if (mapped == MAP_FAILED) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to map buffer");
      return;
    }

    uint8_t *data = reinterpret_cast<uint8_t *>(mapped) + plane.offset;

    // Create ROS2 Image message
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = frame_id_;
    msg->height = image_height_;
    msg->width = image_width_;
    msg->encoding = "yuv420";
    msg->is_bigendian = false;
    msg->step = image_width_;
    msg->data.assign(data, data + frame_size_);

    image_pub_->publish(std::move(msg));

    // Unmap buffer
    munmap(mapped, map_length);

    // Requeue the request
    completed->reuse(FrameBuffer::ReuseFlag::Reuse);
    if (camera_->queueRequest(completed)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to requeue request");
      closeCamera();
    }

  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Error in publishFrame: %s", e.what());
    closeCamera();
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}