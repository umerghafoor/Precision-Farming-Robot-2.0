#include "motor_control/spi_controller_bridge.hpp"

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

SPIControllerBridge::SPIControllerBridge()
: Node("spi_controller_bridge"),
  spi_fd_(-1),
  spi_mode_(0),
  spi_bits_per_word_(8),
  spi_speed_hz_(500000),
  wheel_base_(0.2),
  max_linear_velocity_(1.0),
  cmd_timeout_sec_(0.5),
  latest_cmd_time_(this->now()),
  servo1_angle_(90),
  servo2_angle_(90) {
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<std::string>("servo1_topic", "/servo1/angle");
  this->declare_parameter<std::string>("servo2_topic", "/servo2/angle");
  this->declare_parameter<std::string>("spi_device", "/dev/spidev0.0");
  this->declare_parameter<int>("spi_mode", 0);
  this->declare_parameter<int>("spi_bits_per_word", 8);
  this->declare_parameter<int>("spi_speed_hz", 500000);
  this->declare_parameter<double>("wheel_base", 0.2);
  this->declare_parameter<double>("max_linear_velocity", 1.0);
  this->declare_parameter<double>("cmd_timeout_sec", 0.5);
  this->declare_parameter<double>("tx_rate_hz", 20.0);
  this->declare_parameter<int>("default_servo_angle", 90);

  const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  const auto servo1_topic = this->get_parameter("servo1_topic").as_string();
  const auto servo2_topic = this->get_parameter("servo2_topic").as_string();

  spi_device_ = this->get_parameter("spi_device").as_string();
  spi_mode_ = static_cast<uint8_t>(this->get_parameter("spi_mode").as_int());
  spi_bits_per_word_ = static_cast<uint8_t>(this->get_parameter("spi_bits_per_word").as_int());
  spi_speed_hz_ = static_cast<uint32_t>(this->get_parameter("spi_speed_hz").as_int());
  wheel_base_ = this->get_parameter("wheel_base").as_double();
  max_linear_velocity_ = std::max(0.01, this->get_parameter("max_linear_velocity").as_double());
  cmd_timeout_sec_ = std::max(0.05, this->get_parameter("cmd_timeout_sec").as_double());
  const auto tx_rate_hz = std::max(1.0, this->get_parameter("tx_rate_hz").as_double());
  const int default_servo = this->get_parameter("default_servo_angle").as_int();
  servo1_angle_ = clampServoAngle(default_servo);
  servo2_angle_ = clampServoAngle(default_servo);

  openAndConfigureSPI();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10, std::bind(&SPIControllerBridge::cmdVelCallback, this, std::placeholders::_1));
  servo1_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    servo1_topic, 10, std::bind(&SPIControllerBridge::servo1Callback, this, std::placeholders::_1));
  servo2_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    servo2_topic, 10, std::bind(&SPIControllerBridge::servo2Callback, this, std::placeholders::_1));

  const auto tx_period = std::chrono::duration<double>(1.0 / tx_rate_hz);
  tx_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tx_period),
    std::bind(&SPIControllerBridge::transmitTimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "SPI bridge started. cmd_vel='%s', servo1='%s', servo2='%s', spi='%s', hz=%u",
    cmd_vel_topic.c_str(),
    servo1_topic.c_str(),
    servo2_topic.c_str(),
    spi_device_.c_str(),
    spi_speed_hz_);
}

SPIControllerBridge::~SPIControllerBridge() {
  if (spi_fd_ >= 0) {
    close(spi_fd_);
    spi_fd_ = -1;
  }
}

void SPIControllerBridge::openAndConfigureSPI() {
  spi_fd_ = open(spi_device_.c_str(), O_RDWR);
  if (spi_fd_ < 0) {
    throw std::runtime_error(
            "Failed to open SPI device " + spi_device_ + ": " + std::strerror(errno));
  }

  auto throw_with_cleanup = [this](const std::string & msg) {
    close(spi_fd_);
    spi_fd_ = -1;
    throw std::runtime_error(msg + ": " + std::string(std::strerror(errno)));
  };

  if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_RD_MODE, &spi_mode_) < 0) {
    throw_with_cleanup("Failed to set SPI mode");
  }

  if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word_) < 0) {
    throw_with_cleanup("Failed to set SPI bits per word");
  }

  if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz_) < 0 ||
      ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed_hz_) < 0) {
    throw_with_cleanup("Failed to set SPI max speed");
  }
}

void SPIControllerBridge::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  latest_cmd_vel_ = *msg;
  latest_cmd_time_ = this->now();
}

void SPIControllerBridge::servo1Callback(const std_msgs::msg::Int16::SharedPtr msg) {
  servo1_angle_ = clampServoAngle(static_cast<int>(msg->data));
}

void SPIControllerBridge::servo2Callback(const std_msgs::msg::Int16::SharedPtr msg) {
  servo2_angle_ = clampServoAngle(static_cast<int>(msg->data));
}

void SPIControllerBridge::transmitTimerCallback() {
  const bool timed_out = (this->now() - latest_cmd_time_).seconds() > cmd_timeout_sec_;

  if (timed_out) {
    latest_cmd_vel_.linear.x = 0.0;
    latest_cmd_vel_.angular.z = 0.0;
  }

  const auto packet = buildPacket();
  if (!transmitPacket(packet)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "SPI transfer failed. Check wiring, permissions, and SPI device path.");
  }
}

std::array<uint8_t, 8> SPIControllerBridge::buildPacket() const {
  // Packet format: [Dir1, Speed1, Dir2, Speed2, Dir3, Speed3, Servo1Angle, Servo2Angle]
  const double linear = latest_cmd_vel_.linear.x;
  const double angular = latest_cmd_vel_.angular.z;

  const double left = linear - (angular * wheel_base_ / 2.0);
  const double right = linear + (angular * wheel_base_ / 2.0);

  const double left_normalized = std::clamp(left / max_linear_velocity_, -1.0, 1.0);
  const double right_normalized = std::clamp(right / max_linear_velocity_, -1.0, 1.0);

  const MotorCommand motor1 = toMotorCommand(left_normalized);
  const MotorCommand motor2 = toMotorCommand(right_normalized);
  const MotorCommand motor3 = toMotorCommand(left_normalized);

  return {
    motor1.direction,
    motor1.speed,
    motor2.direction,
    motor2.speed,
    motor3.direction,
    motor3.speed,
    servo1_angle_,
    servo2_angle_};
}

SPIControllerBridge::MotorCommand SPIControllerBridge::toMotorCommand(double normalized_speed) const {
  const double clamped = std::clamp(normalized_speed, -1.0, 1.0);
  if (std::abs(clamped) < 1e-3) {
    return {DIR_STOP, 0};
  }

  const uint8_t direction = clamped >= 0.0 ? DIR_FORWARD : DIR_BACKWARD;
  const uint8_t speed = static_cast<uint8_t>(std::round(std::abs(clamped) * 255.0));
  return {direction, speed};
}

uint8_t SPIControllerBridge::clampServoAngle(int angle) const {
  return static_cast<uint8_t>(std::clamp(angle, 0, 180));
}

bool SPIControllerBridge::transmitPacket(const std::array<uint8_t, 8> & packet) {
  if (spi_fd_ < 0) {
    return false;
  }

  std::array<uint8_t, 8> rx_buffer{};

  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<unsigned long>(packet.data());
  transfer.rx_buf = reinterpret_cast<unsigned long>(rx_buffer.data());
  transfer.len = packet.size();
  transfer.speed_hz = spi_speed_hz_;
  transfer.bits_per_word = spi_bits_per_word_;
  transfer.delay_usecs = 0;

  return ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer) >= 0;
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPIControllerBridge>());
  rclcpp::shutdown();
  return 0;
}
