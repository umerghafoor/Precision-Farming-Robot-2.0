#include "motor_control/spi_controller_bridge.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;

// ── Serial helpers ────────────────────────────────────────────────────────────

static std::vector<std::string> listUsbPorts() {
  std::vector<std::string> ports;
  for (int i = 0; i <= 9; ++i) {
    std::string p = "/dev/ttyUSB" + std::to_string(i);
    if (access(p.c_str(), F_OK) == 0) ports.push_back(p);
  }
  for (int i = 0; i <= 9; ++i) {
    std::string p = "/dev/ttyACM" + std::to_string(i);
    if (access(p.c_str(), F_OK) == 0) ports.push_back(p);
  }
  return ports;
}

static bool configureSerial(int fd) {
  struct termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  cfmakeraw(&tty);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 10;  // 1 s read timeout
  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

static std::string readLine(int fd, int timeout_ms = 1000) {
  std::string line;
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    char c;
    if (read(fd, &c, 1) == 1) {
      if (c == '\n') return line;
      if (c != '\r') line += c;
    }
  }
  return line;
}

// ── SPIControllerBridge ───────────────────────────────────────────────────────

SPIControllerBridge::SPIControllerBridge()
: Node("spi_controller_bridge"),
  wheel_base_(0.2),
  max_linear_velocity_(1.0),
  cmd_timeout_sec_(0.5),
  latest_cmd_time_(this->now()),
  servo1_angle_(90),
  servo2_angle_(90)
{
  this->declare_parameter<std::string>("cmd_vel_topic",   "/cmd_vel");   // listen directly — no safety wrapper needed
  this->declare_parameter<std::string>("servo1_topic",    "/servo1/angle");
  this->declare_parameter<std::string>("servo2_topic",    "/servo2/angle");
  this->declare_parameter<std::string>("serial_port",     "auto");
  this->declare_parameter<double>("wheel_base",           0.2);
  this->declare_parameter<double>("max_linear_velocity",  1.0);
  this->declare_parameter<double>("cmd_timeout_sec",      0.5);
  this->declare_parameter<double>("tx_rate_hz",           20.0);
  this->declare_parameter<int>("default_servo_angle",     90);

  const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  const auto servo1_topic  = this->get_parameter("servo1_topic").as_string();
  const auto servo2_topic  = this->get_parameter("servo2_topic").as_string();
  const auto port_hint     = this->get_parameter("serial_port").as_string();

  wheel_base_          = this->get_parameter("wheel_base").as_double();
  max_linear_velocity_ = std::max(0.01, this->get_parameter("max_linear_velocity").as_double());
  cmd_timeout_sec_     = std::max(0.05, this->get_parameter("cmd_timeout_sec").as_double());
  const double tx_rate_hz = std::max(1.0, this->get_parameter("tx_rate_hz").as_double());
  const int default_servo = this->get_parameter("default_servo_angle").as_int();
  servo1_angle_ = clampServoAngle(default_servo);
  servo2_angle_ = clampServoAngle(default_servo);

  serial_port_ = detectSerialPort(port_hint);
  if (!serial_port_.empty()) {
    if (!openSerial(serial_port_)) {
      RCLCPP_WARN(get_logger(),
        "Could not open %s — motor commands will be dropped.",
        serial_port_.c_str());
      serial_port_.clear();
    }
  }

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic, 10,
    std::bind(&SPIControllerBridge::cmdVelCallback, this, std::placeholders::_1));
  servo1_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    servo1_topic, 10,
    std::bind(&SPIControllerBridge::servo1Callback, this, std::placeholders::_1));
  servo2_sub_ = this->create_subscription<std_msgs::msg::Int16>(
    servo2_topic, 10,
    std::bind(&SPIControllerBridge::servo2Callback, this, std::placeholders::_1));

  const auto tx_period = std::chrono::duration<double>(1.0 / tx_rate_hz);
  tx_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(tx_period),
    std::bind(&SPIControllerBridge::transmitTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "Motor controller bridge ready. port='%s'  cmd_vel='%s'"
    "  servo1='%s'  servo2='%s'  tx=%.0f Hz",
    serial_port_.empty() ? "(none)" : serial_port_.c_str(),
    cmd_vel_topic.c_str(), servo1_topic.c_str(),
    servo2_topic.c_str(), tx_rate_hz);
}

SPIControllerBridge::~SPIControllerBridge() {
  closeSerial();
}

// ── Port detection ────────────────────────────────────────────────────────────

std::string SPIControllerBridge::detectSerialPort(const std::string & hint) {
  if (hint != "auto") {
    RCLCPP_INFO(get_logger(), "Using configured serial port: %s", hint.c_str());
    return hint;
  }

  RCLCPP_INFO(get_logger(), "Auto-detecting motor controller port...");
  const auto ports = listUsbPorts();
  if (ports.empty()) {
    RCLCPP_WARN(get_logger(),
      "No USB serial ports found. Motor commands will be dropped.");
    return {};
  }

  const std::string target = "NODE_ID:motor_controller";

  for (const auto & port : ports) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) continue;
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    if (!configureSerial(fd)) { close(fd); continue; }

    // Drain 1 s of any startup noise (motor controller may be printing)
    auto drain_end = std::chrono::steady_clock::now() + 1000ms;
    while (std::chrono::steady_clock::now() < drain_end) {
      char buf[64];
      if (read(fd, buf, sizeof(buf)) <= 0) break;
    }
    tcflush(fd, TCIFLUSH);

    const char * whoami = "WHOAMI\n";
    write(fd, whoami, strlen(whoami));

    auto reply_end = std::chrono::steady_clock::now() + 2000ms;
    bool found = false;
    while (std::chrono::steady_clock::now() < reply_end) {
      std::string line = readLine(fd, 200);
      if (line == target) { found = true; break; }
      if (!line.empty() && line.rfind("NODE_ID:", 0) == 0) break;  // different node
    }

    close(fd);
    if (found) {
      RCLCPP_INFO(get_logger(), "Found motor_controller on %s", port.c_str());
      return port;
    }
  }

  RCLCPP_WARN(get_logger(),
    "motor_controller not found on any USB port. "
    "Motor commands will be dropped. Pass serial_port:=<path> to override.");
  return {};
}

// ── Serial open/close ─────────────────────────────────────────────────────────

bool SPIControllerBridge::openSerial(const std::string & port) {
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) return false;
  if (!configureSerial(serial_fd_)) {
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  std::this_thread::sleep_for(100ms);
  tcflush(serial_fd_, TCIFLUSH);
  RCLCPP_INFO(get_logger(), "Serial port %s opened at 115200 baud", port.c_str());
  return true;
}

void SPIControllerBridge::closeSerial() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ── ROS callbacks ─────────────────────────────────────────────────────────────

void SPIControllerBridge::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_  = *msg;
  latest_cmd_time_ = this->now();
}

void SPIControllerBridge::servo1Callback(
  const std_msgs::msg::Int16::SharedPtr msg)
{
  const uint8_t angle = clampServoAngle(static_cast<int>(msg->data));
  servo1_angle_ = angle;
  if (angle != prev_servo1_angle_) {
    sendServoCommand(1, angle);
    prev_servo1_angle_ = angle;
  }
}

void SPIControllerBridge::servo2Callback(
  const std_msgs::msg::Int16::SharedPtr msg)
{
  const uint8_t angle = clampServoAngle(static_cast<int>(msg->data));
  servo2_angle_ = angle;
  if (angle != prev_servo2_angle_) {
    sendServoCommand(2, angle);
    prev_servo2_angle_ = angle;
  }
}

void SPIControllerBridge::transmitTimerCallback() {
  const bool timed_out =
    (this->now() - latest_cmd_time_).seconds() > cmd_timeout_sec_;
  if (timed_out) {
    latest_cmd_vel_.linear.x  = 0.0;
    latest_cmd_vel_.angular.z = 0.0;
  }

  if (serial_fd_ < 0) return;  // no hardware — silent

  const auto packet = buildPacket();
  if (!transmitPacket(packet)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *this->get_clock(), 2000,
      "Serial write failed on %s. Check connection.",
      serial_port_.c_str());
  }
}

// ── Packet building ───────────────────────────────────────────────────────────

std::array<uint8_t, 6> SPIControllerBridge::buildPacket() const {
  // Firmware SIGNAL_SIZE = 6: [Dir1,Spd1,Dir2,Spd2,Dir3,Spd3]
  // Servo angles are sent separately as text "S1:<deg>\n" — NOT in this packet.
  const double linear  = latest_cmd_vel_.linear.x;
  const double angular = latest_cmd_vel_.angular.z;

  const double left  = linear - (angular * wheel_base_ / 2.0);
  const double right = linear + (angular * wheel_base_ / 2.0);

  const double left_n  = std::clamp(left  / max_linear_velocity_, -1.0, 1.0);
  const double right_n = std::clamp(right / max_linear_velocity_, -1.0, 1.0);

  const MotorCommand m1 = toMotorCommand(left_n);
  const MotorCommand m2 = toMotorCommand(right_n);
  const MotorCommand m3 = toMotorCommand(left_n);  // motor3 mirrors left side

  return {m1.direction, m1.speed,
          m2.direction, m2.speed,
          m3.direction, m3.speed};
}

SPIControllerBridge::MotorCommand
SPIControllerBridge::toMotorCommand(double normalized_speed) const {
  const double clamped = std::clamp(normalized_speed, -1.0, 1.0);
  if (std::abs(clamped) < 1e-3) return {DIR_STOP, 0};
  return {
    clamped >= 0.0 ? DIR_FORWARD : DIR_BACKWARD,
    static_cast<uint8_t>(std::round(std::abs(clamped) * 255.0))
  };
}

uint8_t SPIControllerBridge::clampServoAngle(int angle) const {
  return static_cast<uint8_t>(std::clamp(angle, 0, 180));
}

bool SPIControllerBridge::transmitPacket(const std::array<uint8_t, 6> & packet) {
  std::lock_guard<std::mutex> lk(write_mutex_);
  if (serial_fd_ < 0) return false;
  return write(serial_fd_, packet.data(), packet.size()) ==
         static_cast<ssize_t>(packet.size());
}

void SPIControllerBridge::sendServoCommand(int servo_id, uint8_t angle) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "S%d:%d\n", servo_id, static_cast<int>(angle));
  std::lock_guard<std::mutex> lk(write_mutex_);
  if (serial_fd_ < 0) return;
  write(serial_fd_, cmd, strlen(cmd));
  RCLCPP_DEBUG(get_logger(), "Servo%d -> %d°", servo_id, static_cast<int>(angle));
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPIControllerBridge>());
  rclcpp::shutdown();
  return 0;
}
