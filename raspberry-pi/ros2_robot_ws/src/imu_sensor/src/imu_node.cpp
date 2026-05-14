#include "imu_sensor/imu_node.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;

// ── Helpers ───────────────────────────────────────────────────────────────────

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

static bool configureSerial(int fd, int baud) {
  struct termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  (void)baud;  // only 115200 used
  cfmakeraw(&tty);
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 10;  // 1 s read timeout (tenths of seconds)
  return tcsetattr(fd, TCSANOW, &tty) == 0;
}

// Read one '\n'-terminated line from fd (blocking up to timeout_ms).
static std::string readLine(int fd, int timeout_ms = 1000) {
  std::string line;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    char c;
    ssize_t n = read(fd, &c, 1);
    if (n == 1) {
      if (c == '\n') return line;
      if (c != '\r') line += c;
    }
  }
  return line;
}

// ── IMUNode ───────────────────────────────────────────────────────────────────

IMUNode::IMUNode() : Node("imu_node") {
  this->declare_parameter<std::string>("serial_port", "auto");
  this->declare_parameter<double>("publish_rate_hz", 50.0);

  const auto port_param = this->get_parameter("serial_port").as_string();
  const double hz = this->get_parameter("publish_rate_hz").as_double();

  serial_port_ = detectPort(port_param);
  if (serial_port_.empty()) {
    throw std::runtime_error("sensor_node not found on any USB port. "
                             "Set serial_port parameter or check connections.");
  }

  if (!openSerial(serial_port_, 115200)) {
    throw std::runtime_error("Failed to open serial port: " + serial_port_);
  }

  RCLCPP_INFO(get_logger(), "Sensor node connected on %s", serial_port_.c_str());

  // Publishers
  imu_pub_         = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  laser_state_pub_ = create_publisher<std_msgs::msg::Bool>("/laser/state", 10);

  // Laser service: call with data=true to turn ON, data=false to turn OFF
  laser_srv_ = create_service<std_srvs::srv::SetBool>(
    "/laser/set",
    std::bind(&IMUNode::laserServiceCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Publish timer
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / hz));
  publish_timer_ = create_wall_timer(period, std::bind(&IMUNode::publishCallback, this));

  // Background serial reader
  stop_reader_ = false;
  reader_thread_ = std::thread(&IMUNode::readerLoop, this);

  RCLCPP_INFO(get_logger(), "IMU node ready. publish=%.0f Hz  laser=/laser/set", hz);
}

IMUNode::~IMUNode() {
  stop_reader_ = true;
  if (reader_thread_.joinable()) reader_thread_.join();
  closeSerial();
}

// ── Port detection ────────────────────────────────────────────────────────────

std::string IMUNode::detectPort(const std::string & hint) {
  if (hint != "auto") {
    RCLCPP_INFO(get_logger(), "Using configured serial port: %s", hint.c_str());
    return hint;
  }

  RCLCPP_INFO(get_logger(), "Auto-detecting sensor node port...");
  const auto ports = listUsbPorts();
  if (ports.empty()) {
    RCLCPP_ERROR(get_logger(), "No USB serial ports found.");
    return {};
  }

  const std::string target = "NODE_ID:sensor_node";

  for (const auto & port : ports) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) continue;
    // Switch to blocking
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    if (!configureSerial(fd, 115200)) { close(fd); continue; }

    // Drain 1 s of IMU flood so WHOAMI reply isn't buried
    auto drain_end = std::chrono::steady_clock::now() + 1000ms;
    while (std::chrono::steady_clock::now() < drain_end) {
      char buf[64];
      if (read(fd, buf, sizeof(buf)) <= 0) break;
    }
    tcflush(fd, TCIFLUSH);

    // Send WHOAMI
    const char * whoami = "WHOAMI\n";
    write(fd, whoami, strlen(whoami));

    // Read reply (up to 2 s)
    auto reply_end = std::chrono::steady_clock::now() + 2000ms;
    bool found = false;
    while (std::chrono::steady_clock::now() < reply_end) {
      std::string line = readLine(fd, 200);
      if (line == target) { found = true; break; }
      if (!line.empty() && line.rfind("NODE_ID:", 0) == 0) break;  // wrong node
    }

    close(fd);
    if (found) {
      RCLCPP_INFO(get_logger(), "Found sensor_node on %s", port.c_str());
      return port;
    }
  }

  RCLCPP_ERROR(get_logger(), "sensor_node not found on any port.");
  return {};
}

// ── Serial open/close ─────────────────────────────────────────────────────────

bool IMUNode::openSerial(const std::string & port, int baud) {
  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) return false;
  if (!configureSerial(serial_fd_, baud)) {
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }
  // Flush startup noise
  std::this_thread::sleep_for(100ms);
  tcflush(serial_fd_, TCIFLUSH);
  return true;
}

void IMUNode::closeSerial() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ── Serial reader thread ──────────────────────────────────────────────────────

void IMUNode::readerLoop() {
  // A:ax,ay,az G:gx,gy,gz
  const std::regex imu_re(
    R"(A:(-?\d+),(-?\d+),(-?\d+)\s+G:(-?\d+),(-?\d+),(-?\d+))");

  while (!stop_reader_) {
    if (serial_fd_ < 0) {
      std::this_thread::sleep_for(500ms);
      continue;
    }

    std::string line = readLine(serial_fd_, 200);
    if (line.empty()) continue;

    std::smatch m;
    if (std::regex_search(line, m, imu_re)) {
      const float ax = std::stof(m[1]) / ACC_SCALE * G_TO_MS2;
      const float ay = std::stof(m[2]) / ACC_SCALE * G_TO_MS2;
      const float az = std::stof(m[3]) / ACC_SCALE * G_TO_MS2;
      const float gx = std::stof(m[4]) / GYR_SCALE * DEG_TO_RAD;
      const float gy = std::stof(m[5]) / GYR_SCALE * DEG_TO_RAD;
      const float gz = std::stof(m[6]) / GYR_SCALE * DEG_TO_RAD;

      std::lock_guard<std::mutex> lk(imu_mutex_);
      ax_ = ax; ay_ = ay; az_ = az;
      gx_ = gx; gy_ = gy; gz_ = gz;
      imu_fresh_ = true;
    }
  }
}

// ── ROS callbacks ─────────────────────────────────────────────────────────────

void IMUNode::publishCallback() {
  std::lock_guard<std::mutex> lk(imu_mutex_);
  if (!imu_fresh_) return;
  imu_fresh_ = false;

  sensor_msgs::msg::Imu msg;
  msg.header.stamp    = now();
  msg.header.frame_id = "imu_link";

  msg.linear_acceleration.x = ax_;
  msg.linear_acceleration.y = ay_;
  msg.linear_acceleration.z = az_;
  msg.linear_acceleration_covariance[0] = 0.01;
  msg.linear_acceleration_covariance[4] = 0.01;
  msg.linear_acceleration_covariance[8] = 0.01;

  msg.angular_velocity.x = gx_;
  msg.angular_velocity.y = gy_;
  msg.angular_velocity.z = gz_;
  msg.angular_velocity_covariance[0] = 0.001;
  msg.angular_velocity_covariance[4] = 0.001;
  msg.angular_velocity_covariance[8] = 0.001;

  // Orientation unknown
  msg.orientation_covariance[0] = -1.0;

  imu_pub_->publish(msg);
}

void IMUNode::laserServiceCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr  req,
  const std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (serial_fd_ < 0) {
    res->success = false;
    res->message = "Serial port not open";
    return;
  }

  const char * cmd = req->data ? "LASER_ON\n" : "LASER_OFF\n";
  ssize_t written = write(serial_fd_, cmd, strlen(cmd));
  if (written < 0) {
    res->success = false;
    res->message = "Serial write failed";
    return;
  }

  laser_on_ = req->data;
  res->success = true;
  res->message = laser_on_ ? "Laser ON" : "Laser OFF";

  std_msgs::msg::Bool state_msg;
  state_msg.data = laser_on_;
  laser_state_pub_->publish(state_msg);

  RCLCPP_INFO(get_logger(), "Laser %s", laser_on_ ? "ON" : "OFF");
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
