#ifndef IMU_SENSOR_IMU_NODE_HPP
#define IMU_SENSOR_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

class IMUNode : public rclcpp::Node {
public:
  IMUNode();
  ~IMUNode() override;

private:
  // ── Serial port ──────────────────────────────────────────────────────────
  std::string serial_port_;
  int         serial_fd_{-1};

  bool openSerial(const std::string & port, int baud);
  void closeSerial();

  // Serialize all writes to serial_fd_ (reader thread reads, callbacks write)
  std::mutex  write_mutex_;
  bool serialWrite(const char * cmd);

  // Auto-detect: drain IMU flood, send WHOAMI, confirm NODE_ID:sensor_node
  std::string detectPort(const std::string & hint);

  // ── Serial reader thread ─────────────────────────────────────────────────
  std::thread       reader_thread_;
  std::atomic<bool> stop_reader_{false};
  void readerLoop();

  // Latest parsed IMU values (updated by reader thread)
  std::mutex  imu_mutex_;
  float ax_{0}, ay_{0}, az_{0};
  float gx_{0}, gy_{0}, gz_{0};
  bool  imu_fresh_{false};

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     laser_state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr    laser_srv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  laser_sub_;
  rclcpp::TimerBase::SharedPtr                          publish_timer_;

  void publishCallback();
  void laserServiceCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr  req,
    const std_srvs::srv::SetBool::Response::SharedPtr res);
  void laserTopicCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void applyLaser(bool on);

  // ── State ─────────────────────────────────────────────────────────────────
  bool laser_on_{false};

  // ── Scaling factors (must match firmware_sensors/main.py) ─────────────────
  // Firmware sends: ax = accel_g * 1000,  gx = gyro_dps * 10
  static constexpr float ACC_SCALE  = 1000.0f;
  static constexpr float GYR_SCALE  = 10.0f;
  static constexpr float G_TO_MS2   = 9.80665f;
  static constexpr float DEG_TO_RAD = 0.017453293f;
};

#endif  // IMU_SENSOR_IMU_NODE_HPP
