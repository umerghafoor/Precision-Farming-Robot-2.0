#ifndef MOTOR_CONTROL_MOTOR_DRIVER_HPP
#define MOTOR_CONTROL_MOTOR_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MotorDriver : public rclcpp::Node {
public:
  MotorDriver();

private:
  // GPIO Pin assignments for L298N motor driver
  // Motor 1 (Front Left)
  static const int MOTOR1_PIN1 = 17;
  static const int MOTOR1_PIN2 = 27;
  static const int MOTOR1_PWM_PIN = 22;

  // Motor 2 (Front Right)
  static const int MOTOR2_PIN1 = 23;
  static const int MOTOR2_PIN2 = 24;
  static const int MOTOR2_PWM_PIN = 25;

  // Motor 3 (Rear Left)
  static const int MOTOR3_PIN1 = 5;
  static const int MOTOR3_PIN2 = 6;
  static const int MOTOR3_PWM_PIN = 12;

  // Motor 4 (Rear Right)
  static const int MOTOR4_PIN1 = 13;
  static const int MOTOR4_PIN2 = 19;
  static const int MOTOR4_PWM_PIN = 26;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Methods
  void initializeGPIO();
  void setupGPIOPins();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void setMotorSpeed(int motor_num, float speed);
  void setMotorDirection(int motor_num, bool forward);
  void pwmWrite(int pin, int value);
};

#endif  // MOTOR_CONTROL_MOTOR_DRIVER_HPP
