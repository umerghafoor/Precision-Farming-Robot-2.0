#ifndef ROS2INTERFACE_H
#define ROS2INTERFACE_H

#include <QObject>
#include <QThread>
#include <QByteArray>
#include <QVariant>
#include <memory>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#endif

/**
 * @brief ROS2 Interface running in separate thread
 * 
 * This class handles all ROS2 communication isolated from Qt main thread
 * Uses signal/slot mechanism to communicate with Qt UI
 */
class ROS2Interface : public QObject
{
    Q_OBJECT

public:
    explicit ROS2Interface(int argc, char** argv, QObject *parent = nullptr);
    ~ROS2Interface();

    bool initialize();
    void shutdown();

    // Publishing methods
    void publishVelocityCommand(double linear_x, double linear_y, double angular_z);
    void publishRobotCommand(const QString& command);

signals:
    // Connection status
    void connected();
    void disconnected();
    void error(const QString& message);

    // Data received signals
    void imageReceived(const QByteArray& imageData, int width, int height);
    void imuDataReceived(double ax, double ay, double az, 
                         double gx, double gy, double gz);
    void robotStatusReceived(const QString& status);
    void sensorDataReceived(const QString& sensorType, const QVariantMap& data);

public slots:
    void start();
    void stop();

private:
    void spinROS2();
    void setupPublishers();
    void setupSubscribers();

#ifdef USE_ROS2
    // Callback methods
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void statusCallback(const std_msgs::msg::String::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> m_node;
    std::unique_ptr<QThread> m_ros2Thread;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocityPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_commandPublisher;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_statusSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_chatterSubscriber;
#ifdef USE_ROS2
    void chatterCallback(const std_msgs::msg::String::SharedPtr msg);
#endif
#else
    std::unique_ptr<QThread> m_ros2Thread;
#endif

    bool m_initialized;
    bool m_running;
    int m_argc;
    char** m_argv;
};

#endif // ROS2INTERFACE_H
