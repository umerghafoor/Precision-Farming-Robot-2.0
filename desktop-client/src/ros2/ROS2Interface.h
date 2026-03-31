#ifndef ROS2INTERFACE_H
#define ROS2INTERFACE_H

#include <QObject>
#include <QThread>
#include <QByteArray>
#include <QVariant>
#include <memory>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#endif

/**
 * @brief ROS2 Interface running in separate thread
 * 
 * This class handles all ROS2 communication isolated from Qt main thread
 * Uses signal/slot mechanism to communicate with Qt UI
 *
 * Image handling notes:
 *  - The imageCallback method consumes CompressedImage frames where format is
 *    expected as "gray4;WxH" or "gray8;WxH".
 *  - Payload is expanded into RGB888 before emitting imageReceived.
 *  - Unsupported/invalid formats are logged as warnings.
 *  - A test hook (IMAGE_PIPELINE_TEST env var) can inject a synthetic
 *    gray8 message during initialization to verify the conversion path.
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

    // Subscription management
    void switchCameraTopic(const QString& topic);
    void subscribeCameraTopic(const QString& topic);
    void unsubscribeCameraTopic();

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
    void coordinatesReceived(double x, double y);
    void coordinatesJsonReceived(const QString& data);

    // Emitted whenever a velocity command is sent (linear_x, angular_z)
    // Used by the digital twin to simulate motion without real robot feedback
    void velocityCommandIssued(double linear, double angular);

public slots:
    void start();
    void stop();

private:
    void spinROS2();
    void setupPublishers();
    void setupSubscribers();
    QString normalizeTopicName(const QString& topic) const;

#ifdef USE_ROS2
    
    using RawImageMsg = sensor_msgs::msg::CompressedImage;

    rclcpp::QoS createImageSubscriptionQos() const;
    bool createImageSubscription(const QString& topic);

    // Callback methods
    void imageCallback(const RawImageMsg::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void statusCallback(const std_msgs::msg::String::SharedPtr msg);
    void coordinatesCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void coordinatesJsonCallback(const std_msgs::msg::String::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> m_node;
    std::unique_ptr<QThread> m_ros2Thread;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_velocityPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_commandPublisher;

    // Subscribers
    rclcpp::Subscription<RawImageMsg>::SharedPtr m_imageSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_statusSubscriber;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_coordinatesSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_coordinatesJsonSubscriber;
#else
    std::unique_ptr<QThread> m_ros2Thread;
#endif

    bool m_initialized;
    bool m_running;
    int m_argc;
    char** m_argv;
};

#endif // ROS2INTERFACE_H
