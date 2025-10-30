#include "ROS2Interface.h"
#include "Logger.h"
#include <QTimer>

ROS2Interface::ROS2Interface(int argc, char** argv, QObject *parent)
    : QObject(parent)
    , m_initialized(false)
    , m_running(false)
    , m_argc(argc)
    , m_argv(argv)
{
}

ROS2Interface::~ROS2Interface()
{
    shutdown();
}

bool ROS2Interface::initialize()
{
#ifdef USE_ROS2
    try {
        // Initialize ROS2
        rclcpp::init(m_argc, m_argv);

        // Create node
        m_node = std::make_shared<rclcpp::Node>("precision_farming_desktop_client");

        setupPublishers();
        setupSubscribers();

        m_initialized = true;
        Logger::instance().info("ROS2 interface initialized successfully");

        return true;
    }
    catch (const std::exception& e) {
        Logger::instance().error(QString("ROS2 initialization failed: %1").arg(e.what()));
        emit error(QString("ROS2 initialization failed: %1").arg(e.what()));
        return false;
    }
#else
    Logger::instance().warning("ROS2 support not compiled. ROS2Interface is in stub mode.");
    m_initialized = true;
    return true;
#endif
}

void ROS2Interface::setupPublishers()
{
#ifdef USE_ROS2
    // Velocity command publisher
    m_velocityPublisher = m_node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    // Robot command publisher
    m_commandPublisher = m_node->create_publisher<std_msgs::msg::String>(
        "/robot_command", 10);

    Logger::instance().debug("ROS2 publishers created");
#endif
}

void ROS2Interface::setupSubscribers()
{
#ifdef USE_ROS2
    // Image subscriber
    m_imageSubscriber = m_node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ROS2Interface::imageCallback, this, std::placeholders::_1));

    // IMU subscriber
    m_imuSubscriber = m_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&ROS2Interface::imuCallback, this, std::placeholders::_1));

    // Status subscriber
    m_statusSubscriber = m_node->create_subscription<std_msgs::msg::String>(
        "/robot_status", 10,
        std::bind(&ROS2Interface::statusCallback, this, std::placeholders::_1));

        // Chatter subscriber
    m_chatterSubscriber = m_node->create_subscription<std_msgs::msg::String>(
        "/chatter", 10,
        std::bind(&ROS2Interface::chatterCallback, this, std::placeholders::_1));

    Logger::instance().debug("ROS2 subscribers created");
#endif
}

void ROS2Interface::start()
{
    if (!m_initialized) {
        Logger::instance().warning("Cannot start ROS2 - not initialized");
        return;
    }

    m_running = true;
    
    // Create and start ROS2 spin thread
    m_ros2Thread = std::make_unique<QThread>();
    
    QTimer* spinTimer = new QTimer();
    spinTimer->setInterval(10); // 100Hz
    spinTimer->moveToThread(m_ros2Thread.get());
    
    connect(m_ros2Thread.get(), &QThread::started, spinTimer, 
            static_cast<void (QTimer::*)()>(&QTimer::start));
    connect(spinTimer, &QTimer::timeout, this, &ROS2Interface::spinROS2);
    connect(m_ros2Thread.get(), &QThread::finished, spinTimer, &QTimer::deleteLater);
    
    m_ros2Thread->start();
    
    emit connected();
    Logger::instance().info("ROS2 interface started");
}

void ROS2Interface::stop()
{
    m_running = false;
    
    if (m_ros2Thread && m_ros2Thread->isRunning()) {
        m_ros2Thread->quit();
        m_ros2Thread->wait();
    }
    
    emit disconnected();
    Logger::instance().info("ROS2 interface stopped");
}

void ROS2Interface::shutdown()
{
    stop();
    
#ifdef USE_ROS2
    if (m_initialized) {
        rclcpp::shutdown();
        m_initialized = false;
        Logger::instance().info("ROS2 shutdown complete");
    }
#else
    if (m_initialized) {
        m_initialized = false;
        Logger::instance().info("ROS2 stub shutdown complete");
    }
#endif
}

void ROS2Interface::spinROS2()
{
#ifdef USE_ROS2
    if (m_node && m_running) {
        rclcpp::spin_some(m_node);
    }
#endif
}

void ROS2Interface::publishVelocityCommand(double linear_x, double linear_y, double angular_z)
{
#ifdef USE_ROS2
    if (!m_velocityPublisher) return;

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z;

    m_velocityPublisher->publish(msg);
#else
    Logger::instance().debug(QString("Velocity command (stub): linear_x=%1, linear_y=%2, angular_z=%3")
                            .arg(linear_x).arg(linear_y).arg(angular_z));
#endif
}

void ROS2Interface::publishRobotCommand(const QString& command)
{
#ifdef USE_ROS2
    if (!m_commandPublisher) return;

    auto msg = std_msgs::msg::String();
    msg.data = command.toStdString();

    m_commandPublisher->publish(msg);
#else
    Logger::instance().debug(QString("Robot command (stub): %1").arg(command));
#endif
}

#ifdef USE_ROS2
void ROS2Interface::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS image to QByteArray
    QByteArray imageData(reinterpret_cast<const char*>(msg->data.data()), 
                         msg->data.size());
    
    emit imageReceived(imageData, msg->width, msg->height);
}

void ROS2Interface::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    emit imuDataReceived(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z
    );
}

void ROS2Interface::statusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    emit robotStatusReceived(QString::fromStdString(msg->data));
}

void ROS2Interface::chatterCallback(const std_msgs::msg::String::SharedPtr msg)
{
    emit robotStatusReceived(QString::fromStdString(msg->data));
}
#endif
