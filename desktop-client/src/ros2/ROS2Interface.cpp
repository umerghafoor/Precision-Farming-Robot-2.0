#include "ROS2Interface.h"
#include "Logger.h"
#include <QImage>
#include <QTimer>
#include <QMutexLocker>

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

QString ROS2Interface::normalizeTopicName(const QString& topic) const
{
    if (topic.isEmpty()) {
        return QString();
    }

    return topic.startsWith('/') ? topic : QString("/") + topic;
}

#ifdef USE_ROS2
rclcpp::QoS ROS2Interface::createImageSubscriptionQos() const
{
    bool ok = false;
    int depth = QString::fromLocal8Bit(qgetenv("IMAGE_QOS_DEPTH")).toInt(&ok);
    if (!ok || depth < 1) {
        depth = 1;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth)));
    if (qgetenv("IMAGE_QOS_RELIABLE") == "1") {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    qos.durability_volatile();
    return qos;
}

bool ROS2Interface::createImageSubscription(const QString& topic)
{
    if (!m_node) {
        Logger::instance().warning("Cannot create camera subscription - node not initialized");
        return false;
    }

    const QString normalizedTopic = normalizeTopicName(topic);
    if (normalizedTopic.isEmpty()) {
        Logger::instance().warning("Cannot create camera subscription - empty topic name");
        return false;
    }

    if (m_imageSubscribers.contains(normalizedTopic)) {
        return true;
    }

    m_imageSubscribers[normalizedTopic] = m_node->create_subscription<RawImageMsg>(
        normalizedTopic.toStdString(),
        createImageSubscriptionQos(),
        [this, normalizedTopic](const RawImageMsg::SharedPtr msg) {
            imageCallback(msg, normalizedTopic);
        });

    Logger::instance().info(QString("Camera subscription active on %1 (reliable=%2, depth=%3)")
                            .arg(normalizedTopic)
                            .arg(qgetenv("IMAGE_QOS_RELIABLE") == "1" ? "true" : "false")
                            .arg(QString::fromLocal8Bit(qgetenv("IMAGE_QOS_DEPTH")).isEmpty()
                                 ? "1"
                                 : QString::fromLocal8Bit(qgetenv("IMAGE_QOS_DEPTH"))));
    return true;
}
#endif

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

        // testing hook: emit a synthetic gray8 frame to verify conversion
        if (qgetenv("IMAGE_PIPELINE_TEST") == "1") {
            RawImageMsg::SharedPtr fake = 
                std::make_shared<RawImageMsg>();
            fake->format = "gray8;2x2";
            // grayscale pixels in row-major order
            fake->data = std::vector<uint8_t>{
                0,
                85,
                170,
                255
            };
            imageCallback(fake, "/camera/color_jpeg");
            Logger::instance().info("IMAGE_PIPELINE_TEST: synthetic frame injected");
        }

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
    // Camera topic is intentionally on-demand and subscribed by UI pages only.

    // IMU subscriber
    m_imuSubscriber = m_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&ROS2Interface::imuCallback, this, std::placeholders::_1));

    // Status subscriber
    m_statusSubscriber = m_node->create_subscription<std_msgs::msg::String>(
        "/robot_status", 10,
        std::bind(&ROS2Interface::statusCallback, this, std::placeholders::_1));

    // Coordinates subscriber
    m_coordinatesSubscriber = m_node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/coordinates", 10,
        std::bind(&ROS2Interface::coordinatesCallback, this, std::placeholders::_1));

    // Coordinates JSON subscriber (for image/coordinates topic)
    m_coordinatesJsonSubscriber = m_node->create_subscription<std_msgs::msg::String>(
        "image/coordinates", 10,
        std::bind(&ROS2Interface::coordinatesJsonCallback, this, std::placeholders::_1));

    Logger::instance().debug("ROS2 subscribers created");
#endif
}

void ROS2Interface::switchCameraTopic(const QString& topic)
{
    subscribeCameraTopic(topic);
}

void ROS2Interface::subscribeCameraTopic(const QString& topic)
{
#ifdef USE_ROS2
    if (!m_node) {
        Logger::instance().warning("Cannot subscribe camera topic - node not initialized");
        return;
    }

    const QString normalizedTopic = normalizeTopicName(topic);
    if (normalizedTopic.isEmpty()) {
        Logger::instance().warning("Cannot subscribe camera topic - empty topic name");
        return;
    }

    QMutexLocker locker(&m_imageSubscriptionMutex);

    int& refCount = m_imageTopicRefCounts[normalizedTopic];
    refCount += 1;
    if (refCount > 1) {
        Logger::instance().debug(QString("Camera topic %1 already active (refCount=%2)")
                                 .arg(normalizedTopic)
                                 .arg(refCount));
        return;
    }

    if (!createImageSubscription(normalizedTopic)) {
        m_imageTopicRefCounts.remove(normalizedTopic);
        Logger::instance().warning(QString("Failed to subscribe camera topic: %1").arg(topic));
    }
#else
    Logger::instance().debug(QString("Camera topic subscribe (stub): %1").arg(topic));
#endif
}

void ROS2Interface::unsubscribeCameraTopic()
{
#ifdef USE_ROS2
    QMutexLocker locker(&m_imageSubscriptionMutex);
    m_imageSubscribers.clear();
    m_imageTopicRefCounts.clear();
    Logger::instance().info("All camera subscriptions disabled (on-demand mode)");
#else
    Logger::instance().debug("Camera topic unsubscribe (stub)");
#endif
}

void ROS2Interface::unsubscribeCameraTopic(const QString& topic)
{
#ifdef USE_ROS2
    const QString normalizedTopic = normalizeTopicName(topic);
    if (normalizedTopic.isEmpty()) {
        unsubscribeCameraTopic();
        return;
    }

    QMutexLocker locker(&m_imageSubscriptionMutex);

    auto it = m_imageTopicRefCounts.find(normalizedTopic);
    if (it == m_imageTopicRefCounts.end()) {
        Logger::instance().debug(QString("Camera topic %1 was not subscribed").arg(normalizedTopic));
        return;
    }

    it.value() -= 1;
    if (it.value() > 0) {
        Logger::instance().debug(QString("Camera topic %1 kept active (refCount=%2)")
                                 .arg(normalizedTopic)
                                 .arg(it.value()));
        return;
    }

    m_imageTopicRefCounts.erase(it);
    m_imageSubscribers.remove(normalizedTopic);
    Logger::instance().info(QString("Camera subscription disabled for %1").arg(normalizedTopic));
#else
    Logger::instance().debug(QString("Camera topic unsubscribe (stub): %1").arg(topic));
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
    connect(spinTimer, &QTimer::timeout, spinTimer, [this]() {
        spinROS2();
    });
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
    emit velocityCommandIssued(linear_x, angular_z);
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
void ROS2Interface::imageCallback(const RawImageMsg::SharedPtr msg, const QString& sourceTopic)
{
    // Supported compressed formats:
    //  1) Standard CompressedImage (jpeg/jpg/png) -> decoded via Qt
    //  2) Custom packed grayscale "gray4;WxH" / "gray8;WxH"

    const QString format = QString::fromStdString(msg->format).trimmed();
    const QString formatLower = format.toLower();
    const QByteArray raw(reinterpret_cast<const char*>(msg->data.data()),
                         static_cast<int>(msg->data.size()));

    if (formatLower == "jpeg" || formatLower == "jpg" || formatLower == "png") {
        QImage decoded;
        if (!decoded.loadFromData(raw)) {
            Logger::instance().warning(
                QString("Failed to decode CompressedImage payload as %1").arg(format));
            return;
        }

        QImage rgb = decoded.convertToFormat(QImage::Format_RGB888);
        if (rgb.isNull()) {
            Logger::instance().warning("Failed to convert decoded compressed image to RGB888");
            return;
        }

        QByteArray imageData(reinterpret_cast<const char*>(rgb.constBits()),
                             rgb.width() * rgb.height() * 3);
        emit imageReceived(imageData, rgb.width(), rgb.height());
        emit imageReceivedForTopic(sourceTopic, imageData, rgb.width(), rgb.height());

        Logger::instance().debug(
            QString("Decoded %1 frame %2x%3")
                .arg(formatLower)
                .arg(rgb.width())
                .arg(rgb.height()));
        return;
    }

    // ── Parse "gray4;WxH" or "gray8;WxH" ─────────────────────────────
    const QStringList parts = format.split(';');
    if (parts.size() != 2) {
        Logger::instance().warning(
            QString("Unsupported CompressedImage format: '%1'").arg(format));
        return;
    }

    const QString encoding = parts[0].trimmed().toLower(); // "gray4" or "gray8"
    const QStringList dims = parts[1].trimmed().split('x');
    if (dims.size() != 2) {
        Logger::instance().warning(
            QString("Cannot parse dimensions from format: '%1'").arg(format));
        return;
    }

    bool okW = false, okH = false;
    const int width  = dims[0].toInt(&okW);
    const int height = dims[1].toInt(&okH);
    if (!okW || !okH || width <= 0 || height <= 0) {
        Logger::instance().warning(
            QString("Invalid dimensions in format: '%1'").arg(format));
        return;
    }

    const int totalPixels = width * height;

    QByteArray imageData; // will be filled as packed RGB888

    // ── gray4: two pixels per byte (high nibble = pixel[2i], low = pixel[2i+1]) ──
    if (encoding == "gray4") {
        const int expectedBytes = (totalPixels + 1) / 2; // ceil(W*H/2)
        if (raw.size() < expectedBytes) {
            Logger::instance().warning(
                QString("gray4 payload too small: expected %1, got %2")
                    .arg(expectedBytes).arg(raw.size()));
            return;
        }

        // Unpack nibbles → grey8 → RGB888 (R=G=B)
        imageData.resize(totalPixels * 3);
        uchar* dst = reinterpret_cast<uchar*>(imageData.data());
        const uchar* src = reinterpret_cast<const uchar*>(raw.constData());

        for (int i = 0; i < totalPixels; ++i) {
            const uchar byte  = src[i / 2];
            // high nibble for even pixels, low nibble for odd
            const uchar nibble = (i % 2 == 0) ? (byte >> 4) : (byte & 0x0F);
            // scale 0-15 → 0-255
            const uchar grey  = static_cast<uchar>(nibble * 17);
            const int   di    = i * 3;
            dst[di]     = grey; // R
            dst[di + 1] = grey; // G
            dst[di + 2] = grey; // B
        }

        Logger::instance().debug(
            QString("Decoded gray4 frame %1x%2 (%3 bytes → %4 bytes RGB)")
                .arg(width).arg(height).arg(raw.size()).arg(imageData.size()));
    }
    // ── gray8: one byte per pixel ────────────────────────────────────
    else if (encoding == "gray8") {
        if (raw.size() < totalPixels) {
            Logger::instance().warning(
                QString("gray8 payload too small: expected %1, got %2")
                    .arg(totalPixels).arg(raw.size()));
            return;
        }

        // Expand grey8 → RGB888 (R=G=B)
        imageData.resize(totalPixels * 3);
        uchar* dst = reinterpret_cast<uchar*>(imageData.data());
        const uchar* src = reinterpret_cast<const uchar*>(raw.constData());

        for (int i = 0; i < totalPixels; ++i) {
            const uchar grey = src[i];
            const int   di   = i * 3;
            dst[di]     = grey;
            dst[di + 1] = grey;
            dst[di + 2] = grey;
        }

        Logger::instance().debug(
            QString("Decoded gray8 frame %1x%2").arg(width).arg(height));
    }
    else {
        Logger::instance().warning(
            QString("Unknown encoding in CompressedImage format: '%1'").arg(encoding));
        return;
    }

    // ── emit — same signal signature as before ────────────────────────
    emit imageReceived(imageData,
                       static_cast<uint32_t>(width),
                       static_cast<uint32_t>(height));
    emit imageReceivedForTopic(sourceTopic,
                               imageData,
                               static_cast<uint32_t>(width),
                               static_cast<uint32_t>(height));

    Logger::instance().debug(
        QString("Image received: %1x%2, encoding: %3")
            .arg(width).arg(height).arg(encoding));
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

void ROS2Interface::coordinatesCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    emit coordinatesReceived(msg->point.x, msg->point.y);

    Logger::instance().debug(QString("Coordinates received: X=%1, Y=%2")
                            .arg(msg->point.x).arg(msg->point.y));
}

void ROS2Interface::coordinatesJsonCallback(const std_msgs::msg::String::SharedPtr msg)
{
    QString jsonData = QString::fromStdString(msg->data);
    emit coordinatesJsonReceived(jsonData);

    Logger::instance().debug(QString("Coordinates JSON received: %1")
                            .arg(jsonData.left(100)));
}
#endif
