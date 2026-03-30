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

        // testing hook: emit a synthetic BGR8 frame to verify conversion
        if (qgetenv("IMAGE_PIPELINE_TEST") == "1") {
            sensor_msgs::msg::Image::SharedPtr fake = 
                std::make_shared<sensor_msgs::msg::Image>();
            fake->width = 2;
            fake->height = 2;
            fake->encoding = "bgr8";
            fake->step = fake->width * 3;
            // pixel order: B,G,R for each pixel
            fake->data = std::vector<uint8_t>{
                0,0,255,   // red pixel (BGR)
                0,255,0,   // green pixel
                255,0,0,   // blue pixel
                255,255,255 // white pixel
            };
            imageCallback(fake);
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
    // Image subscriber - subscribing to camera/raw topic by default
    m_imageSubscriber = m_node->create_subscription<sensor_msgs::msg::Image>(
        "camera/raw", 10,
        std::bind(&ROS2Interface::imageCallback, this, std::placeholders::_1));

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
#ifdef USE_ROS2
    if (!m_node) {
        Logger::instance().warning("Cannot switch camera topic - node not initialized");
        return;
    }

    // Unsubscribe from current topic
    if (m_imageSubscriber) {
        m_imageSubscriber.reset();
        Logger::instance().debug("Unsubscribed from previous camera topic");
    }

    // Subscribe to new topic
    m_imageSubscriber = m_node->create_subscription<sensor_msgs::msg::Image>(
        topic.toStdString(), 10,
        std::bind(&ROS2Interface::imageCallback, this, std::placeholders::_1));

    Logger::instance().info(QString("Switched camera subscription to topic: %1").arg(topic));
#else
    Logger::instance().debug(QString("Camera topic switch (stub): %1").arg(topic));
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
    // Convert ROS image to QByteArray. We must be explicit about BGR→RGB
    // because OpenCV publishers use BGR8, whereas QImage expects RGB888.
    QByteArray imageData;
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);

    auto yuvToRgb = [](int y, int u, int v, uchar* dst) {
        // Full-range BT.601 conversion (common for camera YUV frames)
        const int c = y;
        const int d = u - 128;
        const int e = v - 128;

        const int r = c + ((359 * e) >> 8);
        const int g = c - ((88 * d + 183 * e) >> 8);
        const int b = c + ((454 * d) >> 8);

        dst[0] = static_cast<uchar>(qBound(0, r, 255));
        dst[1] = static_cast<uchar>(qBound(0, g, 255));
        dst[2] = static_cast<uchar>(qBound(0, b, 255));
    };

    if (msg->encoding == "bgr8") {
        // Perform explicit BGR→RGB conversion, honoring ROS row stride (step)
        int srcStep = static_cast<int>(msg->step);
        const size_t payloadSize = msg->data.size();
        const int derivedStep = (height > 0 && (payloadSize % static_cast<size_t>(height) == 0))
                                ? static_cast<int>(payloadSize / static_cast<size_t>(height))
                                : 0;
        const int minStep = width * 3;
        if (srcStep < minStep) {
            if (derivedStep >= minStep) {
                Logger::instance().warning(QString("Invalid BGR8 step %1, using derived step %2 from payload")
                                           .arg(srcStep).arg(derivedStep));
                srcStep = derivedStep;
            } else {
                Logger::instance().warning(QString("Invalid BGR8 step: expected >= %1, got %2")
                                           .arg(minStep).arg(srcStep));
                return;
            }
        }
        if (static_cast<size_t>(srcStep) * static_cast<size_t>(height) > payloadSize) {
            Logger::instance().warning(QString("BGR8 payload too small for step=%1 height=%2 (payload=%3)")
                                       .arg(srcStep).arg(height).arg(payloadSize));
            return;
        }

        imageData.resize(width * height * 3);
        const uint8_t* src = msg->data.data();
        uchar* dst = reinterpret_cast<uchar*>(imageData.data());
        for (int y = 0; y < height; ++y) {
            const uint8_t* row = src + (y * srcStep);
            for (int x = 0; x < width; ++x) {
                const int si = x * 3;
                const int di = (y * width + x) * 3;
                dst[di]     = row[si + 2]; // R
                dst[di + 1] = row[si + 1]; // G
                dst[di + 2] = row[si];     // B
            }
        }
        Logger::instance().debug("Converted BGR8 frame to RGB8 (step-aware)");
    } else if (msg->encoding == "rgb8") {
        // Copy RGB8 data while honoring row stride; optionally swap if source is mislabeled.
        int srcStep = static_cast<int>(msg->step);
        const size_t payloadSize = msg->data.size();
        const int derivedStep = (height > 0 && (payloadSize % static_cast<size_t>(height) == 0))
                                ? static_cast<int>(payloadSize / static_cast<size_t>(height))
                                : 0;
        const int minStep = width * 3;
        if (srcStep < minStep) {
            if (derivedStep >= minStep) {
                Logger::instance().warning(QString("Invalid RGB8 step %1, using derived step %2 from payload")
                                           .arg(srcStep).arg(derivedStep));
                srcStep = derivedStep;
            } else {
                Logger::instance().warning(QString("Invalid RGB8 step: expected >= %1, got %2")
                                           .arg(minStep).arg(srcStep));
                return;
            }
        }
        if (static_cast<size_t>(srcStep) * static_cast<size_t>(height) > payloadSize) {
            Logger::instance().warning(QString("RGB8 payload too small for step=%1 height=%2 (payload=%3)")
                                       .arg(srcStep).arg(height).arg(payloadSize));
            return;
        }

        const bool forceBgrFromRgb = (qgetenv("FORCE_BGR_FROM_RGB8") == "1");
        imageData.resize(width * height * 3);
        const uint8_t* src = msg->data.data();
        uchar* dst = reinterpret_cast<uchar*>(imageData.data());
        const bool has4BytePixels = (srcStep >= width * 4) && (derivedStep >= width * 4);
        const QString rgbaOrder = QString::fromLocal8Bit(qgetenv("RGB8_4BYTE_ORDER")).trimmed().toUpper();

        if (has4BytePixels) {
            // Some pipelines publish 4-byte RGBX/BGRX frames but label as rgb8.
            // Interpret according to RGB8_4BYTE_ORDER (BGRX default).
            const QString order = rgbaOrder.isEmpty() ? "BGRX" : rgbaOrder;
            for (int y = 0; y < height; ++y) {
                const uint8_t* row = src + (y * srcStep);
                for (int x = 0; x < width; ++x) {
                    const int si = x * 4;
                    const int di = (y * width + x) * 3;
                    const uint8_t p0 = row[si];
                    const uint8_t p1 = row[si + 1];
                    const uint8_t p2 = row[si + 2];
                    const uint8_t p3 = row[si + 3];

                    if (order == "RGBX" || order == "RGBA") {
                        dst[di] = p0; dst[di + 1] = p1; dst[di + 2] = p2;
                    } else if (order == "XRGB" || order == "ARGB") {
                        dst[di] = p1; dst[di + 1] = p2; dst[di + 2] = p3;
                    } else if (order == "XBGR" || order == "ABGR") {
                        dst[di] = p3; dst[di + 1] = p2; dst[di + 2] = p1;
                    } else {
                        // Default BGRX/BGRA
                        dst[di] = p2; dst[di + 1] = p1; dst[di + 2] = p0;
                    }
                }
            }
            Logger::instance().warning(QString("RGB8 stream interpreted as 4-byte pixels (order=%1, step=%2)")
                                       .arg(order).arg(srcStep));
        } else {
            for (int y = 0; y < height; ++y) {
                const uint8_t* row = src + (y * srcStep);
                for (int x = 0; x < width; ++x) {
                    const int si = x * 3;
                    const int di = (y * width + x) * 3;
                    if (forceBgrFromRgb) {
                        // Some camera pipelines incorrectly tag BGR as RGB8.
                        dst[di]     = row[si + 2];
                        dst[di + 1] = row[si + 1];
                        dst[di + 2] = row[si];
                    } else {
                        dst[di]     = row[si];
                        dst[di + 1] = row[si + 1];
                        dst[di + 2] = row[si + 2];
                    }
                }
            }
        }

        if (forceBgrFromRgb) {
            Logger::instance().warning("RGB8 stream forced through BGR->RGB swap (FORCE_BGR_FROM_RGB8=1)");
        }
    } else if (msg->encoding == "i420" || msg->encoding == "yv12" ||
               msg->encoding == "nv12" || msg->encoding == "nv21" ||
               msg->encoding == "yuv420") {
        const size_t ySize = static_cast<size_t>(width) * static_cast<size_t>(height);
        const size_t expectedYuv420Size = ySize + (ySize / 2);

        if (msg->data.size() < expectedYuv420Size) {
            Logger::instance().warning(QString("YUV420 payload too small: expected at least %1, got %2")
                                       .arg(expectedYuv420Size)
                                       .arg(msg->data.size()));
            return;
        }

        QString layout = QString::fromStdString(msg->encoding).toLower();
        if (layout == "yuv420") {
            // 'yuv420' is ambiguous in the wild. Default to NV12 for Pi camera pipelines,
            // and allow operator override via YUV420_LAYOUT=nv12|nv21|i420|yv12.
            const QString override = QString::fromLocal8Bit(qgetenv("YUV420_LAYOUT")).trimmed().toLower();
            if (override == "nv12" || override == "nv21" || override == "i420" || override == "yv12") {
                layout = override;
            } else {
                layout = "nv12";
            }
        }

        imageData.resize(width * height * 3);
        const uint8_t* base = msg->data.data();
        const uint8_t* yPlane = base;
        uchar* rgbPtr = reinterpret_cast<uchar*>(imageData.data());

        if (layout == "i420" || layout == "yv12") {
            const uint8_t* planeA = yPlane + ySize;
            const uint8_t* planeB = planeA + (ySize / 4);
            const uint8_t* uPlane = (layout == "i420") ? planeA : planeB;
            const uint8_t* vPlane = (layout == "i420") ? planeB : planeA;

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    const int yIdx = y * width + x;
                    const int uvIdx = (y / 2) * (width / 2) + (x / 2);
                    yuvToRgb(yPlane[yIdx], uPlane[uvIdx], vPlane[uvIdx], &rgbPtr[yIdx * 3]);
                }
            }
        } else {
            // NV12/NV21: Y plane + interleaved chroma plane at half resolution
            const uint8_t* uvPlane = yPlane + ySize;
            const bool isNv21 = (layout == "nv21");

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    const int yIdx = y * width + x;
                    const int uvRow = y / 2;
                    const int uvCol = (x / 2) * 2;
                    const int uvIdx = uvRow * width + uvCol;

                    const uint8_t u = isNv21 ? uvPlane[uvIdx + 1] : uvPlane[uvIdx];
                    const uint8_t v = isNv21 ? uvPlane[uvIdx] : uvPlane[uvIdx + 1];
                    yuvToRgb(yPlane[yIdx], u, v, &rgbPtr[yIdx * 3]);
                }
            }
        }

        Logger::instance().debug(QString("Converted %1 frame to RGB8")
                                 .arg(layout.toUpper()));
    } else {
        // Unknown/unsupported encoding: try to treat as RGB and warn operator
        Logger::instance().warning(QString("Unsupported image encoding '%1'; "
                                        "treating as RGB8").
                                    arg(QString::fromStdString(msg->encoding)));
        imageData = QByteArray(reinterpret_cast<const char*>(msg->data.data()), 
                               msg->data.size());
    }

    emit imageReceived(imageData, msg->width, msg->height);

    Logger::instance().debug(QString("Image received: %1x%2, encoding: %3")
                             .arg(msg->width).arg(msg->height)
                             .arg(QString::fromStdString(msg->encoding)));
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
                            .arg(jsonData.left(100))); // Log first 100 chars
}
#endif
