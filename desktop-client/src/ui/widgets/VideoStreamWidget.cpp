#include "VideoStreamWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>
#include <QImage>
#include <QTabBar>
#include <QFrame>
#include <QPainter>
#include <QStyleOption>

VideoStreamWidget::VideoStreamWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_recording(false)
    , m_lastFrameTime(0)
{
    setupUI();
}

VideoStreamWidget::~VideoStreamWidget()
{
}

void VideoStreamWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Tab bar for stream sources
    m_tabBar = new QTabBar();
    // start timer used for fps calculations
    m_frameTimer.start();
    m_tabBar->addTab("Raw");
    m_tabBar->addTab("Detection");
    m_tabBar->addTab("Depth");
    m_currentTopic = "camera/raw";
    connect(m_tabBar, &QTabBar::currentChanged,
            this, &VideoStreamWidget::onStreamTabChanged);

    // make a container that will hold video and overlays
    m_videoContainer = new QFrame();
    m_videoContainer->setStyleSheet("QFrame { background-color: black; }");
    m_videoContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_videoContainer->setMinimumSize(200, 150);

    QVBoxLayout* containerLayout = new QVBoxLayout(m_videoContainer);
    containerLayout->setContentsMargins(0,0,0,0);

    m_videoLabel = new QLabel();
    m_videoLabel->setAlignment(Qt::AlignCenter);
    m_videoLabel->setStyleSheet("QLabel { background-color: black; }");
    m_videoLabel->setScaledContents(true);
    m_videoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    containerLayout->addWidget(m_videoLabel);

    // overlays
    m_topicOverlay = new QLabel(m_videoContainer);
    m_topicOverlay->setStyleSheet("QLabel { background-color: rgba(0,0,0,128); color: white; padding: 2px 4px; border-radius: 4px; font-size:10px; }");
    m_topicOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);

    m_resolutionOverlay = new QLabel(m_videoContainer);
    m_resolutionOverlay->setStyleSheet("QLabel { background-color: rgba(0,0,0,128); color: white; padding: 2px 4px; border-radius: 4px; font-size:10px; }");
    m_resolutionOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);

    m_statusOverlay = new QLabel(m_videoContainer);
    m_statusOverlay->setStyleSheet("QLabel { background-color: rgba(0,0,0,128); color: white; padding: 2px 4px; border-radius: 4px; font-size:10px; }");
    m_statusOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);

    // live/offline indicator dot
    m_liveDot = new QLabel(m_videoContainer);
    m_liveDot->setFixedSize(8, 8);
    m_liveDot->setStyleSheet("QLabel { background-color: #52C44A; border-radius: 4px; }");
    m_liveDot->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_liveDot->hide();

    m_liveExpiryTimer = new QTimer(this);
    m_liveExpiryTimer->setSingleShot(true);
    connect(m_liveExpiryTimer, &QTimer::timeout, this, [this]() {
        m_liveDot->setStyleSheet("QLabel { background-color: grey; border-radius: 4px; }");
    });

    m_pulseAnimation = new QPropertyAnimation(m_liveDot, "windowOpacity", this);
    m_pulseAnimation->setDuration(1000);
    m_pulseAnimation->setStartValue(0.3);
    m_pulseAnimation->setEndValue(1.0);
    m_pulseAnimation->setLoopCount(-1);
    m_pulseAnimation->start();

    // initial overlay texts
    m_topicOverlay->setText(QString("Topic: %1").arg(m_currentTopic));
    m_resolutionOverlay->setText("");
    m_statusOverlay->setText("");

    // placeholder initial state
    showPlaceholder();

    mainLayout->addWidget(m_tabBar);
    mainLayout->addWidget(m_videoContainer, 1);

    // ensure initial topic state is set
    onStreamTabChanged(m_tabBar->currentIndex());

    // automated exercise for test mode
    if (qgetenv("VIDEO_WIDGET_TEST") == "1") {
        QTimer::singleShot(1000, this, [this]() { m_tabBar->setCurrentIndex(1); });
        QTimer::singleShot(2000, this, [this]() { m_tabBar->setCurrentIndex(2); });
    }
}

bool VideoStreamWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::imageReceived,
                this, &VideoStreamWidget::onImageReceived);
        connect(m_ros2Interface, &ROS2Interface::robotStatusReceived,
                this, &VideoStreamWidget::onRobotStatusUpdated);
        Logger::instance().info("Video stream widget initialized");
        return true;
    }
    
    Logger::instance().warning("Video stream widget initialized without ROS2");
    return false;
}

void VideoStreamWidget::onImageReceived(const QByteArray& imageData, int width, int height)
{
    // The ROS2Interface ensures any BGR8 frames are converted to RGB8 before
    // emitting imageReceived. We therefore treat the incoming QByteArray as
    // RGB data; any unsupported encodings are logged by the interface.
    // Size mismatches are caught below and reported as warnings.
    int expectedSize = width * height * 3;
    if (imageData.size() != expectedSize) {
        Logger::instance().warning(QString("Image size mismatch: expected %1, got %2")
                                  .arg(expectedSize).arg(imageData.size()));
        return;
    }

    // ROS2Interface is responsible for converting any BGR8 frames to RGB8
    // before emitting imageReceived. Here we simply treat the payload as RGB.
    QImage image(reinterpret_cast<const uchar*>(imageData.data()),
                 width, height, width * 3, QImage::Format_RGB888);
    if (image.isNull()) {
        Logger::instance().error("Failed to create QImage from received data");
        return;
    }

    // update fps/resolution overlays
    updateOverlayInfo(width, height);

    m_videoLabel->setPixmap(QPixmap::fromImage(image));

    m_lastFrameTime = m_frameTimer.elapsed();

    // live indicator: show green dot and restart expiry timer
    m_liveDot->setStyleSheet("QLabel { background-color: #52C44A; border-radius: 4px; }");
    m_liveDot->show();
    m_liveExpiryTimer->start(1000);
}

void VideoStreamWidget::onStreamTabChanged(int index)
{
    QString selectedTopic;
    switch (index) {
        case 0: selectedTopic = "camera/raw"; break;
        case 1: selectedTopic = "camera/detection"; break;
        case 2: selectedTopic = "camera/depth"; break;
        default:
            Logger::instance().warning(QString("Unknown stream tab index: %1").arg(index));
            return;
    }

    m_currentTopic = selectedTopic;
    m_topicOverlay->setText(QString("Topic: %1").arg(selectedTopic));
    m_resolutionOverlay->setText("");

    Logger::instance().info(QString("Stream source changed to tab %1 (topic: %2)")
                            .arg(index).arg(selectedTopic));

    // show placeholder until new frames arrive
    showPlaceholder();

    if (m_ros2Interface) {
        m_ros2Interface->switchCameraTopic(selectedTopic);
    } else {
        Logger::instance().warning("Cannot switch camera topic - ROS2 interface not available");
    }
}

void VideoStreamWidget::onToggleRecording()
{
    // kept for backward compatibility but not used in P6
    m_recording = m_recordButton->isChecked();
    if (m_recording) {
        m_recordButton->setText("Stop Recording");
        Logger::instance().info("Video recording started");
    } else {
        m_recordButton->setText("Record");
        Logger::instance().info("Video recording stopped");
    }
}

void VideoStreamWidget::onRobotStatusUpdated(const QString& status)
{
    m_statusOverlay->setText(status);
}

void VideoStreamWidget::showPlaceholder()
{
    // create a simple grey pixmap with text
    QSize sz = m_videoLabel->size().boundedTo(QSize(640,480));
    if (sz.isEmpty()) sz = QSize(640,480);
    QPixmap placeholder(sz);
    placeholder.fill(Qt::darkGray);
    QPainter p(&placeholder);
    p.setPen(Qt::white);
    p.setFont(QFont("Arial", 20, QFont::Bold));
    p.drawText(placeholder.rect(), Qt::AlignCenter, "No Stream");
    p.end();

    m_videoLabel->setPixmap(placeholder);
}

void VideoStreamWidget::updateOverlayInfo(int width, int height)
{
    // update resolution/fps chip
    QString res = QString("%1×%2").arg(width).arg(height);
    QString fps;
    if (m_lastFrameTime > 0) {
        double dt = (m_frameTimer.elapsed() - m_lastFrameTime) / 1000.0;
        if (dt > 0) fps = QString("%1 fps").arg(int(1.0 / dt));
    }
    m_resolutionOverlay->setText(fps.isEmpty() ? res : res + " " + fps);
}

void VideoStreamWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
    if (m_videoContainer) {
        QSize sz = m_videoContainer->size();
        int margin = 8;
        m_topicOverlay->move(margin + 12, margin); // leave space for dot
        m_liveDot->move(margin, margin + 2);

        int w = m_resolutionOverlay->sizeHint().width();
        m_resolutionOverlay->move(sz.width() - w - margin, margin);
        int h = m_statusOverlay->sizeHint().height();
        m_statusOverlay->move(margin, sz.height() - h - margin);
    }
}
