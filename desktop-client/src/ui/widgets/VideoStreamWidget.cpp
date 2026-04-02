#include "VideoStreamWidget.h"
#include "ZoomableImageView.h"
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
#include <QLabel>

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
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(6);

    QWidget* headerRow = new QWidget(this);
    QHBoxLayout* headerLayout = new QHBoxLayout(headerRow);
    headerLayout->setContentsMargins(0, 0, 0, 0);
    headerLayout->setSpacing(6);

    // Tab bar for stream sources
    m_tabBar = new QTabBar(headerRow);
    // start timer used for fps calculations
    m_frameTimer.start();
    m_tabBar->addTab("Raw");
    m_tabBar->addTab("Detection");
    m_tabBar->addTab("Depth");
    m_currentTopic = "camera/raw";
    connect(m_tabBar, &QTabBar::currentChanged,
            this, &VideoStreamWidget::onStreamTabChanged);

    QWidget* zoomControls = new QWidget(headerRow);
    QHBoxLayout* zoomLayout = new QHBoxLayout(zoomControls);
    zoomLayout->setContentsMargins(0, 0, 0, 0);
    zoomLayout->setSpacing(4);

    m_zoomOutButton = new QPushButton("-");
    m_zoomOutButton->setFixedWidth(32);
    m_zoomOutButton->setToolTip("Zoom out");
    connect(m_zoomOutButton, &QPushButton::clicked,
        this, &VideoStreamWidget::onZoomOutClicked);

    m_zoomInButton = new QPushButton("+");
    m_zoomInButton->setFixedWidth(32);
    m_zoomInButton->setToolTip("Zoom in");
    connect(m_zoomInButton, &QPushButton::clicked,
        this, &VideoStreamWidget::onZoomInClicked);

    zoomLayout->addWidget(m_zoomOutButton);
    zoomLayout->addWidget(m_zoomInButton);
    zoomLayout->addStretch(1);

    headerLayout->addWidget(m_tabBar, 1);
    headerLayout->addWidget(zoomControls, 0, Qt::AlignRight);

    // make a container that will hold video and overlays
    m_videoContainer = new QFrame();
    m_videoContainer->setStyleSheet("QFrame { background-color: black; }");
    m_videoContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_videoContainer->setMinimumSize(200, 150);

    QVBoxLayout* containerLayout = new QVBoxLayout(m_videoContainer);
    containerLayout->setContentsMargins(0,0,0,0);

    m_videoLabel = new ZoomableImageView();
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

    mainLayout->addWidget(headerRow);
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

    // Build an owning copy of the RGB888 frame, then convert to RGB32 for
    // robust platform rendering through QPixmap/QLabel.
    QImage view(reinterpret_cast<const uchar*>(imageData.constData()),
                width, height, width * 3, QImage::Format_RGB888);
    if (view.isNull()) {
        Logger::instance().error("Failed to create QImage from received data");
        return;
    }
    QImage image = view.copy().convertToFormat(QImage::Format_RGB32);
    if (image.isNull()) {
        Logger::instance().error("Failed to convert RGB frame to display format");
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

void VideoStreamWidget::onZoomInClicked()
{
    if (m_videoLabel) {
        m_videoLabel->zoomIn();
    }
}

void VideoStreamWidget::onZoomOutClicked()
{
    if (m_videoLabel) {
        m_videoLabel->zoomOut();
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
    if (m_videoLabel) {
        m_videoLabel->resetView();
        m_videoLabel->setPixmap(QPixmap());
    }
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
