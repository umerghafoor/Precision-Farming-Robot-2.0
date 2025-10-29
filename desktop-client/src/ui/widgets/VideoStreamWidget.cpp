#include "VideoStreamWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>
#include <QImage>
#include <QFileDialog>
#include <QDateTime>
#include <QTimer>

VideoStreamWidget::VideoStreamWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_recording(false)
{
    setupUI();
    
    // Setup FPS timer
    m_fpsTimer.start();
    m_fpsUpdateTimer = new QTimer(this);
    connect(m_fpsUpdateTimer, &QTimer::timeout, this, &VideoStreamWidget::updateFPS);
    m_fpsUpdateTimer->start(1000); // Update FPS every second
}

VideoStreamWidget::~VideoStreamWidget()
{
}

void VideoStreamWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Control bar
    QHBoxLayout* controlLayout = new QHBoxLayout();
    
    m_streamSelector = new QComboBox();
    m_streamSelector->addItem("Camera 1");
    m_streamSelector->addItem("Camera 2");
    m_streamSelector->addItem("Depth Camera");
    connect(m_streamSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VideoStreamWidget::onStreamSourceChanged);
    
    m_recordButton = new QPushButton("Record");
    m_recordButton->setCheckable(true);
    connect(m_recordButton, &QPushButton::clicked,
            this, &VideoStreamWidget::onToggleRecording);
    
    m_snapshotButton = new QPushButton("📷 Snapshot");
    connect(m_snapshotButton, &QPushButton::clicked,
            this, &VideoStreamWidget::onSnapshot);
    
    m_fpsLabel = new QLabel("FPS: 0.0");
    m_fpsLabel->setStyleSheet("QLabel { padding: 3px 8px; background-color: #424242; color: #4caf50; border-radius: 3px; font-family: monospace; }");
    
    controlLayout->addWidget(m_streamSelector);
    controlLayout->addWidget(m_recordButton);
    controlLayout->addWidget(m_snapshotButton);
    controlLayout->addWidget(m_fpsLabel);
    controlLayout->addStretch();

    // Video display
    m_videoLabel = new QLabel("No video stream");
    m_videoLabel->setAlignment(Qt::AlignCenter);
    m_videoLabel->setMinimumSize(640, 480);
    m_videoLabel->setStyleSheet("QLabel { background-color: black; color: white; }");
    m_videoLabel->setScaledContents(true);
    m_videoLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    mainLayout->addLayout(controlLayout);
    mainLayout->addWidget(m_videoLabel, 1); // Add stretch factor
}

bool VideoStreamWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::imageReceived,
                this, &VideoStreamWidget::onImageReceived);
        Logger::instance().info("Video stream widget initialized");
        return true;
    }
    
    Logger::instance().warning("Video stream widget initialized without ROS2");
    return false;
}

void VideoStreamWidget::onImageReceived(const QByteArray& imageData, int width, int height)
{
    // Convert image data to QImage and display
    // Assuming RGB8 format
    if (imageData.size() != width * height * 3) {
        return;
    }

    QImage image(reinterpret_cast<const uchar*>(imageData.data()),
                 width, height, width * 3, QImage::Format_RGB888);
    
    if (!image.isNull()) {
        m_currentFrame = QPixmap::fromImage(image);
        m_videoLabel->setPixmap(m_currentFrame);
        
        // Track frame time for FPS calculation
        m_frameTimes.enqueue(m_fpsTimer.elapsed());
        
        // Keep only last 2 seconds of frame times
        while (!m_frameTimes.isEmpty() && m_frameTimes.first() < m_fpsTimer.elapsed() - 2000) {
            m_frameTimes.dequeue();
        }
    }
}

void VideoStreamWidget::updateFPS()
{
    if (m_frameTimes.size() < 2) {
        m_fpsLabel->setText("FPS: 0.0");
        return;
    }
    
    qint64 timeSpan = m_frameTimes.last() - m_frameTimes.first();
    if (timeSpan > 0) {
        double fps = (m_frameTimes.size() - 1) * 1000.0 / timeSpan;
        m_fpsLabel->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    }
}

void VideoStreamWidget::onSnapshot()
{
    if (m_currentFrame.isNull()) {
        Logger::instance().warning("No frame available for snapshot");
        return;
    }
    
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString filename = QFileDialog::getSaveFileName(
        this,
        tr("Save Snapshot"),
        QString("snapshot_%1.png").arg(timestamp),
        tr("Images (*.png *.jpg *.bmp)")
    );
    
    if (!filename.isEmpty()) {
        if (m_currentFrame.save(filename)) {
            Logger::instance().info(QString("Snapshot saved: %1").arg(filename));
        } else {
            Logger::instance().error(QString("Failed to save snapshot: %1").arg(filename));
        }
    }
}

void VideoStreamWidget::onStreamSourceChanged(int index)
{
    Logger::instance().info(QString("Stream source changed to: %1")
                            .arg(m_streamSelector->itemText(index)));
    // In real implementation, switch ROS2 topic subscription
}

void VideoStreamWidget::onToggleRecording()
{
    m_recording = m_recordButton->isChecked();
    
    if (m_recording) {
        m_recordButton->setText("Stop Recording");
        Logger::instance().info("Video recording started");
    } else {
        m_recordButton->setText("Record");
        Logger::instance().info("Video recording stopped");
    }
}
