#include "VideoStreamWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>
#include <QImage>

VideoStreamWidget::VideoStreamWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_recording(false)
{
    setupUI();
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
    
    controlLayout->addWidget(m_streamSelector);
    controlLayout->addWidget(m_recordButton);
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
    // Expecting RGB8 format (3 bytes per pixel)
    int expectedSize = width * height * 3;
    
    if (imageData.size() != expectedSize) {
        Logger::instance().warning(QString("Image size mismatch: expected %1, got %2")
                                  .arg(expectedSize).arg(imageData.size()));
        return;
    }

    QImage image(reinterpret_cast<const uchar*>(imageData.data()),
                 width, height, width * 3, QImage::Format_RGB888);
    
    if (!image.isNull()) {
        m_videoLabel->setPixmap(QPixmap::fromImage(image));
        m_videoLabel->setText(""); // Clear "No video stream" text
    } else {
        Logger::instance().error("Failed to create QImage from received data");
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
