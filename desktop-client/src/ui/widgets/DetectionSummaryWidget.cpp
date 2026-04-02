#include "DetectionSummaryWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QSet>

DetectionSummaryWidget::DetectionSummaryWidget(QWidget *parent)
    : BaseWidget(parent)
{
    setupUI();
}

DetectionSummaryWidget::~DetectionSummaryWidget() = default;

void DetectionSummaryWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(8);

    m_totalDetectionsLabel = new QLabel("Total Detections: 0", this);
    m_uniqueClassesLabel = new QLabel("Unique Classes: 0", this);
    m_averageConfidenceLabel = new QLabel("Average Confidence: 0.00", this);
    m_minConfidenceLabel = new QLabel("Min Confidence: 0.00", this);
    m_maxConfidenceLabel = new QLabel("Max Confidence: 0.00", this);
    m_lastUpdatedLabel = new QLabel("Last update: -", this);

    m_totalDetectionsLabel->setStyleSheet("font-size: 12pt; font-weight: bold;");
    m_uniqueClassesLabel->setStyleSheet("font-size: 11pt;");

    mainLayout->addWidget(m_totalDetectionsLabel);
    mainLayout->addWidget(m_uniqueClassesLabel);
    mainLayout->addWidget(m_averageConfidenceLabel);
    mainLayout->addWidget(m_minConfidenceLabel);
    mainLayout->addWidget(m_maxConfidenceLabel);
    mainLayout->addWidget(m_lastUpdatedLabel);
    mainLayout->addStretch();
}

bool DetectionSummaryWidget::initialize()
{
    if (!m_ros2Interface) {
        Logger::instance().warning("DetectionSummaryWidget initialized without ROS2");
        return false;
    }

    connect(m_ros2Interface, &ROS2Interface::detectionResultsReceived,
            this, &DetectionSummaryWidget::onDetectionResultsReceived);

    Logger::instance().info("DetectionSummaryWidget initialized");
    return true;
}

void DetectionSummaryWidget::onDetectionResultsReceived(const QString& data)
{
    QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    if (doc.isNull() || !doc.isObject()) {
        Logger::instance().warning("DetectionSummaryWidget: invalid detection JSON");
        return;
    }

    QJsonObject obj = doc.object();
    QJsonArray detections = obj.value("detections").toArray();
    int total = detections.size();

    QSet<QString> unique;
    double minConf = 1.0;
    double maxConf = 0.0;
    double sumConf = 0.0;
    int confCount = 0;

    for (const QJsonValue& entry : detections) {
        if (!entry.isObject()) continue;

        QJsonObject det = entry.toObject();
        QString className = det.value("class").toString(det.value("class_id").toVariant().toString());

        unique.insert(className);

        if (det.contains("confidence")) {
            double conf = det.value("confidence").toDouble(-1.0);
            if (conf >= 0.0) {
                sumConf += conf;
                confCount += 1;
                minConf = qMin(minConf, conf);
                maxConf = qMax(maxConf, conf);
            }
        }
    }

    double avgConf = (confCount > 0) ? (sumConf / confCount) : 0.0;
    if (confCount == 0) {
        minConf = 0.0;
        maxConf = 0.0;
    }

    m_totalDetectionsLabel->setText(QString("Total Detections: %1").arg(total));
    m_uniqueClassesLabel->setText(QString("Unique Classes: %1").arg(unique.size()));
    m_averageConfidenceLabel->setText(QString("Average Confidence: %1").arg(avgConf, 0, 'f', 3));
    m_minConfidenceLabel->setText(QString("Min Confidence: %1").arg(minConf, 0, 'f', 3));
    m_maxConfidenceLabel->setText(QString("Max Confidence: %1").arg(maxConf, 0, 'f', 3));
    m_lastUpdatedLabel->setText(QString("Last update: %1").arg(QDateTime::currentDateTime().toString("HH:mm:ss")));

    Logger::instance().debug(QString("DetectionSummaryWidget updated: total=%1 unique=%2 avg=%3")
                             .arg(total)
                             .arg(unique.size())
                             .arg(avgConf, 0, 'f', 3));
}
