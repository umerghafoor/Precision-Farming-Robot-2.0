#include "CurrentDetectionWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

CurrentDetectionWidget::CurrentDetectionWidget(QWidget *parent)
    : BaseWidget(parent)
{
    setupUI();
}

CurrentDetectionWidget::~CurrentDetectionWidget() = default;

void CurrentDetectionWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(8);

    m_countLabel = new QLabel("Detections in frame: 0", this);
    m_currentIndexLabel = new QLabel("Current detection index: -", this);
    m_classLabel = new QLabel("Class: -", this);
    m_classIdLabel = new QLabel("Class ID: -", this);
    m_confidenceLabel = new QLabel("Confidence: -", this);
    m_bboxLabel = new QLabel("Bbox: -", this);

    m_countLabel->setStyleSheet("font-size: 12pt; font-weight: bold;");

    mainLayout->addWidget(m_countLabel);
    mainLayout->addWidget(m_currentIndexLabel);
    mainLayout->addWidget(m_classLabel);
    mainLayout->addWidget(m_classIdLabel);
    mainLayout->addWidget(m_confidenceLabel);
    mainLayout->addWidget(m_bboxLabel);
    mainLayout->addStretch();
}

bool CurrentDetectionWidget::initialize()
{
    if (!m_ros2Interface) {
        Logger::instance().warning("CurrentDetectionWidget initialized without ROS2");
        return false;
    }

    connect(m_ros2Interface, &ROS2Interface::detectionResultsReceived,
            this, &CurrentDetectionWidget::onDetectionResultsReceived);

    Logger::instance().info("CurrentDetectionWidget initialized");
    return true;
}

void CurrentDetectionWidget::onDetectionResultsReceived(const QString& data)
{
    QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    if (doc.isNull() || !doc.isObject()) {
        Logger::instance().warning("CurrentDetectionWidget: invalid detection JSON");
        return;
    }

    QJsonObject obj = doc.object();
    QJsonArray detections = obj.value("detections").toArray();
    int total = detections.size();

    m_countLabel->setText(QString("Detections in frame: %1").arg(total));

    if (total == 0) {
        m_currentIndexLabel->setText("Current detection index: -");
        m_classLabel->setText("Class: -");
        m_classIdLabel->setText("Class ID: -");
        m_confidenceLabel->setText("Confidence: -");
        m_bboxLabel->setText("Bbox: -");
        return;
    }

    int bestIndex = 0;
    double bestConfidence = -1.0;

    for (int i = 0; i < total; ++i) {
        QJsonValue val = detections[i];
        if (!val.isObject()) continue;

        QJsonObject det = val.toObject();
        double conf = det.value("confidence").toDouble(-1.0);
        if (conf > bestConfidence) {
            bestConfidence = conf;
            bestIndex = i;
        }
    }

    QJsonObject best = detections.at(bestIndex).toObject();
    QString className = best.value("class").toString("-");
    int classId = best.value("class_id").toInt(-1);
    QString confidence = QString::number(bestConfidence, 'f', 3);

    QString bboxText = "-";
    if (best.contains("bbox") && best.value("bbox").isArray()) {
        QJsonArray bbox = best.value("bbox").toArray();
        QStringList parts;
        for (int j = 0; j < bbox.size(); ++j) {
            parts << QString::number(bbox[j].toDouble(0.0), 'f', 3);
        }
        bboxText = parts.join(", ");
    }

    m_currentIndexLabel->setText(QString("Current detection index: %1").arg(bestIndex));
    m_classLabel->setText(QString("Class: %1").arg(className));
    m_classIdLabel->setText(QString("Class ID: %1").arg(classId));
    m_confidenceLabel->setText(QString("Confidence: %1").arg(confidence));
    m_bboxLabel->setText(QString("Bbox: [%1]").arg(bboxText));

    Logger::instance().debug(QString("CurrentDetectionWidget updated: index=%1 class=%2 conf=%3")
                             .arg(bestIndex).arg(className).arg(confidence));
}
