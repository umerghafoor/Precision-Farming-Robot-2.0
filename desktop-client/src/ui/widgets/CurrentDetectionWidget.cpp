#include "CurrentDetectionWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QGridLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>

CurrentDetectionWidget::CurrentDetectionWidget(QWidget *parent)
    : BaseWidget(parent)
{
    setupUI();
    setNoDetectionState();
}

CurrentDetectionWidget::~CurrentDetectionWidget()
{
}

void CurrentDetectionWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(10);

    QGridLayout* grid = new QGridLayout();
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(8);

    auto addRow = [grid](int row, const QString& labelText, QLabel*& valueLabel) {
        QLabel* label = new QLabel(labelText);
        label->setStyleSheet("font-weight: 600;");
        valueLabel = new QLabel("-");
        valueLabel->setWordWrap(true);
        valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

        grid->addWidget(label, row, 0, Qt::AlignTop);
        grid->addWidget(valueLabel, row, 1);
    };

    addRow(0, "Detections in View:", m_frameDetectionsValue);
    addRow(1, "Class:", m_classValue);
    addRow(2, "Class ID:", m_classIdValue);
    addRow(3, "Confidence:", m_confidenceValue);
    addRow(4, "BBox:", m_bboxValue);
    addRow(5, "Last Updated:", m_lastUpdatedValue);

    grid->setColumnStretch(1, 1);

    mainLayout->addLayout(grid);
    mainLayout->addStretch(1);

    setMinimumSize(320, 220);
}

bool CurrentDetectionWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::detectionResultsReceived,
                this, &CurrentDetectionWidget::onDetectionResultsReceived);
        Logger::instance().info("Current detection widget initialized");
        return true;
    }

    Logger::instance().warning("Current detection widget initialized without ROS2");
    return false;
}

void CurrentDetectionWidget::setNoDetectionState()
{
    m_frameDetectionsValue->setText("0");
    m_classValue->setText("None");
    m_classIdValue->setText("-");
    m_confidenceValue->setText("-");
    m_bboxValue->setText("-");
    m_lastUpdatedValue->setText("Waiting for /detections/results");
}

void CurrentDetectionWidget::onDetectionResultsReceived(const QString& data)
{
    const QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    if (!doc.isObject()) {
        Logger::instance().warning("CurrentDetectionWidget: invalid detection JSON payload");
        return;
    }

    const QJsonObject root = doc.object();
    const QJsonArray detections = root.value("detections").toArray();
    m_frameDetectionsValue->setText(QString::number(detections.size()));

    if (detections.isEmpty()) {
        setNoDetectionState();
        m_lastUpdatedValue->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
        return;
    }

    QJsonObject bestDetection = detections.first().toObject();
    double bestConfidence = bestDetection.value("confidence").toDouble(0.0);

    for (const QJsonValue& detectionValue : detections) {
        const QJsonObject candidate = detectionValue.toObject();
        const double confidence = candidate.value("confidence").toDouble(0.0);
        if (confidence > bestConfidence) {
            bestConfidence = confidence;
            bestDetection = candidate;
        }
    }

    const QString className = bestDetection.value("class").toString("Unknown");
    const int classId = bestDetection.value("class_id").toInt(-1);

    QString bboxText = "-";
    const QJsonArray bbox = bestDetection.value("bbox").toArray();
    if (bbox.size() >= 4) {
        bboxText = QString("x=%1, y=%2, w=%3, h=%4")
                       .arg(bbox.at(0).toDouble(0.0), 0, 'f', 1)
                       .arg(bbox.at(1).toDouble(0.0), 0, 'f', 1)
                       .arg(bbox.at(2).toDouble(0.0), 0, 'f', 1)
                       .arg(bbox.at(3).toDouble(0.0), 0, 'f', 1);
    }

    m_classValue->setText(className);
    m_classIdValue->setText(classId >= 0 ? QString::number(classId) : "-");
    m_confidenceValue->setText(QString::number(bestConfidence * 100.0, 'f', 1) + "%");
    m_bboxValue->setText(bboxText);
    m_lastUpdatedValue->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
}
