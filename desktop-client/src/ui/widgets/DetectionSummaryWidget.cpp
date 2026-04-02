#include "DetectionSummaryWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

DetectionSummaryWidget::DetectionSummaryWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_totalDetections(0)
    , m_framesWithResults(0)
    , m_lastFrameCount(0)
{
    setupUI();
    updateLabels();
}

DetectionSummaryWidget::~DetectionSummaryWidget()
{
}

void DetectionSummaryWidget::setupUI()
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
        valueLabel = new QLabel("0");
        valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

        grid->addWidget(label, row, 0, Qt::AlignTop);
        grid->addWidget(valueLabel, row, 1);
    };

    addRow(0, "Total Detections:", m_totalDetectionsValue);
    addRow(1, "Frames Processed:", m_framesValue);
    addRow(2, "Avg / Frame:", m_averagePerFrameValue);
    addRow(3, "Unique Classes:", m_uniqueClassesValue);
    addRow(4, "Top Class:", m_topClassValue);
    addRow(5, "Current Frame Count:", m_lastFrameCountValue);

    grid->setColumnStretch(1, 1);

    mainLayout->addLayout(grid);
    mainLayout->addStretch(1);

    setMinimumSize(320, 220);
}

bool DetectionSummaryWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::detectionResultsReceived,
                this, &DetectionSummaryWidget::onDetectionResultsReceived);
        Logger::instance().info("Detection summary widget initialized");
        return true;
    }

    Logger::instance().warning("Detection summary widget initialized without ROS2");
    return false;
}

void DetectionSummaryWidget::onDetectionResultsReceived(const QString& data)
{
    const QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    if (!doc.isObject()) {
        Logger::instance().warning("DetectionSummaryWidget: invalid detection JSON payload");
        return;
    }

    const QJsonArray detections = doc.object().value("detections").toArray();
    m_lastFrameCount = detections.size();
    m_totalDetections += m_lastFrameCount;
    m_framesWithResults += 1;

    for (const QJsonValue& detectionValue : detections) {
        const QString className = detectionValue.toObject().value("class").toString("Unknown");
        m_classCounts[className] = m_classCounts.value(className, 0) + 1;
    }

    updateLabels();
}

void DetectionSummaryWidget::updateLabels()
{
    m_totalDetectionsValue->setText(QString::number(m_totalDetections));
    m_framesValue->setText(QString::number(m_framesWithResults));
    m_lastFrameCountValue->setText(QString::number(m_lastFrameCount));
    m_uniqueClassesValue->setText(QString::number(m_classCounts.size()));

    const double average = m_framesWithResults > 0
        ? static_cast<double>(m_totalDetections) / static_cast<double>(m_framesWithResults)
        : 0.0;
    m_averagePerFrameValue->setText(QString::number(average, 'f', 2));

    QString topClass = "-";
    int topCount = -1;
    for (auto it = m_classCounts.constBegin(); it != m_classCounts.constEnd(); ++it) {
        if (it.value() > topCount) {
            topClass = QString("%1 (%2)").arg(it.key()).arg(it.value());
            topCount = it.value();
        }
    }

    m_topClassValue->setText(topClass);
}
