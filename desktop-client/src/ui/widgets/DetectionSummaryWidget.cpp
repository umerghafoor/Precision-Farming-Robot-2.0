#include "DetectionSummaryWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFrame>
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

// Creates a KPI stat card: big monospace value + small uppercase label
static QFrame* makeStatCard(const QString& label, QLabel*& valueOut, QWidget* parent)
{
    QFrame* card = new QFrame(parent);
    card->setObjectName("summaryStatCard");
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QVBoxLayout* lay = new QVBoxLayout(card);
    lay->setContentsMargins(12, 12, 12, 10);
    lay->setSpacing(4);

    lay->addStretch(1);

    valueOut = new QLabel("0", card);
    valueOut->setObjectName("summaryStatValue");
    valueOut->setAlignment(Qt::AlignCenter);
    lay->addWidget(valueOut);

    QLabel* nameLabel = new QLabel(label.toUpper(), card);
    nameLabel->setObjectName("summaryStatLabel");
    nameLabel->setAlignment(Qt::AlignCenter);
    nameLabel->setWordWrap(true);
    lay->addWidget(nameLabel);

    lay->addStretch(1);
    return card;
}

void DetectionSummaryWidget::setupUI()
{
    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    // ── Row 1: three primary KPI cards ───────────────────────────────────
    QHBoxLayout* row1 = new QHBoxLayout();
    row1->setSpacing(8);
    row1->addWidget(makeStatCard("Total\nDetections", m_totalDetectionsValue, this));
    row1->addWidget(makeStatCard("Frames\nProcessed",  m_framesValue,         this));
    row1->addWidget(makeStatCard("Avg /\nFrame",       m_averagePerFrameValue, this));

    // ── Row 2: two secondary KPI cards ───────────────────────────────────
    QHBoxLayout* row2 = new QHBoxLayout();
    row2->setSpacing(8);
    row2->addWidget(makeStatCard("Current\nFrame",     m_lastFrameCountValue,  this));
    row2->addWidget(makeStatCard("Unique\nClasses",    m_uniqueClassesValue,   this));
    row2->addStretch(1);   // keep cards from stretching to fill 2-col width

    // ── Row 3: top class banner ───────────────────────────────────────────
    QFrame* topClassCard = new QFrame(this);
    topClassCard->setObjectName("summaryTopClass");
    topClassCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QHBoxLayout* topLay = new QHBoxLayout(topClassCard);
    topLay->setContentsMargins(14, 10, 14, 10);
    topLay->setSpacing(8);

    QLabel* topIcon = new QLabel(topClassCard);
    topIcon->setObjectName("summaryTopIcon");
    topIcon->setText("TOP");

    QLabel* topLabel = new QLabel("Top Class", topClassCard);
    topLabel->setObjectName("summaryTopLabel");

    m_topClassValue = new QLabel("-", topClassCard);
    m_topClassValue->setObjectName("summaryTopClassName");
    m_topClassValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_topClassValue->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    topLay->addWidget(topIcon);
    topLay->addWidget(topLabel);
    topLay->addWidget(m_topClassValue);

    // ── Assemble ──────────────────────────────────────────────────────────
    root->addLayout(row1, 3);
    root->addLayout(row2, 2);
    root->addWidget(topClassCard);

    setMinimumSize(320, 240);
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
            topClass = QString("%1  (%2)").arg(it.key()).arg(it.value());
            topCount = it.value();
        }
    }
    m_topClassValue->setText(topClass);
}
