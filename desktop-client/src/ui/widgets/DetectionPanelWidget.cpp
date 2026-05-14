#include "DetectionPanelWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>

// ── Helper: compact inline KPI card ──────────────────────────────────────────
static QFrame* makeMiniStatCard(const QString& label, QLabel*& valueOut, QWidget* parent)
{
    QFrame* card = new QFrame(parent);
    card->setObjectName("panelStatCard");
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QVBoxLayout* lay = new QVBoxLayout(card);
    lay->setContentsMargins(8, 8, 8, 6);
    lay->setSpacing(2);

    valueOut = new QLabel("0", card);
    valueOut->setObjectName("panelStatValue");
    valueOut->setAlignment(Qt::AlignCenter);
    lay->addWidget(valueOut);

    QLabel* nameLabel = new QLabel(label.toUpper(), card);
    nameLabel->setObjectName("panelStatLabel");
    nameLabel->setAlignment(Qt::AlignCenter);
    nameLabel->setWordWrap(true);
    lay->addWidget(nameLabel);

    return card;
}

// ── Constructor / destructor ──────────────────────────────────────────────────
DetectionPanelWidget::DetectionPanelWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_totalDetections(0)
    , m_framesWithResults(0)
    , m_lastFrameCount(0)
{
    setupUI();
    setNoDetectionState();
    updateSummaryLabels();
}

DetectionPanelWidget::~DetectionPanelWidget() {}

// ── UI construction ───────────────────────────────────────────────────────────
void DetectionPanelWidget::setupUI()
{
    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(6);

    // ── Header: count badge + timestamp ──────────────────────────────
    QWidget* header = new QWidget(this);
    QHBoxLayout* headerLay = new QHBoxLayout(header);
    headerLay->setContentsMargins(0, 0, 0, 0);
    headerLay->setSpacing(8);

    m_countBadge = new QLabel("0", header);
    m_countBadge->setObjectName("detectionCountBadge");
    m_countBadge->setAlignment(Qt::AlignCenter);
    m_countBadge->setFixedSize(36, 36);

    QLabel* countLabel = new QLabel("objects\nin frame", header);
    countLabel->setObjectName("detectionCountLabel");

    m_timestampLabel = new QLabel("Waiting…", header);
    m_timestampLabel->setObjectName("detectionTimestamp");
    m_timestampLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    headerLay->addWidget(m_countBadge);
    headerLay->addWidget(countLabel);
    headerLay->addStretch(1);
    headerLay->addWidget(m_timestampLabel);

    // ── Class name card ───────────────────────────────────────────────
    QFrame* classCard = new QFrame(this);
    classCard->setObjectName("detectionClassCard");
    QVBoxLayout* classLay = new QVBoxLayout(classCard);
    classLay->setContentsMargins(16, 10, 16, 10);
    classLay->setSpacing(2);

    QLabel* classLbl = new QLabel("DETECTED CLASS", classCard);
    classLbl->setObjectName("detectionCardLabel");
    classLbl->setAlignment(Qt::AlignCenter);

    m_classValue = new QLabel("-", classCard);
    m_classValue->setObjectName("detectionClassName");
    m_classValue->setAlignment(Qt::AlignCenter);
    m_classValue->setWordWrap(true);

    classLay->addWidget(classLbl);
    classLay->addWidget(m_classValue);

    // ── Confidence row ────────────────────────────────────────────────
    QWidget* confRow = new QWidget(this);
    QHBoxLayout* confLay = new QHBoxLayout(confRow);
    confLay->setContentsMargins(0, 0, 0, 0);
    confLay->setSpacing(10);

    QLabel* confLbl = new QLabel("Confidence", confRow);
    confLbl->setObjectName("detectionFieldLabel");
    confLbl->setFixedWidth(80);

    m_confidenceBar = new QProgressBar(confRow);
    m_confidenceBar->setObjectName("detectionConfBar");
    m_confidenceBar->setRange(0, 100);
    m_confidenceBar->setValue(0);
    m_confidenceBar->setTextVisible(false);
    m_confidenceBar->setFixedHeight(8);

    m_confidenceValue = new QLabel("—", confRow);
    m_confidenceValue->setObjectName("detectionConfValue");
    m_confidenceValue->setFixedWidth(48);
    m_confidenceValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    confLay->addWidget(confLbl);
    confLay->addWidget(m_confidenceBar, 1);
    confLay->addWidget(m_confidenceValue);

    // ── Detail row: class ID + bbox ───────────────────────────────────
    QFrame* detailRow = new QFrame(this);
    detailRow->setObjectName("detectionDetailRow");
    QHBoxLayout* detailLay = new QHBoxLayout(detailRow);
    detailLay->setContentsMargins(10, 6, 10, 6);
    detailLay->setSpacing(16);

    QLabel* idLbl = new QLabel("ID:", detailRow);
    idLbl->setObjectName("detectionFieldLabel");
    m_classIdValue = new QLabel("-", detailRow);
    m_classIdValue->setObjectName("detectionDetailValue");

    QLabel* bboxLbl = new QLabel("BBox:", detailRow);
    bboxLbl->setObjectName("detectionFieldLabel");
    m_bboxValue = new QLabel("-", detailRow);
    m_bboxValue->setObjectName("detectionDetailValue");
    m_bboxValue->setWordWrap(false);

    detailLay->addWidget(idLbl);
    detailLay->addWidget(m_classIdValue);
    detailLay->addSpacing(8);
    detailLay->addWidget(bboxLbl);
    detailLay->addWidget(m_bboxValue, 1);

    // ── Divider ───────────────────────────────────────────────────────
    QFrame* divider = new QFrame(this);
    divider->setObjectName("panelDivider");
    divider->setFrameShape(QFrame::HLine);
    divider->setFrameShadow(QFrame::Plain);
    divider->setFixedHeight(1);

    // ── KPI mini cards (4-up horizontal row) ─────────────────────────
    QWidget* kpiRow = new QWidget(this);
    QHBoxLayout* kpiLay = new QHBoxLayout(kpiRow);
    kpiLay->setContentsMargins(0, 0, 0, 0);
    kpiLay->setSpacing(6);
    kpiLay->addWidget(makeMiniStatCard("Total",   m_totalDetectionsValue,  kpiRow));
    kpiLay->addWidget(makeMiniStatCard("Frames",  m_framesValue,           kpiRow));
    kpiLay->addWidget(makeMiniStatCard("Avg/Fr",  m_averagePerFrameValue,  kpiRow));
    kpiLay->addWidget(makeMiniStatCard("Classes", m_uniqueClassesValue,    kpiRow));

    // ── Top class banner ──────────────────────────────────────────────
    QFrame* topClassCard = new QFrame(this);
    topClassCard->setObjectName("summaryTopClass");
    topClassCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QHBoxLayout* topLay = new QHBoxLayout(topClassCard);
    topLay->setContentsMargins(14, 8, 14, 8);
    topLay->setSpacing(8);

    QLabel* topIcon = new QLabel("TOP", topClassCard);
    topIcon->setObjectName("summaryTopIcon");

    QLabel* topLabel = new QLabel("Top Class", topClassCard);
    topLabel->setObjectName("summaryTopLabel");

    m_topClassValue = new QLabel("-", topClassCard);
    m_topClassValue->setObjectName("summaryTopClassName");
    m_topClassValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    m_topClassValue->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    topLay->addWidget(topIcon);
    topLay->addWidget(topLabel);
    topLay->addWidget(m_topClassValue);

    // ── Assemble ──────────────────────────────────────────────────────
    root->addWidget(header);
    root->addWidget(classCard);
    root->addWidget(confRow);
    root->addWidget(detailRow);
    root->addWidget(divider);
    root->addWidget(kpiRow);
    root->addWidget(topClassCard);

    setMinimumSize(300, 320);
}

// ── Initialization ────────────────────────────────────────────────────────────
bool DetectionPanelWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::detectionResultsReceived,
                this, &DetectionPanelWidget::onDetectionResultsReceived);
        Logger::instance().info("Detection panel widget initialized");
        return true;
    }
    Logger::instance().warning("Detection panel widget initialized without ROS2");
    return false;
}

// ── Helpers ───────────────────────────────────────────────────────────────────
void DetectionPanelWidget::setNoDetectionState()
{
    m_countBadge->setText("0");
    m_classValue->setText("None");
    m_classIdValue->setText("-");
    m_confidenceValue->setText("—");
    m_confidenceBar->setValue(0);
    m_bboxValue->setText("-");
    m_timestampLabel->setText("Waiting for data…");
}

void DetectionPanelWidget::updateSummaryLabels()
{
    m_totalDetectionsValue->setText(QString::number(m_totalDetections));
    m_framesValue->setText(QString::number(m_framesWithResults));
    m_uniqueClassesValue->setText(QString::number(m_classCounts.size()));

    const double average = m_framesWithResults > 0
        ? static_cast<double>(m_totalDetections) / static_cast<double>(m_framesWithResults)
        : 0.0;
    m_averagePerFrameValue->setText(QString::number(average, 'f', 1));

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

// ── Slot ──────────────────────────────────────────────────────────────────────
void DetectionPanelWidget::onDetectionResultsReceived(const QString& data)
{
    const QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    if (!doc.isObject()) {
        Logger::instance().warning("DetectionPanelWidget: invalid JSON payload");
        return;
    }

    const QJsonArray detections = doc.object().value("detections").toArray();

    // ── Update header ─────────────────────────────────────────────────
    m_countBadge->setText(QString::number(detections.size()));
    m_timestampLabel->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));

    // ── Update cumulative stats ───────────────────────────────────────
    m_lastFrameCount = detections.size();
    m_totalDetections += m_lastFrameCount;
    m_framesWithResults += 1;

    for (const QJsonValue& v : detections) {
        const QString cls = v.toObject().value("class").toString("Unknown");
        m_classCounts[cls] = m_classCounts.value(cls, 0) + 1;
    }

    // ── Update current-frame details ──────────────────────────────────
    if (detections.isEmpty()) {
        m_classValue->setText("None");
        m_classIdValue->setText("-");
        m_confidenceValue->setText("—");
        m_confidenceBar->setValue(0);
        m_bboxValue->setText("-");
    } else {
        // Pick highest-confidence detection
        QJsonObject best = detections.first().toObject();
        double bestConf = best.value("confidence").toDouble(0.0);
        for (const QJsonValue& v : detections) {
            const QJsonObject c = v.toObject();
            const double conf = c.value("confidence").toDouble(0.0);
            if (conf > bestConf) { bestConf = conf; best = c; }
        }

        const QString className = best.value("class").toString("Unknown");
        const int classId = best.value("class_id").toInt(-1);

        QString bboxText = "-";
        const QJsonArray bbox = best.value("bbox").toArray();
        if (bbox.size() >= 4) {
            bboxText = QString("%1, %2, %3×%4")
                .arg(bbox.at(0).toDouble(), 0, 'f', 0)
                .arg(bbox.at(1).toDouble(), 0, 'f', 0)
                .arg(bbox.at(2).toDouble(), 0, 'f', 0)
                .arg(bbox.at(3).toDouble(), 0, 'f', 0);
        }

        const int confPct = int(bestConf * 100.0);
        m_classValue->setText(className);
        m_classIdValue->setText(classId >= 0 ? QString::number(classId) : "-");
        m_confidenceValue->setText(QString::number(confPct) + "%");
        m_confidenceBar->setValue(confPct);
        m_bboxValue->setText(bboxText);
    }

    updateSummaryLabels();
}
