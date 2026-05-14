#include "CurrentDetectionWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
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

CurrentDetectionWidget::~CurrentDetectionWidget() {}

void CurrentDetectionWidget::setupUI()
{
    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);

    // ── Header: count badge + timestamp ──────────────────────────────────
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

    // ── Primary: class name card ──────────────────────────────────────────
    QFrame* classCard = new QFrame(this);
    classCard->setObjectName("detectionClassCard");
    QVBoxLayout* classLay = new QVBoxLayout(classCard);
    classLay->setContentsMargins(16, 14, 16, 14);

    QLabel* classLbl = new QLabel("DETECTED CLASS", classCard);
    classLbl->setObjectName("detectionCardLabel");
    classLbl->setAlignment(Qt::AlignCenter);

    m_classValue = new QLabel("-", classCard);
    m_classValue->setObjectName("detectionClassName");
    m_classValue->setAlignment(Qt::AlignCenter);
    m_classValue->setWordWrap(true);

    classLay->addWidget(classLbl);
    classLay->addWidget(m_classValue);

    // ── Confidence row ────────────────────────────────────────────────────
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

    // ── Detail row: class ID + bbox ───────────────────────────────────────
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

    // ── Assemble ──────────────────────────────────────────────────────────
    root->addWidget(header);
    root->addWidget(classCard, 1);
    root->addWidget(confRow);
    root->addWidget(detailRow);

    setMinimumSize(300, 220);
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
    m_countBadge->setText("0");
    m_classValue->setText("None");
    m_classIdValue->setText("-");
    m_confidenceValue->setText("—");
    m_confidenceBar->setValue(0);
    m_bboxValue->setText("-");
    m_timestampLabel->setText("Waiting for data…");
}

void CurrentDetectionWidget::onDetectionResultsReceived(const QString& data)
{
    const QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    if (!doc.isObject()) {
        Logger::instance().warning("CurrentDetectionWidget: invalid JSON payload");
        return;
    }

    const QJsonArray detections = doc.object().value("detections").toArray();
    m_countBadge->setText(QString::number(detections.size()));
    m_timestampLabel->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));

    if (detections.isEmpty()) {
        m_classValue->setText("None");
        m_classIdValue->setText("-");
        m_confidenceValue->setText("—");
        m_confidenceBar->setValue(0);
        m_bboxValue->setText("-");
        return;
    }

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
