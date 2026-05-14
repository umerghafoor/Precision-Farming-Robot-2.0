#ifndef DETECTIONPANELWIDGET_H
#define DETECTIONPANELWIDGET_H

#include "BaseWidget.h"
#include <QHash>
#include <QLabel>
#include <QProgressBar>

/**
 * @brief Merged detection panel: current-frame details + cumulative KPI summary.
 *
 * Replaces the separate CurrentDetectionWidget and DetectionSummaryWidget docks
 * with a single, space-efficient panel.
 */
class DetectionPanelWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit DetectionPanelWidget(QWidget *parent = nullptr);
    ~DetectionPanelWidget() override;

    bool    initialize() override;
    QString displayName() const override { return "Detection Panel"; }

private slots:
    void onDetectionResultsReceived(const QString& data);

private:
    void setupUI();
    void setNoDetectionState();
    void updateSummaryLabels();

    // ── Current detection ─────────────────────────────────────────────
    QLabel*       m_countBadge;
    QLabel*       m_timestampLabel;
    QLabel*       m_classValue;
    QProgressBar* m_confidenceBar;
    QLabel*       m_confidenceValue;
    QLabel*       m_classIdValue;
    QLabel*       m_bboxValue;

    // ── Cumulative summary ────────────────────────────────────────────
    int            m_totalDetections;
    int            m_framesWithResults;
    int            m_lastFrameCount;
    QHash<QString, int> m_classCounts;

    QLabel*       m_totalDetectionsValue;
    QLabel*       m_framesValue;
    QLabel*       m_averagePerFrameValue;
    QLabel*       m_uniqueClassesValue;
    QLabel*       m_topClassValue;
};

#endif // DETECTIONPANELWIDGET_H
