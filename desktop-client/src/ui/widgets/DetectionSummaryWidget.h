#ifndef DETECTIONSUMMARYWIDGET_H
#define DETECTIONSUMMARYWIDGET_H

#include "BaseWidget.h"

#include <QLabel>

class DetectionSummaryWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit DetectionSummaryWidget(QWidget *parent = nullptr);
    ~DetectionSummaryWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Detection Summary"; }

private slots:
    void onDetectionResultsReceived(const QString& data);

private:
    void setupUI();

    QLabel* m_totalDetectionsLabel;
    QLabel* m_uniqueClassesLabel;
    QLabel* m_averageConfidenceLabel;
    QLabel* m_minConfidenceLabel;
    QLabel* m_maxConfidenceLabel;
    QLabel* m_lastUpdatedLabel;
};

#endif // DETECTIONSUMMARYWIDGET_H
