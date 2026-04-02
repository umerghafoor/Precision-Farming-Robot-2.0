#ifndef DETECTIONSUMMARYWIDGET_H
#define DETECTIONSUMMARYWIDGET_H

#include "BaseWidget.h"
#include <QHash>

class QLabel;

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
    void updateLabels();

    int m_totalDetections;
    int m_framesWithResults;
    int m_lastFrameCount;
    QHash<QString, int> m_classCounts;

    QLabel* m_totalDetectionsValue;
    QLabel* m_framesValue;
    QLabel* m_averagePerFrameValue;
    QLabel* m_uniqueClassesValue;
    QLabel* m_topClassValue;
    QLabel* m_lastFrameCountValue;
};

#endif // DETECTIONSUMMARYWIDGET_H