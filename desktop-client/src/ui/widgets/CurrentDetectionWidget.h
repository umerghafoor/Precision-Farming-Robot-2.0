#ifndef CURRENTDETECTIONWIDGET_H
#define CURRENTDETECTIONWIDGET_H

#include "BaseWidget.h"
#include <QLabel>

class CurrentDetectionWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit CurrentDetectionWidget(QWidget *parent = nullptr);
    ~CurrentDetectionWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Current Detection"; }

private slots:
    void onDetectionResultsReceived(const QString& data);

private:
    void setupUI();
    void setNoDetectionState();

    QLabel* m_frameDetectionsValue;
    QLabel* m_classValue;
    QLabel* m_classIdValue;
    QLabel* m_confidenceValue;
    QLabel* m_bboxValue;
    QLabel* m_lastUpdatedValue;
};

#endif // CURRENTDETECTIONWIDGET_H