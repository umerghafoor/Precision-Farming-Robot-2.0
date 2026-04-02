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

    QLabel* m_currentIndexLabel;
    QLabel* m_classLabel;
    QLabel* m_classIdLabel;
    QLabel* m_confidenceLabel;
    QLabel* m_bboxLabel;
    QLabel* m_countLabel;
};

#endif // CURRENTDETECTIONWIDGET_H
