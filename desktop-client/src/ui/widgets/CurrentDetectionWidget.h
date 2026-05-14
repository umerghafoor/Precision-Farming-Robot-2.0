#ifndef CURRENTDETECTIONWIDGET_H
#define CURRENTDETECTIONWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QProgressBar>

class CurrentDetectionWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit CurrentDetectionWidget(QWidget *parent = nullptr);
    ~CurrentDetectionWidget() override;

    bool    initialize() override;
    QString displayName() const override { return "Current Detection"; }

private slots:
    void onDetectionResultsReceived(const QString& data);

private:
    void setupUI();
    void setNoDetectionState();

    // Header row
    QLabel*       m_countBadge;
    QLabel*       m_timestampLabel;

    // Primary — class name
    QLabel*       m_classValue;

    // Confidence row
    QProgressBar* m_confidenceBar;
    QLabel*       m_confidenceValue;

    // Detail row
    QLabel*       m_classIdValue;
    QLabel*       m_bboxValue;
};

#endif // CURRENTDETECTIONWIDGET_H
