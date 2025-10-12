#ifndef VIDEOSTREAMWIDGET_H
#define VIDEOSTREAMWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QComboBox>
#include <QPushButton>

/**
 * @brief Widget for displaying video streams from robot cameras
 */
class VideoStreamWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit VideoStreamWidget(QWidget *parent = nullptr);
    ~VideoStreamWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Video Stream"; }

private slots:
    void onImageReceived(const QByteArray& imageData, int width, int height);
    void onStreamSourceChanged(int index);
    void onToggleRecording();

private:
    void setupUI();

    QLabel* m_videoLabel;
    QComboBox* m_streamSelector;
    QPushButton* m_recordButton;
    bool m_recording;
};

#endif // VIDEOSTREAMWIDGET_H
