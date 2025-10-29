#ifndef VIDEOSTREAMWIDGET_H
#define VIDEOSTREAMWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QElapsedTimer>
#include <QQueue>

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
    void onSnapshot();
    void updateFPS();

private:
    void setupUI();

    QLabel* m_videoLabel;
    QComboBox* m_streamSelector;
    QPushButton* m_recordButton;
    QPushButton* m_snapshotButton;
    QLabel* m_fpsLabel;
    bool m_recording;
    
    // FPS calculation
    QElapsedTimer m_fpsTimer;
    QQueue<qint64> m_frameTimes;
    QTimer* m_fpsUpdateTimer;
    QPixmap m_currentFrame;
};

#endif // VIDEOSTREAMWIDGET_H
