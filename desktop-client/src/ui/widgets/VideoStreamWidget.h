#ifndef VIDEOSTREAMWIDGET_H
#define VIDEOSTREAMWIDGET_H

#include "BaseWidget.h"
#include <QLabel>
#include <QTabBar>
#include <QPushButton>
#include <QElapsedTimer>
#include <QTimer>
#include <QPropertyAnimation>

class ZoomableImageView;

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

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void onImageReceived(const QByteArray& imageData, int width, int height);
    void onStreamTabChanged(int index);
    void onZoomInClicked();
    void onZoomOutClicked();
    void onResetZoomClicked();
    void onToggleRecording();
    void onRobotStatusUpdated(const QString& status);

private:
    void setupUI();
    void showPlaceholder();
    void updateOverlayInfo(int width, int height);

    QWidget* m_videoContainer;
    ZoomableImageView* m_videoLabel;
    QTabBar* m_tabBar;
    QPushButton* m_zoomInButton;
    QPushButton* m_zoomOutButton;
    QPushButton* m_resetZoomButton;
    QLabel* m_topicOverlay;
    QLabel* m_resolutionOverlay;
    QLabel* m_statusOverlay;
    QLabel* m_liveDot;
    QPushButton* m_recordButton;
    bool m_recording;
    QElapsedTimer m_frameTimer;
    qint64 m_lastFrameTime;
    QString m_currentTopic;
    QTimer* m_liveExpiryTimer;
    QPropertyAnimation* m_pulseAnimation;
};

#endif // VIDEOSTREAMWIDGET_H
