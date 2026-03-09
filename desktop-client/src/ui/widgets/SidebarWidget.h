#ifndef SIDEBARWIDGET_H
#define SIDEBARWIDGET_H

#include "BaseWidget.h"
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QRadioButton>
#include <QScrollArea>
#include <QMap>

/**
 * @brief Unified sidebar containing motion and command controls
 *
 * This widget replaces the separate MotionControl and CommandControl docks.
 * It provides a 3x3 D-pad, speed/radius sliders, spin buttons, and the
 * standard stop/emergency buttons anchored to the bottom of the panel.  All
 * ROS2 publishers/subscriptions are unchanged; this class simply consolidates
 * UI elements in a single scrollable container.
 */
class SidebarWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit SidebarWidget(QWidget *parent = nullptr);
    ~SidebarWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Controls"; }

private slots:
    // motion control slots
    void onSpeedChanged(int value);
    void onMotionPressed();
    void onMotionReleased();

    // command slider slots (legacy, may be removed later)
    void onLinearSpeedChanged(int value);
    void onAngularSpeedChanged(int value);

    // stop/emergency slots
    void onStopClicked();
    void onEmergencyStopClicked();

private:
    enum class Motion {
        Forward,
        Backward,
        Left,
        Right,
        FwdLeft,
        FwdRight,
        BackLeft,
        BackRight,
        SpinLeft,
        SpinRight
    };

    void setupUI();
    void sendVelocityFor(Motion motion);
    void sendStop();
    void sendVelocityCommand();

    // scrolling content container
    QScrollArea* m_scrollArea;
    QWidget* m_scrollContent;

    // motion controls
    QSlider* m_speedSlider;
    QLabel* m_speedLabel;
    QMap<Motion, QPushButton*> m_buttons;
    double m_speed; // 0.0 - 1.0
    double m_maxLinear;
    double m_maxAngular;

    // curve radius
    QSlider* m_radiusSlider;
    QLabel* m_radiusLabel;
    double m_radius;

    // spin
    QPushButton* m_spinLeftButton;
    QPushButton* m_spinRightButton;

    // pin
    QRadioButton* m_pinRadioButton;
    bool m_isPinned;
    Motion m_pinnedMotion;

    // legacy velocity sliders/labels (will serve as read-only feedback later)
    double m_linearSpeed;
    double m_angularSpeed;
    QSlider* m_linearSpeedSlider;
    QSlider* m_angularSpeedSlider;
    QLabel* m_linearSpeedLabel;
    QLabel* m_angularSpeedLabel;

    // bottom area buttons
    QPushButton* m_stopButton;
    QPushButton* m_emergencyStopButton;
};

#endif // SIDEBARWIDGET_H
