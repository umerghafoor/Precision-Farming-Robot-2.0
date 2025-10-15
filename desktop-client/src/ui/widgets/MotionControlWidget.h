#ifndef MOTIONCONTROLWIDGET_H
#define MOTIONCONTROLWIDGET_H

#include "BaseWidget.h"
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QRadioButton>

/**
 * @brief Widget providing 8-direction motion buttons and a speed slider
 *
 * Publishes velocity commands through ROS2Interface (geometry_msgs::Twist)
 */
class MotionControlWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit MotionControlWidget(QWidget *parent = nullptr);
    ~MotionControlWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Motion Control"; }

private slots:
    void onSpeedChanged(int value);
    void onMotionPressed();
    void onMotionReleased();

private:
    enum class Motion { Forward, Backward, Left, Right, FwdLeft, FwdRight, BackLeft, BackRight, SpinLeft, SpinRight };

    void setupUI();
    void sendVelocityFor(Motion motion);
    void sendStop();

    // UI
    QSlider* m_speedSlider;
    QLabel* m_speedLabel;
    QMap<Motion, QPushButton*> m_buttons;

    double m_speed; // 0.0 - 1.0 (slider fraction)
    // Differential drive configuration
    double m_maxLinear;   // m/s
    double m_maxAngular;  // rad/s

    // Curve radius control (meters)
    QSlider* m_radiusSlider;
    QLabel* m_radiusLabel;
    double m_radius; // meters

    // Spin in place buttons
    QPushButton* m_spinLeftButton;
    QPushButton* m_spinRightButton;
    QPushButton* m_stopButton;

    // Pin / latch behavior
    QRadioButton* m_pinRadioButton;
    bool m_isPinned;
    Motion m_pinnedMotion;
};

#endif // MOTIONCONTROLWIDGET_H
