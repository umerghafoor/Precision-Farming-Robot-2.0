#ifndef COMMANDCONTROLWIDGET_H
#define COMMANDCONTROLWIDGET_H

#include "BaseWidget.h"
#include <QSlider>
#include <QLabel>
#include <QPushButton>

/**
 * @brief Widget for sending commands and controlling the robot
 */
class CommandControlWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit CommandControlWidget(QWidget *parent = nullptr);
    ~CommandControlWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Command & Control"; }

private slots:
    void onLinearSpeedChanged(int value);
    void onAngularSpeedChanged(int value);
    void onStopClicked();
    void onEmergencyStopClicked();
    void onSendCommand();

private:
    void setupUI();
    void sendVelocityCommand();

    QSlider* m_linearSpeedSlider;
    QSlider* m_angularSpeedSlider;
    QLabel* m_linearSpeedLabel;
    QLabel* m_angularSpeedLabel;
    QPushButton* m_stopButton;
    QPushButton* m_emergencyStopButton;

    double m_linearSpeed;
    double m_angularSpeed;
};

#endif // COMMANDCONTROLWIDGET_H
