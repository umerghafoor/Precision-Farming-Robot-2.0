#include "CommandControlWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QGridLayout>

CommandControlWidget::CommandControlWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_linearSpeed(0.0)
    , m_angularSpeed(0.0)
{
    setupUI();
}

CommandControlWidget::~CommandControlWidget()
{
}

void CommandControlWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Velocity control group
    QGroupBox* velocityGroup = new QGroupBox("Velocity Control");
    QGridLayout* velocityLayout = new QGridLayout();

    // Linear speed control
    velocityLayout->addWidget(new QLabel("Linear Speed:"), 0, 0);
    m_linearSpeedSlider = new QSlider(Qt::Horizontal);
    m_linearSpeedSlider->setRange(-100, 100);
    m_linearSpeedSlider->setValue(0);
    connect(m_linearSpeedSlider, &QSlider::valueChanged,
            this, &CommandControlWidget::onLinearSpeedChanged);
    velocityLayout->addWidget(m_linearSpeedSlider, 0, 1);
    
    m_linearSpeedLabel = new QLabel("0.0 m/s");
    m_linearSpeedLabel->setMinimumWidth(80);
    velocityLayout->addWidget(m_linearSpeedLabel, 0, 2);

    // Angular speed control
    velocityLayout->addWidget(new QLabel("Angular Speed:"), 1, 0);
    m_angularSpeedSlider = new QSlider(Qt::Horizontal);
    m_angularSpeedSlider->setRange(-100, 100);
    m_angularSpeedSlider->setValue(0);
    connect(m_angularSpeedSlider, &QSlider::valueChanged,
            this, &CommandControlWidget::onAngularSpeedChanged);
    velocityLayout->addWidget(m_angularSpeedSlider, 1, 1);
    
    m_angularSpeedLabel = new QLabel("0.0 rad/s");
    m_angularSpeedLabel->setMinimumWidth(80);
    velocityLayout->addWidget(m_angularSpeedLabel, 1, 2);

    velocityGroup->setLayout(velocityLayout);

    // Control buttons
    QGroupBox* controlGroup = new QGroupBox("Robot Control");
    QVBoxLayout* controlLayout = new QVBoxLayout();

    m_stopButton = new QPushButton("STOP");
    m_stopButton->setStyleSheet("QPushButton { background-color: orange; color: white; font-weight: bold; padding: 10px; }");
    connect(m_stopButton, &QPushButton::clicked,
            this, &CommandControlWidget::onStopClicked);

    m_emergencyStopButton = new QPushButton("EMERGENCY STOP");
    m_emergencyStopButton->setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; padding: 10px; }");
    connect(m_emergencyStopButton, &QPushButton::clicked,
            this, &CommandControlWidget::onEmergencyStopClicked);

    controlLayout->addWidget(m_stopButton);
    controlLayout->addWidget(m_emergencyStopButton);
    controlGroup->setLayout(controlLayout);

    mainLayout->addWidget(velocityGroup);
    mainLayout->addWidget(controlGroup);
    mainLayout->addStretch(1); // Give stretch to push controls to top
}

bool CommandControlWidget::initialize()
{
    Logger::instance().info("Command control widget initialized");
    return true;
}

void CommandControlWidget::onLinearSpeedChanged(int value)
{
    m_linearSpeed = value / 100.0; // Convert to -1.0 to 1.0
    m_linearSpeedLabel->setText(QString("%1 m/s").arg(m_linearSpeed, 0, 'f', 2));
    sendVelocityCommand();
}

void CommandControlWidget::onAngularSpeedChanged(int value)
{
    m_angularSpeed = value / 100.0 * 3.14159; // Convert to radians
    m_angularSpeedLabel->setText(QString("%1 rad/s").arg(m_angularSpeed, 0, 'f', 2));
    sendVelocityCommand();
}

void CommandControlWidget::sendVelocityCommand()
{
    if (m_ros2Interface) {
        m_ros2Interface->publishVelocityCommand(m_linearSpeed, 0.0, m_angularSpeed);
        Logger::instance().debug(QString("Velocity command: linear=%1, angular=%2")
                                .arg(m_linearSpeed).arg(m_angularSpeed));
    }
}

void CommandControlWidget::onStopClicked()
{
    m_linearSpeedSlider->setValue(0);
    m_angularSpeedSlider->setValue(0);
    
    if (m_ros2Interface) {
        m_ros2Interface->publishVelocityCommand(0.0, 0.0, 0.0);
    }
    
    Logger::instance().info("Robot stopped");
}

void CommandControlWidget::onEmergencyStopClicked()
{
    onStopClicked();
    
    if (m_ros2Interface) {
        m_ros2Interface->publishRobotCommand("EMERGENCY_STOP");
    }
    
    Logger::instance().warning("EMERGENCY STOP activated");
}

void CommandControlWidget::onSendCommand()
{
    // Implementation for custom commands
}
