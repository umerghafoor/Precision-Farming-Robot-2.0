#include "MotionControlWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QGridLayout>
#include <QGroupBox>
#include <QSpacerItem>
#include <QRadioButton>

MotionControlWidget::MotionControlWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_speed(0.5)
    , m_maxLinear(1.0)    // default 1 m/s
    , m_maxAngular(1.5)   // default 1.5 rad/s
    , m_radius(1.0)
    , m_isPinned(false)
{
    setupUI();
}

MotionControlWidget::~MotionControlWidget()
{
}

void MotionControlWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    QGroupBox* motionGroup = new QGroupBox("Motion");
    QGridLayout* grid = new QGridLayout();

    // Create 8 buttons in a 3x3 grid (center empty)
    auto makeButton = [&](Motion m, const QString& text){
        QPushButton* b = new QPushButton(text);
        b->setAutoRepeat(true);
        b->setAutoRepeatDelay(300);
        b->setAutoRepeatInterval(100);
        connect(b, &QPushButton::pressed, this, &MotionControlWidget::onMotionPressed);
        connect(b, &QPushButton::released, this, &MotionControlWidget::onMotionReleased);
        m_buttons[m] = b;
        return b;
    };

    grid->addWidget(makeButton(Motion::FwdLeft, "↖"), 0, 0);
    grid->addWidget(makeButton(Motion::Forward, "↑"), 0, 1);
    grid->addWidget(makeButton(Motion::FwdRight, "↗"), 0, 2);

    grid->addWidget(makeButton(Motion::SpinLeft, "⟲"), 1, 0);
    grid->addItem(new QSpacerItem(40, 40), 1, 1);
    grid->addWidget(makeButton(Motion::SpinRight, "⟳"), 1, 2);

    grid->addWidget(makeButton(Motion::BackLeft, "↙"), 2, 0);
    grid->addWidget(makeButton(Motion::Backward, "↓"), 2, 1);
    grid->addWidget(makeButton(Motion::BackRight, "↘"), 2, 2);

    motionGroup->setLayout(grid);

    // Speed slider
    QGroupBox* speedGroup = new QGroupBox("Speed");
    QHBoxLayout* speedLayout = new QHBoxLayout();
    m_speedSlider = new QSlider(Qt::Horizontal);
    m_speedSlider->setRange(0, 100);
    m_speedSlider->setValue(static_cast<int>(m_speed * 100));
    connect(m_speedSlider, &QSlider::valueChanged, this, &MotionControlWidget::onSpeedChanged);
    m_speedLabel = new QLabel(QString("%1%").arg(static_cast<int>(m_speed * 100)));
    m_speedLabel->setMinimumWidth(40);

    speedLayout->addWidget(m_speedSlider);
    speedLayout->addWidget(m_speedLabel);
    speedGroup->setLayout(speedLayout);

    // Curve radius slider
    QGroupBox* radiusGroup = new QGroupBox("Curve Radius (m)");
    QHBoxLayout* radiusLayout = new QHBoxLayout();
    m_radiusSlider = new QSlider(Qt::Horizontal);
    m_radiusSlider->setRange(1, 500); // 0.01m - 5.00m represented as 1..500 (centimeters)
    m_radiusSlider->setValue(100); // 1.00 m default
    connect(m_radiusSlider, &QSlider::valueChanged, this, [this](int v){
        m_radius = v / 100.0; // convert to meters
        m_radiusLabel->setText(QString("%1 m").arg(m_radius, 0, 'f', 2));
    });
    m_radiusLabel = new QLabel("1.00 m");
    m_radiusLabel->setMinimumWidth(60);
    radiusLayout->addWidget(m_radiusSlider);
    radiusLayout->addWidget(m_radiusLabel);
    radiusGroup->setLayout(radiusLayout);

    // Spin buttons
    QGroupBox* spinGroup = new QGroupBox("Spin");
    QHBoxLayout* spinLayout = new QHBoxLayout();
    m_spinLeftButton = new QPushButton("Spin Left");
    m_spinRightButton = new QPushButton("Spin Right");
    connect(m_spinLeftButton, &QPushButton::pressed, this, [this](){
        // spin left
        if (m_ros2Interface) m_ros2Interface->publishVelocityCommand(0.0, 0.0, m_maxAngular * m_speed);
    });
    connect(m_spinLeftButton, &QPushButton::released, this, [this](){
        if (m_ros2Interface) m_ros2Interface->publishVelocityCommand(0.0, 0.0, 0.0);
    });
    connect(m_spinRightButton, &QPushButton::pressed, this, [this](){
        if (m_ros2Interface) m_ros2Interface->publishVelocityCommand(0.0, 0.0, -m_maxAngular * m_speed);
    });
    connect(m_spinRightButton, &QPushButton::released, this, [this](){
        if (m_ros2Interface) m_ros2Interface->publishVelocityCommand(0.0, 0.0, 0.0);
    });
    spinLayout->addWidget(m_spinLeftButton);
    spinLayout->addWidget(m_spinRightButton);
    spinGroup->setLayout(spinLayout);

    // Pin and stop controls
    QHBoxLayout* pinStopLayout = new QHBoxLayout();
    m_pinRadioButton = new QRadioButton("Pin Command");
    m_stopButton = new QPushButton("STOP");
    m_stopButton->setStyleSheet("QPushButton { background-color: orange; color: white; font-weight: bold; padding: 8px; }");
    connect(m_stopButton, &QPushButton::clicked, this, [this](){
        // Clear pinned motion and send stop
        m_isPinned = false;
        m_pinRadioButton->setChecked(false);
        m_pinnedMotion = Motion::Forward; // default clear
        if (m_ros2Interface) m_ros2Interface->publishVelocityCommand(0.0, 0.0, 0.0);
    });
    pinStopLayout->addWidget(m_pinRadioButton);
    pinStopLayout->addWidget(m_stopButton);

    mainLayout->addLayout(pinStopLayout);

    mainLayout->addWidget(motionGroup);
    mainLayout->addWidget(speedGroup);
    mainLayout->addWidget(radiusGroup);
    mainLayout->addWidget(spinGroup);
    mainLayout->addStretch(1);
}

bool MotionControlWidget::initialize()
{
    Logger::instance().info("Motion control widget initialized");
    return true;
}

void MotionControlWidget::onSpeedChanged(int value)
{
    m_speed = value / 100.0;
    m_speedLabel->setText(QString("%1%").arg(value));
}

void MotionControlWidget::onMotionPressed()
{
    // Find which button sent the signal
    QObject* s = sender();
    if (!s) return;

    for (auto it = m_buttons.begin(); it != m_buttons.end(); ++it) {
        if (it.value() == s) {
            // If pinned mode is active, latch the motion until STOP
            if (m_pinRadioButton && m_pinRadioButton->isChecked()) {
                m_isPinned = true;
                m_pinnedMotion = it.key();
                sendVelocityFor(m_pinnedMotion);
            } else {
                sendVelocityFor(it.key());
            }
            return;
        }
    }
}

void MotionControlWidget::onMotionReleased()
{
    // Only stop on release if not pinned
    if (!m_isPinned) sendStop();
}

void MotionControlWidget::sendVelocityFor(Motion motion)
{
    // Differential-drive mapping: compute linear.x and angular.z
    double linear = 0.0;
    double angular = 0.0;

    switch (motion) {
        case Motion::Forward:
            linear =  m_speed * m_maxLinear;
            angular = 0.0;
            break;
        case Motion::Backward:
            linear = -m_speed * m_maxLinear;
            angular = 0.0;
            break;
        case Motion::Left:
            // Turn while moving using curve radius: omega = v / r
            linear =  m_speed * m_maxLinear;
            if (m_radius > 0.0) angular = linear / m_radius;
            else angular =  m_speed * m_maxAngular;
            break;
        case Motion::Right:
            linear =  m_speed * m_maxLinear;
            if (m_radius > 0.0) angular = -linear / m_radius;
            else angular = -m_speed * m_maxAngular;
            break;
        case Motion::FwdLeft:
            linear =  m_speed * m_maxLinear * 0.85;
            if (m_radius > 0.0) angular = linear / m_radius;
            else angular =  m_speed * m_maxAngular * 0.6;
            break;
        case Motion::FwdRight:
            linear =  m_speed * m_maxLinear * 0.85;
            if (m_radius > 0.0) angular = -linear / m_radius;
            else angular = -m_speed * m_maxAngular * 0.6;
            break;
        case Motion::BackLeft:
            linear = -m_speed * m_maxLinear * 0.85;
            if (m_radius > 0.0) angular = linear / m_radius;
            else angular =  m_speed * m_maxAngular * 0.6;
            break;
        case Motion::BackRight:
            linear = -m_speed * m_maxLinear * 0.85;
            if (m_radius > 0.0) angular = -linear / m_radius;
            else angular = -m_speed * m_maxAngular * 0.6;
            break;
        case Motion::SpinLeft:
            linear = 0.0;
            angular = m_maxAngular * m_speed;
            break;
        case Motion::SpinRight:
            linear = 0.0;
            angular = -m_maxAngular * m_speed;
            break;
    }

    if (m_ros2Interface) {
        // publish linear.x and angular.z only (differential drive)
        m_ros2Interface->publishVelocityCommand(linear, 0.0, angular);
        Logger::instance().debug(QString("Motion command: linear=%1 angular=%2").arg(linear).arg(angular));
    }
}

void MotionControlWidget::sendStop()
{
    if (m_ros2Interface) {
        m_ros2Interface->publishVelocityCommand(0.0, 0.0, 0.0);
    }
}
