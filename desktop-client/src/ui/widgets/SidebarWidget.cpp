#include "SidebarWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSpacerItem>
#include <QTimer>

SidebarWidget::SidebarWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_speed(0.5)
    , m_maxLinear(1.0)
    , m_maxAngular(1.5)
    , m_radius(1.0)
    , m_isPinned(false)
    , m_linearSpeed(0.0)
    , m_angularSpeed(0.0)
{
    setupUI();
}

SidebarWidget::~SidebarWidget()
{
}

bool SidebarWidget::initialize()
{
    Logger::instance().info("Sidebar widget initialized");

    // initial connection gating - assume offline until signalled otherwise
    setMotionEnabled(false);

    // watch for ROS2 connection state if interface already assigned
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::connected, this, [this]() { setMotionEnabled(true); });
        connect(m_ros2Interface, &ROS2Interface::disconnected, this, [this]() { setMotionEnabled(false); });
    }

    return true;
}

void SidebarWidget::setupUI()
{
    // main layout holds a scrollarea and a fixed bottom control bar
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0,0,0,0);
    mainLayout->setSpacing(0);

    m_scrollArea = new QScrollArea(this);
    m_scrollArea->setWidgetResizable(true);
    m_scrollArea->setFrameShape(QFrame::NoFrame);

    m_scrollContent = new QWidget();
    QVBoxLayout* contentLayout = new QVBoxLayout(m_scrollContent);
    contentLayout->setContentsMargins(8,8,8,8);
    contentLayout->setSpacing(10);

    // --- Motion section ---
    QGroupBox* motionGroup = new QGroupBox("Motion");
    QVBoxLayout* motionLayout = new QVBoxLayout();

    // directional pad grid
    QGridLayout* grid = new QGridLayout();
    auto makeButton = [&](Motion m, const QString& text){
        QPushButton* b = new QPushButton(text);
        b->setAutoRepeat(true);
        b->setAutoRepeatDelay(300);
        b->setAutoRepeatInterval(100);
        connect(b, &QPushButton::pressed, this, &SidebarWidget::onMotionPressed);
        connect(b, &QPushButton::released, this, &SidebarWidget::onMotionReleased);
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

    motionLayout->addLayout(grid);

    // speed slider
    QHBoxLayout* speedLayout = new QHBoxLayout();
    m_speedSlider = new QSlider(Qt::Horizontal);
    m_speedSlider->setRange(0, 100);
    m_speedSlider->setValue(static_cast<int>(m_speed * 100));
    connect(m_speedSlider, &QSlider::valueChanged, this, &SidebarWidget::onSpeedChanged);
    m_speedLabel = new QLabel(QString("%1%").arg(static_cast<int>(m_speed * 100)));
    m_speedLabel->setMinimumWidth(40);
    speedLayout->addWidget(m_speedSlider);
    speedLayout->addWidget(m_speedLabel);
    motionLayout->addLayout(speedLayout);

    // radius slider (0.00–3.00 m in 1cm steps)
    QHBoxLayout* radiusLayout = new QHBoxLayout();
    m_radiusSlider = new QSlider(Qt::Horizontal);
    m_radiusSlider->setRange(0, 300); // centimetres
    m_radiusSlider->setValue(100); // 1.00 m default
    connect(m_radiusSlider, &QSlider::valueChanged, this, [this](int v){
        m_radius = v / 100.0;
        m_radiusLabel->setText(QString("%1 m").arg(m_radius, 0, 'f', 2));
    });
    m_radiusLabel = new QLabel("1.00 m");
    m_radiusLabel->setMinimumWidth(60);
    radiusLayout->addWidget(m_radiusSlider);
    radiusLayout->addWidget(m_radiusLabel);
    motionLayout->addLayout(radiusLayout);

    // spin buttons
    QHBoxLayout* spinLayout = new QHBoxLayout();
    m_spinLeftButton = new QPushButton("Spin Left");
    m_spinRightButton = new QPushButton("Spin Right");
    connect(m_spinLeftButton, &QPushButton::pressed, this, [this](){
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
    motionLayout->addLayout(spinLayout);

    // pin radio (keep for now)
    QHBoxLayout* pinLayout = new QHBoxLayout();
    m_pinRadioButton = new QRadioButton("Pin Command");
    pinLayout->addWidget(m_pinRadioButton);
    motionLayout->addLayout(pinLayout);

    motionGroup->setLayout(motionLayout);
    contentLayout->addWidget(motionGroup);

    // --- Legacy velocity sliders from old CommandControlWidget ---
    QGroupBox* velocityGroup = new QGroupBox("Velocity Control");
    QGridLayout* velocityLayout = new QGridLayout();

    velocityLayout->addWidget(new QLabel("Linear Speed:"), 0, 0);
    m_linearSpeedSlider = new QSlider(Qt::Horizontal);
    m_linearSpeedSlider->setRange(-100, 100);
    m_linearSpeedSlider->setValue(0);
    connect(m_linearSpeedSlider, &QSlider::valueChanged,
            this, &SidebarWidget::onLinearSpeedChanged);
    velocityLayout->addWidget(m_linearSpeedSlider, 0, 1);
    m_linearSpeedLabel = new QLabel("0.0 m/s");
    m_linearSpeedLabel->setMinimumWidth(80);
    velocityLayout->addWidget(m_linearSpeedLabel, 0, 2);

    velocityLayout->addWidget(new QLabel("Angular Speed:"), 1, 0);
    m_angularSpeedSlider = new QSlider(Qt::Horizontal);
    m_angularSpeedSlider->setRange(-100, 100);
    m_angularSpeedSlider->setValue(0);
    connect(m_angularSpeedSlider, &QSlider::valueChanged,
            this, &SidebarWidget::onAngularSpeedChanged);
    velocityLayout->addWidget(m_angularSpeedSlider, 1, 1);
    m_angularSpeedLabel = new QLabel("0.0 rad/s");
    m_angularSpeedLabel->setMinimumWidth(80);
    velocityLayout->addWidget(m_angularSpeedLabel, 1, 2);

    velocityGroup->setLayout(velocityLayout);
    contentLayout->addWidget(velocityGroup);

    contentLayout->addStretch(1);

    m_scrollArea->setWidget(m_scrollContent);
    mainLayout->addWidget(m_scrollArea);

    // bottom buttons
    QWidget* bottomBar = new QWidget(this);
    QHBoxLayout* bottomLayout = new QHBoxLayout(bottomBar);
    bottomLayout->setContentsMargins(8,4,8,4);

    m_stopButton = new QPushButton("STOP");
    m_stopButton->setObjectName("stopButton");
    Q_ASSERT(m_stopButton->objectName() == "stopButton");
    connect(m_stopButton, &QPushButton::clicked, this, &SidebarWidget::onStopClicked);
    m_emergencyStopButton = new QPushButton("EMERGENCY STOP");
    m_emergencyStopButton->setObjectName("emergencyStopButton");
    Q_ASSERT(m_emergencyStopButton->objectName() == "emergencyStopButton");
    connect(m_emergencyStopButton, &QPushButton::clicked, this, &SidebarWidget::onEmergencyStopClicked);

    bottomLayout->addWidget(m_stopButton);
    bottomLayout->addWidget(m_emergencyStopButton);

    mainLayout->addWidget(bottomBar);

    // automated exercise for sidebar test mode (motion control + sliders)
    if (qgetenv("SIDEBAR_WIDGET_TEST") == "1") {
        // automated sequence to exercise motion controls and stop buttons
        QTimer::singleShot(500, this, [this]() { m_speedSlider->setValue(80); });
        QTimer::singleShot(1000, this, [this]() { m_radiusSlider->setValue(200); });
        QTimer::singleShot(1500, this, [this]() {
            if (m_buttons.contains(Motion::Forward)) m_buttons[Motion::Forward]->animateClick();
        });
        QTimer::singleShot(2000, this, [this]() {
            if (m_buttons.contains(Motion::BackRight)) m_buttons[Motion::BackRight]->animateClick();
        });
        QTimer::singleShot(2500, this, [this]() {
            if (m_buttons.contains(Motion::SpinLeft)) m_buttons[Motion::SpinLeft]->animateClick();
        });
        QTimer::singleShot(3000, this, [this]() { m_stopButton->click(); });
        // also click emergency stop slightly later to verify its availability and styling
        QTimer::singleShot(3300, this, [this]() {
            Logger::instance().debug("SidebarWidget test: clicking emergency stop");
            m_emergencyStopButton->click();
        });
        // log palette colour so we can eyeball that it resolves to red
        QTimer::singleShot(100, this, [this]() {
            QPalette pal = m_emergencyStopButton->palette();
            Logger::instance().debug(QString("Emergency button palette button color: %1").arg(pal.color(QPalette::Button).name()));
        });
        // also simulate connection events via ROS2Interface to drive gating
        QTimer::singleShot(3500, this, [this]() {
            Logger::instance().debug("SidebarWidget test: simulating ROS2 start (should enable controls)");
            if (m_ros2Interface) m_ros2Interface->start();
        });
        QTimer::singleShot(4500, this, [this]() {
            Logger::instance().debug("SidebarWidget test: simulating ROS2 stop (should disable controls)");
            if (m_ros2Interface) m_ros2Interface->stop();
        });
        // finally exercise the gating helper directly
        QTimer::singleShot(6000, this, [this]() {
            Logger::instance().debug("SidebarWidget test: directly disabling motion controls");
            setMotionEnabled(false);
        });
        QTimer::singleShot(6500, this, [this]() {
            Logger::instance().debug("SidebarWidget test: directly re-enabling motion controls");
            setMotionEnabled(true);
        });    }
}


void SidebarWidget::onSpeedChanged(int value)
{
    m_speed = value / 100.0;
    m_speedLabel->setText(QString("%1%").arg(value));
}

void SidebarWidget::onMotionPressed()
{
    QObject* s = sender();
    if (!s) return;
    for (auto it = m_buttons.begin(); it != m_buttons.end(); ++it) {
        if (it.value() == s) {
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

void SidebarWidget::onMotionReleased()
{
    if (!m_isPinned) sendStop();
}

void SidebarWidget::sendVelocityFor(Motion motion)
{
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
        m_ros2Interface->publishVelocityCommand(linear, 0.0, angular);
        Logger::instance().debug(QString("Motion command: linear=%1 angular=%2").arg(linear).arg(angular));
    }
}

void SidebarWidget::sendStop()
{
    if (m_ros2Interface) {
        m_ros2Interface->publishVelocityCommand(0.0, 0.0, 0.0);
    }
}


void SidebarWidget::setMotionEnabled(bool enabled)
{
    // motion buttons
    for (auto btn : m_buttons.values()) {
        btn->setEnabled(enabled);
    }
    // speed and radius controls
    if (m_speedSlider) m_speedSlider->setEnabled(enabled);
    if (m_radiusSlider) m_radiusSlider->setEnabled(enabled);
    // spin controls and pin toggle
    if (m_spinLeftButton) m_spinLeftButton->setEnabled(enabled);
    if (m_spinRightButton) m_spinRightButton->setEnabled(enabled);
    if (m_pinRadioButton) m_pinRadioButton->setEnabled(enabled);
}

void SidebarWidget::onLinearSpeedChanged(int value)
{
    m_linearSpeed = value / 100.0;
    m_linearSpeedLabel->setText(QString("%1 m/s").arg(m_linearSpeed, 0, 'f', 2));
    sendVelocityCommand();
}

void SidebarWidget::onAngularSpeedChanged(int value)
{
    m_angularSpeed = value / 100.0 * 3.14159;
    m_angularSpeedLabel->setText(QString("%1 rad/s").arg(m_angularSpeed, 0, 'f', 2));
    sendVelocityCommand();
}

void SidebarWidget::sendVelocityCommand()
{
    if (m_ros2Interface) {
        m_ros2Interface->publishVelocityCommand(m_linearSpeed, 0.0, m_angularSpeed);
        Logger::instance().debug(QString("Velocity command: linear=%1, angular=%2")
                                .arg(m_linearSpeed).arg(m_angularSpeed));
    }
}

void SidebarWidget::onStopClicked()
{
    // reset sliders as well
    m_linearSpeedSlider->setValue(0);
    m_angularSpeedSlider->setValue(0);
    sendStop();
    Logger::instance().info("Robot stopped");
}

void SidebarWidget::onEmergencyStopClicked()
{
    onStopClicked();
    if (m_ros2Interface) {
        m_ros2Interface->publishRobotCommand("EMERGENCY_STOP");
    }
    Logger::instance().warning("EMERGENCY STOP activated");
}
