#include "TwinVisualizationWidget.h"
#include "DigitalTwin.h"
#include "TwinState.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

TwinVisualizationWidget::TwinVisualizationWidget(QWidget *parent)
    : BaseWidget(parent)
{
    setupUI();
}

TwinVisualizationWidget::~TwinVisualizationWidget()
{
}

void TwinVisualizationWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Mode control
    QGroupBox* controlGroup = new QGroupBox("Digital Twin Control");
    QHBoxLayout* controlLayout = new QHBoxLayout();

    m_modeLabel = new QLabel("Mode: Offline");
    m_modeLabel->setStyleSheet("QLabel { font-weight: bold; }");

    m_modeSelector = new QComboBox();
    m_modeSelector->addItem("Synchronized");
    m_modeSelector->addItem("Simulated");
    m_modeSelector->addItem("Offline");
    connect(m_modeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TwinVisualizationWidget::onToggleMode);

    m_resetButton = new QPushButton("Reset Twin");
    connect(m_resetButton, &QPushButton::clicked,
            this, &TwinVisualizationWidget::onResetTwin);

    controlLayout->addWidget(m_modeLabel);
    controlLayout->addWidget(m_modeSelector);
    controlLayout->addWidget(m_resetButton);
    controlLayout->addStretch();
    controlGroup->setLayout(controlLayout);

    // State display
    QGroupBox* stateGroup = new QGroupBox("Twin State");
    QVBoxLayout* stateLayout = new QVBoxLayout();

    m_stateDisplay = new QTextEdit();
    m_stateDisplay->setReadOnly(true);
    m_stateDisplay->setMinimumHeight(300);
    m_stateDisplay->setStyleSheet("QTextEdit { font-family: monospace; }");
    m_stateDisplay->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    stateLayout->addWidget(m_stateDisplay);
    stateGroup->setLayout(stateLayout);

    mainLayout->addWidget(controlGroup);
    mainLayout->addWidget(stateGroup, 1); // Add stretch factor
}

bool TwinVisualizationWidget::initialize()
{
    if (m_digitalTwin) {
        connect(m_digitalTwin, &DigitalTwin::stateChanged,
                this, &TwinVisualizationWidget::onTwinStateChanged);
        
        updateTwinDisplay();
        Logger::instance().info("Twin visualization widget initialized");
        return true;
    }
    
    Logger::instance().warning("Twin visualization widget initialized without Digital Twin");
    return false;
}

void TwinVisualizationWidget::onTwinStateChanged()
{
    updateTwinDisplay();
}

void TwinVisualizationWidget::updateTwinDisplay()
{
    if (!m_digitalTwin || !m_digitalTwin->state()) {
        m_stateDisplay->setText("Digital Twin not available");
        return;
    }

    TwinState* state = m_digitalTwin->state();
    const TwinState::Pose& pose = state->pose();
    const TwinState::Velocity& velocity = state->velocity();

    QString displayText;
    displayText += "=== ROBOT STATE ===\n\n";
    
    displayText += "Position:\n";
    displayText += QString("  X: %1 m\n").arg(pose.position.x(), 0, 'f', 3);
    displayText += QString("  Y: %1 m\n").arg(pose.position.y(), 0, 'f', 3);
    displayText += QString("  Z: %1 m\n\n").arg(pose.position.z(), 0, 'f', 3);
    
    displayText += "Velocity:\n";
    displayText += QString("  Linear X: %1 m/s\n").arg(velocity.linear.x(), 0, 'f', 3);
    displayText += QString("  Linear Y: %1 m/s\n").arg(velocity.linear.y(), 0, 'f', 3);
    displayText += QString("  Angular Z: %1 rad/s\n\n").arg(velocity.angular.z(), 0, 'f', 3);
    
    displayText += QString("Battery: %1%\n").arg(state->batteryLevel(), 0, 'f', 1);
    displayText += QString("Status: %1\n").arg(state->robotStatus());

    // IMU Data
    const QVariantMap& imu = state->sensorData().imu;
    if (!imu.isEmpty()) {
        displayText += "\n=== IMU DATA ===\n";
        displayText += QString("Accel: (%1, %2, %3) m/s²\n")
            .arg(imu.value("accel_x", 0.0).toDouble(), 0, 'f', 3)
            .arg(imu.value("accel_y", 0.0).toDouble(), 0, 'f', 3)
            .arg(imu.value("accel_z", 0.0).toDouble(), 0, 'f', 3);
        displayText += QString("Gyro: (%1, %2, %3) rad/s\n")
            .arg(imu.value("gyro_x", 0.0).toDouble(), 0, 'f', 3)
            .arg(imu.value("gyro_y", 0.0).toDouble(), 0, 'f', 3)
            .arg(imu.value("gyro_z", 0.0).toDouble(), 0, 'f', 3);
    }

    m_stateDisplay->setText(displayText);

    // Update mode label
    QString modeStr;
    switch (m_digitalTwin->mode()) {
        case DigitalTwin::Mode::Synchronized: modeStr = "Synchronized"; break;
        case DigitalTwin::Mode::Simulated: modeStr = "Simulated"; break;
        case DigitalTwin::Mode::Offline: modeStr = "Offline"; break;
    }
    m_modeLabel->setText("Mode: " + modeStr);
}

void TwinVisualizationWidget::onToggleMode()
{
    if (!m_digitalTwin) return;

    int index = m_modeSelector->currentIndex();
    
    switch (index) {
        case 0: // Synchronized
            m_digitalTwin->setMode(DigitalTwin::Mode::Synchronized);
            m_digitalTwin->stopSimulation();
            break;
        case 1: // Simulated
            m_digitalTwin->startSimulation();
            break;
        case 2: // Offline
            m_digitalTwin->setMode(DigitalTwin::Mode::Offline);
            m_digitalTwin->stopSimulation();
            break;
    }
}

void TwinVisualizationWidget::onResetTwin()
{
    Logger::instance().info("Reset digital twin requested");
    // Implementation for resetting twin state
}
