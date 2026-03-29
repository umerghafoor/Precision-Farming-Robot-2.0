#include "TwinVisualizationWidget.h"
#include "RobotMapView.h"
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
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(4);

    // ── Top control bar ──────────────────────────────────────────────────
    QHBoxLayout* controlLayout = new QHBoxLayout();

    m_modeLabel = new QLabel("Mode:");
    m_modeLabel->setStyleSheet("QLabel { font-weight: bold; }");

    m_modeSelector = new QComboBox();
    m_modeSelector->addItem("Synchronized");
    m_modeSelector->addItem("Simulated");
    m_modeSelector->addItem("Offline");
    connect(m_modeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TwinVisualizationWidget::onToggleMode);

    m_resetButton = new QPushButton("Reset");
    connect(m_resetButton, &QPushButton::clicked,
            this, &TwinVisualizationWidget::onResetTwin);

    controlLayout->addWidget(m_modeLabel);
    controlLayout->addWidget(m_modeSelector);
    controlLayout->addStretch();
    controlLayout->addWidget(m_resetButton);

    // ── 2D map ───────────────────────────────────────────────────────────
    m_mapView = new RobotMapView();

    // ── Coordinate readout ───────────────────────────────────────────────
    m_coordLabel = new QLabel("X: — m   Y: — m   Yaw: —°");
    m_coordLabel->setAlignment(Qt::AlignCenter);
    m_coordLabel->setStyleSheet("QLabel { font-family: monospace; font-size: 11px; }");

    mainLayout->addLayout(controlLayout);
    mainLayout->addWidget(m_mapView, 1);
    mainLayout->addWidget(m_coordLabel);
}

bool TwinVisualizationWidget::initialize()
{
    if (!m_digitalTwin) {
        Logger::instance().warning("Twin visualization widget initialized without Digital Twin");
        return false;
    }

    connect(m_digitalTwin, &DigitalTwin::stateChanged,
            this, &TwinVisualizationWidget::onTwinStateChanged);
    connect(m_digitalTwin, &DigitalTwin::modeChanged,
            this, [this](DigitalTwin::Mode) { syncModeCombo(); });

    syncModeCombo();
    Logger::instance().info("Twin visualization widget initialized");
    return true;
}

void TwinVisualizationWidget::onTwinStateChanged()
{
    if (!m_digitalTwin || !m_digitalTwin->state()) return;

    TwinState* state = m_digitalTwin->state();
    const TwinState::Pose& pose = state->pose();

    m_mapView->setRobotPose(pose.position.x(), pose.position.y(), pose.orientation);

    // Extract yaw for the label
    float qw = pose.orientation.scalar();
    float qx = pose.orientation.x();
    float qy = pose.orientation.y();
    float qz = pose.orientation.z();
    double yawRad = std::atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz));
    double yawDeg = qRadiansToDegrees(yawRad);

    m_coordLabel->setText(
        QString("X: %1 m   Y: %2 m   Yaw: %3°")
            .arg(pose.position.x(), 0, 'f', 3)
            .arg(pose.position.y(), 0, 'f', 3)
            .arg(yawDeg,            0, 'f', 1));
}

void TwinVisualizationWidget::syncModeCombo()
{
    if (!m_digitalTwin) return;

    // Block signals to avoid re-triggering onToggleMode
    const QSignalBlocker blocker(m_modeSelector);
    switch (m_digitalTwin->mode()) {
        case DigitalTwin::Mode::Synchronized: m_modeSelector->setCurrentIndex(0); break;
        case DigitalTwin::Mode::Simulated:    m_modeSelector->setCurrentIndex(1); break;
        case DigitalTwin::Mode::Offline:      m_modeSelector->setCurrentIndex(2); break;
    }
    m_modeLabel->setText("Mode: " + m_modeSelector->currentText());
}

void TwinVisualizationWidget::onToggleMode()
{
    if (!m_digitalTwin) return;

    switch (m_modeSelector->currentIndex()) {
        case 0:
            m_digitalTwin->stopSimulation();
            m_digitalTwin->setMode(DigitalTwin::Mode::Synchronized);
            break;
        case 1:
            m_digitalTwin->startSimulation();
            break;
        case 2:
            m_digitalTwin->stopSimulation();
            m_digitalTwin->setMode(DigitalTwin::Mode::Offline);
            break;
    }
    m_modeLabel->setText("Mode: " + m_modeSelector->currentText());
}

void TwinVisualizationWidget::onResetTwin()
{
    m_mapView->clear();
    Logger::instance().info("Digital twin map reset");
}
