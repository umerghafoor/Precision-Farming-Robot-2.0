#include "DigitalTwin.h"
#include "TwinState.h"
#include "TwinSimulator.h"
#include "ROS2Interface.h"
#include "Logger.h"

DigitalTwin::DigitalTwin(QObject *parent)
    : QObject(parent)
    , m_ros2Interface(nullptr)
    , m_mode(Mode::Offline)
{
}

DigitalTwin::~DigitalTwin()
{
    stopSimulation();
}

bool DigitalTwin::initialize()
{
    try {
        // Create twin state
        m_state = std::make_unique<TwinState>();
        
        // Create simulator
        m_simulator = std::make_unique<TwinSimulator>(m_state.get());
        
        // Connect simulator signals
        connect(m_simulator.get(), &TwinSimulator::stateUpdated,
                this, &DigitalTwin::onSimulationUpdate);
        
        Logger::instance().info("Digital Twin initialized");
        return true;
    }
    catch (const std::exception& e) {
        Logger::instance().error(QString("Digital Twin initialization failed: %1").arg(e.what()));
        return false;
    }
}

void DigitalTwin::connectToROS2(ROS2Interface* ros2)
{
    if (!ros2) {
        Logger::instance().warning("Cannot connect to null ROS2 interface");
        return;
    }

    m_ros2Interface = ros2;
    setupConnections(ros2);
    setMode(Mode::Synchronized);
    
    Logger::instance().info("Digital Twin connected to ROS2");
}

void DigitalTwin::setupConnections(ROS2Interface* ros2)
{
    connect(ros2, &ROS2Interface::imageReceived,
            this, &DigitalTwin::onROS2ImageReceived);
    connect(ros2, &ROS2Interface::imuDataReceived,
            this, &DigitalTwin::onROS2IMUReceived);
    connect(ros2, &ROS2Interface::robotStatusReceived,
            this, &DigitalTwin::onROS2StatusReceived);
}

void DigitalTwin::setMode(Mode mode)
{
    if (m_mode == mode) return;
    
    m_mode = mode;
    emit modeChanged(mode);
    
    QString modeStr;
    switch (mode) {
        case Mode::Synchronized: modeStr = "Synchronized"; break;
        case Mode::Simulated: modeStr = "Simulated"; break;
        case Mode::Offline: modeStr = "Offline"; break;
    }
    
    Logger::instance().info(QString("Digital Twin mode changed to: %1").arg(modeStr));
}

void DigitalTwin::startSimulation()
{
    if (m_simulator) {
        m_simulator->start();
        setMode(Mode::Simulated);
        emit simulationStarted();
        Logger::instance().info("Digital Twin simulation started");
    }
}

void DigitalTwin::stopSimulation()
{
    if (m_simulator) {
        m_simulator->stop();
        emit simulationStopped();
        Logger::instance().info("Digital Twin simulation stopped");
    }
}

void DigitalTwin::onROS2ImageReceived(const QByteArray& imageData, int width, int height)
{
    // Update twin state with image data
    // This can be used for vision processing in the twin
    Logger::instance().debug(QString("Received image: %1x%2").arg(width).arg(height));
}

void DigitalTwin::onROS2IMUReceived(double ax, double ay, double az, double gx, double gy, double gz)
{
    if (m_state && m_mode == Mode::Synchronized) {
        m_state->updateIMU(ax, ay, az, gx, gy, gz);
    }
}

void DigitalTwin::onROS2StatusReceived(const QString& status)
{
    if (m_state) {
        m_state->setRobotStatus(status);
    }
}

void DigitalTwin::onSimulationUpdate()
{
    emit stateChanged();
}
