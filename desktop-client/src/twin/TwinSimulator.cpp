#include "TwinSimulator.h"
#include "TwinState.h"
#include "Logger.h"
#include <QDateTime>
#include <QRandomGenerator>
#include <QtMath>
#include <cmath>

TwinSimulator::TwinSimulator(TwinState* state, QObject *parent)
    : QObject(parent)
    , m_state(state)
    , m_running(false)
    , m_updateRate(50) // 50 Hz default
    , m_simulationTime(0.0)
{
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &TwinSimulator::update);
}

TwinSimulator::~TwinSimulator()
{
    stop();
}

void TwinSimulator::start()
{
    if (m_running) return;

    m_running = true;
    m_lastUpdateTime = QDateTime::currentMSecsSinceEpoch();
    m_simulationTime = 0.0;
    
    int interval = 1000 / m_updateRate;
    m_updateTimer->start(interval);
    
    Logger::instance().info(QString("Twin simulator started at %1 Hz").arg(m_updateRate));
}

void TwinSimulator::stop()
{
    if (!m_running) return;

    m_running = false;
    m_updateTimer->stop();
    
    Logger::instance().info("Twin simulator stopped");
}

void TwinSimulator::setUpdateRate(int hz)
{
    if (hz <= 0 || hz > 1000) {
        Logger::instance().warning("Invalid update rate");
        return;
    }

    m_updateRate = hz;
    
    if (m_running) {
        int interval = 1000 / m_updateRate;
        m_updateTimer->setInterval(interval);
    }
}

void TwinSimulator::update()
{
    if (!m_state) return;

    // Calculate delta time
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    double dt = (currentTime - m_lastUpdateTime) / 1000.0;
    m_lastUpdateTime = currentTime;
    m_simulationTime += dt;

    // Run simulation steps
    simulatePhysics(dt);
    simulateSensors();

    emit stateUpdated();
}

void TwinSimulator::simulatePhysics(double dt)
{
    const TwinState::Velocity& vel = m_state->velocity();
    TwinState::Pose pose = m_state->pose();

    // Extract current yaw from orientation quaternion
    float qw = pose.orientation.scalar();
    float qx = pose.orientation.x();
    float qy = pose.orientation.y();
    float qz = pose.orientation.z();
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz));

    // Rotate linear velocity from robot frame into world frame
    double lx = vel.linear.x() * std::cos(yaw) - vel.linear.y() * std::sin(yaw);
    double ly = vel.linear.x() * std::sin(yaw) + vel.linear.y() * std::cos(yaw);
    pose.position += QVector3D(lx * dt, ly * dt, 0.0);

    // Integrate angular velocity (yaw rate around Z)
    double dAngle = vel.angular.z() * dt;
    QQuaternion rotation = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), qRadiansToDegrees(dAngle));
    pose.orientation = rotation * pose.orientation;
    pose.orientation.normalize();

    m_state->setPose(pose);
}

void TwinSimulator::simulateSensors()
{
    // Simulate sensor data
    // Add some noise and variation for realism
    
    double noise = (QRandomGenerator::global()->bounded(100) - 50) / 1000.0; // -0.05 to 0.05
    
    // Simulate IMU with gravity and small variations
    m_state->updateIMU(
        noise, noise, 9.81 + noise,  // Accelerometer (with gravity)
        noise, noise, noise           // Gyroscope
    );
    
    // Simulate battery drain (very slow)
    double currentBattery = m_state->batteryLevel();
    if (currentBattery > 0) {
        m_state->setBatteryLevel(currentBattery - 0.0001);
    }
}
