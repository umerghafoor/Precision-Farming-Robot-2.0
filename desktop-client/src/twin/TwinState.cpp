#include "TwinState.h"

TwinState::TwinState(QObject *parent)
    : QObject(parent)
    , m_batteryLevel(100.0)
    , m_robotStatus("Idle")
{
    m_pose.position = QVector3D(0, 0, 0);
    m_pose.orientation = QQuaternion();
    m_velocity.linear = QVector3D(0, 0, 0);
    m_velocity.angular = QVector3D(0, 0, 0);
}

void TwinState::setPose(const Pose& pose)
{
    m_pose = pose;
    emit poseChanged(pose);
    emit stateChanged();
}

void TwinState::setVelocity(const Velocity& velocity)
{
    m_velocity = velocity;
    emit velocityChanged(velocity);
    emit stateChanged();
}

void TwinState::setSensorData(const SensorData& data)
{
    m_sensorData = data;
    emit sensorDataChanged();
    emit stateChanged();
}

void TwinState::setBatteryLevel(double level)
{
    m_batteryLevel = level;
    emit batteryLevelChanged(level);
    emit stateChanged();
}

void TwinState::setRobotStatus(const QString& status)
{
    m_robotStatus = status;
    emit statusChanged(status);
    emit stateChanged();
}

void TwinState::updateIMU(double ax, double ay, double az, double gx, double gy, double gz)
{
    m_sensorData.imu["accel_x"] = ax;
    m_sensorData.imu["accel_y"] = ay;
    m_sensorData.imu["accel_z"] = az;
    m_sensorData.imu["gyro_x"] = gx;
    m_sensorData.imu["gyro_y"] = gy;
    m_sensorData.imu["gyro_z"] = gz;
    
    emit sensorDataChanged();
    emit stateChanged();
}

void TwinState::updatePosition(const QVector3D& position)
{
    m_pose.position = position;
    emit poseChanged(m_pose);
    emit stateChanged();
}

void TwinState::updateOrientation(const QQuaternion& orientation)
{
    m_pose.orientation = orientation;
    emit poseChanged(m_pose);
    emit stateChanged();
}

QVariantMap TwinState::toVariantMap() const
{
    QVariantMap map;
    
    map["position_x"] = m_pose.position.x();
    map["position_y"] = m_pose.position.y();
    map["position_z"] = m_pose.position.z();
    
    map["battery_level"] = m_batteryLevel;
    map["status"] = m_robotStatus;
    
    return map;
}

void TwinState::fromVariantMap(const QVariantMap& map)
{
    if (map.contains("position_x")) {
        m_pose.position.setX(map["position_x"].toDouble());
    }
    if (map.contains("position_y")) {
        m_pose.position.setY(map["position_y"].toDouble());
    }
    if (map.contains("position_z")) {
        m_pose.position.setZ(map["position_z"].toDouble());
    }
    
    if (map.contains("battery_level")) {
        setBatteryLevel(map["battery_level"].toDouble());
    }
    if (map.contains("status")) {
        setRobotStatus(map["status"].toString());
    }
    
    emit stateChanged();
}
