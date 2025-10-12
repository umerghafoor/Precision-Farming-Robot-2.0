#ifndef TWINSTATE_H
#define TWINSTATE_H

#include <QObject>
#include <QVector3D>
#include <QQuaternion>
#include <QVariantMap>

/**
 * @brief Represents the state of the digital twin
 * 
 * This class encapsulates all state information of the robot
 * in the digital twin
 */
class TwinState : public QObject
{
    Q_OBJECT

public:
    struct Pose {
        QVector3D position;
        QQuaternion orientation;
    };

    struct Velocity {
        QVector3D linear;
        QVector3D angular;
    };

    struct SensorData {
        QVariantMap imu;
        QVariantMap gps;
        QVariantMap camera;
        QVariantMap custom;
    };

    explicit TwinState(QObject *parent = nullptr);

    // Getters
    const Pose& pose() const { return m_pose; }
    const Velocity& velocity() const { return m_velocity; }
    const SensorData& sensorData() const { return m_sensorData; }
    double batteryLevel() const { return m_batteryLevel; }
    QString robotStatus() const { return m_robotStatus; }

    // Setters
    void setPose(const Pose& pose);
    void setVelocity(const Velocity& velocity);
    void setSensorData(const SensorData& data);
    void setBatteryLevel(double level);
    void setRobotStatus(const QString& status);

    // Update methods
    void updateIMU(double ax, double ay, double az, double gx, double gy, double gz);
    void updatePosition(const QVector3D& position);
    void updateOrientation(const QQuaternion& orientation);

    // Serialization
    QVariantMap toVariantMap() const;
    void fromVariantMap(const QVariantMap& map);

signals:
    void stateChanged();
    void poseChanged(const Pose& pose);
    void velocityChanged(const Velocity& velocity);
    void sensorDataChanged();
    void batteryLevelChanged(double level);
    void statusChanged(const QString& status);

private:
    Pose m_pose;
    Velocity m_velocity;
    SensorData m_sensorData;
    double m_batteryLevel;
    QString m_robotStatus;
};

#endif // TWINSTATE_H
