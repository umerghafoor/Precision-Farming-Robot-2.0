#ifndef SENSORDATAWIDGET_H
#define SENSORDATAWIDGET_H

#include "BaseWidget.h"
#include <QTableWidget>
#include <QTimer>

/**
 * @brief Widget for displaying real-time sensor data
 */
class SensorDataWidget : public BaseWidget
{
    Q_OBJECT

public:
    explicit SensorDataWidget(QWidget *parent = nullptr);
    ~SensorDataWidget() override;

    bool initialize() override;
    QString displayName() const override { return "Sensor Data"; }

private slots:
    void onIMUDataReceived(double ax, double ay, double az, double gx, double gy, double gz);
    void onRobotStatusReceived(const QString& status);
    void updateDisplay();

private:
    void setupUI();
    void updateSensorRow(const QString& sensor, const QString& value);

    QTableWidget* m_sensorTable;
    QTimer* m_updateTimer;

    struct IMUData {
        double ax, ay, az;
        double gx, gy, gz;
    } m_imuData;

    QString m_robotStatus;
};

#endif // SENSORDATAWIDGET_H
