#ifndef SENSORDATAWIDGET_H
#define SENSORDATAWIDGET_H

#include "BaseWidget.h"
#include <QTimer>
#include <QFrame>
#include <QLabel>
#include <QProgressBar>
#include <QVector>

class SensorDataWidget : public BaseWidget
{
    Q_OBJECT

public:
    struct TelemetryCell {
        QFrame*       frame;
        QLabel*       nameLabel;
        QLabel*       valueLabel;
        QLabel*       unitLabel;
        QProgressBar* miniBar;
        double        minValue;
        double        maxValue;
    };

    explicit SensorDataWidget(QWidget *parent = nullptr);
    ~SensorDataWidget() override;

    bool    initialize() override;
    QString displayName() const override { return "Sensor Data"; }

private slots:
    void onIMUDataReceived(double ax, double ay, double az,
                           double gx, double gy, double gz);
    void onRobotStatusReceived(const QString& status);
    void onCoordinatesReceived(double x, double y);
    void updateDisplay();

private:
    void setupUI();

    QVector<TelemetryCell> m_cells;
    QTimer* m_updateTimer;

    struct IMUData { double ax, ay, az, gx, gy, gz; } m_imuData;
    QString m_robotStatus;
    double  m_posX{0.0};
    double  m_posY{0.0};
};

#endif // SENSORDATAWIDGET_H
