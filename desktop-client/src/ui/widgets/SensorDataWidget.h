#ifndef SENSORDATAWIDGET_H
#define SENSORDATAWIDGET_H

#include "BaseWidget.h"
#include <QTimer>
#include <QFrame>
#include <QLabel>
#include <QProgressBar>
#include <QVector>

/**
 * @brief Widget for displaying real-time sensor telemetry as a fixed bar
 *
 * The previous implementation used a scrolling table. For the UI overhaul
 * this has been replaced with a horizontal row of six metric cells; each
 * cell shows a name, large value, unit and a 4px mini progress bar. The
 * widget is expected to live in a bottom-fixed dock with a constant height.
 */
class SensorDataWidget : public BaseWidget
{
    Q_OBJECT

public:
    struct TelemetryCell {
        QFrame* frame;
        QLabel* nameLabel;
        QLabel* valueLabel;
        QLabel* unitLabel;
        QProgressBar* miniBar;
        double minValue;
        double maxValue;
    };

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

    QVector<TelemetryCell> m_cells;
    QTimer* m_updateTimer;

    struct IMUData {
        double ax, ay, az;
        double gx, gy, gz;
    } m_imuData;

    QString m_robotStatus;
};

#endif // SENSORDATAWIDGET_H
