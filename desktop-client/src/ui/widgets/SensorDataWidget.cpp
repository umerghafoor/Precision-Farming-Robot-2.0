#include "SensorDataWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHeaderView>

SensorDataWidget::SensorDataWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_robotStatus("Unknown")
{
    m_imuData = {0, 0, 0, 0, 0, 0};
    setupUI();
}

SensorDataWidget::~SensorDataWidget()
{
}

void SensorDataWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    m_sensorTable = new QTableWidget();
    m_sensorTable->setColumnCount(2);
    m_sensorTable->setHorizontalHeaderLabels({"Sensor", "Value"});
    m_sensorTable->horizontalHeader()->setStretchLastSection(true);
    m_sensorTable->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // Initialize sensor rows
    QStringList sensors = {
        "Robot Status",
        "Accelerometer X",
        "Accelerometer Y",
        "Accelerometer Z",
        "Gyroscope X",
        "Gyroscope Y",
        "Gyroscope Z",
        "Battery Level",
        "GPS Latitude",
        "GPS Longitude"
    };

    m_sensorTable->setRowCount(sensors.size());
    for (int i = 0; i < sensors.size(); ++i) {
        m_sensorTable->setItem(i, 0, new QTableWidgetItem(sensors[i]));
        m_sensorTable->setItem(i, 1, new QTableWidgetItem("--"));
    }

    mainLayout->addWidget(m_sensorTable);

    // Update timer
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &SensorDataWidget::updateDisplay);
    m_updateTimer->start(100); // 10 Hz update rate
}

bool SensorDataWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::imuDataReceived,
                this, &SensorDataWidget::onIMUDataReceived);
        connect(m_ros2Interface, &ROS2Interface::robotStatusReceived,
                this, &SensorDataWidget::onRobotStatusReceived);
        Logger::instance().info("Sensor data widget initialized");
        return true;
    }
    
    Logger::instance().warning("Sensor data widget initialized without ROS2");
    return false;
}

void SensorDataWidget::onIMUDataReceived(double ax, double ay, double az, double gx, double gy, double gz)
{
    m_imuData.ax = ax;
    m_imuData.ay = ay;
    m_imuData.az = az;
    m_imuData.gx = gx;
    m_imuData.gy = gy;
    m_imuData.gz = gz;
}

void SensorDataWidget::onRobotStatusReceived(const QString& status)
{
    m_robotStatus = status;
}

void SensorDataWidget::updateDisplay()
{
    updateSensorRow("Robot Status", m_robotStatus);
    updateSensorRow("Accelerometer X", QString::number(m_imuData.ax, 'f', 3) + " m/s²");
    updateSensorRow("Accelerometer Y", QString::number(m_imuData.ay, 'f', 3) + " m/s²");
    updateSensorRow("Accelerometer Z", QString::number(m_imuData.az, 'f', 3) + " m/s²");
    updateSensorRow("Gyroscope X", QString::number(m_imuData.gx, 'f', 3) + " rad/s");
    updateSensorRow("Gyroscope Y", QString::number(m_imuData.gy, 'f', 3) + " rad/s");
    updateSensorRow("Gyroscope Z", QString::number(m_imuData.gz, 'f', 3) + " rad/s");
}

void SensorDataWidget::updateSensorRow(const QString& sensor, const QString& value)
{
    for (int i = 0; i < m_sensorTable->rowCount(); ++i) {
        if (m_sensorTable->item(i, 0)->text() == sensor) {
            m_sensorTable->item(i, 1)->setText(value);
            break;
        }
    }
}
