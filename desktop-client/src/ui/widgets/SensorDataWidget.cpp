#include "SensorDataWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QTimer>

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

static SensorDataWidget::TelemetryCell makeCell(const QString &name,
                                                const QString &unit,
                                                double minVal,
                                                double maxVal,
                                                QWidget *parent)
{
    SensorDataWidget::TelemetryCell c;
    c.frame = new QFrame(parent);
    c.frame->setFrameShape(QFrame::StyledPanel);
    c.frame->setObjectName("telemetryCell");
    c.frame->setMinimumWidth(110);
    QVBoxLayout *lay = new QVBoxLayout(c.frame);
    lay->setContentsMargins(14, 14, 14, 14);
    lay->setSpacing(4);

    lay->addStretch(1);

    c.nameLabel = new QLabel(name.toUpper(), c.frame);
    c.nameLabel->setObjectName("cellNameLabel");
    c.nameLabel->setAlignment(Qt::AlignCenter);
    c.nameLabel->setWordWrap(true);
    lay->addWidget(c.nameLabel);

    c.valueLabel = new QLabel("--", c.frame);
    c.valueLabel->setObjectName("cellValueLabel");
    c.valueLabel->setAlignment(Qt::AlignCenter);
    lay->addWidget(c.valueLabel);

    c.unitLabel = new QLabel(unit, c.frame);
    c.unitLabel->setObjectName("cellUnitLabel");
    c.unitLabel->setAlignment(Qt::AlignCenter);
    lay->addWidget(c.unitLabel);

    lay->addSpacing(8);

    c.miniBar = new QProgressBar(c.frame);
    c.miniBar->setTextVisible(false);
    c.miniBar->setFixedHeight(4);
    c.miniBar->setRange(0, 100);
    c.miniBar->setValue(0);
    lay->addWidget(c.miniBar);

    lay->addStretch(1);

    c.minValue = minVal;
    c.maxValue = maxVal;
    return c;
}

void SensorDataWidget::setupUI()
{
    // 2-row × 3-column grid
    QGridLayout* mainLayout = new QGridLayout(this);
    mainLayout->setContentsMargins(12, 12, 12, 12);
    mainLayout->setSpacing(10);

    // create cells in specification order
    m_cells.append(makeCell("Accel X", "m/s²", -10.0, 10.0, this));
    m_cells.append(makeCell("Accel Y", "m/s²", -10.0, 10.0, this));
    m_cells.append(makeCell("Accel Z", "m/s²", -10.0, 10.0, this));
    m_cells.append(makeCell("Gyro X", "rad/s", -5.0, 5.0, this));
    m_cells.append(makeCell("Gyro Y", "rad/s", -5.0, 5.0, this));
    // Robot status cell has no mini-bar
    SensorDataWidget::TelemetryCell statusCell = makeCell("Robot Status", "", 0.0, 1.0, this);
    statusCell.miniBar->hide();
    m_cells.append(statusCell);

    // Row 0: Accel X, Accel Y, Accel Z
    // Row 1: Gyro X,  Gyro Y,  Robot Status
    for (int i = 0; i < m_cells.size(); ++i) {
        mainLayout->addWidget(m_cells[i].frame, i / 3, i % 3);
        mainLayout->setColumnStretch(i % 3, 1);
    }
    mainLayout->setRowStretch(0, 1);
    mainLayout->setRowStretch(1, 1);

    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &SensorDataWidget::updateDisplay);
    m_updateTimer->start(100);

    // automated exercise for telemetry bar test mode
    if (qgetenv("TELEMETRY_WIDGET_TEST") == "1") {
        QTimer::singleShot(500, this, [this]() {
            onIMUDataReceived(1.2, -3.4, 5.6, 0.5, -0.5, 0.0);
            onRobotStatusReceived("Online");
        });
        QTimer::singleShot(1500, this, [this]() {
            onIMUDataReceived(-9.8, 0.0, 2.2, -1.0, 1.1, 0.0);
            onRobotStatusReceived("Busy");
        });
    }
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
    if (m_cells.size() < 6) return;

    // Accel X/Y/Z
    m_cells[0].valueLabel->setText(QString::number(m_imuData.ax, 'f', 3));
    m_cells[1].valueLabel->setText(QString::number(m_imuData.ay, 'f', 3));
    m_cells[2].valueLabel->setText(QString::number(m_imuData.az, 'f', 3));
    // Gyro X/Y
    m_cells[3].valueLabel->setText(QString::number(m_imuData.gx, 'f', 3));
    m_cells[4].valueLabel->setText(QString::number(m_imuData.gy, 'f', 3));
    // Robot status
    m_cells[5].valueLabel->setText(m_robotStatus);

    // update mini-bars (map value to 0..100)
    auto scale = [](double val, double minv, double maxv) -> int {
        if (val <= minv) return 0;
        if (val >= maxv) return 100;
        return int((val - minv)/(maxv - minv)*100.0);
    };

    m_cells[0].miniBar->setValue(scale(m_imuData.ax, m_cells[0].minValue, m_cells[0].maxValue));
    m_cells[1].miniBar->setValue(scale(m_imuData.ay, m_cells[1].minValue, m_cells[1].maxValue));
    m_cells[2].miniBar->setValue(scale(m_imuData.az, m_cells[2].minValue, m_cells[2].maxValue));
    m_cells[3].miniBar->setValue(scale(m_imuData.gx, m_cells[3].minValue, m_cells[3].maxValue));
    m_cells[4].miniBar->setValue(scale(m_imuData.gy, m_cells[4].minValue, m_cells[4].maxValue));
    // status cell has no bar

    if (qgetenv("TELEMETRY_WIDGET_TEST") == "1") {
        Logger::instance().debug(QString("Telemetry updated ax=%1 ay=%2 az=%3 gx=%4 gy=%5 status=%6")
                                 .arg(m_imuData.ax)
                                 .arg(m_imuData.ay)
                                 .arg(m_imuData.az)
                                 .arg(m_imuData.gx)
                                 .arg(m_imuData.gy)
                                 .arg(m_robotStatus));
    }
}
