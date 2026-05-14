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

SensorDataWidget::~SensorDataWidget() {}

static SensorDataWidget::TelemetryCell makeCell(
        const QString& name, const QString& unit,
        double minVal, double maxVal, QWidget* parent)
{
    SensorDataWidget::TelemetryCell c;
    c.frame = new QFrame(parent);
    c.frame->setFrameShape(QFrame::StyledPanel);
    c.frame->setObjectName("telemetryCell");
    c.frame->setMinimumWidth(100);

    QVBoxLayout* lay = new QVBoxLayout(c.frame);
    lay->setContentsMargins(12, 10, 12, 8);
    lay->setSpacing(3);
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

    lay->addSpacing(6);

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
    QGridLayout* mainLayout = new QGridLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(8);

    // Row 0: Accel X / Y / Z
    m_cells.append(makeCell("Accel X",  "m/s²",  -10.0,  10.0, this));
    m_cells.append(makeCell("Accel Y",  "m/s²",  -10.0,  10.0, this));
    m_cells.append(makeCell("Accel Z",  "m/s²",  -10.0,  10.0, this));

    // Row 1: Gyro X / Y / Robot Status
    m_cells.append(makeCell("Gyro X",   "rad/s",  -5.0,   5.0, this));
    m_cells.append(makeCell("Gyro Y",   "rad/s",  -5.0,   5.0, this));
    SensorDataWidget::TelemetryCell statusCell = makeCell("Robot Status", "", 0.0, 1.0, this);
    statusCell.miniBar->hide();
    m_cells.append(statusCell);

    // Row 2: Position X / Y  (merged from CoordinatesWidget)
    SensorDataWidget::TelemetryCell posXCell = makeCell("Pos X", "m", -50.0, 50.0, this);
    posXCell.frame->setObjectName("telemetryCellAlt");
    m_cells.append(posXCell);
    SensorDataWidget::TelemetryCell posYCell = makeCell("Pos Y", "m", -50.0, 50.0, this);
    posYCell.frame->setObjectName("telemetryCellAlt");
    m_cells.append(posYCell);

    // Place all cells: row = i/3, col = i%3
    for (int i = 0; i < m_cells.size(); ++i) {
        mainLayout->addWidget(m_cells[i].frame, i / 3, i % 3);
        mainLayout->setColumnStretch(i % 3, 1);
    }
    for (int r = 0; r < 3; ++r) mainLayout->setRowStretch(r, 1);

    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &SensorDataWidget::updateDisplay);
    m_updateTimer->start(100);
}

bool SensorDataWidget::initialize()
{
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::imuDataReceived,
                this, &SensorDataWidget::onIMUDataReceived);
        connect(m_ros2Interface, &ROS2Interface::robotStatusReceived,
                this, &SensorDataWidget::onRobotStatusReceived);
        connect(m_ros2Interface, &ROS2Interface::coordinatesReceived,
                this, &SensorDataWidget::onCoordinatesReceived);
        Logger::instance().info("Sensor data widget initialized");
        return true;
    }
    Logger::instance().warning("Sensor data widget initialized without ROS2");
    return false;
}

void SensorDataWidget::onIMUDataReceived(double ax, double ay, double az,
                                          double gx, double gy, double gz)
{
    m_imuData = {ax, ay, az, gx, gy, gz};
}

void SensorDataWidget::onRobotStatusReceived(const QString& status)
{
    m_robotStatus = status;
}

void SensorDataWidget::onCoordinatesReceived(double x, double y)
{
    m_posX = x;
    m_posY = y;
}

void SensorDataWidget::updateDisplay()
{
    if (m_cells.size() < 8) return;

    auto scale = [](double val, double minv, double maxv) -> int {
        if (val <= minv) return 0;
        if (val >= maxv) return 100;
        return int((val - minv) / (maxv - minv) * 100.0);
    };

    // Accel
    m_cells[0].valueLabel->setText(QString::number(m_imuData.ax, 'f', 3));
    m_cells[1].valueLabel->setText(QString::number(m_imuData.ay, 'f', 3));
    m_cells[2].valueLabel->setText(QString::number(m_imuData.az, 'f', 3));
    m_cells[0].miniBar->setValue(scale(m_imuData.ax, m_cells[0].minValue, m_cells[0].maxValue));
    m_cells[1].miniBar->setValue(scale(m_imuData.ay, m_cells[1].minValue, m_cells[1].maxValue));
    m_cells[2].miniBar->setValue(scale(m_imuData.az, m_cells[2].minValue, m_cells[2].maxValue));

    // Gyro
    m_cells[3].valueLabel->setText(QString::number(m_imuData.gx, 'f', 3));
    m_cells[4].valueLabel->setText(QString::number(m_imuData.gy, 'f', 3));
    m_cells[3].miniBar->setValue(scale(m_imuData.gx, m_cells[3].minValue, m_cells[3].maxValue));
    m_cells[4].miniBar->setValue(scale(m_imuData.gy, m_cells[4].minValue, m_cells[4].maxValue));

    // Robot Status (index 5)
    m_cells[5].valueLabel->setText(m_robotStatus);

    // Position (indices 6, 7)
    m_cells[6].valueLabel->setText(QString::number(m_posX, 'f', 2));
    m_cells[7].valueLabel->setText(QString::number(m_posY, 'f', 2));
    m_cells[6].miniBar->setValue(scale(m_posX, m_cells[6].minValue, m_cells[6].maxValue));
    m_cells[7].miniBar->setValue(scale(m_posY, m_cells[7].minValue, m_cells[7].maxValue));
}
