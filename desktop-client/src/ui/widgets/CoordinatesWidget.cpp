#include "CoordinatesWidget.h"
#include "ROS2Interface.h"
#include "Logger.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFont>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>

CoordinatesWidget::CoordinatesWidget(QWidget *parent)
    : BaseWidget(parent)
    , m_currentX(0.0)
    , m_currentY(0.0)
{
    setupUI();
}

CoordinatesWidget::~CoordinatesWidget()
{
}

void CoordinatesWidget::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(10, 10, 10, 10);

    // Create group box for coordinates
    QGroupBox* coordGroup = new QGroupBox("Robot Position");
    QVBoxLayout* groupLayout = new QVBoxLayout(coordGroup);

    // X coordinate row
    QHBoxLayout* xLayout = new QHBoxLayout();
    m_xLabel = new QLabel("X:");
    m_xLabel->setStyleSheet("font-weight: bold; font-size: 14pt;");
    m_xValueLabel = new QLabel("0.00 m");
    m_xValueLabel->setStyleSheet("font-size: 18pt; color: #4CAF50;");
    m_xValueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    
    xLayout->addWidget(m_xLabel);
    xLayout->addStretch();
    xLayout->addWidget(m_xValueLabel);

    // Y coordinate row
    QHBoxLayout* yLayout = new QHBoxLayout();
    m_yLabel = new QLabel("Y:");
    m_yLabel->setStyleSheet("font-weight: bold; font-size: 14pt;");
    m_yValueLabel = new QLabel("0.00 m");
    m_yValueLabel->setStyleSheet("font-size: 18pt; color: #2196F3;");
    m_yValueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    
    yLayout->addWidget(m_yLabel);
    yLayout->addStretch();
    yLayout->addWidget(m_yValueLabel);

    groupLayout->addLayout(xLayout);
    groupLayout->addLayout(yLayout);

    mainLayout->addWidget(coordGroup);
    mainLayout->addStretch();

    // Set minimum size
    setMinimumSize(250, 120);
}

bool CoordinatesWidget::initialize()
{
    if (m_ros2Interface) {
        // NOTE: You must implement the signal coordinatesJsonReceived(const QString&) in ROS2Interface
        connect(m_ros2Interface, SIGNAL(coordinatesJsonReceived(QString)),
                this, SLOT(onCoordinatesJsonReceived(QString)));
        Logger::instance().info("Coordinates widget initialized");
        return true;
    }
    Logger::instance().warning("Coordinates widget initialized without ROS2");
    return false;
}

void CoordinatesWidget::onCoordinatesJsonReceived(const QString& data)
{
    // Parse the JSON data
    QJsonDocument doc = QJsonDocument::fromJson(data.toUtf8());
    
    if (doc.isNull()) {
        Logger::instance().warning("CoordinatesWidget: Invalid JSON received");
        return;
    }

    bool foundMarker = false;
    double x = 0.0;
    double y = 0.0;
    
    // Check if it's an array or object
    if (doc.isArray()) {
        QJsonArray rootArray = doc.array();
        
        // Look for the first marker in the array
        for (const QJsonValue& itemVal : rootArray) {
            if (!itemVal.isObject() || foundMarker) continue;
            
            QJsonObject item = itemVal.toObject();
            QJsonArray markers = item.value("markers").toArray();
            
            if (!markers.isEmpty()) {
                QJsonObject marker = markers[0].toObject();
                QJsonObject center = marker.value("center").toObject();
                x = center.value("x").toDouble();
                y = center.value("y").toDouble();
                foundMarker = true;
                break;
            }
        }
    } else if (doc.isObject()) {
        // Handle single object case
        QJsonObject obj = doc.object();
        QJsonArray markers = obj.value("markers").toArray();
        
        if (!markers.isEmpty()) {
            QJsonObject marker = markers[0].toObject();
            QJsonObject center = marker.value("center").toObject();
            x = center.value("x").toDouble();
            y = center.value("y").toDouble();
            foundMarker = true;
        }
    }
    
    // Update the display values
    m_currentX = x;
    m_currentY = y;
    m_xValueLabel->setText(QString("%1 px").arg(x, 0, 'f', 2));
    m_yValueLabel->setText(QString("%1 px").arg(y, 0, 'f', 2));
    
    if (foundMarker) {
        Logger::instance().debug(QString("CoordinatesWidget updated: X=%1, Y=%2").arg(x).arg(y));
    }
}
