#include "WidgetManager.h"
#include "BaseWidget.h"
#include "VideoStreamWidget.h"
#include "CommandControlWidget.h"
#include "SensorDataWidget.h"
#include "TwinVisualizationWidget.h"
#include "Logger.h"

WidgetManager::WidgetManager(QObject *parent)
    : QObject(parent)
    , m_widgetCounter(0)
{
    // Register default widget types
    registerWidget(WidgetType::VideoStream, "Video Stream");
    registerWidget(WidgetType::CommandControl, "Command & Control");
    registerWidget(WidgetType::SensorData, "Sensor Data");
    registerWidget(WidgetType::TwinVisualization, "Digital Twin");
}

WidgetManager::~WidgetManager()
{
    // Clean up active widgets
    for (auto widget : m_activeWidgets) {
        if (widget) {
            widget->deleteLater();
        }
    }
    m_activeWidgets.clear();
}

void WidgetManager::registerWidget(WidgetType type, const QString& name)
{
    m_registeredWidgets[type] = name;
    Logger::instance().debug(QString("Registered widget type: %1").arg(name));
}

BaseWidget* WidgetManager::createWidget(WidgetType type, QWidget* parent)
{
    BaseWidget* widget = nullptr;

    switch (type) {
        case WidgetType::VideoStream:
            widget = new VideoStreamWidget(parent);
            break;
        case WidgetType::CommandControl:
            widget = new CommandControlWidget(parent);
            break;
        case WidgetType::SensorData:
            widget = new SensorDataWidget(parent);
            break;
        case WidgetType::TwinVisualization:
            widget = new TwinVisualizationWidget(parent);
            break;
        case WidgetType::Custom:
            Logger::instance().warning("Custom widget type not implemented");
            break;
    }

    if (widget) {
        QString widgetId = generateWidgetId(type);
        widget->setWidgetId(widgetId);
        addActiveWidget(widgetId, widget);
        
        emit widgetCreated(widget);
        Logger::instance().info(QString("Created widget: %1 (ID: %2)")
                                .arg(widgetName(type))
                                .arg(widgetId));
    }

    return widget;
}

QList<WidgetManager::WidgetType> WidgetManager::registeredTypes() const
{
    return m_registeredWidgets.keys();
}

QString WidgetManager::widgetName(WidgetType type) const
{
    return m_registeredWidgets.value(type, "Unknown");
}

void WidgetManager::addActiveWidget(const QString& id, BaseWidget* widget)
{
    m_activeWidgets[id] = widget;
}

void WidgetManager::removeActiveWidget(const QString& id)
{
    if (m_activeWidgets.contains(id)) {
        m_activeWidgets.remove(id);
        emit widgetRemoved(id);
        Logger::instance().info(QString("Removed widget: %1").arg(id));
    }
}

BaseWidget* WidgetManager::getWidget(const QString& id) const
{
    return m_activeWidgets.value(id, nullptr);
}

QList<BaseWidget*> WidgetManager::activeWidgets() const
{
    return m_activeWidgets.values();
}

QString WidgetManager::generateWidgetId(WidgetType type)
{
    QString typeName;
    switch (type) {
        case WidgetType::VideoStream: typeName = "video"; break;
        case WidgetType::CommandControl: typeName = "control"; break;
        case WidgetType::SensorData: typeName = "sensor"; break;
        case WidgetType::TwinVisualization: typeName = "twin"; break;
        case WidgetType::Custom: typeName = "custom"; break;
    }
    
    return QString("%1_%2").arg(typeName).arg(++m_widgetCounter);
}
