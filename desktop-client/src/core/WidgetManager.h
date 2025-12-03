#ifndef WIDGETMANAGER_H
#define WIDGETMANAGER_H

#include <QObject>
#include <QMap>
#include <QString>
#include <memory>

class BaseWidget;

/**
 * @brief Manages widget lifecycle and registration
 * 
 * This class implements the Factory and Registry patterns to manage
 * modular widgets dynamically
 */
class WidgetManager : public QObject
{
    Q_OBJECT

public:
    enum class WidgetType {
        VideoStream,
        MotionControl,
        CommandControl,
        SensorData,
        Coordinates,
        TwinVisualization,
        Custom
    };

    explicit WidgetManager(QObject *parent = nullptr);
    ~WidgetManager();

    /**
     * @brief Register a widget type
     */
    void registerWidget(WidgetType type, const QString& name);

    /**
     * @brief Create a widget instance
     */
    BaseWidget* createWidget(WidgetType type, QWidget* parent = nullptr);

    /**
     * @brief Get all registered widget types
     */
    QList<WidgetType> registeredTypes() const;

    /**
     * @brief Get widget name by type
     */
    QString widgetName(WidgetType type) const;

    /**
     * @brief Track an active widget
     */
    void addActiveWidget(const QString& id, BaseWidget* widget);

    /**
     * @brief Remove a widget from tracking
     */
    void removeActiveWidget(const QString& id);

    /**
     * @brief Get active widget by ID
     */
    BaseWidget* getWidget(const QString& id) const;

    /**
     * @brief Get all active widgets
     */
    QList<BaseWidget*> activeWidgets() const;

signals:
    void widgetCreated(BaseWidget* widget);
    void widgetRemoved(const QString& id);

private:
    QMap<WidgetType, QString> m_registeredWidgets;
    QMap<QString, BaseWidget*> m_activeWidgets;
    int m_widgetCounter;

    QString generateWidgetId(WidgetType type);
};

#endif // WIDGETMANAGER_H
