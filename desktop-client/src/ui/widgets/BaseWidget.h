#ifndef BASEWIDGET_H
#define BASEWIDGET_H

#include <QWidget>
#include <QString>

class ROS2Interface;
class DigitalTwin;

/**
 * @brief Base class for all modular widgets
 * 
 * Provides common interface and functionality for all widgets
 */
class BaseWidget : public QWidget
{
    Q_OBJECT

public:
    explicit BaseWidget(QWidget *parent = nullptr);
    virtual ~BaseWidget();

    /**
     * @brief Initialize the widget
     */
    virtual bool initialize() = 0;

    /**
     * @brief Get widget unique ID
     */
    QString widgetId() const { return m_widgetId; }

    /**
     * @brief Set widget unique ID
     */
    void setWidgetId(const QString& id) { m_widgetId = id; }

    /**
     * @brief Set ROS2 interface
     */
    void setROS2Interface(ROS2Interface* ros2) { m_ros2Interface = ros2; }

    /**
     * @brief Set Digital Twin
     */
    void setDigitalTwin(DigitalTwin* twin) { m_digitalTwin = twin; }

    /**
     * @brief Get widget display name
     */
    virtual QString displayName() const = 0;

signals:
    void widgetClosed(const QString& id);
    void errorOccurred(const QString& error);

protected:
    void closeEvent(QCloseEvent *event) override;

    ROS2Interface* m_ros2Interface;
    DigitalTwin* m_digitalTwin;
    QString m_widgetId;
};

#endif // BASEWIDGET_H
