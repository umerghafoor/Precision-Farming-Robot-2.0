#ifndef APPLICATION_H
#define APPLICATION_H

#include <QObject>
#include <memory>

class MainWindow;
class ROS2Interface;
class DigitalTwin;
class WidgetManager;

/**
 * @brief Main Application class - orchestrates all components
 * 
 * This class follows the Facade pattern to provide a unified interface
 * to the subsystems (ROS2, Digital Twin, UI)
 */
class Application : public QObject
{
    Q_OBJECT

public:
    explicit Application(int argc, char** argv, QObject *parent = nullptr);
    ~Application();

    /**
     * @brief Initialize all subsystems
     * @return true if successful
     */
    bool initialize();

    /**
     * @brief Show the main window
     */
    void show();

    /**
     * @brief Get ROS2 interface
     */
    ROS2Interface* ros2Interface() const { return m_ros2Interface.get(); }

    /**
     * @brief Get Digital Twin
     */
    DigitalTwin* digitalTwin() const { return m_digitalTwin.get(); }

    /**
     * @brief Get Widget Manager
     */
    WidgetManager* widgetManager() const { return m_widgetManager.get(); }

signals:
    void initializationComplete();
    void shutdownRequested();

private slots:
    void onROS2Connected();
    void onROS2Disconnected();
    void onTwinStateChanged();

private:
    bool initializeROS2(int argc, char** argv);
    bool initializeDigitalTwin();
    bool initializeUI();
    void setupConnections();

    std::unique_ptr<MainWindow> m_mainWindow;
    std::unique_ptr<ROS2Interface> m_ros2Interface;
    std::unique_ptr<DigitalTwin> m_digitalTwin;
    std::unique_ptr<WidgetManager> m_widgetManager;

    int m_argc;
    char** m_argv;
};

#endif // APPLICATION_H
