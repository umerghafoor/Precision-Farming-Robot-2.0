#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QMap>
#include <memory>

class WidgetManager;
class ROS2Interface;
class DigitalTwin;
class BaseWidget;

/**
 * @brief Main application window with dockable widget system
 * 
 * Provides a flexible workspace where users can arrange widgets
 * in a modular fashion
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setWidgetManager(WidgetManager* manager);
    void setROS2Interface(ROS2Interface* ros2);
    void setDigitalTwin(DigitalTwin* twin);

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void onAddVideoStream();
    void onAddMotionControl();
    void onAddCommandControl();
    void onAddSensorData();
    void onAddTwinVisualization();
    void onRemoveWidget();
    void onToggleROS2Connection();
    void onToggleSimulation();
    void onAbout();

private:
    void setupUI();
    void createMenus();
    void createToolBar();
    void createStatusBar();
    void addWidgetToDock(BaseWidget* widget, const QString& title);
    void createDefaultLayout();

    WidgetManager* m_widgetManager;
    ROS2Interface* m_ros2Interface;
    DigitalTwin* m_digitalTwin;

    QMap<QString, QDockWidget*> m_dockWidgets;

    // UI elements
    QAction* m_connectAction;
    QAction* m_simulateAction;
    bool m_ros2Connected;
};

#endif // MAINWINDOW_H
