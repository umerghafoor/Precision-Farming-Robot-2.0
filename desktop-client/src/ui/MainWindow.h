#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QMap>
#include <QSettings>
#include <memory>
#include "MaterialDockWidget.h"

class WidgetManager;
class ROS2Interface;
class BaseWidget;

#include "twin/DigitalTwin.h"    // needed for DigitalTwin::Mode in slot signature

/**
 * @brief Main application window with dockable widget system
 * 
 * Provides a flexible workspace where users can arrange widgets
 * in a modular fashion
 */
#include "widgets/StatusBadge.h"

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
    void onAddMotionControl();   // deprecated: forwards to sidebar
    void onAddCommandControl();  // deprecated: forwards to sidebar
    void onAddSidebar();         // unified controls panel
    void onAddSensorData();
    void onAddCoordinates();
    void onAddTwinVisualization();
    void onRemoveWidget();
    void onToggleROS2Connection();
    void onToggleSimulation();
    void onAbout();

    // status badge updates
    void onROS2Connected();
    void onROS2Disconnected();
    void onSimulationModeChanged(DigitalTwin::Mode mode);

private:
    void setupUI();
    void createMenus();
    void createToolBar();
    void createStatusBar();
    void addWidgetToDock(BaseWidget* widget, const QString& title);
    void createDefaultLayout();

    // layout persistence helpers
    bool restoreLayout();
    void saveLayout();

    bool m_layoutRestored{false};

    WidgetManager* m_widgetManager;
    ROS2Interface* m_ros2Interface;
    DigitalTwin* m_digitalTwin;

    QMap<QString, MaterialDockWidget*> m_dockWidgets;

    // UI elements
    QAction* m_connectAction;
    QAction* m_simulateAction;
    bool m_ros2Connected;

    // persistent badges in title/menu bar
    StatusBadge *m_ros2Badge;
    StatusBadge *m_simBadge;
};

#endif // MAINWINDOW_H
