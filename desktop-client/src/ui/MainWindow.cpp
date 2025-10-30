#include "MainWindow.h"
#include "WidgetManager.h"
#include "ROS2Interface.h"
#include "DigitalTwin.h"
#include "BaseWidget.h"
#include "Logger.h"

#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QMessageBox>
#include <QCloseEvent>
#include <QLabel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_widgetManager(nullptr)
    , m_ros2Interface(nullptr)
    , m_digitalTwin(nullptr)
    , m_ros2Connected(false)
{
    setupUI();
}

MainWindow::~MainWindow()
{
    Logger::instance().info("Main window destroyed");
}

void MainWindow::setupUI()
{
    setWindowTitle("Precision Farming Robot - Desktop Client");
    setMinimumSize(1280, 720);
    resize(1600, 900);

    // Enable docking
    setDockNestingEnabled(true);
    setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    createMenus();
    createToolBar();
    createStatusBar();

    // Set central widget as empty - we'll use dock widgets
    QWidget* centralWidget = new QWidget(this);
    centralWidget->setStyleSheet("background-color: #2b2b2b;");
    setCentralWidget(centralWidget);
}

void MainWindow::createMenus()
{
    // File menu
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(tr("&Exit"), QKeySequence::Quit, this, &QWidget::close);

    // Widgets menu
    QMenu* widgetsMenu = menuBar()->addMenu(tr("&Widgets"));
    widgetsMenu->addAction(tr("Add &Video Stream"), this, &MainWindow::onAddVideoStream);
    widgetsMenu->addAction(tr("Add &Command Control"), this, &MainWindow::onAddCommandControl);
    widgetsMenu->addAction(tr("Add &Motion Control"), this, &MainWindow::onAddMotionControl);
    widgetsMenu->addAction(tr("Add &Sensor Data"), this, &MainWindow::onAddSensorData);
    widgetsMenu->addAction(tr("Add C&oordinates"), this, &MainWindow::onAddCoordinates);
    // widgetsMenu->addAction(tr("Add &Digital Twin"), this, &MainWindow::onAddTwinVisualization); // Disabled: Digital Twin widget hidden
    widgetsMenu->addSeparator();
    widgetsMenu->addAction(tr("&Remove Widget"), this, &MainWindow::onRemoveWidget);

    // ROS2 menu
    QMenu* ros2Menu = menuBar()->addMenu(tr("&ROS2"));
    m_connectAction = ros2Menu->addAction(tr("&Connect"), this, &MainWindow::onToggleROS2Connection);
    m_connectAction->setCheckable(true);

    // Simulation menu
    QMenu* simMenu = menuBar()->addMenu(tr("&Simulation"));
    m_simulateAction = simMenu->addAction(tr("&Start Simulation"), this, &MainWindow::onToggleSimulation);
    m_simulateAction->setCheckable(true);

    // Help menu
    QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(tr("&About"), this, &MainWindow::onAbout);
}

void MainWindow::createToolBar()
{
    QToolBar* toolbar = addToolBar(tr("Main Toolbar"));
    toolbar->setMovable(false);

    toolbar->addAction(tr("Video"), this, &MainWindow::onAddVideoStream);
    toolbar->addAction(tr("Control"), this, &MainWindow::onAddCommandControl);
    toolbar->addAction(tr("Motion"), this, &MainWindow::onAddMotionControl);
    //toolbar->addAction(tr("Sensors"), this, &MainWindow::onAddSensorData);
   // toolbar->addAction(tr("Twin"), this, &MainWindow::onAddTwinVisualization);
    toolbar->addSeparator();
    toolbar->addAction(m_connectAction);
    toolbar->addAction(m_simulateAction);
}

void MainWindow::createStatusBar()
{
    statusBar()->showMessage(tr("Ready"));
}

void MainWindow::setWidgetManager(WidgetManager* manager)
{
    m_widgetManager = manager;
}

void MainWindow::setROS2Interface(ROS2Interface* ros2)
{
    m_ros2Interface = ros2;
    
    if (m_ros2Interface) {
        connect(m_ros2Interface, &ROS2Interface::connected, this, [this]() {
            statusBar()->showMessage(tr("ROS2 Connected"), 3000);
            m_ros2Connected = true;
        });
        connect(m_ros2Interface, &ROS2Interface::disconnected, this, [this]() {
            statusBar()->showMessage(tr("ROS2 Disconnected"), 3000);
            m_ros2Connected = false;
        });
    }
}

void MainWindow::setDigitalTwin(DigitalTwin* twin)
{
    m_digitalTwin = twin;
    
    // Create default layout after all dependencies are set
    if (m_widgetManager && m_ros2Interface && m_digitalTwin) {
        createDefaultLayout();
    }
}

void MainWindow::createDefaultLayout()
{
    Logger::instance().info("Creating default widget layout");
    
    // Create a sensible default layout
    onAddVideoStream();      // Left side
    onAddCoordinates();      // Left side, next to video
    // onAddTwinVisualization(); // Disabled: Digital Twin widget hidden
    onAddMotionControl();     // Right side
    onAddCommandControl();    // Right side (will be tabbed with motion)
    onAddSensorData();        // Bottom
    
    Logger::instance().info("Default layout created");
}

void MainWindow::addWidgetToDock(BaseWidget* widget, const QString& title)
{
    if (!widget) return;

    QDockWidget* dock = new QDockWidget(title, this);
    dock->setWidget(widget);
    dock->setObjectName(widget->widgetId());
    
    // Make dockable, movable, and closable
    dock->setFeatures(QDockWidget::DockWidgetMovable | 
                      QDockWidget::DockWidgetClosable | 
                      QDockWidget::DockWidgetFloatable);

    // Determine dock area based on widget type
    Qt::DockWidgetArea area = Qt::RightDockWidgetArea;
    QDockWidget* splitWith = nullptr;
    
    // Smart placement based on widget type
    if (title.contains("Video", Qt::CaseInsensitive)) {
        area = Qt::LeftDockWidgetArea;
    } else if (title.contains("Coordinates", Qt::CaseInsensitive)) {
        area = Qt::LeftDockWidgetArea;
        // Find video widget to split with
        for (auto it = m_dockWidgets.begin(); it != m_dockWidgets.end(); ++it) {
            if (it.key().contains("video", Qt::CaseInsensitive)) {
                splitWith = it.value();
                break;
            }
        }
    } else if (title.contains("Twin", Qt::CaseInsensitive)) {
        area = Qt::LeftDockWidgetArea;
        // Find video widget to split with
        for (auto it = m_dockWidgets.begin(); it != m_dockWidgets.end(); ++it) {
            if (it.key().contains("video", Qt::CaseInsensitive)) {
                splitWith = it.value();
                break;
            }
        }
    } else if (title.contains("Motion", Qt::CaseInsensitive)) {
        area = Qt::RightDockWidgetArea;
    } else if (title.contains("Command", Qt::CaseInsensitive)) {
        area = Qt::RightDockWidgetArea;
        // Find motion widget to split with
        for (auto it = m_dockWidgets.begin(); it != m_dockWidgets.end(); ++it) {
            if (it.key().contains("_", Qt::CaseInsensitive) && dockWidgetArea(it.value()) == Qt::RightDockWidgetArea) {
                splitWith = it.value();
                break;
            }
        }
    } else if (title.contains("Sensor", Qt::CaseInsensitive)) {
        area = Qt::BottomDockWidgetArea;
    }
    
    addDockWidget(area, dock);
    
    // Split with existing widget if found
    if (splitWith) {
        splitDockWidget(splitWith, dock, Qt::Vertical);
    }
    
    m_dockWidgets[widget->widgetId()] = dock;

    // Connect widget to data sources
    widget->setROS2Interface(m_ros2Interface);
    widget->setDigitalTwin(m_digitalTwin);
    widget->initialize();

    Logger::instance().info(QString("Added widget to dock: %1 in area %2").arg(title).arg(area));
}

void MainWindow::onAddVideoStream()
{
    if (!m_widgetManager) return;
    
    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::VideoStream, this);
    addWidgetToDock(widget, "Video Stream");
}

void MainWindow::onAddCommandControl()
{
    if (!m_widgetManager) return;
    
    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::CommandControl, this);
    addWidgetToDock(widget, "Command & Control");
}

void MainWindow::onAddMotionControl()
{
    if (!m_widgetManager) return;

    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::MotionControl, this);
    addWidgetToDock(widget, "Motion Control");
}

void MainWindow::onAddSensorData()
{
    if (!m_widgetManager) return;
    
    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::SensorData, this);
    addWidgetToDock(widget, "Sensor Data");
}

void MainWindow::onAddCoordinates()
{
    if (!m_widgetManager) return;
    
    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::Coordinates, this);
    addWidgetToDock(widget, "Coordinates");
}

void MainWindow::onAddTwinVisualization()
{
    // Disabled: Digital Twin widget creation is intentionally turned off
    Logger::instance().info("Digital Twin widget creation is disabled");
    return;
}

void MainWindow::onRemoveWidget()
{
    // Implementation for removing widgetsDigital 
    statusBar()->showMessage(tr("Right-click on widget title to close"), 2000);
}

void MainWindow::onToggleROS2Connection()
{
    if (!m_ros2Interface) {
        QMessageBox::warning(this, tr("Error"), tr("ROS2 interface not initialized"));
        m_connectAction->setChecked(false);
        return;
    }

    if (m_connectAction->isChecked()) {
        m_ros2Interface->start();
        statusBar()->showMessage(tr("Connecting to ROS2..."), 2000);
    } else {
        m_ros2Interface->stop();
        statusBar()->showMessage(tr("Disconnecting from ROS2..."), 2000);
    }
}

void MainWindow::onToggleSimulation()
{
    if (!m_digitalTwin) {
        QMessageBox::warning(this, tr("Error"), tr("Digital Twin not initialized"));
        m_simulateAction->setChecked(false);
        return;
    }

    if (m_simulateAction->isChecked()) {
        m_digitalTwin->startSimulation();
        statusBar()->showMessage(tr("Simulation started"), 2000);
    } else {
        m_digitalTwin->stopSimulation();
        statusBar()->showMessage(tr("Simulation stopped"), 2000);
    }
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, tr("About"),
        tr("<h2>Precision Farming Robot</h2>"
           "<p>Desktop Client v1.0.0</p>"
           "<p>A modular Qt-based interface for robot control and monitoring.</p>"
           "<p>Features:</p>"
           "<ul>"
           "<li>ROS2 Integration</li>"
           "<li>Digital Twin Simulation</li>"
           "<li>Modular Widget System</li>"
           "<li>Real-time Sensor Data Visualization</li>"
           "</ul>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    Logger::instance().info("Main window closing");
    event->accept();
}
