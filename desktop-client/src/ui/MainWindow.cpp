#include "MainWindow.h"
#include "MaterialDockWidget.h"
#include "WidgetManager.h"
#include "ROS2Interface.h"
#include "DigitalTwin.h"
#include "BaseWidget.h"
#include "Logger.h"
#include "SidebarWidget.h"

#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QDockWidget>
#include <QMessageBox>
#include <QCloseEvent>
#include <QLabel>
#include <QToolButton>
#include <QHBoxLayout>

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
    centralWidget->setObjectName("mainCentralWidget");
    setCentralWidget(centralWidget);

    // 6px gap between docks exposes the surface background, creating M3 card separation
    setContentsMargins(6, 6, 6, 6);
}

void MainWindow::createMenus()
{
    // File menu
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    QAction* exitAction = fileMenu->addAction(tr("&Exit"));
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QWidget::close);

    // Widgets menu
    QMenu* widgetsMenu = menuBar()->addMenu(tr("&Widgets"));
    widgetsMenu->addAction(tr("Add &Video Stream"), this, &MainWindow::onAddVideoStream);
    widgetsMenu->addAction(tr("Add &Controls"), this, &MainWindow::onAddSidebar);
    // legacy entries kept for compatibility but hidden from default UI
    // widgetsMenu->addAction(tr("Add &Command Control"), this, &MainWindow::onAddCommandControl);
    // widgetsMenu->addAction(tr("Add &Motion Control"), this, &MainWindow::onAddMotionControl);
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

    // create persistent badges container
    QWidget *badgeContainer = new QWidget(this);
    QHBoxLayout *badgeLayout = new QHBoxLayout(badgeContainer);
    badgeLayout->setContentsMargins(0,0,0,0);
    badgeLayout->setSpacing(8);
    m_ros2Badge = new StatusBadge(tr("ROS2 Offline"), this);
    m_simBadge  = new StatusBadge(tr("Simulation Off"), this);
    badgeLayout->addWidget(m_ros2Badge);
    badgeLayout->addWidget(m_simBadge);
    badgeContainer->setObjectName("appBarBadgeContainer");
    menuBar()->setCornerWidget(badgeContainer, Qt::TopRightCorner);

    // WeedX brand label in left corner — M3 app bar branding
    QLabel* brandLabel = new QLabel(this);
    brandLabel->setObjectName("appBarBrandLabel");
    brandLabel->setText("WeedX  <span style='font-weight:400;color:#7A9B79;'>Precision Farming</span>");
    brandLabel->setTextFormat(Qt::RichText);
    menuBar()->setCornerWidget(brandLabel, Qt::TopLeftCorner);
}

void MainWindow::createToolBar()
{
    QToolBar* toolbar = addToolBar(tr("Main Toolbar"));
    toolbar->setMovable(false);

    toolbar->addAction(tr("Video"), this, &MainWindow::onAddVideoStream);
    toolbar->addAction(tr("Controls"), this, &MainWindow::onAddSidebar);
    // toolbar->addAction(tr("Control"), this, &MainWindow::onAddCommandControl);
    // toolbar->addAction(tr("Motion"), this, &MainWindow::onAddMotionControl);
    //toolbar->addAction(tr("Sensors"), this, &MainWindow::onAddSensorData);
   // toolbar->addAction(tr("Twin"), this, &MainWindow::onAddTwinVisualization);
    toolbar->addSeparator();
    toolbar->addAction(m_connectAction);
    // style hook: name the underlying toolbutton created for the connect action
    if (auto btn = qobject_cast<QToolButton*>(toolbar->widgetForAction(m_connectAction))) {
        btn->setObjectName("connectButton");
    }
    toolbar->addAction(m_simulateAction);
    if (auto btn = qobject_cast<QToolButton*>(toolbar->widgetForAction(m_simulateAction))) {
        btn->setObjectName("simulateButton");
    }
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
        connect(m_ros2Interface, &ROS2Interface::connected, this, &MainWindow::onROS2Connected);
        connect(m_ros2Interface, &ROS2Interface::disconnected, this, &MainWindow::onROS2Disconnected);
    }
}

void MainWindow::setDigitalTwin(DigitalTwin* twin)
{
    m_digitalTwin = twin;
    
    // connect badge update for simulation mode
    if (m_digitalTwin) {
        connect(m_digitalTwin, &DigitalTwin::modeChanged, this, &MainWindow::onSimulationModeChanged);
        // set initial state
        onSimulationModeChanged(m_digitalTwin->mode());
    }

    // Create or restore layout after all dependencies are set
    if (m_widgetManager && m_ros2Interface && m_digitalTwin) {
        // always create the standard set of docks first so restoreState can act on them
        createDefaultLayout();

        if (!m_layoutRestored) {
            if (restoreLayout()) {
                Logger::instance().info("Layout restored from previous session");
            } else {
                Logger::instance().info("No saved layout found; using default");
            }
            m_layoutRestored = true;
        }
    }
}

void MainWindow::createDefaultLayout()
{
    Logger::instance().info("Creating default widget layout");
    
    // Build the three-zone layout using the existing dock system.
    // The order of addition determines splitting behaviour.
    onAddVideoStream();      // left main video area
    // coordinates dock is no longer part of default layout – position will be shown inline in sidebar
    // onAddCoordinates();      // left, split beside video
    // onAddTwinVisualization(); // Disabled: Digital Twin widget hidden
    onAddSidebar();          // right sidebar unified control panel
    onAddSensorData();       // bottom telemetry bar

    // Enforce proportions: video should be ~3× wider than sidebar
    if (m_dockWidgets.contains("Video Stream") && m_dockWidgets.contains("Controls")) {
        QList<QDockWidget*> docks;
        docks << m_dockWidgets["Video Stream"] << m_dockWidgets["Controls"];
        QList<int> sizes;
        sizes << 3 << 1; // relative weights
        resizeDocks(docks, sizes, Qt::Horizontal);
    }

    // Fix telemetry bar height to ~160px so it remains visible and constant
    if (m_dockWidgets.contains("Sensor Data")) {
        MaterialDockWidget* telemetry = m_dockWidgets["Sensor Data"];
        telemetry->setMinimumHeight(160);
        telemetry->setMaximumHeight(160);
        // On some platforms the dock may still float/rescale; restricting both min/max helps
    }

    Logger::instance().info("Default layout created");
}

void MainWindow::addWidgetToDock(BaseWidget* widget, const QString& title)
{
    if (!widget) return;

    MaterialDockWidget* dock = new MaterialDockWidget(title, this);
    dock->setWidget(widget);
    dock->setObjectName(widget->widgetId());

    // Make dockable, movable, and closable
    dock->setFeatures(QDockWidget::DockWidgetMovable |
                      QDockWidget::DockWidgetClosable |
                      QDockWidget::DockWidgetFloatable);

    // Determine dock area based on widget type
    Qt::DockWidgetArea area = Qt::RightDockWidgetArea;
    MaterialDockWidget* splitWith = nullptr;
    
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

    // if this is a sidebar control panel, make sure motion gating reflects
    // current connection state (in case ROS2 was already connected)
    if (m_ros2Connected) {
        if (auto sb = qobject_cast<SidebarWidget*>(widget)) {
            sb->setMotionEnabled(true);
        }
    }

    Logger::instance().info(QString("Added widget to dock: %1 in area %2").arg(title).arg(area));
}


void MainWindow::onAddCommandControl()
{
    Logger::instance().info("CommandControl widget deprecated; using unified sidebar instead");
    onAddSidebar();
}

void MainWindow::onAddSidebar()
{
    if (!m_widgetManager) return;

    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::Sidebar, this);
    addWidgetToDock(widget, "Controls");
}

void MainWindow::onAddMotionControl()
{
    Logger::instance().info("MotionControl widget deprecated; using unified sidebar instead");
    onAddSidebar();
}

void MainWindow::onAddVideoStream()
{
    if (!m_widgetManager) return;

    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::VideoStream, this);
    addWidgetToDock(widget, "Video Stream");
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

// -----------------------------------------------------------------------------
// layout persistence helpers
// -----------------------------------------------------------------------------

bool MainWindow::restoreLayout()
{
    QSettings settings;
    if (settings.contains("windowGeometry") && settings.contains("windowState")) {
        restoreGeometry(settings.value("windowGeometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray());

        // after restoring we may need to reapply telemetry height constraint
        if (m_dockWidgets.contains("Sensor Data")) {
            MaterialDockWidget* telemetry = m_dockWidgets["Sensor Data"];
            telemetry->setMinimumHeight(160);
            telemetry->setMaximumHeight(160);
        }
        return true;
    }
    return false;
}

void MainWindow::saveLayout()
{
    QSettings settings;
    settings.setValue("windowGeometry", saveGeometry());
    settings.setValue("windowState", saveState());
}


// -----------------------------------------------------------------------------
// status badge slot implementations
// -----------------------------------------------------------------------------

void MainWindow::onROS2Connected()
{
    Logger::instance().info("MainWindow: ROS2 connected, updating badge");
    statusBar()->showMessage(tr("ROS2 Connected"), 3000);
    m_ros2Connected = true;
    if (m_ros2Badge) {
        m_ros2Badge->setText(tr("ROS2 Connected"));
        m_ros2Badge->setDotColor(QColor("#52C44A")); // live green
    }
}

void MainWindow::onROS2Disconnected()
{
    Logger::instance().info("MainWindow: ROS2 disconnected, updating badge");
    statusBar()->showMessage(tr("ROS2 Disconnected"), 3000);
    m_ros2Connected = false;
    if (m_ros2Badge) {
        m_ros2Badge->setText(tr("ROS2 Offline"));
        m_ros2Badge->setDotColor(QColor("#7A9B79")); // grey
    }
}

void MainWindow::onSimulationModeChanged(DigitalTwin::Mode mode)
{
    Logger::instance().info(QString("MainWindow: simulation mode changed to %1").arg(static_cast<int>(mode)));
    if (!m_simBadge) return;

    switch (mode) {
    case DigitalTwin::Mode::Simulated:
        m_simBadge->setText(tr("Simulation On"));
        m_simBadge->setDotColor(QColor("#52C44A"));
        break;
    default:
        m_simBadge->setText(tr("Simulation Off"));
        m_simBadge->setDotColor(QColor("#7A9B79"));
        break;
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
    saveLayout();
    event->accept();
}
