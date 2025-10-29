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

    // ROS2 menu
    QMenu* ros2Menu = menuBar()->addMenu(tr("&ROS2"));
    m_connectAction = ros2Menu->addAction(tr("&Connect"), this, &MainWindow::onToggleROS2Connection);
    m_connectAction->setCheckable(true);

    // Help menu
    QMenu* helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(tr("&About"), this, &MainWindow::onAbout);
}

void MainWindow::createToolBar()
{
    QToolBar* toolbar = addToolBar(tr("Main Toolbar"));
    toolbar->setMovable(false);

    toolbar->addAction(m_connectAction);
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
    if (m_widgetManager && m_ros2Interface) {
        createDefaultLayout();
    }
}

void MainWindow::createDefaultLayout()
{
    Logger::instance().info("Creating default widget layout");
    
    // Create a simple 2-panel layout: Video + Motion Control
    onAddVideoStream();      // Left side
    onAddMotionControl();    // Right side
    
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

    // Simple placement: Video on left, Motion control on right
    Qt::DockWidgetArea area = Qt::LeftDockWidgetArea;
    
    if (title.contains("Motion", Qt::CaseInsensitive)) {
        area = Qt::RightDockWidgetArea;
    }
    
    addDockWidget(area, dock);
    
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

void MainWindow::onAddMotionControl()
{
    if (!m_widgetManager) return;

    auto widget = m_widgetManager->createWidget(WidgetManager::WidgetType::MotionControl, this);
    addWidgetToDock(widget, "Motion Control");
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

void MainWindow::onAbout()
{
    QMessageBox::about(this, tr("About"),
        tr("<h2>Precision Farming Robot</h2>"
           "<p>Desktop Client v1.0.0</p>"
           "<p>Robot movement control and camera monitoring interface.</p>"
           "<p>Features:</p>"
           "<ul>"
           "<li>Live Camera Feed</li>"
           "<li>Motion Control</li>"
           "<li>ROS2 Integration</li>"
           "</ul>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    Logger::instance().info("Main window closing");
    event->accept();
}
