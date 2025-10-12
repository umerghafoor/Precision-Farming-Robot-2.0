#include "Application.h"
#include "MainWindow.h"
#include "WidgetManager.h"
#include "ROS2Interface.h"
#include "DigitalTwin.h"
#include "Logger.h"

Application::Application(int argc, char** argv, QObject *parent)
    : QObject(parent)
    , m_argc(argc)
    , m_argv(argv)
{
}

Application::~Application()
{
    Logger::instance().info("Application destroyed");
}

bool Application::initialize()
{
    Logger::instance().info("Initializing application components...");

    // Initialize in order of dependency
    if (!initializeROS2(m_argc, m_argv)) {
        Logger::instance().error("Failed to initialize ROS2 interface");
        return false;
    }

    if (!initializeDigitalTwin()) {
        Logger::instance().error("Failed to initialize Digital Twin");
        return false;
    }

    if (!initializeUI()) {
        Logger::instance().error("Failed to initialize UI");
        return false;
    }

    setupConnections();

    emit initializationComplete();
    return true;
}

bool Application::initializeROS2(int argc, char** argv)
{
    try {
        m_ros2Interface = std::make_unique<ROS2Interface>(argc, argv);
        
        if (!m_ros2Interface->initialize()) {
            return false;
        }

        Logger::instance().info("ROS2 interface initialized");
        return true;
    }
    catch (const std::exception& e) {
        Logger::instance().error(QString("ROS2 initialization error: %1").arg(e.what()));
        return false;
    }
}

bool Application::initializeDigitalTwin()
{
    try {
        m_digitalTwin = std::make_unique<DigitalTwin>();
        
        if (!m_digitalTwin->initialize()) {
            return false;
        }

        // Connect digital twin to ROS2
        if (m_ros2Interface) {
            m_digitalTwin->connectToROS2(m_ros2Interface.get());
        }

        Logger::instance().info("Digital Twin initialized");
        return true;
    }
    catch (const std::exception& e) {
        Logger::instance().error(QString("Digital Twin initialization error: %1").arg(e.what()));
        return false;
    }
}

bool Application::initializeUI()
{
    try {
        // Create widget manager
        m_widgetManager = std::make_unique<WidgetManager>();

        // Create main window
        m_mainWindow = std::make_unique<MainWindow>();
        m_mainWindow->setWidgetManager(m_widgetManager.get());
        m_mainWindow->setROS2Interface(m_ros2Interface.get());
        m_mainWindow->setDigitalTwin(m_digitalTwin.get());

        Logger::instance().info("UI initialized");
        return true;
    }
    catch (const std::exception& e) {
        Logger::instance().error(QString("UI initialization error: %1").arg(e.what()));
        return false;
    }
}

void Application::setupConnections()
{
    // Connect ROS2 signals
    if (m_ros2Interface) {
        connect(m_ros2Interface.get(), &ROS2Interface::connected,
                this, &Application::onROS2Connected);
        connect(m_ros2Interface.get(), &ROS2Interface::disconnected,
                this, &Application::onROS2Disconnected);
    }

    // Connect Digital Twin signals
    if (m_digitalTwin) {
        connect(m_digitalTwin.get(), &DigitalTwin::stateChanged,
                this, &Application::onTwinStateChanged);
    }
}

void Application::show()
{
    if (m_mainWindow) {
        m_mainWindow->show();
    }
}

void Application::onROS2Connected()
{
    Logger::instance().info("ROS2 connected");
}

void Application::onROS2Disconnected()
{
    Logger::instance().warning("ROS2 disconnected");
}

void Application::onTwinStateChanged()
{
    Logger::instance().debug("Digital Twin state changed");
}
