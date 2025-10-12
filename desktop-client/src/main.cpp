#include <QApplication>
#include <memory>
#include "core/Application.h"
#include "utils/Logger.h"

int main(int argc, char *argv[])
{
    // Initialize Qt Application
    QApplication app(argc, argv);
    app.setApplicationName("Precision Farming Desktop Client");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("PrecisionFarming");

    // Initialize Logger
    Logger::instance().initialize("PrecisionFarmingClient.log");
    Logger::instance().info("Application starting...");

    try {
        // Create and run the main application
        auto farmingApp = std::make_unique<Application>(argc, argv);
        
        if (!farmingApp->initialize()) {
            Logger::instance().error("Failed to initialize application");
            return -1;
        }

        farmingApp->show();
        Logger::instance().info("Application initialized successfully");

        int result = app.exec();
        
        Logger::instance().info("Application shutting down...");
        return result;
    }
    catch (const std::exception& e) {
        Logger::instance().error(QString("Fatal error: %1").arg(e.what()));
        return -1;
    }
}
