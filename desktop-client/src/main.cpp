#include <QApplication>
#include <QDockWidget>
#include <memory>
#include "core/Application.h"
#include "utils/Logger.h"
#include "ui/MaterialDockWidget.h"

// Subclass QApplication to guard against QTBUG-43698.
//
// Qt's restoreState() creates bare QDockWidget placeholder objects for any
// dock widget names in the saved state that don't exist in the current layout
// (e.g. a widget added manually in a previous session). These placeholders are
// not MaterialDockWidget instances, so they have no event() override and crash
// on any mouse event via a null internal drag-state dereference.  Additionally,
// Qt's dock group window machinery can briefly route mouse events through a plain
// QDockWidget before the widget is properly set up.
//
// notify() is the only interception point that fires for ALL events to ALL
// objects before any virtual dispatch — so we block mouse events to any dock
// widget that is not one of our MaterialDockWidget instances here.
class PrecisionFarmingApp : public QApplication {
public:
    using QApplication::QApplication;

    bool notify(QObject* receiver, QEvent* event) override {
        const QEvent::Type t = event->type();
        if (t == QEvent::MouseButtonPress   ||
            t == QEvent::MouseButtonRelease ||
            t == QEvent::MouseMove          ||
            t == QEvent::MouseButtonDblClick) {
            if (auto* dw = qobject_cast<QDockWidget*>(receiver)) {
                if (!qobject_cast<MaterialDockWidget*>(dw))
                    return false;   // drop event — prevents null drag-state crash
            }
        }
        return QApplication::notify(receiver, event);
    }
};

int main(int argc, char *argv[])
{
    // Initialize Qt Application
    PrecisionFarmingApp app(argc, argv);
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
