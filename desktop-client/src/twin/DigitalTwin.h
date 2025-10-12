#ifndef DIGITALTWIN_H
#define DIGITALTWIN_H

#include <QObject>
#include <memory>

class TwinState;
class TwinSimulator;
class ROS2Interface;

/**
 * @brief Digital Twin main class
 * 
 * Manages the digital twin of the robot, synchronizes with real robot
 * via ROS2 and runs simulation when needed
 */
class DigitalTwin : public QObject
{
    Q_OBJECT

public:
    enum class Mode {
        Synchronized,  // Synchronized with real robot
        Simulated,     // Running in simulation mode
        Offline        // Not connected
    };

    explicit DigitalTwin(QObject *parent = nullptr);
    ~DigitalTwin();

    bool initialize();
    
    /**
     * @brief Connect to ROS2 interface for data synchronization
     */
    void connectToROS2(ROS2Interface* ros2);

    /**
     * @brief Get current twin state
     */
    TwinState* state() const { return m_state.get(); }

    /**
     * @brief Get current mode
     */
    Mode mode() const { return m_mode; }

    /**
     * @brief Set twin mode
     */
    void setMode(Mode mode);

    /**
     * @brief Start simulation
     */
    void startSimulation();

    /**
     * @brief Stop simulation
     */
    void stopSimulation();

signals:
    void stateChanged();
    void modeChanged(Mode mode);
    void simulationStarted();
    void simulationStopped();
    void errorOccurred(const QString& error);

private slots:
    void onROS2ImageReceived(const QByteArray& imageData, int width, int height);
    void onROS2IMUReceived(double ax, double ay, double az, double gx, double gy, double gz);
    void onROS2StatusReceived(const QString& status);
    void onSimulationUpdate();

private:
    void setupConnections(ROS2Interface* ros2);

    std::unique_ptr<TwinState> m_state;
    std::unique_ptr<TwinSimulator> m_simulator;
    ROS2Interface* m_ros2Interface;
    Mode m_mode;
};

#endif // DIGITALTWIN_H
