#ifndef TWINSIMULATOR_H
#define TWINSIMULATOR_H

#include <QObject>
#include <QTimer>

class TwinState;

/**
 * @brief Simulates robot behavior for the digital twin
 * 
 * This class runs physics simulation and behavior modeling
 * when the twin is in simulation mode
 */
class TwinSimulator : public QObject
{
    Q_OBJECT

public:
    explicit TwinSimulator(TwinState* state, QObject *parent = nullptr);
    ~TwinSimulator();

    void start();
    void stop();
    bool isRunning() const { return m_running; }

    void setUpdateRate(int hz);
    int updateRate() const { return m_updateRate; }

signals:
    void stateUpdated();
    void simulationError(const QString& error);

private slots:
    void update();

private:
    void simulatePhysics(double dt);
    void simulateSensors();

    TwinState* m_state;
    QTimer* m_updateTimer;
    bool m_running;
    int m_updateRate; // Hz
    qint64 m_lastUpdateTime;
    double m_simulationTime;
};

#endif // TWINSIMULATOR_H
