#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QRecursiveMutex>

/**
 * @brief Singleton logger class for application-wide logging
 */
class Logger : public QObject
{
    Q_OBJECT

public:
    enum class Level {
        Debug,
        Info,
        Warning,
        Error,
        Critical
    };

    static Logger& instance();

    void initialize(const QString& logFilePath);
    void setLogLevel(Level level);

    void debug(const QString& message);
    void info(const QString& message);
    void warning(const QString& message);
    void error(const QString& message);
    void critical(const QString& message);

signals:
    void logMessage(const QString& message);

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log(Level level, const QString& message);
    QString levelToString(Level level) const;

    QFile m_logFile;
    QTextStream m_logStream;
    QRecursiveMutex m_mutex;
    Level m_logLevel;
    bool m_initialized;
};

#endif // LOGGER_H
