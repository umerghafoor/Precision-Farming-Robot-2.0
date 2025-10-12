#include "Logger.h"
#include <QDateTime>
#include <QDebug>
#include <QRecursiveMutex>

Logger::Logger()
    : m_logLevel(Level::Debug)
    , m_initialized(false)
{
}

Logger::~Logger()
{
    if (m_logFile.isOpen()) {
        m_logStream.flush();
        m_logFile.close();
    }
}

Logger& Logger::instance()
{
    static Logger instance;
    return instance;
}

void Logger::initialize(const QString& logFilePath)
{
    QMutexLocker locker(&m_mutex);

    if (m_initialized) {
        return;
    }

    m_logFile.setFileName(logFilePath);
    if (m_logFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        m_logStream.setDevice(&m_logFile);
        m_initialized = true;
        log(Level::Info, "Logger initialized");
    } else {
        qWarning() << "Failed to open log file:" << logFilePath;
    }
}

void Logger::setLogLevel(Level level)
{
    m_logLevel = level;
}

void Logger::debug(const QString& message)
{
    log(Level::Debug, message);
}

void Logger::info(const QString& message)
{
    log(Level::Info, message);
}

void Logger::warning(const QString& message)
{
    log(Level::Warning, message);
}

void Logger::error(const QString& message)
{
    log(Level::Error, message);
}

void Logger::critical(const QString& message)
{
    log(Level::Critical, message);
}

void Logger::log(Level level, const QString& message)
{
    if (level < m_logLevel) {
        return;
    }

    QMutexLocker locker(&m_mutex);

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz");
    QString levelStr = levelToString(level);
    QString formattedMessage = QString("[%1] [%2] %3").arg(timestamp, levelStr, message);

    // Write to file
    if (m_initialized && m_logFile.isOpen()) {
        m_logStream << formattedMessage << Qt::endl;
        m_logStream.flush();
    }

    // Write to console
    switch (level) {
        case Level::Debug:
            qDebug().noquote() << formattedMessage;
            break;
        case Level::Info:
            qInfo().noquote() << formattedMessage;
            break;
        case Level::Warning:
            qWarning().noquote() << formattedMessage;
            break;
        case Level::Error:
        case Level::Critical:
            qCritical().noquote() << formattedMessage;
            break;
    }

    emit logMessage(formattedMessage);
}

QString Logger::levelToString(Level level) const
{
    switch (level) {
        case Level::Debug: return "DEBUG";
        case Level::Info: return "INFO";
        case Level::Warning: return "WARN";
        case Level::Error: return "ERROR";
        case Level::Critical: return "CRITICAL";
    }
    return "UNKNOWN";
}
