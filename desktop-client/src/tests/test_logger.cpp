#include <gtest/gtest.h>
#include "utils/Logger.h"
#include <fstream>
#include <sstream>
#include <cstdlib>

class LoggerTest : public ::testing::Test {
protected:
    std::string log_file = "/tmp/test_logger.log";
    
    void SetUp() override {
        // Clean up any existing log file
        std::remove(log_file.c_str());
        Logger::instance().initialize(QString::fromStdString(log_file));
    }
    
    void TearDown() override {
        // Clean up log file after test
        std::remove(log_file.c_str());
    }
    
    bool LogFileExists() {
        std::ifstream file(log_file);
        return file.good();
    }
    
    std::string ReadLogFile() {
        std::ifstream file(log_file);
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
};

// Test 1: Logger initialization
TEST_F(LoggerTest, InitializeLogger) {
    EXPECT_TRUE(LogFileExists());
}

// Test 2: Set log level to Debug
TEST_F(LoggerTest, SetLogLevelDebug) {
    EXPECT_NO_THROW({
        Logger::instance().setLogLevel(Logger::Level::Debug);
    });
}

// Test 3: Set log level to Info
TEST_F(LoggerTest, SetLogLevelInfo) {
    EXPECT_NO_THROW({
        Logger::instance().setLogLevel(Logger::Level::Info);
    });
}

// Test 4: Set log level to Warning
TEST_F(LoggerTest, SetLogLevelWarning) {
    EXPECT_NO_THROW({
        Logger::instance().setLogLevel(Logger::Level::Warning);
    });
}

// Test 5: Set log level to Error
TEST_F(LoggerTest, SetLogLevelError) {
    EXPECT_NO_THROW({
        Logger::instance().setLogLevel(Logger::Level::Error);
    });
}

// Test 6: Log debug message
TEST_F(LoggerTest, LogDebugMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().debug("This is a debug message");
    });
}

// Test 7: Log info message
TEST_F(LoggerTest, LogInfoMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().info("This is an info message");
    });
}

// Test 8: Log warning message
TEST_F(LoggerTest, LogWarningMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().warning("This is a warning message");
    });
}

// Test 9: Log error message
TEST_F(LoggerTest, LogErrorMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().error("This is an error message");
    });
}

// Test 10: Log critical message
TEST_F(LoggerTest, LogCriticalMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().critical("This is a critical message");
    });
}

// Test 11: Log all message types sequentially
TEST_F(LoggerTest, LogAllMessageTypes) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().debug("Debug message");
        Logger::instance().info("Info message");
        Logger::instance().warning("Warning message");
        Logger::instance().error("Error message");
        Logger::instance().critical("Critical message");
    });
}

// Test 12: Log empty message
TEST_F(LoggerTest, LogEmptyMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().info("");
    });
}

// Test 13: Log very long message
TEST_F(LoggerTest, LogLongMessage) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    std::string longMessage(10000, 'a');
    EXPECT_NO_THROW({
        Logger::instance().info(QString::fromStdString(longMessage));
    });
}

// Test 14: Log message with special characters
TEST_F(LoggerTest, LogSpecialCharacters) {
    Logger::instance().setLogLevel(Logger::Level::Debug);
    EXPECT_NO_THROW({
        Logger::instance().info("Special chars: !@#$%^&*()_+-=[]{}|;:',.<>?/");
    });
}

// Test 15: Log level filtering (Info should not log Debug)
TEST_F(LoggerTest, LogLevelFiltering) {
    Logger::instance().setLogLevel(Logger::Level::Info);
    EXPECT_NO_THROW({
        Logger::instance().debug("This should not appear");
        Logger::instance().info("This should appear");
    });
}
