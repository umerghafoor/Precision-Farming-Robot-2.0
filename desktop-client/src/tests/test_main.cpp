#include <gtest/gtest.h>
#include <QApplication>

int main(int argc, char** argv) {
    // Initialize Qt application
    QApplication app(argc, argv);
    
    // Initialize Google Test
    ::testing::InitGoogleTest(&argc, argv);
    
    // Run all tests
    return RUN_ALL_TESTS();
}
