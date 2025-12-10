#include <gtest/gtest.h>
#include "core/WidgetManager.h"

class WidgetManagerTest : public ::testing::Test {
protected:
    WidgetManager wm;
    
    void SetUp() override {
        // Initialize any test fixtures here
    }
    
    void TearDown() override {
        // Cleanup after each test
    }
};

// Test 1: Widget registration success
TEST_F(WidgetManagerTest, RegisterWidgetSuccess) {
    EXPECT_NO_THROW({
        wm.registerWidget(WidgetManager::WidgetType::VideoStream, "TestWidget");
    });
}

// Test 2: Multiple widget registration
TEST_F(WidgetManagerTest, RegisterMultipleWidgets) {
    EXPECT_NO_THROW({
        wm.registerWidget(WidgetManager::WidgetType::VideoStream, "Widget1");
        wm.registerWidget(WidgetManager::WidgetType::MotionControl, "Widget2");
        wm.registerWidget(WidgetManager::WidgetType::CommandControl, "Widget3");
    });
}

// Test 3: Add active widget
TEST_F(WidgetManagerTest, AddActiveWidget) {
    BaseWidget* widget = nullptr;
    wm.addActiveWidget("widget_001", widget);
    SUCCEED();
}

// Test 4: Remove active widget
TEST_F(WidgetManagerTest, RemoveActiveWidget) {
    BaseWidget* widget = nullptr;
    wm.addActiveWidget("widget_001", widget);
    wm.removeActiveWidget("widget_001");
    SUCCEED();
}

// Test 5: Add and remove multiple widgets
TEST_F(WidgetManagerTest, AddRemoveMultipleActiveWidgets) {
    BaseWidget* widget1 = nullptr;
    BaseWidget* widget2 = nullptr;
    BaseWidget* widget3 = nullptr;
    
    wm.addActiveWidget("id_1", widget1);
    wm.addActiveWidget("id_2", widget2);
    wm.addActiveWidget("id_3", widget3);
    
    wm.removeActiveWidget("id_1");
    wm.removeActiveWidget("id_2");
    wm.removeActiveWidget("id_3");
    
    SUCCEED();
}

// Test 6: Remove non-existent widget (should not crash)
TEST_F(WidgetManagerTest, RemoveNonExistentWidget) {
    EXPECT_NO_THROW({
        wm.removeActiveWidget("non_existent_id");
    });
}

// Test 7: Duplicate widget registration
TEST_F(WidgetManagerTest, DuplicateWidgetRegistration) {
    EXPECT_NO_THROW({
        wm.registerWidget(WidgetManager::WidgetType::VideoStream, "DuplicateWidget");
        wm.registerWidget(WidgetManager::WidgetType::VideoStream, "DuplicateWidget");
    });
}

// Test 8: Empty widget name
TEST_F(WidgetManagerTest, EmptyWidgetName) {
    EXPECT_NO_THROW({
        wm.registerWidget(WidgetManager::WidgetType::VideoStream, "");
    });
}

// Test 9: Add widget with empty ID
TEST_F(WidgetManagerTest, AddWidgetWithEmptyID) {
    BaseWidget* widget = nullptr;
    EXPECT_NO_THROW({
        wm.addActiveWidget("", widget);
    });
}
