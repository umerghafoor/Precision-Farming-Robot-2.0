# Code Reference Guide

Comprehensive technical reference for the Precision Farming Robot 2.0 codebase.

## Table of Contents

- [Desktop Client](#desktop-client)
  - [Core Module](#core-module)
  - [ROS2 Module](#ros2-module)
  - [Digital Twin Module](#digital-twin-module)
  - [UI Module](#ui-module)
  - [Utilities Module](#utilities-module)
- [ROS2 Workspace (Raspberry Pi)](#ros2-workspace-raspberry-pi)
  - [Motor Control Package](#motor-control-package)
  - [IMU Sensor Package](#imu-sensor-package)
  - [Encoder Odometry Package](#encoder-odometry-package)
  - [Robot Controller Package](#robot-controller-package)
- [Configuration Files](#configuration-files)
- [Build System](#build-system)

---

# Desktop Client

Location: `desktop-client/src/`

## Core Module

Location: `desktop-client/src/core/`

### Application Class

**File**: `Application.h`, `Application.cpp`

**Purpose**: Main application orchestrator implementing the Facade pattern

**Key Methods**:

```cpp
class Application : public QObject
{
public:
    Application(int argc, char *argv[]);
    ~Application();
    
    // Initialize all subsystems
    void initialize();
    
    // Show main window
    void show();
    
    // Get component instances
    MainWindow* getMainWindow();
    ROS2Interface* getROS2Interface();
    DigitalTwin* getDigitalTwin();
    WidgetManager* getWidgetManager();

private:
    // Component instances
    std::unique_ptr<MainWindow> m_mainWindow;
    std::unique_ptr<ROS2Interface> m_ros2Interface;
    std::unique_ptr<DigitalTwin> m_digitalTwin;
    std::unique_ptr<WidgetManager> m_widgetManager;
    QApplication* m_app;
};
```

**Usage Example**:
```cpp
int main(int argc, char *argv[]) {
    Application app(argc, argv);
    app.initialize();
    app.show();
    return app.exec();
}
```

---

### WidgetManager Class

**File**: `WidgetManager.h`, `WidgetManager.cpp`

**Purpose**: Factory and registry for managing modular widgets

**Key Methods**:

```cpp
class WidgetManager : public QObject
{
public:
    enum class WidgetType {
        VideoStream,
        CommandControl,
        SensorData,
        TwinVisualization,
        MotionControl
    };
    
    // Factory method to create widgets
    BaseWidget* createWidget(WidgetType type, QWidget* parent = nullptr);
    
    // Register a widget instance
    void registerWidget(BaseWidget* widget);
    
    // Get widget by type
    BaseWidget* getWidget(WidgetType type);
    
    // Get all registered widgets
    QVector<BaseWidget*> getAllWidgets() const;
    
    // Set dependencies for all widgets
    void setROS2Interface(ROS2Interface* interface);
    void setDigitalTwin(DigitalTwin* twin);

private:
    QMap<WidgetType, BaseWidget*> m_widgets;
    ROS2Interface* m_ros2Interface;
    DigitalTwin* m_digitalTwin;
};
```

**Usage Example**:
```cpp
WidgetManager* manager = app.getWidgetManager();
BaseWidget* videoWidget = manager->createWidget(
    WidgetManager::WidgetType::VideoStream
);
```

---

## ROS2 Module

Location: `desktop-client/src/ros2/`

### ROS2Interface Class

**File**: `ROS2Interface.h`, `ROS2Interface.cpp`

**Purpose**: Thread-safe ROS2 communication bridge

**Key Methods**:

```cpp
class ROS2Interface : public QObject
{
public:
    ROS2Interface();
    ~ROS2Interface();
    
    // Lifecycle
    void initialize();
    void shutdown();
    bool isInitialized() const;
    
    // Publishing
    void publishVelocity(double linear_x, double angular_z);
    void publishCustomCommand(const QString& command);
    
    // Topic management
    QString getImageTopic() const;
    QString getIMUTopic() const;
    QString getStatusTopic() const;

signals:
    // Signals emitted when data received (thread-safe)
    void imageReceived(const QByteArray& data, int width, int height);
    void imuDataReceived(const QVariantMap& data);
    void statusReceived(const QString& status);
    void errorOccurred(const QString& error);
    
private slots:
    void spinROS2();  // Called by timer in separate thread
    
private:
    #ifdef USE_ROS2
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imageSubscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_statusSubscriber;
    #endif
    
    QThread* m_spinThread;
    QTimer* m_spinTimer;
    bool m_initialized;
};
```

**ROS2 Topics**:
- Publishers:
  - `/cmd_vel` (geometry_msgs/Twist)
  - `/robot_command` (std_msgs/String)
- Subscribers:
  - `/camera/image_raw` (sensor_msgs/Image)
  - `/imu/data` (sensor_msgs/Imu)
  - `/robot_status` (std_msgs/String)

**Usage Example**:
```cpp
ROS2Interface* ros2 = app.getROS2Interface();
ros2->initialize();

// Publish velocity
ros2->publishVelocity(0.5, 0.0);  // Forward at 0.5 m/s

// Connect to signals
connect(ros2, &ROS2Interface::imuDataReceived, 
        this, &MyWidget::handleIMU);
```

---

## Digital Twin Module

Location: `desktop-client/src/twin/`

### DigitalTwin Class

**File**: `DigitalTwin.h`, `DigitalTwin.cpp`

**Purpose**: Main controller for digital twin functionality

**Key Methods**:

```cpp
class DigitalTwin : public QObject
{
public:
    enum class Mode {
        Synchronized,  // Mirror real robot via ROS2
        Simulated,     // Run physics simulation
        Offline        // Disconnected
    };
    
    DigitalTwin(TwinState* state, ROS2Interface* ros2);
    ~DigitalTwin();
    
    // Mode management
    void setMode(Mode mode);
    Mode getMode() const;
    
    // State access
    TwinState* getState();
    
    // Simulation control
    void startSimulation();
    void stopSimulation();
    bool isSimulationRunning() const;
    void setSimulationRate(int hz);

signals:
    void modeChanged(Mode newMode);
    void simulationStateChanged(bool running);
    
private:
    TwinState* m_state;
    TwinSimulator* m_simulator;
    ROS2Interface* m_ros2Interface;
    Mode m_currentMode;
};
```

**Usage Example**:
```cpp
DigitalTwin* twin = app.getDigitalTwin();

// Switch to simulation mode
twin->setMode(DigitalTwin::Mode::Simulated);
twin->startSimulation();

// Get current state
TwinState* state = twin->getState();
QVector3D position = state->getPosition();
```

---

### TwinState Class

**File**: `TwinState.h`, `TwinState.cpp`

**Purpose**: Container for all robot state data

**Data Structures**:

```cpp
class TwinState : public QObject
{
public:
    struct Pose {
        QVector3D position;      // (x, y, z) in meters
        QQuaternion orientation; // Quaternion rotation
    };
    
    struct Velocity {
        QVector3D linear;   // Linear velocity (m/s)
        QVector3D angular;  // Angular velocity (rad/s)
    };
    
    struct SensorData {
        QVariantMap imu;     // IMU data
        QVariantMap gps;     // GPS data (future)
        QVariantMap camera;  // Camera metadata
    };
    
    // Getters
    QVector3D getPosition() const;
    QQuaternion getOrientation() const;
    QVector3D getLinearVelocity() const;
    QVector3D getAngularVelocity() const;
    double getBatteryLevel() const;
    QString getRobotStatus() const;
    
    // Setters (emit stateChanged signal)
    void setPosition(const QVector3D& pos);
    void setOrientation(const QQuaternion& orient);
    void setVelocity(const QVector3D& linear, const QVector3D& angular);
    void setBatteryLevel(double level);
    void setRobotStatus(const QString& status);
    
    // Sensor data
    void updateIMUData(const QVariantMap& data);
    QVariantMap getIMUData() const;

signals:
    void stateChanged();
    void positionChanged(const QVector3D& position);
    void velocityChanged(const QVector3D& linear, const QVector3D& angular);
    void batteryLevelChanged(double level);
    
private:
    Pose m_pose;
    Velocity m_velocity;
    SensorData m_sensorData;
    double m_batteryLevel;
    QString m_robotStatus;
};
```

---

### TwinSimulator Class

**File**: `TwinSimulator.h`, `TwinSimulator.cpp`

**Purpose**: Physics simulation engine

**Key Methods**:

```cpp
class TwinSimulator : public QObject
{
public:
    TwinSimulator(TwinState* state);
    
    // Simulation control
    void start();
    void stop();
    bool isRunning() const;
    void setUpdateRate(int hz);  // Default: 50 Hz
    
    // Apply commands
    void applyVelocityCommand(double linear, double angular);
    
private slots:
    void updatePhysics();  // Called by timer
    
private:
    void integrateKinematics(double dt);
    void simulateSensors();
    
    TwinState* m_state;
    QTimer* m_updateTimer;
    QElapsedTimer m_elapsedTimer;
    double m_commandLinear;
    double m_commandAngular;
    int m_updateRate;
};
```

**Physics Model**:
- Simple differential drive kinematics
- Forward Euler integration
- Configurable update rate (10-100 Hz)

---

## UI Module

Location: `desktop-client/src/ui/`

### MainWindow Class

**File**: `MainWindow.h`, `MainWindow.cpp`

**Purpose**: Main application window with dockable layout

**Key Methods**:

```cpp
class MainWindow : public QMainWindow
{
public:
    MainWindow(WidgetManager* widgetManager,
               ROS2Interface* ros2,
               DigitalTwin* twin);
    
    // Widget management
    void addWidget(BaseWidget* widget, Qt::DockWidgetArea area);
    void removeWidget(BaseWidget* widget);

private slots:
    // Menu actions
    void onAddVideoWidget();
    void onAddCommandWidget();
    void onAddSensorWidget();
    void onAddTwinWidget();
    void onAbout();
    
private:
    void createMenus();
    void createToolBar();
    void setupDockLayout();
    
    WidgetManager* m_widgetManager;
    ROS2Interface* m_ros2Interface;
    DigitalTwin* m_digitalTwin;
    QMap<QString, QDockWidget*> m_dockWidgets;
};
```

---

### BaseWidget Class (Abstract)

**File**: `BaseWidget.h`, `BaseWidget.cpp`

**Purpose**: Base class for all modular widgets

**Interface**:

```cpp
class BaseWidget : public QWidget
{
public:
    virtual ~BaseWidget() = default;
    
    // Must be implemented by derived classes
    virtual QString displayName() const = 0;
    virtual void initialize() = 0;
    
    // Dependency injection
    void setROS2Interface(ROS2Interface* interface);
    void setDigitalTwin(DigitalTwin* twin);
    
protected:
    ROS2Interface* m_ros2Interface;
    DigitalTwin* m_digitalTwin;
    
    // Lifecycle
    virtual void closeEvent(QCloseEvent* event) override;
};
```

---

### VideoStreamWidget

**File**: `VideoStreamWidget.h`, `VideoStreamWidget.cpp`

**Purpose**: Display camera feeds

**Key Features**:
- Real-time video display
- Camera switching
- Recording capability (planned)
- FPS counter

```cpp
class VideoStreamWidget : public BaseWidget
{
public:
    QString displayName() const override { return "Video Stream"; }
    void initialize() override;
    
private slots:
    void onImageReceived(const QByteArray& data, int w, int h);
    void onRecordToggled(bool checked);
    
private:
    QLabel* m_videoLabel;
    QPushButton* m_recordButton;
    QComboBox* m_cameraSelector;
};
```

---

### CommandControlWidget

**File**: `CommandControlWidget.h`, `CommandControlWidget.cpp`

**Purpose**: Manual robot control

**Features**:
- Linear velocity slider (-1.0 to 1.0 m/s)
- Angular velocity slider (-2.0 to 2.0 rad/s)
- Emergency stop button
- Custom command input

```cpp
class CommandControlWidget : public BaseWidget
{
public:
    QString displayName() const override { return "Command Control"; }
    void initialize() override;
    
private slots:
    void onLinearChanged(int value);
    void onAngularChanged(int value);
    void onEmergencyStop();
    void onSendCommand();
    
private:
    QSlider* m_linearSlider;
    QSlider* m_angularSlider;
    QPushButton* m_stopButton;
    QLineEdit* m_commandInput;
};
```

---

### SensorDataWidget

**File**: `SensorDataWidget.h`, `SensorDataWidget.cpp`

**Purpose**: Display sensor data in table format

**Features**:
- Real-time sensor updates (10 Hz)
- Multiple sensor types
- Scrollable table

```cpp
class SensorDataWidget : public BaseWidget
{
public:
    QString displayName() const override { return "Sensor Data"; }
    void initialize() override;
    
private slots:
    void onIMUDataReceived(const QVariantMap& data);
    void updateDisplay();
    
private:
    struct IMUData {
        double accel_x, accel_y, accel_z;
        double gyro_x, gyro_y, gyro_z;
    } m_imuData;
    
    QTableWidget* m_table;
    QTimer* m_updateTimer;
};
```

---

### TwinVisualizationWidget

**File**: `TwinVisualizationWidget.h`, `TwinVisualizationWidget.cpp`

**Purpose**: Visualize digital twin state

**Features**:
- Mode selection (Sync/Simulated/Offline)
- State display (position, velocity, battery)
- Simulation control

```cpp
class TwinVisualizationWidget : public BaseWidget
{
public:
    QString displayName() const override { return "Digital Twin"; }
    void initialize() override;
    
private slots:
    void onModeChanged(int index);
    void onStateChanged();
    void onSimulationToggled(bool checked);
    
private:
    QComboBox* m_modeSelector;
    QLabel* m_positionLabel;
    QLabel* m_velocityLabel;
    QLabel* m_batteryLabel;
    QPushButton* m_simulationButton;
};
```

---

### MotionControlWidget

**File**: `MotionControlWidget.h`, `MotionControlWidget.cpp`

**Purpose**: Directional motion control

**Features**:
- 10 motion buttons (Forward, Back, Left, Right, combinations, spins)
- Visual grid layout
- Customizable speed

```cpp
class MotionControlWidget : public BaseWidget
{
public:
    enum class Motion {
        Forward, Backward, Left, Right,
        FwdLeft, FwdRight, BackLeft, BackRight,
        SpinLeft, SpinRight
    };
    
    QString displayName() const override { return "Motion Control"; }
    void initialize() override;
    
private slots:
    void onMotionTriggered(Motion motion);
    void onStopMotion();
    
private:
    QMap<Motion, QPushButton*> m_motionButtons;
    QSlider* m_speedSlider;
    double m_currentSpeed;
};
```

---

## Utilities Module

Location: `desktop-client/src/utils/`

### Logger Class

**File**: `Logger.h`, `Logger.cpp`

**Purpose**: Centralized logging (Singleton)

**Key Methods**:

```cpp
class Logger : public QObject
{
public:
    enum class Level {
        Debug,
        Info,
        Warning,
        Error,
        Critical
    };
    
    // Singleton access
    static Logger& instance();
    
    // Logging methods
    void debug(const QString& message);
    void info(const QString& message);
    void warning(const QString& message);
    void error(const QString& message);
    void critical(const QString& message);
    
    // Configuration
    void setLogFile(const QString& filePath);
    void setMinLevel(Level level);
    
private:
    Logger();  // Private constructor
    ~Logger();
    
    void writeLog(Level level, const QString& message);
    
    QFile m_logFile;
    QMutex m_mutex;  // Thread-safe
    Level m_minLevel;
};
```

**Usage Example**:
```cpp
Logger::instance().info("Application started");
Logger::instance().error("Failed to connect to ROS2");
```

---

# ROS2 Workspace (Raspberry Pi)

Location: `raspberry-pi/ros2_robot_ws/src/`

## Motor Control Package

Location: `raspberry-pi/ros2_robot_ws/src/motor_control/`

### MotorDriver Node

**Files**: 
- `include/motor_control/motor_driver.hpp`
- `src/motor_driver.cpp`

**Purpose**: Convert velocity commands to motor PWM signals

**Node Name**: `motor_driver`

**Subscribed Topics**:
- `/cmd_vel` (geometry_msgs/msg/Twist)

**Parameters**:
```cpp
// Robot parameters
double wheel_base_;     // Distance between wheels (m), default: 0.2
double wheel_radius_;   // Wheel radius (m), default: 0.05
double max_speed_;      // Maximum speed (m/s), default: 1.0

// GPIO pins for 4 motors
// Motor 1 (Front Left)
static const int MOTOR1_PIN1 = 17;
static const int MOTOR1_PIN2 = 27;
static const int MOTOR1_PWM_PIN = 22;

// Motor 2 (Front Right)
static const int MOTOR2_PIN1 = 23;
static const int MOTOR2_PIN2 = 24;
static const int MOTOR2_PWM_PIN = 25;

// Motor 3 (Rear Left)
static const int MOTOR3_PIN1 = 5;
static const int MOTOR3_PIN2 = 6;
static const int MOTOR3_PWM_PIN = 12;

// Motor 4 (Rear Right)
static const int MOTOR4_PIN1 = 13;
static const int MOTOR4_PIN2 = 19;
static const int MOTOR4_PWM_PIN = 26;
```

**Key Methods**:
```cpp
class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver();
    
private:
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void calculateMotorSpeeds(double linear, double angular,
                              double& left, double& right);
    void setMotorSpeed(int motor_num, double speed);
    void setupGPIO();
    void setPWM(int pin, double duty_cycle);
    void setDirection(int pin1, int pin2, bool forward);
};
```

**Differential Drive Kinematics**:
```cpp
// Left and right wheel velocities
left_speed = linear_x - (angular_z * wheel_base / 2.0);
right_speed = linear_x + (angular_z * wheel_base / 2.0);
```

---

## IMU Sensor Package

Location: `raspberry-pi/ros2_robot_ws/src/imu_sensor/`

### IMU Node

**Files**:
- `include/imu_sensor/imu_node.hpp`
- `src/imu_node.cpp`

**Purpose**: Read MPU6050 sensor via I2C

**Node Name**: `imu_node`

**Published Topics**:
- `/imu/data` (sensor_msgs/msg/Imu)

**Parameters**:
```cpp
double update_rate_;    // Hz, default: 50.0
int i2c_bus_;          // I2C bus number, default: 1
int i2c_address_;      // MPU6050 address, default: 0x68
```

**Key Methods**:
```cpp
class IMUNode : public rclcpp::Node
{
public:
    IMUNode();
    
private:
    void timerCallback();
    void readIMUData(double& ax, double& ay, double& az,
                     double& gx, double& gy, double& gz);
    void publishIMU(double ax, double ay, double az,
                    double gx, double gy, double gz);
    void initializeI2C();
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_;  // I2C file descriptor
};
```

**I2C Communication**:
```cpp
// Open I2C device
i2c_fd = open("/dev/i2c-1", O_RDWR);

// Set slave address
ioctl(i2c_fd, I2C_SLAVE, 0x68);

// Read data
read(i2c_fd, buffer, length);
```

---

## Encoder Odometry Package

Location: `raspberry-pi/ros2_robot_ws/src/encoder_odometry/`

### Encoder Node

**Files**:
- `include/encoder_odometry/encoder_node.hpp`
- `src/encoder_node.cpp`

**Purpose**: Calculate odometry from wheel encoders

**Node Name**: `encoder_node`

**Published Topics**:
- `/odom` (nav_msgs/msg/Odometry)

**Parameters**:
```cpp
double wheel_radius_;      // m, default: 0.05
double wheel_base_;        // m, default: 0.2
int counts_per_rev_;       // Encoder CPR, default: 20
double update_rate_;       // Hz, default: 20.0

// GPIO pins for encoders
static const int ENCODER1_A = 4;
static const int ENCODER1_B = 14;
static const int ENCODER2_A = 15;
static const int ENCODER2_B = 18;
static const int ENCODER3_A = 2;
static const int ENCODER3_B = 3;
static const int ENCODER4_A = 7;
static const int ENCODER4_B = 8;
```

**Key Methods**:
```cpp
class EncoderNode : public rclcpp::Node
{
public:
    EncoderNode();
    
private:
    void timerCallback();
    void readEncoders(int& left, int& right);
    void calculateOdometry(int left_counts, int right_counts);
    void publishOdometry();
    void setupGPIOInterrupts();
    
    // State variables
    double x_, y_, theta_;  // Robot pose
    int prev_left_counts_, prev_right_counts_;
    rclcpp::Time last_time_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

**Odometry Calculations**:
```cpp
// Distance traveled by each wheel
double left_distance = (left_counts * 2 * M_PI * wheel_radius_) / counts_per_rev_;
double right_distance = (right_counts * 2 * M_PI * wheel_radius_) / counts_per_rev_;

// Robot displacement
double distance = (left_distance + right_distance) / 2.0;
double delta_theta = (right_distance - left_distance) / wheel_base_;

// Update pose
x_ += distance * cos(theta_ + delta_theta / 2.0);
y_ += distance * sin(theta_ + delta_theta / 2.0);
theta_ += delta_theta;
```

---

## Robot Controller Package

Location: `raspberry-pi/ros2_robot_ws/src/robot_controller/`

### RobotController Node

**Files**:
- `include/robot_controller/robot_controller.hpp`
- `src/robot_controller.cpp`

**Purpose**: Main robot coordinator and safety monitor

**Node Name**: `robot_controller`

**Subscribed Topics**:
- `/cmd_vel` (geometry_msgs/msg/Twist)
- `/imu/data` (sensor_msgs/msg/Imu)
- `/odom` (nav_msgs/msg/Odometry)

**Published Topics**:
- `/cmd_vel_filtered` (geometry_msgs/msg/Twist)
- `/robot_status` (std_msgs/msg/String)

**Parameters**:
```cpp
double control_loop_rate_;      // Hz, default: 20.0
double max_linear_velocity_;    // m/s, default: 1.0
double max_angular_velocity_;   // rad/s, default: 2.0
double min_battery_voltage_;    // V, default: 7.0
```

**Key Methods**:
```cpp
class RobotController : public rclcpp::Node
{
public:
    RobotController();
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    
    bool validateVelocity(geometry_msgs::msg::Twist& cmd);
    void checkBattery();
    void publishStatus(const std::string& status);
    
    geometry_msgs::msg::Twist current_cmd_;
    sensor_msgs::msg::Imu current_imu_;
    nav_msgs::msg::Odometry current_odom_;
    
    bool emergency_stop_;
    double battery_voltage_;
};
```

**Safety Features**:
- Velocity limiting
- Emergency stop
- Battery monitoring
- Command timeout

---

# Configuration Files

## Desktop Client Package XML

**File**: `desktop-client/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>precision_farming_desktop_client</name>
  <version>1.0.0</version>
  <description>Desktop client for Precision Farming Robot</description>
  
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
</package>
```

---

## Robot Configuration YAML

**File**: `raspberry-pi/ros2_robot_ws/config/robot_config.yaml`

```yaml
motor_driver:
  ros__parameters:
    wheel_base: 0.2      # meters
    wheel_radius: 0.05   # meters
    max_speed: 1.0       # m/s

imu_sensor:
  ros__parameters:
    update_rate: 50.0    # Hz
    i2c_bus: 1
    i2c_address: 0x68

encoder_odometry:
  ros__parameters:
    wheel_radius: 0.05   # meters
    wheel_base: 0.2      # meters
    counts_per_rev: 20
    update_rate: 20.0    # Hz

robot_controller:
  ros__parameters:
    control_loop_rate: 20.0         # Hz
    max_linear_velocity: 1.0        # m/s
    max_angular_velocity: 2.0       # rad/s
    min_battery_voltage: 7.0        # V
```

---

# Build System

## Desktop Client CMakeLists

**File**: `desktop-client/CMakeLists.txt`

**Key Sections**:

```cmake
cmake_minimum_required(VERSION 3.16)
project(PrecisionFarmingDesktopClient VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt6 packages
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Network)

# Optional ROS2
option(USE_ROS2 "Build with ROS2 support" ON)

if(USE_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
endif()

# Source files
set(SOURCES ...)
set(HEADERS ...)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Network
)

if(USE_ROS2)
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp std_msgs sensor_msgs geometry_msgs
    )
endif()
```

---

## ROS2 Package CMakeLists

**Example**: `raspberry-pi/ros2_robot_ws/src/motor_control/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(motor_control)

# C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable
add_executable(motor_driver src/motor_driver.cpp)

ament_target_dependencies(motor_driver
  rclcpp
  geometry_msgs
)

target_include_directories(motor_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS motor_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## Launch File

**File**: `raspberry-pi/ros2_robot_ws/launch/robot.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_control',
            executable='motor_driver',
            name='motor_driver',
            parameters=[{
                'wheel_base': 0.2,
                'wheel_radius': 0.05,
                'max_speed': 1.0
            }]
        ),
        Node(
            package='imu_sensor',
            executable='imu_node',
            name='imu_node',
            parameters=[{
                'update_rate': 50.0,
                'i2c_bus': 1,
                'i2c_address': 0x68
            }]
        ),
        Node(
            package='encoder_odometry',
            executable='encoder_node',
            name='encoder_node',
            parameters=[{
                'wheel_radius': 0.05,
                'wheel_base': 0.2,
                'counts_per_rev': 20,
                'update_rate': 20.0
            }]
        ),
        Node(
            package='robot_controller',
            executable='robot_controller',
            name='robot_controller',
            parameters=[{
                'control_loop_rate': 20.0,
                'max_linear_velocity': 1.0,
                'max_angular_velocity': 2.0,
                'min_battery_voltage': 7.0
            }]
        )
    ])
```

---

## Quick Reference Tables

### Desktop Client Classes

| Class | Module | Purpose | Pattern |
|-------|--------|---------|---------|
| Application | Core | Main orchestrator | Facade |
| WidgetManager | Core | Widget factory/registry | Factory + Registry |
| ROS2Interface | ROS2 | ROS2 communication | Bridge |
| DigitalTwin | Twin | Twin controller | Strategy |
| TwinState | Twin | State container | Observer |
| TwinSimulator | Twin | Physics engine | Command |
| MainWindow | UI | Main window | Mediator |
| BaseWidget | UI | Widget interface | Template Method |
| Logger | Utils | Logging | Singleton |

### ROS2 Nodes

| Node | Package | Subscriptions | Publications |
|------|---------|---------------|--------------|
| motor_driver | motor_control | /cmd_vel | - |
| imu_node | imu_sensor | - | /imu/data |
| encoder_node | encoder_odometry | - | /odom |
| robot_controller | robot_controller | /cmd_vel, /imu/data, /odom | /cmd_vel_filtered, /robot_status |

### GPIO Pin Assignments

| Component | GPIO Pins | Description |
|-----------|-----------|-------------|
| Motor 1 (FL) | 17, 27, 22 | IN1, IN2, PWM |
| Motor 2 (FR) | 23, 24, 25 | IN1, IN2, PWM |
| Motor 3 (RL) | 5, 6, 12 | IN1, IN2, PWM |
| Motor 4 (RR) | 13, 19, 26 | IN1, IN2, PWM |
| Encoder 1 | 4, 14 | A, B |
| Encoder 2 | 15, 18 | A, B |
| Encoder 3 | 2, 3 | A, B |
| Encoder 4 | 7, 8 | A, B |

---

## Code Statistics

### Desktop Client
- **Total Lines**: ~3,850
- **C++ Files**: 28 (14 headers, 14 implementations)
- **Classes**: 14
- **Design Patterns**: 6

### ROS2 Workspace
- **Total Lines**: ~600
- **C++ Files**: 8 (4 headers, 4 implementations)
- **ROS2 Nodes**: 4
- **Topics**: 5

---

## See Also

- [ARCHITECTURE.md](desktop-client/docs/ARCHITECTURE.md) - System architecture
- [README.md](README.md) - Project overview
- [CHANGELOG.md](CHANGELOG.md) - Version history
- [LICENSE](LICENSE) - License information

---

*This code reference is maintained alongside the codebase. Last updated: 2025-10-29*
