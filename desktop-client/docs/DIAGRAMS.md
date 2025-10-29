# System Overview Diagrams

## 1. High-Level System Architecture

```
┌────────────────────────────────────────────────────────────────────┐
│                  Precision Farming Desktop Client                   │
│                         (Qt6 Application)                           │
├────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │                    Application (Facade)                       │ │
│  │  • Initialize all subsystems                                  │ │
│  │  • Coordinate components                                      │ │
│  │  • Manage lifecycle                                           │ │
│  └───────┬──────────────────┬──────────────────┬────────────────┘ │
│          │                  │                  │                   │
│  ┌───────▼────────┐  ┌──────▼──────┐  ┌───────▼────────┐         │
│  │ ROS2Interface  │  │DigitalTwin  │  │  UI System     │         │
│  │  (Threaded)    │  │   Module    │  │  (MainWindow)  │         │
│  ├────────────────┤  ├─────────────┤  ├────────────────┤         │
│  │• Publishers    │  │• Twin State │  │• Widget Mgr    │         │
│  │• Subscribers   │  │• Simulator  │  │• 4 Widgets     │         │
│  │• Callbacks     │  │• 3 Modes    │  │• Dock System   │         │
│  └────────────────┘  └─────────────┘  └────────────────┘         │
│          │                  │                  │                   │
└──────────┼──────────────────┼──────────────────┼───────────────────┘
           │                  │                  │
           ▼                  ▼                  ▼
     ┌──────────┐       ┌──────────┐      ┌──────────┐
     │   ROS2   │       │ Physics  │      │   User   │
     │  Network │       │  Engine  │      │Interface │
     └──────────┘       └──────────┘      └──────────┘
```

## 2. Widget System Architecture

```
                    ┌─────────────────┐
                    │  WidgetManager  │
                    │   (Factory)     │
                    └────────┬────────┘
                             │ creates
                             ▼
                    ┌─────────────────┐
                    │   BaseWidget    │
                    │   (Abstract)    │
                    └────────┬────────┘
                             │ inherits
             ┌───────────────┼───────────────┬────────────────┐
             │               │               │                │
    ┌────────▼────────┐ ┌───▼────────┐ ┌───▼──────────┐ ┌───▼────────────┐
    │ VideoStream     │ │ Command    │ │ SensorData   │ │ Twin           │
    │ Widget          │ │ Control    │ │ Widget       │ │ Visualization  │
    │                 │ │ Widget     │ │              │ │ Widget         │
    ├─────────────────┤ ├────────────┤ ├──────────────┤ ├────────────────┤
    │• Camera feed    │ │• Vel ctrl  │ │• IMU data    │ │• State display │
    │• Recording      │ │• E-stop    │ │• GPS data    │ │• Mode switch   │
    │• Multi-cam      │ │• Sliders   │ │• Battery     │ │• Simulation    │
    └─────────────────┘ └────────────┘ └──────────────┘ └────────────────┘
```

## 3. Data Flow Diagram

```
┌─────────────┐
│ Real Robot  │
└──────┬──────┘
       │ ROS2 Topics
       ▼
┌──────────────────────────────────────────────────────────┐
│              ROS2Interface (Separate Thread)              │
├──────────────────────────────────────────────────────────┤
│  Subscribers:                 Publishers:                 │
│  • /camera/image_raw    ─→    /cmd_vel                   │
│  • /imu/data            ─→    /robot_command             │
│  • /robot_status                                          │
└───────┬──────────────────────────────┬───────────────────┘
        │ Qt Signals                   │ Qt Signals
        │ (Thread-safe)                │
        ▼                              ▼
┌───────────────────┐          ┌──────────────────┐
│  Digital Twin     │          │   UI Widgets     │
│                   │          │                  │
│  ┌─────────────┐ │          │ ┌──────────────┐ │
│  │ TwinState   │ │◄─────────┤ │ Video Widget │ │
│  │             │ │          │ └──────────────┘ │
│  │ • Position  │ │          │ ┌──────────────┐ │
│  │ • Velocity  │ │◄─────────┤ │Control Widget│ │
│  │ • Sensors   │ │          │ └──────────────┘ │
│  └─────────────┘ │          │ ┌──────────────┐ │
│  ┌─────────────┐ │◄─────────┤ │Sensor Widget │ │
│  │ Simulator   │ │          │ └──────────────┘ │
│  │             │ │          │ ┌──────────────┐ │
│  │ • Physics   │ │◄─────────┤ │ Twin Widget  │ │
│  │ • Sensors   │ │          │ └──────────────┘ │
│  └─────────────┘ │          │                  │
└───────────────────┘          └──────────────────┘
```

## 4. Thread Model

```
┌─────────────────────────────────────────────────────────┐
│                    Process Threads                       │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │              │  │              │  │              │ │
│  │ Main Thread  │  │ ROS2 Thread  │  │ Timer Thread │ │
│  │  (Qt GUI)    │  │  (Spin)      │  │ (Simulator)  │ │
│  │              │  │              │  │              │ │
│  ├──────────────┤  ├──────────────┤  ├──────────────┤ │
│  │• UI Updates  │  │• spin_some() │  │• Physics     │ │
│  │• Widgets     │  │• Callbacks   │  │• Update 50Hz │ │
│  │• Events      │  │• Pub/Sub     │  │• State calc  │ │
│  └───┬──────────┘  └───┬──────────┘  └───┬──────────┘ │
│      │                 │                 │             │
│      │    Signals/     │    Signals/     │             │
│      │◄────Slots──────►│◄────Slots──────►│             │
│      │  (Queued)       │  (Queued)       │             │
│                                                          │
└─────────────────────────────────────────────────────────┘

Thread Safety Mechanisms:
• Qt Signal/Slot (Queued connections)
• QMutex (in Logger)
• No direct cross-thread calls
```

## 5. Digital Twin State Machine

```
                    ┌──────────────┐
                    │   Offline    │
                    │   (Initial)  │
                    └───┬──────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
        ▼               ▼               ▼
┌───────────────┐ ┌─────────────┐ ┌────────────┐
│ Synchronized  │ │  Simulated  │ │  Offline   │
├───────────────┤ ├─────────────┤ ├────────────┤
│• Mirror robot │ │• Run physics│ │• No data   │
│• ROS2 data    │ │• Simulated  │ │• Idle      │
│• Real-time    │ │  sensors    │ │            │
└───────┬───────┘ └──────┬──────┘ └─────┬──────┘
        │                │              │
        └────────────────┼──────────────┘
                         │
                    User selects
                    mode from UI
```

## 6. Build & Deployment Flow

```
┌────────────────┐
│  Source Code   │
│  (28 files)    │
└────────┬───────┘
         │
         ▼
┌────────────────┐
│   build.sh     │
│  (Build Script)│
└────────┬───────┘
         │
         ├─→ Check ROS2
         ├─→ Run CMake
         └─→ Run Make
         │
         ▼
┌────────────────────┐
│  Compilation       │
│  • Qt MOC          │
│  • C++ Compiler    │
│  • Linker          │
└────────┬───────────┘
         │
         ▼
┌────────────────────┐
│   Executable       │
│ PrecisionFarming   │
│ DesktopClient      │
└────────┬───────────┘
         │
         ▼
┌────────────────────┐
│   Run Application  │
│   • Load Qt libs   │
│   • Init ROS2      │
│   • Show UI        │
└────────────────────┘
```

## 7. Widget Lifecycle

```
┌─────────────────┐
│ User Request    │
│ "Add Widget"    │
└────────┬────────┘
         │
         ▼
┌─────────────────────┐
│ WidgetManager       │
│ createWidget(type)  │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ Widget Constructor  │
│ setupUI()           │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ setROS2Interface()  │
│ setDigitalTwin()    │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ initialize()        │
│ Connect signals     │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ Add to MainWindow   │
│ as QDockWidget      │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│   Active State      │
│   • Receive data    │
│   • Update UI       │
│   • Send commands   │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ User closes widget  │
│ closeEvent()        │
└────────┬────────────┘
         │
         ▼
┌─────────────────────┐
│ Cleanup & Destroy   │
└─────────────────────┘
```

## 8. ROS2 Communication Flow

```
Robot Hardware
     │
     ▼
┌─────────────────────┐
│  ROS2 Network       │
│  (DDS)              │
└──────────┬──────────┘
           │
    Topics Published:
    • /camera/image_raw
    • /imu/data  
    • /robot_status
           │
           ▼
┌──────────────────────────┐
│  ROS2Interface Node      │
│  (Separate Thread)       │
├──────────────────────────┤
│  Subscribers:            │
│  ┌────────────────────┐ │
│  │ imageCallback()    │─┼─→ emit imageReceived()
│  │ imuCallback()      │─┼─→ emit imuDataReceived()
│  │ statusCallback()   │─┼─→ emit robotStatusReceived()
│  └────────────────────┘ │
│                          │
│  Publishers:             │
│  ┌────────────────────┐ │
│  │ m_velocityPublisher│◄┼─── publishVelocityCommand()
│  │ m_commandPublisher │◄┼─── publishRobotCommand()
│  └────────────────────┘ │
└──────────────────────────┘
           │
    Qt Signals (cross-thread)
           │
           ▼
    ┌──────────────┐
    │   Widgets    │
    │   • Video    │
    │   • Control  │
    │   • Sensors  │
    │   • Twin     │
    └──────────────┘
```

## 9. Memory Management

```
Application
├── unique_ptr<ROS2Interface>      [Owner: Application]
├── unique_ptr<DigitalTwin>        [Owner: Application]
│   ├── unique_ptr<TwinState>      [Owner: DigitalTwin]
│   └── unique_ptr<TwinSimulator>  [Owner: DigitalTwin]
├── unique_ptr<MainWindow>         [Owner: Application]
│   └── unique_ptr<WidgetManager>  [Owner: Application]
│       └── QMap<QString, BaseWidget*>  [Qt parent-child]
│           ├── VideoStreamWidget*      [Parent: QDockWidget]
│           ├── CommandControlWidget*   [Parent: QDockWidget]
│           ├── SensorDataWidget*       [Parent: QDockWidget]
│           └── TwinVisualizationWidget*[Parent: QDockWidget]
└── Logger (Singleton)              [Static lifetime]

Memory Safety:
✓ RAII for all resources
✓ Smart pointers for ownership
✓ Qt parent-child for widgets
✓ No manual delete needed
```

---

These diagrams provide a complete visual understanding of the system architecture, 
data flow, threading model, and component interactions.
