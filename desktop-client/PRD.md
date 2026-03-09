# WeedX Desktop Client — UI Overhaul PRD

| Field | Value |
|---|---|
| Version | 1.0 — Draft |
| Date | March 2026 |
| Project | Precision Farming Robot — Final Year Project |
| Scope | Desktop Client UI Overhaul (Qt6 / C++17) |
| Audience | Developer / Researcher |

---

## 1. Overview

This document defines requirements, design decisions, and acceptance criteria for the WeedX Desktop Client UI overhaul. The goal is to replace the current default-Qt visual treatment with a focused, production-quality operator interface that is visually consistent with the WeedX mobile application.

This is **not** a feature expansion. It is a deliberate reduction and refinement — removing clutter, consolidating redundant panels, and establishing a consistent design language across the WeedX product family.

### 1.1 Problem Statement

The current desktop client has the following issues:

- Default Qt grey styling with no visual hierarchy — all panels look identical in weight and importance
- Two separate control panels (Motion Control and Command & Control) with overlapping responsibilities
- Sensor data rendered as a raw scrolling table — unreadable at a glance during active operation
- Large dead space in the centre of the layout with no purpose
- No persistent status indicators — connection state, robot mode, and simulation state buried in menus
- Video feed is undersized relative to its operational importance
- No visual consistency with the mobile app the robot operator may also be using

### 1.2 Goals

- Mirror the mobile app's visual language: dark background, deep forest green accents, card-based surfaces
- Make the video feed the dominant element — it is the primary operator view
- Consolidate Motion Control + Command & Control into a single sidebar panel
- Replace the raw sensor table with a glanceable telemetry bar (matching mobile's "Today's Summary" style)
- Surface connection and robot status persistently without menu navigation
- Deliver a production-ready UI suitable for ongoing field use

### 1.3 Non-Goals

- Adding new widget types not already in the codebase
- Redesigning ROS2 integration or communication protocols
- Changing Digital Twin simulation logic
- Mobile or web versions — this PRD is desktop only
- Weed analytics charts, field mapping, GPS visualisation (future PRD)

---

## 2. Design Reference — Mobile App Alignment

The WeedX mobile app (Android) establishes the visual identity this desktop overhaul must match. Key elements to carry over:

| Mobile Element | Desktop Equivalent |
|---|---|
| Dark background (`#0D1A0C`) with deep green surfaces | Main window + all panel backgrounds |
| Bright green accent (`#52C44A`) on live/active states | ROS2 connected badge, live value labels |
| Card-based sections (Live Monitoring, Today's Summary) | Sidebar sections separated by dividers, stat cards for position |
| "Live Monitoring — Robot Active" banner | Persistent status badge in title bar |
| Today's Summary metric rows (Weeds Detected, Herbicide Used, Area Covered) | Telemetry bar metric cells (Accel X/Y/Z, Gyro X/Y, Robot Status) |
| Recent Images grid | Video stream panel (full-bleed, dominant) |
| Minimal, all-caps section labels in muted green | Panel titles: 10px ALL CAPS, muted green |
| White text on dark surfaces for values | Live numeric values in `#52C44A` monospace |
| Rounded cards with subtle borders | Widget panels with `border-radius: 8px`, `#2A3D29` borders |

The desktop is an operator-focused tool so it prioritises density and control over the mobile's monitoring-first layout, but the visual language (colours, type, card treatment) must feel like the same product family.

---

## 3. Layout Architecture

### 3.1 Three-Zone Layout

The redesigned layout replaces the freeform dockable workspace with a structured three-zone layout. Qt dock widgets remain in the implementation for flexibility, but the default layout is enforced.

```
┌─────────────────────────────────────┬──────────────┐
│                                     │              │
│          VIDEO STREAM               │   SIDEBAR    │
│          (~75% width)               │  (~25% width)│
│                                     │              │
│  Camera feed — dominant element     │  Status      │
│  Source tabs in panel header        │  Position    │
│  Overlays: topic, fps, status       │  Motion ctrl │
│                                     │  Stop btns   │
├─────────────────────────────────────┴──────────────┤
│  TELEMETRY BAR  —  6 metric cells  —  fixed ~160px │
└────────────────────────────────────────────────────┘
```

| Zone | Size | Content |
|---|---|---|
| **Main (Video)** | ~75% width, full height | Full-bleed video feed. Camera source tabs embedded in header. Detection overlays rendered on feed. |
| **Sidebar (Right)** | ~25% width, full height | Scrollable single panel: ROS2 status → robot position → motion D-pad → speed sliders → stop actions |
| **Telemetry Bar (Bottom)** | Full width, fixed ~160px | Six metric cells: Accel X/Y/Z, Gyro X/Y, Robot Status |

### 3.2 Title Bar / Menu Bar

The native Qt menu bar is retained (File, Widgets, ROS2, Simulation, Help). Two persistent status badges are added to the right side — matching the mobile app's "Robot Active" live banner:

- **ROS2 badge** — green pulsing dot + "Connected" or grey dot + "Offline"
- **Simulation badge** — "Simulation On" or "Simulation Off"

These replace the need to open menus to check current state.

---

## 4. Widget Specifications

Five widgets are designated core. The standalone Coordinates dock is removed — position data surfaces inline in the sidebar.

### 4.1 Video Stream

- Full-height panel, fills available width
- Camera source selector embedded in panel header as tab buttons: **Raw** / **Detection** / **Depth**
- Overlay chips (matching mobile's card style):
  - Top-left: current ROS2 topic name + live/offline indicator with pulsing dot
  - Top-right: stream resolution + frame rate when active
  - Bottom-left: robot status string
- BGR8 → RGB conversion transparent to operator
- Placeholder shown when no stream active (not blank black rectangle)
- Camera topic switchable at runtime via `ROS2Interface::switchCameraTopic()`

### 4.2 Motion Control *(merged with Command & Control)*

The current two-panel split is eliminated. One sidebar widget houses everything:

- 3×3 D-pad grid — directional arrows, blank centre cell
- **Speed** slider: 0–100%, live value label
- **Curve Radius** slider: 0.00–3.00 m, live value label
- **Spin Left** / **Spin Right** buttons
- Publishes to `/cmd_vel` — logic unchanged, only container changes

### 4.3 Command & Control *(embedded in sidebar, not a separate dock)*

- **Stop** button — muted red border, transparent background
- **Emergency Stop** button — solid red `#E05050`, high contrast white text, always visible at sidebar bottom
- Linear Speed + Angular Speed shown as read-only labels updated from `/cmd_vel` feedback
- Both buttons permanently visible (sticky bottom of sidebar, not scrollable away)

### 4.4 Sensor Data / Telemetry Bar

Replaces the scrolling `QTableWidget` with a fixed 6-cell horizontal grid — styled like the mobile app's "Today's Summary" row:

| Cell | Topic | Unit |
|---|---|---|
| Accel X | `/imu/data` | m/s² |
| Accel Y | `/imu/data` | m/s² |
| Accel Z | `/imu/data` | m/s² |
| Gyro X | `/imu/data` | rad/s |
| Gyro Y | `/imu/data` | rad/s |
| Robot Status | `/robot_status` | string |

Each cell contains: metric name (small, muted), numeric value (large, monospace, green), unit label, and a 4px proportional mini-bar indicator.

### 4.5 Digital Twin

- Dock panel creation remains disabled in MainWindow (as currently)
- Mode selector (Synchronized / Simulated / Offline) exposed via **Simulation** menu only
- Active mode reflected in the simulation badge in the title bar
- No visual panel in the default layout

---

## 5. Visual Design System

### 5.1 Colour Palette

Derived directly from the WeedX mobile app colour scheme:

| Token | Hex | Usage |
|---|---|---|
| `--bg` | `#0D1A0C` | Main window, video background |
| `--surface` | `#142213` | Sidebar, panel headers, menu bar |
| `--surface-2` | `#1B2E1A` | Stat cards, input fields |
| `--surface-3` | `#223B21` | Hover states, active elements |
| `--border` | `#2A3D29` | All panel and widget borders |
| `--green` | `#3D8B37` | Slider fill, active button backgrounds |
| `--green-lit` | `#52C44A` | Live values, connected status dot |
| `--green-dim` | `#2A6126` | Connect button background |
| `--text` | `#E4F0E3` | Primary text |
| `--text-dim` | `#7A9B79` | Labels, secondary text |
| `--text-mute` | `#4A6B49` | Section titles, placeholders |
| `--red` | `#E05050` | Emergency Stop |
| `--amber` | `#D4882A` | Warning states |

### 5.2 Typography

Font stack applied via `QFont` before any widget is constructed:

```
"Inter", "Segoe UI", "Ubuntu", sans-serif
```

| Role | Size | Weight | Colour | Notes |
|---|---|---|---|---|
| Panel titles | 10px | 600 | `--text-mute` | ALL CAPS, 1px letter-spacing |
| Body / labels | 12px | 400 | `--text-dim` | — |
| Live numeric values | 14–22px | 500 | `--green-lit` | Courier New / monospace |
| Buttons | 12px | 600 | varies | 0.5px letter-spacing |
| Overlay badges | 10px | 700 | varies | colour-coded |

### 5.3 Component Rules

- **Panels** — no border-radius (full bleed to edges)
- **Cards / stat cells** — `border-radius: 8–10px`, `1px solid --border`
- **Buttons** — `border-radius: 7px`, one-step background change on hover, slightly darker on press
- **Sliders** — 4px groove, 14px circular handle with `--bg` inner border
- **Scrollbars** — 6px width, `--border` track, `--green` on hover
- **Status dots** — 6–7px circle, `box-shadow: 0 0 6px --green-lit` when live

### 5.4 Stylesheet Delivery

All styling delivered as a single `theme.qss` applied globally at startup:

```cpp
// In Application::initialize() — before MainWindow is shown
QFile f(":/theme.qss");
f.open(QFile::ReadOnly | QFile::Text);
QTextStream ts(&f);
qApp->setStyleSheet(ts.readAll());
```

Add `theme.qss` to your `.qrc` file. Apply `QFont` to `qApp` before the stylesheet so the font stack takes effect consistently across Linux distributions.

Object names used for targeted button styling in QSS:

```cpp
stopButton->setObjectName("stopButton");
emergencyStopButton->setObjectName("emergencyStopButton");
connectButton->setObjectName("connectButton");
```

---

## 6. Requirements

> Priority scale: **P0** = must have · **P1** = should have · **P2** = nice to have

### 6.1 Layout

| ID | Priority | Requirement | Status |
|---|---|---|---|
| L-01 | P0 | Default layout uses three-zone structure: main video, right sidebar, bottom telemetry bar | Open |
| L-02 | P0 | Video panel occupies ≥70% of horizontal screen space at 1920×1080 | Open |
| L-03 | P0 | Sidebar is a single scrollable panel — no separate Motion Control and Command & Control docks | Open |
| L-04 | P0 | Telemetry bar is fixed ~160px height and always visible | Open |
| L-05 | P1 | Dock widgets remain in codebase but default layout is enforced on first launch | Open |
| L-06 | P1 | Layout state saved and restored between sessions via `QSettings` | Open |

### 6.2 Video Stream

| ID | Priority | Requirement | Status |
|---|---|---|---|
| V-01 | P0 | Video panel fills available space with correct aspect ratio, no letterboxing artefacts | Open |
| V-02 | P0 | Camera source tabs (Raw / Detection / Depth) embedded in panel header, not a separate dropdown | Open |
| V-03 | P0 | BGR8 → RGB conversion transparent to operator — colour correct by default | Open |
| V-04 | P0 | Top-left overlay chip shows current ROS2 topic name and live/offline indicator | Open |
| V-05 | P1 | Top-right overlay chip shows stream resolution and frame rate when active | Open |
| V-06 | P1 | Placeholder graphic shown when no stream active — not a blank black rectangle | Open |

### 6.3 Motion Control

| ID | Priority | Requirement | Status |
|---|---|---|---|
| M-01 | P0 | D-pad rendered as 3×3 grid with clearly styled directional buttons and blank centre | Open |
| M-02 | P0 | Speed slider 0–100%, labelled, shows current value in real time | Open |
| M-03 | P0 | Curve Radius slider 0–3.00 m, labelled, shows current value in real time | Open |
| M-04 | P0 | Spin Left / Spin Right buttons present and publishing correct `/cmd_vel` payloads | Open |
| M-05 | P0 | Stop and Emergency Stop buttons permanently visible at sidebar bottom | Open |
| M-06 | P0 | Emergency Stop has solid red background, visually distinct from all other buttons | Open |
| M-07 | P1 | Motion controls disabled (greyed out) when ROS2 is not connected | Open |

### 6.4 Telemetry Bar

| ID | Priority | Requirement | Status |
|---|---|---|---|
| T-01 | P0 | Telemetry bar shows six fixed metric cells: Accel X/Y/Z, Gyro X/Y, Robot Status | Open |
| T-02 | P0 | Each numeric cell shows: metric name, current value, unit, and a 4px proportional mini-bar | Open |
| T-03 | P0 | Robot Status cell shows latest string from `/robot_status` topic | Open |
| T-04 | P1 | Numeric values update at ROS2 subscription rate (10ms spin loop) | Open |
| T-05 | P2 | Values outside expected range trigger amber colouring on the affected cell | Open |

### 6.5 Status & Connection

| ID | Priority | Requirement | Status |
|---|---|---|---|
| S-01 | P0 | ROS2 connection state shown as persistent badge in title bar — always visible | Open |
| S-02 | P0 | Simulation mode state shown as persistent badge in title bar — always visible | Open |
| S-03 | P0 | Robot position (X, Y in metres) displayed inline in sidebar — no separate Coordinates dock | Open |
| S-04 | P1 | Connect/Disconnect accessible from both ROS2 menu and a button in the sidebar | Open |

### 6.6 Visual / Styling

| ID | Priority | Requirement | Status |
|---|---|---|---|
| D-01 | P0 | `theme.qss` applied globally at startup — no default Qt grey visible in the final UI | Open |
| D-02 | P0 | All colour values conform to the palette in Section 5.1 | Open |
| D-03 | P0 | `objectName` set on stopButton, emergencyStopButton, connectButton | Open |
| D-04 | P0 | Font stack set via `QFont` before window construction | Open |
| D-05 | P1 | Panel titles use 10px ALL CAPS label style | Open |
| D-06 | P1 | Hover and pressed states defined for all interactive controls | Open |
| D-07 | P2 | 6px styled scrollbars — default OS scrollbars not visible | Open |

---

## 7. Implementation Notes

### 7.1 Consolidating Motion Control + Command & Control

The current codebase has two separate dock widgets. To consolidate:

1. Create a unified `SidebarWidget` (or extend `CommandControlWidget`) that houses all motion controls
2. Move D-pad, speed sliders, and spin buttons from `MotionControlWidget` into this widget
3. Move Stop and Emergency Stop buttons to the sticky bottom of the same widget
4. Remove the now-redundant dock registration from `WidgetManager`
5. `/cmd_vel` publish logic is unchanged — only the container moves

### 7.2 Surfacing Position Inline

Remove the `CoordinatesWidget` dock from the default layout. Instead, subscribe to `/coordinates` directly in the sidebar widget and update two `QLabel` values (X, Y). No data is lost — it just appears inline rather than in a separate panel.

### 7.3 Replacing the Sensor Table

Replace the `QTableWidget` in `SensorDataWidget` with a `QHBoxLayout` of six `QFrame` cells. Each cell contains:

- `QLabel` — metric name (small, muted, ALL CAPS)
- `QLabel` — numeric value (large, monospace, `--green-lit`)
- `QLabel` — unit (small, muted)
- `QProgressBar` — styled to 4px height, proportional to value range

This is a **presentation change only** — the underlying `/imu/data` and `/robot_status` subscriptions are unchanged.

### 7.4 Digital Twin Panel

The `DigitalTwin` widget type exists but UI creation is currently disabled in `MainWindow`. It stays disabled as a dock. The `Simulation` menu actions surface twin controls. The active mode (`Synchronized` / `Simulated` / `Offline`) is reflected in the simulation badge in the title bar.

### 7.5 Persistent Status Badges

Add two `QLabel` or custom `QWidget` badges to the right side of `QMenuBar` using `QMenuBar::setCornerWidget()` or by adding them to a `QToolBar` pinned below the menu bar:

```cpp
// Example — in MainWindow::setupMenuBar()
auto* ros2Badge = new StatusBadge("ROS2 Offline", this);
auto* simBadge  = new StatusBadge("Simulation Off", this);
menuBar()->setCornerWidget(createBadgeContainer(ros2Badge, simBadge));
```

Connect these to existing signals from `ROS2Interface` and `DigitalTwin`.

---

## 8. Acceptance Criteria

All P0 requirements must pass before the overhaul is considered complete.

| # | Check | Pass Condition |
|---|---|---|
| 1 | No default Qt grey visible | Launch app — all surfaces use the dark green palette |
| 2 | Video panel is dominant | Video occupies ≥70% horizontal space at 1920×1080 |
| 3 | Single sidebar panel | Only one right-side dock exists; motion and stop controls in same widget |
| 4 | Stop buttons always visible | Stop and Emergency Stop visible regardless of sidebar scroll position |
| 5 | Emergency Stop is distinct | Solid red background, clearly different from all other buttons |
| 6 | Telemetry bar replaces sensor table | No scrolling table visible; six metric cells present in bottom bar |
| 7 | Position shown in sidebar | X and Y update in sidebar when `/coordinates` messages received |
| 8 | ROS2 status visible without menus | Connection state readable in title bar without opening any menu |
| 9 | Camera source switching works | Switching Raw / Detection / Depth triggers correct topic change |
| 10 | Standalone mode builds and runs | Build without ROS2 sourced — app launches, stylesheet applied, no crashes |

---

## 9. Out of Scope & Future Considerations

### 9.1 Out of Scope for this PRD

- Weed detection analytics (heatmaps, detection count history)
- Field mapping or GPS coordinate visualisation
- Battery level monitoring
- Alert / notification system
- Any changes to ROS2 topic structure or message types
- Digital Twin 3D visualisation

### 9.2 Future Backlog

Items for potential future PRDs once the core overhaul is stable:

- **Detection overlay** — bounding boxes + confidence scores drawn on the video feed in real time (mirrors mobile's live weed detection visualisation)
- **Weed count cell** — add a 7th telemetry cell showing detections in the current session (mirrors mobile's "Weeds Detected: 26" summary card)
- **Session recording** — log video and telemetry to disk with a single toggle
- **Keyboard shortcuts** — WASD / arrow keys mapped to D-pad commands
- **Light theme** — for indoor / presentation use

---

## 10. Atomic Implementation Tasks (GitHub-Committable)

> Notes:
> - Each task is intentionally scoped to a single logical change and can be shipped as one commit.
> - This repository currently has no automated UI test suite; test cases below are defined as build + manual verification scenarios unless otherwise noted.
> - Commit order preserves dependency flow (theme foundation first, then layout, then widget-level changes).

### Task 1 — Add global theme resource and startup application hook

**Commit goal**
- Add `theme.qss` to Qt resources and apply it globally during app initialization.
- Apply app-wide font stack (`Inter`, `Segoe UI`, `Ubuntu`, sans-serif) before window construction.

**Primary files**
- `src/core/Application.cpp`
- `src/core/Application.h` (if helper needed)
- `theme.qss` (new)
- Resource file (`.qrc`) where UI resources are declared
- `CMakeLists.txt` (only if resource file wiring requires update)

**PRD coverage**
- D-01, D-04

**Test case (required)**
- **Name:** Global theme bootstrap applies without ROS2.
- **Type:** Build + manual smoke test.
- **Steps:**
  1. Build in standalone mode (without ROS2 sourced).
  2. Launch app.
  3. Inspect main window, menu bar, and first-render panels.
- **Expected:** No default Qt grey is visible; dark palette and new font are applied from first frame; app launches without crash.

---

### Task 2 — Define complete WeedX QSS tokens and core component styling

**Commit goal**
- Populate `theme.qss` with palette tokens and core styles for panels, cards, buttons, sliders, scrollbars, and status dots.
- Add object-name-targeted styles for `#stopButton`, `#emergencyStopButton`, and `#connectButton`.

**Primary files**
- `theme.qss`

**PRD coverage**
- D-02, D-03, D-06, D-07 (P2)

**Test case (required)**
- **Name:** Critical control styling and interaction states.
- **Type:** Manual visual/interaction test.
- **Steps:**
  1. Open app and locate connect, stop, and emergency stop controls.
  2. Hover and press each control.
  3. Scroll any scrollable panel.
- **Expected:** Colors match token palette; emergency stop is solid red and visually distinct; hover/pressed states are present; scrollbar styling is custom and visible.

---

### Task 3 — Enforce default three-zone window layout shell

**Commit goal**
- Restructure default `MainWindow` layout to: dominant video region, right sidebar region, fixed bottom telemetry region.
- Keep dock infrastructure in codebase but enforce this as default first-launch arrangement.

**Primary files**
- `src/ui/MainWindow.cpp`
- `src/ui/MainWindow.h`

**PRD coverage**
- L-01, L-02, L-04, L-05

**Test case (required)**
- **Name:** 1920×1080 default layout proportions.
- **Type:** Manual layout verification.
- **Steps:**
  1. Launch app at 1920×1080.
  2. Reset layout to default (if action exists) or use fresh config.
  3. Measure approximate widths of video vs sidebar.
- **Expected:** Video area is ≥70% width; sidebar ≈25%; telemetry bar remains visible with fixed ~160px height.

---

### Task 4 — Add layout persistence via QSettings

**Commit goal**
- Persist and restore window/dock layout state between sessions.
- Preserve new three-zone default on first run; restore user-customized state on subsequent runs.

**Primary files**
- `src/ui/MainWindow.cpp`
- `src/ui/MainWindow.h`

**PRD coverage**
- L-06

**Test case (required)**
- **Name:** Layout survives restart.
- **Type:** Manual persistence test.
- **Steps:**
  1. Resize sidebar width and reposition allowed panes.
  2. Close app fully.
  3. Relaunch app.
- **Expected:** Previous layout state is restored exactly; first-run default enforcement remains intact for clean profiles.

---

### Task 5 — Implement persistent title/menu bar status badges

**Commit goal**
- Add always-visible ROS2 and Simulation badges at menu/title bar right side.
- Connect badge state to existing ROS2 connection and Digital Twin mode signals.

**Primary files**
- `src/ui/MainWindow.cpp`
- `src/ui/MainWindow.h`
- Optional new small helper widget in `src/ui/widgets/` (if needed)

**PRD coverage**
- S-01, S-02

**Test case (required)**
- **Name:** Badge state updates without opening menus.
- **Type:** Manual interaction test.
- **Steps:**
  1. Start app disconnected.
  2. Connect ROS2 from menu; then disconnect.
  3. Toggle simulation mode through Simulation menu.
- **Expected:** ROS2 badge transitions Offline/Connected immediately; simulation badge reflects current mode continuously.

---

### Task 6 — Refactor video panel to dominant operator view with overlays

**Commit goal**
- Ensure video fills main area with correct aspect-ratio handling.
- Move camera source controls to header tabs (Raw/Detection/Depth).
- Add overlay chips for topic/live state, resolution/fps, and robot status.
- Add non-black placeholder when no active stream.

**Primary files**
- Existing video widget implementation in `src/ui/widgets/`
- `src/ros2/ROS2Interface.cpp` / `src/ros2/ROS2Interface.h` (only if topic switch wiring needs adjustment)
- `src/ui/MainWindow.cpp` (if hosting changes are needed)

**PRD coverage**
- V-01, V-02, V-04, V-05, V-06

**Test case (required)**
- **Name:** Camera source switching and overlay correctness.
- **Type:** Manual functional test.
- **Steps:**
  1. Open video panel and switch Raw → Detection → Depth.
  2. Observe top-left topic chip and live/offline indicator.
  3. With active stream, observe resolution/fps chip.
  4. Stop stream/input and observe placeholder.
- **Expected:** Topic switches correctly at runtime via `ROS2Interface::switchCameraTopic()`; overlays update correctly; placeholder appears when inactive.

---

### Task 7 — Keep BGR8→RGB correctness explicit in stream pipeline

**Commit goal**
- Confirm and lock-in color conversion behavior so operator always sees correct colors.
- Add/adjust logging around unsupported encodings if needed.

**Primary files**
- `src/ros2/ROS2Interface.cpp`
- `src/ros2/ROS2Interface.h` (if interface update is required)

**PRD coverage**
- V-03

**Test case (necessary if code touched)**
- **Name:** Color correctness for BGR8 stream.
- **Type:** Manual visual validation with known color target.
- **Steps:**
  1. Publish a test frame containing known red/green/blue regions.
  2. View stream in desktop client.
- **Expected:** Rendered colors match source target; no swapped red/blue channels.

---

### Task 8 — Create unified sidebar widget (Motion + Command)

**Commit goal**
- Consolidate Motion Control and Command & Control into one right-side scrollable container.
- Keep `/cmd_vel` command semantics unchanged.
- Ensure stop controls are anchored/sticky at sidebar bottom.

**Primary files**
- Motion/command widget source files in `src/ui/widgets/`
- `src/core/WidgetManager.cpp`
- `src/core/WidgetManager.h`
- `src/ui/MainWindow.cpp` (dock placement)
- `CMakeLists.txt` (if file additions/removals occur)

**PRD coverage**
- L-03, M-04, M-05

**Test case (required)**
- **Name:** Single sidebar architecture and sticky stop controls.
- **Type:** Manual structural/UX test.
- **Steps:**
  1. Launch app and inspect right side.
  2. Verify only one sidebar dock exists for control operations.
  3. Scroll sidebar content.
- **Expected:** Motion + command controls are in one panel; Stop and Emergency Stop stay visible at bottom and never scroll away.

---

### Task 9 — Implement D-pad and control primitives in unified sidebar

**Commit goal**
- Render 3×3 D-pad with blank centre.
- Add Speed slider (0–100%) with live value label.
- Add Curve Radius slider (0.00–3.00 m) with live value label.
- Add Spin Left/Spin Right buttons.

**Primary files**
- Unified sidebar widget implementation in `src/ui/widgets/`

**PRD coverage**
- M-01, M-02, M-03, M-04

**Test case (required)**
- **Name:** Motion controls emit correct command behavior.
- **Type:** Manual functional test (with ROS2 monitor).
- **Steps:**
  1. Move each D-pad direction and spin controls.
  2. Sweep both sliders and observe live labels.
  3. Inspect `/cmd_vel` messages.
- **Expected:** Direction/spin commands map correctly; slider ranges enforce spec limits; label values update in real time.

---

### Task 10 — Add distinct Stop and Emergency Stop behavior/styling hooks

**Commit goal**
- Ensure `stopButton` and `emergencyStopButton` object names are set in code.
- Confirm emergency stop style remains high-contrast red regardless of generic button theme.

**Primary files**
- Unified sidebar widget source in `src/ui/widgets/`
- `theme.qss`

**PRD coverage**
- M-06, D-03

**Test case (required)**
- **Name:** Emergency stop visual and action priority check.
- **Type:** Manual safety/UX test.
- **Steps:**
  1. Verify emergency stop button is always visible and red.
  2. Trigger Emergency Stop during robot motion simulation.
- **Expected:** Emergency control is visually unique and immediately accessible; stop command behavior executes as designed.

---

### Task 11 — Disable motion controls when ROS2 is offline

**Commit goal**
- Bind ROS2 connection state to enabled/disabled state of motion controls.
- Keep stop/emergency visibility unaffected.

**Primary files**
- Unified sidebar widget source in `src/ui/widgets/`
- `src/ui/MainWindow.cpp` or signal wiring location

**PRD coverage**
- M-07

**Test case (required)**
- **Name:** Offline safety gating.
- **Type:** Manual state test.
- **Steps:**
  1. Start disconnected.
  2. Verify motion controls are disabled.
  3. Connect ROS2 and verify controls re-enable.
  4. Disconnect again.
- **Expected:** Enablement tracks connection state reliably without requiring app restart.

---

### Task 12 — Surface coordinates inline in sidebar and remove standalone dock from default UI

**Commit goal**
- Display X/Y position labels inline in sidebar from `/coordinates` updates.
- Remove Coordinates widget from default active layout (dock code may remain available if desired for fallback, but not shown by default).

**Primary files**
- Unified sidebar widget source in `src/ui/widgets/`
- `src/core/WidgetManager.cpp`
- `src/ui/MainWindow.cpp`

**PRD coverage**
- S-03

**Test case (required)**
- **Name:** Inline coordinate display update.
- **Type:** Manual data-flow test.
- **Steps:**
  1. Publish changing `/coordinates` values.
  2. Observe X/Y labels in sidebar.
  3. Confirm no separate coordinates dock appears in default layout.
- **Expected:** Sidebar values track latest coordinates; standalone coordinates panel is absent by default.

---

### Task 13 — Replace sensor table with fixed 6-cell telemetry bar

**Commit goal**
- Replace `QTableWidget` sensor presentation with fixed horizontal card/cell layout.
- Include metric label, value, unit, and 4px mini-bar per cell.

**Primary files**
- Sensor/telemetry widget implementation in `src/ui/widgets/`
- `src/ui/MainWindow.cpp` (placement to bottom telemetry bar)

**PRD coverage**
- T-01, T-02

**Test case (required)**
- **Name:** Telemetry bar structure and visual format.
- **Type:** Manual UI structure test.
- **Steps:**
  1. Launch app and inspect bottom telemetry zone.
  2. Confirm exactly six fixed cells are present.
  3. Inspect one cell for label/value/unit/mini-bar composition.
- **Expected:** No scrolling table remains; all six cells follow specified card structure.

---

### Task 14 — Bind telemetry values to `/imu/data` and `/robot_status`

**Commit goal**
- Wire Accel X/Y/Z and Gyro X/Y cells to IMU updates.
- Wire Robot Status cell to latest `/robot_status` string.

**Primary files**
- Sensor/telemetry widget implementation in `src/ui/widgets/`
- Signal wiring points in `src/ui/MainWindow.cpp` or `src/core/Application.cpp`

**PRD coverage**
- T-03, T-04

**Test case (required)**
- **Name:** Telemetry live update rate and status string propagation.
- **Type:** Manual functional test.
- **Steps:**
  1. Publish varying IMU and robot status messages.
  2. Observe telemetry values and status text.
- **Expected:** Values refresh at subscription cadence (bounded by UI refresh constraints); status cell always shows latest string.

---

### Task 15 — Keep Digital Twin dock disabled and sync simulation badge/menu only

**Commit goal**
- Preserve disabled Digital Twin dock creation path.
- Ensure Simulation menu remains control surface and title badge reflects active mode.

**Primary files**
- `src/ui/MainWindow.cpp`
- `src/twin/DigitalTwin.cpp` / `src/twin/DigitalTwin.h` (only if signal emission adjustments needed)

**PRD coverage**
- Section 4.5, S-02

**Test case (required)**
- **Name:** Simulation control path regression check.
- **Type:** Manual behavior test.
- **Steps:**
  1. Attempt to add/open Digital Twin panel via widget actions.
  2. Change simulation mode from menu.
  3. Observe simulation badge.
- **Expected:** No Digital Twin dock is created; mode changes function through menu and badge updates correctly.

---

### Task 16 — Final regression pass: standalone + ROS2-enabled launch

**Commit goal**
- Validate overhaul end-to-end in both standalone and ROS2-enabled environments.
- Capture known issues and UI acceptance pass/fail summary.

**Primary files**
- No mandatory code changes; optional doc updates in `README.md` or `docs/` if run instructions/UI screenshots need refresh.

**PRD coverage**
- Acceptance Criteria 1–10 (verification pass)

**Test case (required)**
- **Name:** Dual-mode launch and UI acceptance sweep.
- **Type:** Build + manual regression.
- **Steps:**
  1. Build/run without ROS2 sourced.
  2. Build/run with ROS2 environment sourced.
  3. Execute acceptance checklist items 1–10.
- **Expected:** App launches in both modes without crash; all P0 acceptance checks pass; gaps logged for follow-up commits.

---

## 11. Suggested Branch / Commit Workflow

- Branch naming: `feature/ui-overhaul-task-<n>-<slug>`
- Commit style: `feat(ui): <atomic change description>` or `refactor(ui): <atomic change description>`
- One task per PR when possible; if batching, combine only tightly-coupled tasks (e.g., Task 13 + 14)
- Merge gate for each task:
  1. Build succeeds
  2. Task test case passes
  3. No regression in already-completed P0 checks

*End of Document — WeedX Desktop Client UI Overhaul PRD v1.1*