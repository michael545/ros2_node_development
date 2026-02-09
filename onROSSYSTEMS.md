# ROS 2 Robot Systems and ezmap for system Architects
## 1. Essentials of Distributed ROS 2 Systems

A **ROS 2 System** is a collection of nodes where running on multiple distinct compute units (e.g., a Robot CPU (Pi5, Intel NUC, Jetson) + a User Laptop  + a STM32 MCU) connected via a network. Unlike monolithic systems, distributed systems have peculiar challenges in Connectivity, Latency, and Consistency.

### Key Considerations that jump to mind

1.  **Discovery & The Shared Databus**:
    *   ROS 2 uses DDS (Data Distribution Service). Nodes find each other via **Multicast** (default) or **Discovery Servers**.
    *   **In `ezmap` context**: If the Motor Driver and the Main CPU are separate devices, they share a network (UART). Configuring `ROS_DOMAIN_ID` is critical to isolate distinct robot fleets on the same Wifi.

2.  **Quality of Service (QoS)**:
    *   In distributed systems, Wi-Fi is unreliable.
    *   **Best Practice**: Use `RELIABLE` for critical commands (Start/Stop) but `BEST_EFFORT` for high-bandwidth sensor data (Lidar/Camera) to prevent head-of-line blocking.

3.  **Clock Synchronization**:
    *   `tf2` transforms are time-sensitive. If the MCU clock drifts or even worse missynchronizes with the Robot CPU clock by >0.01s, transforms can fail.
    *   **Requirement**: PTP (Precision Time Protocol) or NTP is mandatory across all compute nodes.

---

## 2. On `ezmap` Packages and Entanglement

The `ezmap` repository exhibits a **"Feature-Centric"** packaging strategy rather than an **"Architectural"** or **"Layered"** strategy. While this groups related files together, it leads to monolithic packages and tight coupling.

### Case Study A: `ezpkg_map_screen` – The Monolith

**Contents:**
*   **Nodes**: `map_web_worker`, `actions`, `triggers`, `route_setup`, `route_executor`, `odom_tf_broadcaster`, `twist_bridge`.
*   **Web Assets**: HTML/JS in `public/`.

**The Architectural Flaw:**
This package violates the **Single Responsibility Principle**.
1.  **Logic Logic**: It contains `route_executor_node` (Navigation logic) and `triggers_node` (Business logic).
2.  **Driver Logic**: It contains `twist_bridge` (Hardware interface) and `odom_tf_broadcaster`.
3.  **UI Logic**: It contains `map_web_worker` and front-end assets.

**Why this is bad:**
*   **Forced Dependencies**: If another UI (e.g., `route_select_ui`) needs to use the `route_executor`, it effectively has to depend on the entire `map_screen` package. The "Map Screen" is no longer just a screen; it's the implicit "Navigation Server" too.
*   **Deployment Bloat**: Installing the navigation logic necessitates installing the web server dependencies, even on a headless robot.
*   **Testing Nightmare**: You cannot unit test the `route_executor` without navigating past the folder structures of the web server.

### Case Study B: `route_select_ui` – The Hybrid Driver/UI

**Contents:**
*   **Nodes**: `touchscreen.py`, `lidar_ui_interface.py`, `services_worker.py`.

**The Architectural Flaw:**
*   **Driver in UI**: `touchscreen.py` implies a hardware driver for the specific 7-inch display is bundled inside the UI package.
*   **Coupling**: It seems to exist to control the logic housed in `ezpkg_map_screen`, creating a circular or confusing dependency graph where a "UI" controls another "UI's" internal logic.

### Case Study C: `ezmap_core` – The Misnomer

**Contents:**
*   `ezmap_web`, `ezmap_autoloader_web`, `ezpkg_views`.

**The Architectural Irony:**
A package named `core` usually implies business logic or central algorithms. Here, it contains **Web Infrastructure**.
*   This confirms that the repository is organized around implementation details (Web vs Screen) rather than functional domains (Navigation, Hardware).
*   The actual "Core" logic (executing routes) is misplaced in `ezpkg_map_screen`, while the `ezmap_core` package handles web loading. This inversion makes onboarding new developers extremely difficult, as they will look for route logic in `core` and fail to find it.

---

## 3. Effective Node Bundling: A Better Way

To fix the "too many nodes in one repo/package" issue, the system should be refactored into **Layered Capabilities**.

### Refactoring Strategy

#### 1. Extract The "Core" Logic
Move the business logic and navigation engines out of the UI packages.
*   **New Package**: `ezmap_navigation_core`
    *   *Contains*: `route_executor_node`, `route_setup_node`.
    *   *Role*: Pure logic. Accepts generic Plans, executes Routes. No UI code.

#### 2. Extract The "Drivers"
Hardware interfaces should never be in a "pkg" or "screen" package.
*   **New Package**: `ezmap_hardware_drivers`
    *   *Contains*: `touchscreen.py`, `twist_bridge`, `odom_tf_broadcaster`.
    *   *Role*: Publish `sensor_msgs` and subscribe to `cmd_vel`. implementation-agnostic.

#### 3. Pure UI Packages
The "Screen" packages should only contain the Code required to bridge ROS to that specific Screen.
*   **Refactored**: `ezpkg_map_screen`
    *   *Contains*: `map_web_worker_node`, Web Assets.
    *   *Role*: Subscribes to map/status, Publishes User Intents. Does **not** execute the route.
*   **Refactored**: `route_select_ui`
    *   *Contains*: `lidar_ui_interface`, UI Scripts.
    *   *Role*: Bridges the 7-inch Touchscreen to ROS messaging.

### The ideal "Repo" Structure

Don't fear multiple `package.xml` files. A granular structure is cleaner:

```text
src/
  ezmap_core/               # Business Logic (Routes, Triggers) · [C++/Python logic]
  ezmap_drivers/            # Hardware Abstraction · [Drivers]
  screens/
    ezmap_web_ui/           # The Web Dashboard (was map_screen)
    ezmap_touch_ui/         # The 7-inch Screen (was route_select_ui)
  ezmap_bringup/            # Launch files that compose the above
```

### Why this fixes the problem:
*   **Composable**: You can launch `ezmap_core` + `ezmap_web_ui` for a remote-control robot.
*   **Composable**: You can launch `ezmap_core` + `ezmap_touch_ui` for a standalone robot.
*   **Testable**: You can test `ezmap_core` without any UI running.
*   **Clean**: No more "Why is the TF broadcaster inside the Map Screen?" confusion.
