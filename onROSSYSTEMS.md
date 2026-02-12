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

---

## 4. Lifecycle Nodes and Debuggability

### 4.1 Why Lifecycle Nodes Matter in Real Systems

A standard `rclcpp::Node` starts doing work the moment it's constructor is called. In a distributed system this is dangerous:
- A controller starts publishing `cmd_vel` before the firmware motor driver is ready.
- A SLAM processing scans before the TF tree is published.
- A camera driver opens the device while calibration parameters are still loading.

**Lifecycle Nodes** (`rclcpp_lifecycle::LifecycleNode`) solve this by enforcing a state machine:

```
[Unconfigured] → configure() → [Inactive] → activate() → [Active]
                                     ↑            ↓
                                 cleanup()   deactivate()
                                     ↓            ↑
                              [Unconfigured]   [Inactive]
```

**When to use them:**
- **Always** for hardware drivers (sensors, actuators, serial ports). If opening the port fails during `on_configure()`, the transition fails and the system halts cleanly instead of silently running broken.
- **Always** for nodes that depend on other nodes being ready (e.g., navigation depends on costmap).
- **Optional** for stateless utility nodes (e.g., a simple topic relay or a math converter).

### 4.2 The `lifecycle_manager` Pattern (Nav2 Style)

Nav2 pioneered the pattern of a centralized `lifecycle_manager` that transitions a list of nodes in order:

```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    node_names:
      - "map_server"
      - "costmap"
      - "controller_server"
      - "planner_server"
```

The manager calls `configure()` then `activate()` on each node **sequentially**. If `map_server` fails to configure, none of the downstream nodes are started. This is deterministic startup.

**In `ezmap` context:** None of the nodes right now in `ezpkg_map_screen` use lifecycle management. 7 nodes start simultaneously and spray and pray for their cpu resources. This results in nondeterministic launch results and system behaviour. If the camera driver isn't ready when `map_web_worker` tries to subscribe to its image topic, the system drops frames with no error **for now weithout consequences**. A lifecycle manager would catch and prevent this.

### 4.3 Debugging a Running ROS 2 System

Debugging distributed ROS 2 systems is fundamentally different from debugging a single application. The bugs are often **emergent** — they only appear when nodes interact.

#### Command-Line Introspection

These are your first line of defense:

```bash
# List all active nodes
ros2 node list

# Inspect a specific node (topics, services, params)
ros2 node info /route_executor_node

# Check topic bandwidth and frequency
ros2 topic hz /cmd_vel
ros2 topic bw /camera/image_raw

# Check if a topic is actually being published
ros2 topic echo /odom --once

# Inspect the full TF tree
ros2 run tf2_tools view_frames

# Check parameter values at runtime
ros2 param get /my_node my_parameter
ros2 param list /my_node

# Lifecycle state inspection
ros2 lifecycle get /my_lifecycle_node
ros2 lifecycle list /my_lifecycle_node
```

#### `ros2 doctor`

Run this first when something feels wrong:

```bash
ros2 doctor --report
```

It checks for:
- Mismatched QoS between publishers and subscribers (the #1 silent failure)
- Network configuration issues
- Missing environment variables
- DDS discovery problems

#### Logging Discipline

Logging is the most underrated debugging tool. Rules:

- **`DEBUG`**: Verbose data dumps. Off by default. Enable per-node when hunting a specific bug.
- **`INFO`**: Normal operation milestones ("Route loaded", "Navigation started"). Should be readable at a glance.
- **`WARN`**: Something unexpected but recoverable ("GPS fix lost, using dead reckoning").
- **`ERROR`**: Something broke ("Failed to open serial port /dev/ttyUSB0").

**Throttled logging** for high-frequency callbacks:

```cpp
// C++ — logs at most once per second
RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "IMU data arriving late: %.2f ms", delay_ms);
```

```python
# Python — logs at most once per second
self.get_logger().warn("IMU data arriving late", throttle_duration_sec=1.0)
```

**Pitfall:** Putting `RCLCPP_INFO` inside a 100Hz callback produces 100 log lines per second. This floods the terminal, fills disk, and can actually slow down the node. Always throttle or use `DEBUG` level for hot-path logging.

#### Diagnostics (`diagnostic_msgs`)

For production systems, structured diagnostics beat log grep:

```cpp
// Publish structured health data
diagnostic_msgs::msg::DiagnosticStatus status;
status.name = "GPS Driver";
status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
status.message = "Low satellite count";
status.values.push_back({"satellite_count", "3"});
status.values.push_back({"hdop", "4.2"});
```

This feeds into `rqt_robot_monitor` and `diagnostic_aggregator`, giving you a dashboard view of system health rather than grepping through log files.

### 4.4 Common Debugging Scenarios and Solutions

| Symptom | Likely Cause | Debug Command |
|---------|-------------|---------------|
| Topic published but subscriber gets nothing | QoS mismatch (RELIABLE vs BEST_EFFORT) | `ros2 doctor --report`, check QoS |
| TF transform lookup fails | Missing broadcaster, clock skew | `ros2 run tf2_tools view_frames` |
| Node starts but does nothing | Lifecycle node stuck in Unconfigured | `ros2 lifecycle get /node_name` |
| Intermittent message drops on Wi-Fi | RELIABLE QoS + packet loss = head-of-line blocking | Switch to BEST_EFFORT for sensor data |
| High CPU usage on bridge node | Serializing large messages (images) to JSON | Switch to `foxglove_bridge` (binary) |
| Robot jerks or oscillates | Control loop too slow, or stale TF data | `ros2 topic hz /cmd_vel`, check TF timestamps |
| Node crashes silently | Segfault in C++ callback, no lifecycle cleanup | Run under `gdb`, use AddressSanitizer |
| System works in sim but not on hardware | `use_sim_time` still true, or wrong frame IDs | `ros2 param get /node use_sim_time` |

### 4.5 Profiling and Performance Debugging

When something is "slow" but you don't know what:

```bash
# Measure callback execution time — add to your node
auto start = this->now();
// ... callback work ...
auto elapsed = (this->now() - start).seconds() * 1000.0;
RCLCPP_DEBUG(this->get_logger(), "Callback took %.2f ms", elapsed);

# System-level CPU profiling
top -H -p $(pgrep -f my_node)  # Per-thread CPU usage

# ROS 2 tracing (requires ros2-tracing package)
ros2 trace start my_trace
# ... run the system ...
ros2 trace stop my_trace
```

**Memory debugging for C++ nodes:**
```bash
# Detect leaks and use-after-free
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address"

# Or run under Valgrind
valgrind --leak-check=full ros2 run my_package my_node
```

---

## 5. Dependency Management

Managing dependencies across distributed ROS 2 system is critical for reproducibility, deployment, and avoiding "it works on my machine" derangements syndrome. Dependencies span multiple layers: **System-level**, **ROS-level**, and **Language-specific** (Python, C++, Rust).

### 5.1 System-Level Dependencies (apt/dnf/pacman)

These are OS packages installed you get via Apt or snap.

**Examples:**
- `libgmock-dev`, `libopencv-dev`, `libeigen3-dev` (C++ libraries)
- `python3-numpy`, `python3-scipy` (Python system packages)
- `ros-jazzy-desktop`, `ros-jazzy-navigation2` (ROS 2 metapackages)

**Best Management Strategy:**

**Using `rosdep`:**
```bash
# Install all dependencies declared in package.xml files
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` reads the `<depend>` tags in `package.xml` and maps them to system packages using `rosdep` rules.

**Example `package.xml`:**
```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>opencv</depend>           <!-- Mapped to libopencv-dev -->
<depend>libgmock-dev</depend>     <!-- System package -->
```

**Best Practices:**
- **Always run `rosdep update` before install** to sync the latest package mappings.
- **Declare ALL system dependencies explicitly** in `package.xml`, even if they're installed. This ensures reproducibility on new systems.
- **Use Docker for complete isolation**: System dependencies are baked into the container image, ensuring identical environments across dev/CI/production.

**Pitfall:** Installing system packages manually (`sudo apt install libfoo-dev`) bypasses `rosdep` tracking. Other developers won't know about this dependency.

---

### 5.2 ROS-Level Dependencies (Other ROS Packages)

These are other ROS 2 packages your package depends on.

**Declaration in `package.xml`:**
```xml
<depend>nav2_msgs</depend>
<depend>tf2_ros</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>robot_localization</exec_depend>
```

**Dependency Types:**
- `<depend>`: Required for both build and runtime.
- `<build_depend>`: Only needed during compilation (e.g., message generators).
- `<exec_depend>`: Only needed at runtime (e.g., a Python script you call).
- `<test_depend>`: Only for tests (e.g., `launch_testing`).

**Management Strategy:**

**For Source Builds (Workspace Development):**
Use `vcstool` to pull all repositories into your workspace:

```bash
# Create a .repos file
cat > my_robot.repos << EOF
repositories:
  navigation2:
    type: git
    url: https://github.com/ros-planning/navigation2.git
    version: jazzy
  robot_localization:
    type: git
    url: https://github.com/cra-ros-pkg/robot_localization.git
    version: jazzy
EOF

# Import into workspace
vcs import src < my_robot.repos
```

**For Binary Dependencies:**
If the dependency is released, install via apt:
```bash
sudo apt install ros-jazzy-nav2-msgs ros-jazzy-robot-localization
```

**Best Practices:**
- **Pin versions in `.repos` files** using commit hashes for reproducibility.
- **Minimize custom dependencies**: Prefer released ROS packages over forked/custom versions.
- **Use `colcon graph`** to visualize dependency chains and detect circular dependencies.

---

### 5.3 Python-Specific Dependencies

Python nodes may depend on packages not available via `rosdep` (e.g., ML libraries, web frameworks).

**Declaration Strategies:**

**Option 1: `package.xml` (Preferred for ROS Integration)**
```xml
<exec_depend>python3-requests</exec_depend>
<exec_depend>python3-flask</exec_depend>
```

If the package is available in the system repositories, `rosdep` will install it.

**Option 2: `requirements.txt` (For pip-only packages)**
```text
# requirements.txt
torch==2.0.0
fastapi==0.95.0
uvicorn[standard]
```

Install manually:
```bash
pip3 install -r requirements.txt
```

**Option 3: `setup.py` (For ament_python packages)**
```python
setup(
    name='my_package',
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'scipy',
        'opencv-python',
    ],
)
```

When you `colcon build` or `pip install .`, these dependencies are installed automatically.

**Best Practices:**
- **Prefer system packages** (`python3-opencv`) over pip (`opencv-python`) for stability.
- **Use virtual environments** (`venv`) or `pipx` to isolate dependencies during development.
- **For deployment**, bake dependencies into Docker images or use `requirements.txt` with locked versions.

**Pitfall:** The Python Global Interpreter Lock (GIL) means pip-installed native extensions (e.g., PyTorch) can conflict with system packages. Use `--user` or virtual environments to avoid polluting the system Python.

---

### 5.4 C++ Dependencies (CMake/pkg-config)

C++ packages depend on external libraries (Eigen, OpenCV, Boost, etc.).

**Declaration in `CMakeLists.txt`:**
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

target_include_directories(my_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(my_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
```

**And in `package.xml`:**
```xml
<depend>libopencv-dev</depend>
<depend>libeigen3-dev</depend>
<depend>libpcl-dev</depend>
```

**Management Strategy:**

**System Libraries:**
- Install via `apt`: `sudo apt install libopencv-dev libeigen3-dev`.
- Let `rosdep` handle it: Declare in `package.xml`, run `rosdep install`.

**Vendored/Custom Libraries:**
If you need a specific version or a library not in apt:
1. **Git Submodules**: Include the library as a submodule in your repo.
   ```bash
   git submodule add https://github.com/fmtlib/fmt.git external/fmt
   ```
2. **ExternalProject_Add**: Download and build during CMake configuration.
   ```cmake
   include(ExternalProject)
   ExternalProject_Add(fmt
     GIT_REPOSITORY https://github.com/fmtlib/fmt.git
     GIT_TAG 9.1.0
     CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
   )
   ```

**Best Practices:**
- **Avoid vendoring unless necessary**: System packages are tested and patched by distro maintainers.
- **Document custom build steps** in README.md.
- **Use `ccache`** to speed up rebuilds when dependencies change.

**Pitfall:** CMake's `find_package` searches in system paths. If you have multiple versions of OpenCV installed (system + conda), CMake may pick the wrong one. Use `CMAKE_PREFIX_PATH` to force the correct location.

---

### 5.5 Rust Dependencies (Cargo)

Rust packages (using `rclrs`) depend on crates from crates.io.

**Declaration in `Cargo.toml`:**
```toml
[dependencies]
rclrs = "0.4"
sensor_msgs = { version = "0.1", package = "sensor-msgs-rs" }
tokio = { version = "1", features = ["full"] }
serde = { version = "1.0", features = ["derive"] }
```

**Management Strategy:**

**Lock File (`Cargo.lock`):**
- Cargo automatically generates `Cargo.lock` to pin exact versions of all transitive dependencies.
- **Commit `Cargo.lock`** to version control for reproducible builds.

**Integration with Colcon:**
Use the `colcon-cargo` extension to build Rust packages in ROS 2 workspaces.

```bash
# Install colcon-cargo
pip3 install colcon-cargo

# Build Rust package
colcon build --packages-select my_rust_package
```

**Best Practices:**
- **Use semantic versioning** (`^1.0` means `>=1.0.0, <2.0.0`).
- **Audit dependencies** with `cargo audit` to detect security vulnerabilities.
- **Minimize dependencies**: Each crate increases build time. Rust's monomorphization can lead to long compile times.

**Pitfall:** ROS message bindings for Rust (`rosidl_generator_rs`) are generated at build time. If you change a `.msg` file, you must rebuild the Rust package. Use `cargo clean` if you encounter stale bindings.

---

### 5.6 Cross-Language Dependencies and Interoperability

In distributed systems with multiple languages (C++ on robot, Python on laptop, Rust on safety monitor), dependencies must be **version-aligned**.

**Example Scenario:**
- Robot runs `rclcpp` nodes (C++).
- Laptop runs `rclpy` nodes (Python).
- Safety monitor runs `rclrs` (Rust).

**Critical Alignment:**
1. **ROS 2 Distro**: All nodes must use the same ROS 2 distro (Jazzy, Humble, etc.). Mixing distros breaks ABI compatibility.
2. **DDS Vendor**: If using Fast DDS on the robot, use Fast DDS on the laptop. Mixing vendors (Fast DDS + Cyclone DDS) *can* work but introduces discovery and performance issues.
3. **Message Definitions**: All nodes must have the same `.msg` file versions. Use a shared `my_robot_msgs` package as a single source of truth.

**Management Strategy:**
- **Centralize message definitions** in a dedicated package (e.g., `ezmap_interfaces`).
- **Use Docker images with identical ROS 2 base** (`ros:jazzy-desktop-full`) for all compute nodes.
- **Test interoperability** with integration tests that span all languages:
  ```python
  # test_interop.py
  def test_cpp_to_python_to_rust():
      # Start C++ publisher, Python relay, Rust subscriber
      # Assert message flows correctly
  ```

---

### 5.7 Dependency Management Best Practices Summary

| Layer | Tool | Declaration | Best Practice |
|-------|------|-------------|---------------|
| **System** | `rosdep`, `apt` | `package.xml` (`<depend>`) | Always run `rosdep install`, use Docker for isolation |
| **ROS Packages** | `vcstool`, `colcon` | `package.xml`, `.repos` | Pin versions in `.repos`, minimize custom deps |
| **Python** | `pip`, `venv` | `setup.py`, `requirements.txt` | Prefer system packages, lock versions |
| **C++** | CMake, `pkg-config` | `CMakeLists.txt`, `package.xml` | Avoid vendoring, use `ccache` |
| **Rust** | Cargo | `Cargo.toml`, `Cargo.lock` | Commit `Cargo.lock`, audit with `cargo audit` |
| **Cross-Language** | Docker, CI | Centralized message package | Align ROS distro and DDS vendor |

**Golden Rule:** If a dependency is not declared in `package.xml`, `requirements.txt`, `Cargo.toml`, or `CMakeLists.txt`, it doesn't exist for reproducibility purposes. Always make dependencies explicit.


