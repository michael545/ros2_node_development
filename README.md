# ROS2 Node Development Guide

This repository serves as a guide for building efficient ROS2 nodes, primarily in C++ with rclcpp. It covers over best practices, project structure, building, deployment, and more learnt during working with a ros2 system.

## Table of Contents

1. [Introduction to ROS2 Nodes](#introduction-to-ros2-nodes)
2. [Shortly on: C++](#language-choice-c)
3. [Project Structure:](#project-structure-separating-functionality-from-ros-wrapper)
4. [Building with Colcon](#building-with-colcon)
5. [Symlink-Install for Development](#symlink-install-for-development)
6. [Launch Files and Configuration](#launch-files-and-configuration)
7. [Managing External Dependencies](#managing-external-dependencies)
8. [Best Practices for Efficient Nodes](#best-practices-for-efficient-nodes)
9. [Examples](#examples)

## Introduction to ROS2 Nodes

Key concepts:
- Publishers/Subscribers for data streaming
- Services for request-response
- Actions for long-running tasks
- Parameters for configuration
- Lifecycle management

## Language Choice: C++

C++ is most used because of:
- In General: High Performance: Low latency, high throughput
- Control: Direct memory and thread control
- Maturity: Extensive libraries and tools and rclcpp, rust is still imature

## Project Structure: Strive to separate core Functionality from ROS Wrapper

A must practice: Separate core logic from ROS-specific code.

```
include/my_package/
  core.hpp          # Core functionality interfaces
  algorithms.hpp    # Business logic
src/
  core.cpp          # Core implementations
  ros_wrapper.cpp   # ROS node wrapper
  main.cpp          # Entry point
```

This allows:
- Isolated testing of core logic, your driver or hardware interface should always first work without ROS, the tests should be written for core logic only also.
- Portability to other frameworks
- Cleaner code organization, easier maintanance

## Building with Colcon

### How it  Works

Colcon:
1. Discovers packages in workspace
2. Builds packages in topological order based on dependencies
3. Handles dependencies automatically
4. Supports multiple build types (Debug, Release)

### Workspace Management

```bash
# Initialize workspace
mkdir -p colcon_ws/src
cd colcon_ws

```

### Basic Usage

```bash
# Build all packages in that workspace
colcon build 

# Build specific package
colcon build --packages-select my_package

colcon build --symlink-install

# Build with merged install (for large workspaces)
colcon build --merge-install --symlink-install

# Every once in a while during development clean the workspace a bit
rm -rf build/ install/ log/
```

### Package Structure

Each ROS2 package needs:
- `package.xml`: Package metadata and dependencies
- `CMakeLists.txt`: Build configuration
- Source files

### The don'ts 

- **Multiple workspaces**: Avoid having separate workspaces that aren't properly overlaid
- **Nested monorepos**: Don't nest repositories inside src/ subdirectories
- **Missing dependencies**: Always run `rosdep install --from-paths src --ignore-src -r -y` before building
- **Build order issues**: Ensure all dependencies are explicitly declared in package.xml

## Symlink-Install for Development

Symlink-install creates symbolic links instead of copying files, enabling:
- Faster rebuilds
- Direct sourcing of Config files
- Python changes without rebuild

Use during development:
```bash
colcon build --symlink-install
```

## Launch Files and Configuration

### Python Launch Files named node_name.launch.py

Python launch files are most standard and easy:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory for config files
    pkg_share = get_package_share_directory('my_package')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_instance',
            parameters=[config_file],
            output='screen'
        )
    ])
```

### YAML Configuration

Store configurations in package's config/ directory:

```yaml
my_node:
  ros__parameters:
    param1: value1
    param2: 42
    rate: 10.0
```

### Configuration Management Best Practices

- **Install config files**: Add to CMakeLists.txt:
  ```cmake
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  ```

- **Use get_package_share_directory**: Never use relative paths in launch files

- **Centralize configurations**: Keep all config files in dedicated directories, not root

- **Environment-specific configs**: Use launch arguments for different environments

### Lifecycle Nodes

For stateful applications (drivers, SLAM):

```cpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : LifecycleNode("my_lifecycle_node") {}

  // Implement lifecycle callbacks
  LifecycleNode::CallbackReturn on_configure();
  LifecycleNode::CallbackReturn on_activate();
  LifecycleNode::CallbackReturn on_deactivate();
  LifecycleNode::CallbackReturn on_cleanup();
  LifecycleNode::CallbackReturn on_shutdown();
};
```

Launch with lifecycle management:
```python
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

# Configure then activate
lifecycle_node = LifecycleNode(
    package='my_package',
    executable='my_lifecycle_node'
)

return LaunchDescription([
    lifecycle_node,
    # Auto-transition to configure
    EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
        )
    ),
    # Then to activate
    RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lifecycle_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )
])
```

### System Dependencies

Use `package.xml` for ROS packages:
```xml
<depend>opencv</depend>
<depend>pcl</depend>
<depend>libgmock-dev</depend>
```

Install with rosdep:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### CMake Integration

In `CMakeLists.txt`:
```cmake
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

target_include_directories(my_target PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(my_target
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
```

### Vendoring vs System Packages

- **System packages**: Faster builds, consistent versions, managed by package manager
- **Vendoring**: Full control, reproducibility, but increases repository size

### Handling ROS1/ROS2 Migration

When migrating from ROS1:
- Remove ROS1 packages (catkin-based)
- Use ROS2 equivalents or port code
- Avoid mixing catkin and ament packages in same workspace
- Consider ros1_bridge 

### Custom Interfaces

**Choosing standard vs custom messages**

- **Default to standard types:**
  - Use when: The concept fits (e.g., geometry_msgs/Twist for velocity, sensor_msgs/Imu for inertial data)
  - Benefit: Immediate tooling support (RViz/rqt), ecosystem familiarity, fewer packages to maintain

- **Create custom types sparingly:**
  - Use when: You need domain-specific fields, constraints, or units that standard types don't express without overload
  - Benefit: Clear semantics, reduced misuse, tighter validation

- **Hybrid pattern:**
  - Publish both: A standard message for interoperability and a minimal custom message carrying domain semantics (units, mode flags, invariants)
  - Adapter nodes: Translate internally precise messages to standard ones at integration boundaries

**Designing messages that age well**

- **Express intent in fields, not names:**
  - Include units, frames, ranges, and mode/state fields explicitly. Avoid "Float64 that actually means temperature"

- **Prefer stamped messages:**
  - Use Header where time/frame matter; it prevents ambiguity and aids synchronization

- **Keep hot-path lean:**
  - Limit size and nesting; avoid unbounded strings/arrays in real-time paths. Use bounded sequences or fixed-size arrays

- **Use additive evolution:**
  - Add new fields rather than changing types/meaning. Deprecate instead of removing; maintain backward compatibility for at least one release

- **Document invariants inside .msg:**
  - Add concise comments for units, valid ranges, and assumptions. Future readers will thank you

**Practical patterns and checklists**

- **Message profile for standard types:**
  - Define which fields are used, required zero/empty fields, units, and frames. Enforce via linters/tests

- **Validation at the edges:**
  - Validate ranges/units before publishing and upon receipt. Fail fast with clear logs if data violates constraints

- **Version your interfaces package:**
  - Semantic versioning, changelog entries for message changes, and CI that rebuilds dependents to catch breakage early

- **Adapters and bridges:**
  - Provide small nodes/functions that convert between your custom schema and standard messages to keep tooling compatibility

- **QoS awareness:**
  - Align message size/frequency with QoS: reliability, history depth, and deadlines. Large messages plus reliable QoS can stall low-bandwidth links

**Common pitfalls to avoid**

- **Overloading generic types:**
  - Don't shove domain-specific data into an unrelated standard message just to avoid a custom type

- **Ambiguous blanks:**
  - Leaving fields empty can mean "unknown" or "unused." Document the difference in your profile and ensure consumers agree

- **Hidden unit/frame assumptions:**
  - Always state units (e.g., m/s, rpm) and reference frames (e.g., base_link, map) in fields or comments

- **Unbounded data in real-time loops:**
  - Strings and variable-length arrays can break determinism. Use bounds or move heavy data off the hot path

**Minimal examples**

*Standard message (velocity command)*
```yaml
# geometry_msgs/Twist
# Profile:
# - Use linear.x (m/s), angular.z (rad/s)
# - All other fields must be zero
```

*Custom message with clear semantics*
```yaml
# my_robot_interfaces/msg/WheelSpeed.msg
# Units: rpm
# Valid range: [0, 5000]
# Timestamp: Header.stamp is controller time
std_msgs/Header header
float32 left_rpm
float32 right_rpm
bool closed_loop_enabled
```

Create separate packages for custom messages/services ONLY when absolutely unavoidable:
```
my_interfaces/
├── msg/
│   └── MyMessage.msg  # Use only as last resort
├── srv/
│   └── MyService.srv
├── CMakeLists.txt
└── package.xml
```

Keep interfaces minimal and depend only on standard ROS2 types.

## Best Practices for Efficient Nodes

1. **Asynchronous Programming**: Use rclcpp's executors for non-blocking operations
2. **Memory Management**: Avoid allocations in hot paths, use object pools for frequent allocations
3. **Threading**: Use multi-threaded executors for concurrent processing
4. **QoS Settings**: Configure Quality of Service for reliability/latency tradeoffs
5. **Parameter Validation**: Validate parameters at startup with descriptive error messages
6. **Logging**: Use appropriate log levels, throttle frequent logs
7. **Testing**: Unit tests for core logic, integration tests for ROS interactions

### Runtime Data Management

- **Log management**: ROS logs go to `~/.ros/log/`, implement log rotation
- **Application data**: Store runtime data in dedicated directories (e.g., `/data/my_app/`)
- **Build artifacts**: Keep workspace clean, add `build/`, `install/`, `log/` to `.gitignore` to avoid accidentally commiting.

### Web Interfaces

For web-based UIs, prefer standard solutions:
- **rosbridge_server**: Standard ROS2-web bridge
- **foxglove_bridge**: High-performance alternative

Avoid custom web servers - use established bridges instead.

### Hardware Drivers and Permissions

- **Lifecycle nodes**: Use for hardware drivers that need controlled startup/shutdown
- **Permissions**: Don't run entire ROS stack as root
  - Use systemd services with appropriate user
  - Or launch specific nodes with sudo via launch files

### Dependency Management

- **Explicit dependencies**: Declare all dependencies in package.xml
- **Version pinning**: Use specific versions in .repos files
- **Clean builds**: Regularly clean build artifacts to avoid cached issues

### Code Organization

- **Separate interfaces**: Keep .msg/.srv files in dedicated packages
- **Core logic separation**: Keep ROS-specific code separate from business logic
- **Configuration**: Externalize all configurable parameters

## Common Architectural Pitfalls and Solutions

Based on real-world ROS2 project audits, here are common issues and how to avoid them:

### Workspace Proliferation

**Problem**: Multiple separate workspaces (ament_ws, ros2_ws, zenoh_ws) that aren't properly overlaid, leading to undefined behavior and dependency hell.

**Solution**: Use a single workspace for all nodes. Create overlays when needed for different environments.

### Nested Monorepos

**Problem**: Nesting multiple packages inside a subdirectory (like ezmap_pro/), breaking colcon discovery.

**Solution**: Keep all packages flat in src/. Use github submodules to manage multiple repos.

### Package Duplication

**Problem**: Same package in multiple locations, causing unpredictable behavior.

**Solution**: Consolidate to single source of truth, remove duplicates.

### ROS1/ROS2 Mixing

**Problem**: Catkin (ROS1) and ament (ROS2) packages in same workspace can't build togetherm, old packages just clutter the workspace.

**Solution**: Complete migration to ROS2, don't use ros1_bridge temporarily because it is easy.

### Build Instability

**Problem**: Hundreds of build logs indicating persistent failures.

**Solution**: Clean builds regularly, ensure all dependencies are declared, use proper build order.

### Configuration Pollution

**Problem**: Config files and runtime artifacts in project root.

**Solution**: Install configs to share/ directory, use get_package_share_directory() in launch files.

### Runtime Data Management

**Problem**: Logs and data scattered across filesystem.

**Solution**: Centralized data directories, log rotation, proper .gitignore.

### Web Integration Anti-patterns

**Problem**: Custom C++ web servers instead of standard bridges.

**Solution**: Use rosbridge_server or foxglove_bridge for ROS2-web communication.

## Examples

See the `examples/` directory for complete C++ ROS2 node implementations demonstrating these concepts.

- [Basic Publisher/Subscriber](examples/basic_pub_sub/)
- [Parameter Management](examples/parameters/)
- [Lifecycle Nodes](examples/lifecycle_node/)
- [Service Server/Client](examples/service_example/)
- [Action Server/Client](examples/action_example/)
- [Launch Configuration](examples/launch_config/)