# README Guidelines for ROS 2 Projects

## Context & The Organizational Standard

Documentation is the drudgerous beaurocratic part of any new engineering engineering endevor. In a complex distributed system like a full ROS2 powered AMR software stack, an undocumented node is a "black box" that requires reverse-engineering to use, debug and fix.

**The Golden Rule:** **Every repository** within the UbiquityRobotics GitHub organization **MUST** have a `README.md` that is strictly compliant with these standards. 

We acknowledge that some older repositories are non-compliant today; this has identified as a major hindrance to development flow, technical debt, and onboarding delays. Moving forward, no new repository will be considered "done" without a compliant README, and older repositories should be updated as they are updated and fixed.

**The Folder Rule:** If you have multiple significant directories within a repository that contain unique logic or configuration (eg, a muliple unrealated ROS packages each encompassing a uniqure ROS node) those directories MUST also have its own `README.md`.


*   **Root Level:** High-level architecture, build instructions.
*   **Package Level:** Node details, interfaces, and specific build quirks.
*   **Sub-folders (e.g., `launch/`, `config/`):** If complex, add a short README explaining the strategy (or a pointer to the main package README).
*   **"Pointer" READMEs:** If a folder is self-explanatory or detailed elsewhere, the README must explicitly say so (eg, *"See `docs/architecture.md` for details on this module"*). **Never leave it empty.**

---

## 1. The Anatomy of a good README for robot development

A good README answers three questions immediately:
1.  **What** does this do?
2.  **How** do I run it?
3.  **How** does it communicate with the rest of my system?

### 2. Core Description
Start with a concise high-level summary. No implementation details here; what is the *functionality* of this repo and why is it needed.

> **Bad:** "This node subscribes to /scan and uses DBSCAN1. to stop before obstacles"
> **Good:** "The `obstacle_detector` node clusters raw LiDAR points obtained fro /scna into tracked object instances for the local planner. It differentiates static vs. dynamic obstacles."

---

## 3. The Interface Specification 

This is the most critical section. It defines the "ROS2 API" of your code. New developers should be able to integrate your node without ever looking at the raw source code.

### 3.1 Nodes
List every executable/node provided by the package.

#### Node: `my_node_name`

**Subscribers**
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/scan` | `sensor_msgs/LaserScan` | Raw 2D lidar data. |
| `/odom` | `nav_msgs/Odometry` | Robot odometry for motion compensation. |

**Publishers**
| Topic | Type | Description |
| :--- | :--- | :--- |
| `/obstacles` | `derived_object_msgs/ObjectArray` | List of detected obstacles with velocity. |
| `/visualization_marker` | `visualization_msgs/MarkerArray` | Rviz debug markers. |

**Services (Servers)**
| Service | Type | Description |
| :--- | :--- | :--- |
| `reset_tracking` | `std_srvs/Trigger` | Clears all internal object history. |
| `set_region` | `my_pkg/SetRegion` | Updates the active detection ROI. |

**Actions**
| Action | Type | Description |
| :--- | :--- | :--- |
| `calibrate_floor` | `my_pkg/Calibrate` | Runs a floor plane detection routine (blocking). |

**Parameters**
| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cluster_tolerance` | double | 0.5 | Max distance between points to consider them one cluster. |
| `max_objects` | int | 100 | Hard limit on tracked objects to prevent CPU overload. |

---

## 4. Custom Interface Definitions

If your package defines custom `.msg`, `.srv`, or `.action` files, it is generally a good idea or a **must** to document their fields and invariants. Do not force users to read the raw definition files to understand the units.

**Example: `msg/DetectedObject.msg`**

```text
std_msgs/Header header    # Timestamp of detection
uint32 id                 # Unique tracking ID (persists across frames)
geometry_msgs/Pose pose   # Centroid position in 'map' frame
geometry_msgs/Twist velocity # Linear velocity in m/s (covariance required)
float32 confidence        # [0.0 - 1.0] Detection certainty
```

*   **Important Side Note:** Please do specify **Units** (m, rad, s) and **Reference Frames** (map, base_link) in the comments/description.

---

## 5. Usage & Examples

Provide copy-pasteable commands.

**Building:**
```bash
colcon build --packages-select my_package --symlink-install
```

**Running:**
```bash
# Standard run
ros2 launch my_package main.launch.py

# With debug output
ros2 run my_package my_node --ros-args --log-level debug
```

---

## 6. Copy-Paste Template

*(Copy the section below into your new `README.md`)*

```markdown
# Package Name

## Overview
[Brief description of what this package does.]

## Installation
### Dependencies
*   `roscpp`
*   `pcl_ros`
*   ...

## Nodes

### `node_name`
[Description of the node's specific role.]

#### Subscribed Topics
*   `topic_name` (`type`): Description

#### Published Topics
*   `topic_name` (`type`): Description

#### Parameters
*   `param_name` (type, default): Description

## Launch Files
*   `my_node.launch.py`: Starts the node with default config.
    *   Args: `use_sim_time` (bool): ...
```
