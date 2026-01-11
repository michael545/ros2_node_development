# GNSS Navigation Stack

A lightweight ROS 2 package for real-time position and orientation tracking using GNSS and Orientation data only!. This provides a simple and efficient alternative to full SLAM systems when a global pose estimate is needed without local map building and lidar scan processing.

## Overview

This repository is a ROS 2 "stack" containing two key packages:

*   `gnss_navigation`: A core, non-ROS C++ library that contains the `IMUGNSSTracker` logic. It is fully isolated from ROS and can be used in other C++ projects.
*   `gnss_navigation_ros`: The ROS 2 wrapper that provides a node to interface the `IMUGNSSTracker` with the ROS ecosystem. It subscribes to NavSatFIx messages and publishes the resulting pose.

## Features

*   **Lightweight Tracking**: Fuses `sensor_msgs/msg/NavSatFix` and `sensor_msgs/msg/Imu` (for orientation) into a `geometry_msgs/msg/PoseWithCovarianceStamped`.
*   **Two Fusion Methods**:
    1.  **Simple Exponential Smoothing**: A basic filter for quick and simple integration.
    2.  **Simplified Kalman Filter**: A more robust method for smoother and more accurate pose estimation.
*   **Minimal Dependencies**: No LIDAR, cameras, or complex mapping libraries required.
*   **Clean Architecture**: Separates the core tracking logic from the ROS wrapper for better portability and easier testing.

## Prerequisites

*   ROS 2 (tested on Jazzy)
*   Eigen3 library (should be installed via `libeigen3-dev` on Linux)

## Building the Stack

To build the packages, clone this repository into your ROS 2 workspace `src` directory and use `colcon`.

```bash
# 1. Navigate to your ROS 2 workspace source directory
cd your_ros2_ws/src

# git clone <this_repo_url>

# 3. Navigate back to the root of your workspace
cd ..

# 4. Source your ROS 2 installation (if you haven't already)

# 5. Build the packages
colcon build --symlink-install --packages-up-to gnss_navigation_ros
```

## Usage

After building, source your `setup` file and use the provided launch file to run the tracker node.

```bash
# 1. Source your workspace's overlay
source install/setup.bash

# 2. Launch the tracker node
ros2 launch gnss_navigation_ros imu_gnss_tracker.launch.py
```

The node will start and wait for messages on the `/gnss/imu` and `/gnss/fix` topics.

### Configuration

You can modify the tracker's behavior by editing the parameters in `gnss_navigation_ros/param/imu_gnss_tracker.yaml`.

| Parameter                         | Type   | Default | Description                                                                                                                            |
| --------------------------------- | ------ | ------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `fusion_method`                   | string | "kalman"  | The fusion algorithm to use. Options: `"kalman"` or `"simple"`.                                                                          |
| `world_frame`                     | string | "map"     | The fixed world frame for the output pose (e.g., "map", "odom", "utm").                                                                |
| `base_frame`                      | string | "base_link" | The frame ID of the robot's base.                                                                                                      |
| `process_noise_covariance`        | double[] | `[0.1, ...]` | (Kalman only) The uncertainty in the motion model prediction. Increase if the robot's movement is erratic.                             |
| `gnss_measurement_noise_covariance` | double[] | `[1.0, ...]` | (Kalman only) The uncertainty in the GNSS sensor measurements. Increase if the GNSS signal is noisy or has low accuracy.            |
| `smoothing_factor`                | double | 0.1     | (Simple Fusion only) The alpha value for the exponential smoothing filter (0.0 to 1.0). Higher is more responsive, lower is smoother. |


### Subscribed Topics

*   **/gnss/imu** (`sensor_msgs/msg/Imu`): IMU data for orientation. The tracker uses the `orientation` field from this message.
*   **/gnss/fix** (`sensor_msgs/msg/NavSatFix`): GNSS data for position. The tracker converts latitude/longitude to UTM coordinates.

### Published Topics

*   **/pose** (`geometry_msgs/msg/PoseWithCovarianceStamped`): The estimated pose of the robot in the `world_frame`.
*   **/tf** (`tf2_msgs/msg/TFMessage`): Publishes the transform from `world_frame` -> `base_frame`.
