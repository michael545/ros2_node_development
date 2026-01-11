import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('gnss_navigation_ros'),
        'param',
        'imu_gnss_tracker.yaml'
    )

    node = Node(
        package='gnss_navigation_ros',
        name='imu_gnss_tracker_ros',
        executable='imu_gnss_tracker_ros',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'imu_gnss_tracker_ros:=debug']
    )

    ld.add_action(node)

    return ld
