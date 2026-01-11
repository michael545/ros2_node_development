import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gnss_navigation_ros')

    # Include the main tracker launch file
    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'imu_gnss_tracker.launch.py')
        )
    )

    # Start the simulation spoofer
    spoofer_node = Node(
        package='gnss_navigation_ros',
        executable='gnss_spoofer.py',
        name='gnss_spoofer',
        output='screen'
    )

    return LaunchDescription([
        tracker_launch,
        spoofer_node
    ])
