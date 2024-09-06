# wp_launcher.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('waypoint_navigator'),
        'config',
        'area.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_navigator',
            executable='waypoint_navigator',
            name='wp_waypoint'
            # parameters=[config]
        )
    ])