import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('esp_driver')



    nav2_params_file = 'nav2_params.yaml'
    nav2_params = os.path.join(package_dir, 'config', nav2_params_file)
    nav2_map = os.path.join(package_dir, 'map', 'map.yaml')
    
    navigation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('esp_driver'), 'launch', 'navigation_bringup', 'bringup_launch.py')),
        launch_arguments=[
            ('map', nav2_map),
            ('params_file', nav2_params),
            ('use_sim_time', "False"),
            ('slam', "False")
        ],
    )

    # rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory(
    #             'nav2_bringup'), 'launch', 'rviz_launch.py')
    #     ),
    #     launch_arguments={'namespace': '', 'use_namespace': 'False'}.items(),
    # )
    return LaunchDescription([
        navigation_nodes,
        # rviz_cmd
    ])
