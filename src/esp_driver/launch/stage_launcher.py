#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    stage_ros2_dir = get_package_share_directory('stage_ros2')
    stage_launch_dir = os.path.join(stage_ros2_dir, 'launch')

    stage_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            stage_launch_dir, 'demo.launch.py')),
        launch_arguments={'use_sim_time':'false','world': 'cave','use_static_transformations': 'true'}.items())
    return LaunchDescription([
        stage_launcher,
        footprint_publisher,
    ])
