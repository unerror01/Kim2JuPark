#!/usr/bin/env python3
"""SLAM 매핑: bringup + slam_toolbox(mapping) + rviz2."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_wheelchair')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='rviz2 실행 여부')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup.launch.py')),
    )

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_mapping.yaml')
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_rviz,
        bringup_launch,
        slam_toolbox_node,
        rviz_node,
    ])
