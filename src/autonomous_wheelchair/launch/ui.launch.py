#!/usr/bin/env python3
"""태블릿/관리자 웹 UI: rosbridge_server + ui_bridge + web/ 정적 파일 서버."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_wheelchair')
    web_dir = os.path.join(pkg_share, 'web')

    web_port = LaunchConfiguration('web_port')
    declare_web_port = DeclareLaunchArgument(
        'web_port', default_value='8080', description='태블릿/관리자 웹 UI 정적 파일 서버 포트')

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
    )

    ui_bridge_node = Node(
        package='autonomous_wheelchair',
        executable='ui_bridge',
        name='ui_bridge',
        output='screen',
    )

    web_server_process = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', web_port],
        cwd=web_dir,
        output='screen',
    )

    return LaunchDescription([
        declare_web_port,
        rosbridge_launch,
        ui_bridge_node,
        web_server_process,
    ])
