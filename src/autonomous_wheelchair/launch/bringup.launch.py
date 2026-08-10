#!/usr/bin/env python3
"""로봇 기본 구동: robot_state_publisher + ydlidar + base_driver + ekf."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('autonomous_wheelchair')

    use_base = LaunchConfiguration('use_base')
    use_ekf = LaunchConfiguration('use_ekf')
    serial_port = LaunchConfiguration('serial_port')

    declare_use_base = DeclareLaunchArgument(
        'use_base', default_value='true', description='base_driver 노드 실행 여부')
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf', default_value='true', description='robot_localization ekf_node 실행 여부')
    # 라이다와 ESP32 가 동일한 CP2102 칩(idVendor/idProduct/serial 동일)을 쓰기 때문에
    # /dev/ttyUSB* 대신 udev/99-ydlidar.rules 가 물리 포트 기준으로 만들어주는
    # 안정적인 심볼릭 링크 /dev/esp32 를 기본값으로 사용한다.
    declare_serial_port = DeclareLaunchArgument(
        'serial_port', default_value='/dev/esp32', description='ESP32 모터 컨트롤러 시리얼 포트')

    xacro_file = os.path.join(pkg_share, 'urdf', 'wheelchair.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # 참고: ydlidar_ros2_driver_node 는 실제로는 rclcpp_lifecycle 을 구현하지 않은
    # 일반 rclcpp::Node 이며(get_state 등 lifecycle 서비스 없음), 시작하자마자 바로
    # 스캔을 시작한다. 그래서 lifecycle_manager 로 관리하지 않고 일반 Node 로 구동한다.
    ydlidar_params_file = os.path.join(pkg_share, 'config', 'ydlidar_g2.yaml')
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params_file],
    )

    base_driver_node = Node(
        package='autonomous_wheelchair',
        executable='base_driver',
        name='base_driver',
        output='screen',
        parameters=[{'serial_port': serial_port}],
        condition=IfCondition(use_base),
    )

    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        condition=IfCondition(use_ekf),
    )

    return LaunchDescription([
        declare_use_base,
        declare_use_ekf,
        declare_serial_port,
        robot_state_publisher_node,
        ydlidar_node,
        base_driver_node,
        ekf_node,
    ])
