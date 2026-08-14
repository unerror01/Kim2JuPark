#!/usr/bin/env python3
"""SLAM 매핑: bringup + slam_toolbox(mapping) + rviz2."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                             IncludeLaunchDescription, LogInfo,
                             RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


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
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params_file],
    )

    # async_slam_toolbox_node는 lifecycle 노드라 configure/activate를 명시적으로
    # 호출해주지 않으면 unconfigured 상태로 멈춰서 /scan을 구독하지 않는다
    # (2026-08-14 실차 매핑 테스트에서 /map이 안 나와서 발견됨).
    configure_slam_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )
    activate_slam_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[mapping.launch] slam_toolbox activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
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
        configure_slam_event,
        activate_slam_event,
        rviz_node,
    ])
