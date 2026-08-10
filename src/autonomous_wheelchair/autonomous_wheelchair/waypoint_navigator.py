#!/usr/bin/env python3
"""waypoints.yaml 에 정의된 이름으로 목표 지점 이동 (nav2_simple_commander 사용)."""

import math
import os

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_pose(navigator, name, wp):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(wp['x'])
    pose.pose.position.y = float(wp['y'])
    yaw = float(wp.get('yaw', 0.0))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def load_waypoints(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get('waypoints', {})


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    navigator.declare_parameter('waypoints_file', '')
    navigator.declare_parameter('waypoint', '')
    navigator.declare_parameter('tour', False)

    waypoints_file = navigator.get_parameter('waypoints_file').value
    if not waypoints_file:
        waypoints_file = os.path.join(
            get_package_share_directory('autonomous_wheelchair'),
            'config', 'waypoints.yaml')

    waypoint_name = navigator.get_parameter('waypoint').value
    tour = navigator.get_parameter('tour').value

    waypoints = load_waypoints(waypoints_file)
    if not waypoints:
        navigator.get_logger().error('waypoints.yaml 에 waypoints 항목이 없습니다: %s' % waypoints_file)
        rclpy.shutdown()
        return

    navigator.waitUntilNav2Active()

    if tour:
        poses = [yaw_to_pose(navigator, name, wp) for name, wp in waypoints.items()]
        navigator.get_logger().info('전체 순회 시작: %s' % ', '.join(waypoints.keys()))
        navigator.followWaypoints(poses)
    else:
        if waypoint_name not in waypoints:
            navigator.get_logger().error(
                "알 수 없는 waypoint 이름 '%s'. 사용 가능: %s" %
                (waypoint_name, ', '.join(waypoints.keys())))
            rclpy.shutdown()
            return
        pose = yaw_to_pose(navigator, waypoint_name, waypoints[waypoint_name])
        navigator.get_logger().info("'%s' 로 이동 시작" % waypoint_name)
        navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('목표 도달 성공')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('작업이 취소되었습니다')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('목표 도달 실패')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
