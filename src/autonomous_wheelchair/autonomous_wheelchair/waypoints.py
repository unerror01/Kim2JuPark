#!/usr/bin/env python3
"""waypoints.yaml 로드/저장 + PoseStamped 변환 공용 헬퍼 (waypoint_navigator, ui_bridge 공유)."""

import math

import yaml

from geometry_msgs.msg import PoseStamped


def yaw_to_pose(clock, name, wp):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = clock.now().to_msg()
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


def save_waypoints(path, waypoints):
    with open(path, 'w') as f:
        yaml.safe_dump({'waypoints': waypoints}, f, allow_unicode=True, sort_keys=True)
