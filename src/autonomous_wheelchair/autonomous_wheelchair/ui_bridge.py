#!/usr/bin/env python3
"""태블릿/관리자 웹 UI와 Nav2 사이를 잇는 상시 구동 노드.

- 목적지 이름 -> NavigateToPose 액션 목표로 변환해 전송 (재사용 가능한 액션 클라이언트,
  waypoint_navigator.py 의 1회성 스크립트와 달리 여러 목표를 반복 수신).
- 목적지 선택 시점의 /amcl_pose 를 trip_start_pose 로 저장해 반납(원래 위치 복귀)에 사용.
- 정지/지원호출/재개, 수동제어 전환, 웨이포인트 추가/삭제(관리자 페이지) 처리.
- /trip_state, /waypoints_list, /admin/support_calls 를 웹 UI에 발행.
"""

import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String

from autonomous_wheelchair.waypoints import load_waypoints, save_waypoints, yaw_to_pose


def _latched_qos(depth=1):
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UiBridge(Node):

    def __init__(self):
        super().__init__('ui_bridge')

        self.declare_parameter('waypoints_file', '')
        waypoints_file = self.get_parameter('waypoints_file').value
        if not waypoints_file:
            waypoints_file = os.path.join(
                get_package_share_directory('autonomous_wheelchair'),
                'config', 'waypoints.yaml')
        self.waypoints_file = waypoints_file
        self.waypoints = load_waypoints(self.waypoints_file)

        self.trip_state = 'IDLE'
        self.pre_stop_state = None
        self.control_mode = 'AUTO'
        self.estop = False
        self.current_destination = None
        self.trip_start_pose = None
        self.latest_amcl_pose = None
        self.goal_handle = None

        # --- publishers ---
        self.control_mode_pub = self.create_publisher(String, '/control_mode', _latched_qos())
        self.estop_pub = self.create_publisher(Bool, '/estop', _latched_qos())
        self.trip_state_pub = self.create_publisher(String, '/trip_state', _latched_qos())
        self.waypoints_list_pub = self.create_publisher(String, '/waypoints_list', _latched_qos())
        self.support_call_pub = self.create_publisher(String, '/admin/support_calls', 20)
        self.cmd_vel_manual_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)

        # --- subscriptions ---
        self.create_subscription(String, '/ui/select_destination', self.on_select_destination, 10)
        self.create_subscription(Bool, '/ui/return_to_start', self.on_return_to_start, 10)
        self.create_subscription(Bool, '/ui/set_manual_mode', self.on_set_manual_mode, 10)
        self.create_subscription(Bool, '/ui/stop', self.on_stop, 10)
        self.create_subscription(Bool, '/ui/support_call', self.on_support_call, 10)
        self.create_subscription(Bool, '/ui/resume', self.on_resume, 10)
        self.create_subscription(Twist, '/ui/joystick', self.on_joystick, 10)
        self.create_subscription(String, '/ui/waypoint_add', self.on_waypoint_add, 10)
        self.create_subscription(String, '/ui/waypoint_delete', self.on_waypoint_delete, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 초기 상태를 latched 로 한 번 발행해 늦게 붙는 클라이언트도 즉시 동기화되게 함
        self._publish_control_mode(self.control_mode)
        self._publish_estop(self.estop)
        self._publish_trip_state(self.trip_state)
        self._publish_waypoints_list()

        self.get_logger().info('ui_bridge started, %d waypoint(s) loaded' % len(self.waypoints))

    # ---------------- 상태 발행 헬퍼 ----------------

    def _publish_control_mode(self, mode):
        self.control_mode = mode
        msg = String()
        msg.data = mode
        self.control_mode_pub.publish(msg)

    def _publish_estop(self, value):
        self.estop = value
        msg = Bool()
        msg.data = value
        self.estop_pub.publish(msg)

    def _publish_trip_state(self, state):
        self.trip_state = state
        msg = String()
        msg.data = state
        self.trip_state_pub.publish(msg)

    def _publish_waypoints_list(self):
        payload = json.dumps(
            [{'name': name, 'x': wp['x'], 'y': wp['y'], 'yaw': wp.get('yaw', 0.0)}
             for name, wp in sorted(self.waypoints.items())],
            ensure_ascii=False)
        msg = String()
        msg.data = payload
        self.waypoints_list_pub.publish(msg)

    def _publish_support_call(self, call_type):
        payload = json.dumps({
            'type': call_type,
            'destination': self.current_destination,
            'timestamp': time.time(),
        }, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self.support_call_pub.publish(msg)

    # ---------------- Nav2 액션 ----------------

    def _pose_stamped_from_amcl(self, amcl_msg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = amcl_msg.pose.pose
        return pose

    def _cancel_active_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _send_pose_goal(self, pose: PoseStamped):
        if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose 액션 서버를 찾을 수 없습니다')
            self._publish_trip_state('FAILED')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _send_goal_to_waypoint(self, name):
        wp = self.waypoints.get(name)
        if wp is None:
            self.get_logger().error("알 수 없는 waypoint 이름 '%s'" % name)
            self._publish_trip_state('FAILED')
            return
        self._send_pose_goal(yaw_to_pose(self.get_clock(), name, wp))

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 목표가 거부되었습니다')
            self._publish_trip_state('FAILED')
            self.goal_handle = None
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        status = future.result().status
        was_returning = (self.trip_state == 'RETURNING')
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            if was_returning:
                self.current_destination = None
                self._publish_trip_state('IDLE')
            else:
                self._publish_trip_state('ARRIVED')
        elif status == GoalStatus.STATUS_CANCELED:
            pass  # 취소를 시작한 쪽(정지/지원호출/수동전환)이 이미 trip_state 를 설정함
        else:
            self._publish_trip_state('FAILED')

    # ---------------- UI 명령 핸들러 ----------------

    def on_amcl_pose(self, msg):
        self.latest_amcl_pose = msg

    def on_select_destination(self, msg):
        name = msg.data
        if name not in self.waypoints:
            self.get_logger().error("알 수 없는 목적지 '%s'" % name)
            return
        if self.latest_amcl_pose is None:
            self.get_logger().error('초기 위치(/amcl_pose)가 아직 없어 출발 지점을 기록할 수 없습니다')
            return
        self.trip_start_pose = self._pose_stamped_from_amcl(self.latest_amcl_pose)
        self.current_destination = name
        # 수동제어(대기 화면) 중에 목적지를 바로 고를 수도 있으므로, nav2 출력이
        # safety_monitor 에서 걸러지지 않도록 여기서 자동 모드로 되돌려놓는다.
        self._publish_control_mode('AUTO')
        self._send_goal_to_waypoint(name)
        self._publish_trip_state('NAVIGATING')

    def on_return_to_start(self, msg):
        if not msg.data:
            return
        if self.trip_start_pose is None:
            self.get_logger().error('기록된 출발 위치가 없어 반납할 수 없습니다')
            return
        self._cancel_active_goal()
        self._publish_control_mode('AUTO')
        self._send_pose_goal(self.trip_start_pose)
        self._publish_trip_state('RETURNING')

    def on_set_manual_mode(self, msg):
        if msg.data:
            self._cancel_active_goal()
            self._publish_control_mode('MANUAL')
            self._publish_trip_state('MANUAL')
        else:
            self._publish_control_mode('AUTO')
            self._publish_trip_state('IDLE')

    def on_joystick(self, msg):
        if self.control_mode != 'MANUAL' or self.estop:
            return
        self.cmd_vel_manual_pub.publish(msg)

    def _trigger_stop(self, notify):
        self.pre_stop_state = self.trip_state
        self._cancel_active_goal()
        self._publish_estop(True)
        self._publish_trip_state('STOPPED')
        if notify:
            self._publish_support_call('support')

    def on_stop(self, msg):
        if msg.data:
            self._trigger_stop(notify=False)

    def on_support_call(self, msg):
        if msg.data:
            self._trigger_stop(notify=True)

    def on_resume(self, msg):
        if not msg.data:
            return
        self._publish_estop(False)
        prev = self.pre_stop_state
        self.pre_stop_state = None
        if prev == 'NAVIGATING' and self.current_destination in self.waypoints:
            self._send_goal_to_waypoint(self.current_destination)
            self._publish_trip_state('NAVIGATING')
        elif prev == 'MANUAL':
            self._publish_trip_state('MANUAL')
        elif prev == 'ARRIVED':
            self._publish_trip_state('ARRIVED')
        else:
            self._publish_trip_state('IDLE')

    # ---------------- 웨이포인트 관리 (관리자 페이지) ----------------

    def on_waypoint_add(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().error('잘못된 waypoint_add 페이로드: %r' % msg.data)
            return
        name = data.get('name')
        if not name:
            self.get_logger().error('waypoint_add 에 name 이 없습니다')
            return
        if 'x' in data and 'y' in data:
            x, y = float(data['x']), float(data['y'])
            yaw = float(data.get('yaw', 0.0))
        else:
            if self.latest_amcl_pose is None:
                self.get_logger().error('현재 위치(/amcl_pose)가 없어 waypoint 를 저장할 수 없습니다')
                return
            p = self.latest_amcl_pose.pose.pose
            x, y = p.position.x, p.position.y
            yaw = _yaw_from_quaternion(p.orientation)
        self.waypoints[name] = {'x': x, 'y': y, 'yaw': yaw}
        save_waypoints(self.waypoints_file, self.waypoints)
        self._publish_waypoints_list()

    def on_waypoint_delete(self, msg):
        name = msg.data
        if name in self.waypoints:
            del self.waypoints[name]
            save_waypoints(self.waypoints_file, self.waypoints)
            self._publish_waypoints_list()


def main(args=None):
    rclpy.init(args=args)
    node = UiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
