#!/usr/bin/env python3
"""라이다 기반 최종 안전 계층. /cmd_vel_raw + /scan -> /cmd_vel."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('slow_distance', 1.60)
        self.declare_parameter('robot_half_width', 0.40)
        self.declare_parameter('front_offset', 0.55)
        self.declare_parameter('rear_offset', 1.00)
        self.declare_parameter('min_scale', 0.15)
        self.declare_parameter('smooth_alpha', 0.25)
        self.declare_parameter('scan_timeout', 0.6)
        self.declare_parameter('cmd_timeout', 0.5)

        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.robot_half_width = self.get_parameter('robot_half_width').value
        self.front_offset = self.get_parameter('front_offset').value
        self.rear_offset = self.get_parameter('rear_offset').value
        self.min_scale = self.get_parameter('min_scale').value
        self.smooth_alpha = self.get_parameter('smooth_alpha').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.last_scan = None
        self.last_scan_time = None

        self.out_v = 0.0
        self.out_w = 0.0

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, scan_qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dist_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.state_pub = self.create_publisher(String, '/safety_state', 10)

        self.timer = self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info('safety_monitor started')

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _closest_corridor_distance(self, scan: LaserScan, forward: bool):
        min_dist = None
        offset = self.front_offset if forward else self.rear_offset

        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment

            if not math.isfinite(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            x = r * math.cos(a)
            y = r * math.sin(a)

            if abs(y) > self.robot_half_width:
                continue

            if forward:
                if x <= 0.0:
                    continue
                dist = x - offset
            else:
                if x >= 0.0:
                    continue
                dist = (-x) - offset

            if dist < 0.0:
                dist = 0.0

            if min_dist is None or dist < min_dist:
                min_dist = dist

        return min_dist

    def _scale_from_distance(self, distance):
        if distance is None:
            return 1.0
        if distance <= self.stop_distance:
            return 0.0
        if distance >= self.slow_distance:
            return 1.0
        t = (distance - self.stop_distance) / (self.slow_distance - self.stop_distance)
        s = smoothstep(t)
        return self.min_scale + (1.0 - self.min_scale) * s

    def _smooth(self, target, prev):
        # 감속(목표 크기가 더 작음)은 즉시 반영, 가속은 저역통과 필터로 서서히
        if abs(target) <= abs(prev):
            return target
        return prev + self.smooth_alpha * (target - prev)

    def control_loop(self):
        now = self.get_clock().now()

        cmd_ok = (self.last_cmd_time is not None and
                  (now - self.last_cmd_time).nanoseconds / 1e9 <= self.cmd_timeout)
        scan_ok = (self.last_scan_time is not None and
                   (now - self.last_scan_time).nanoseconds / 1e9 <= self.scan_timeout)

        if not cmd_ok or not scan_ok:
            self.out_v = 0.0
            self.out_w = 0.0
            self._publish(0.0, 0.0, None, 'TIMEOUT')
            return

        v_raw = self.last_cmd.linear.x
        w_raw = self.last_cmd.angular.z

        if abs(v_raw) < 0.02:
            # 제자리 회전: 직사각 통로 판정 예외 (전진/후진이 아니므로 장애물 감속 미적용)
            distance = None
            scale = 1.0
            state = 'ROTATE'
        else:
            forward = v_raw > 0.0
            distance = self._closest_corridor_distance(self.last_scan, forward)
            scale = self._scale_from_distance(distance)
            if distance is not None and distance <= self.stop_distance:
                state = 'STOP'
            elif distance is not None and distance < self.slow_distance:
                state = 'SLOW'
            else:
                state = 'OK'

        target_v = v_raw * scale
        target_w = w_raw * scale

        self.out_v = self._smooth(target_v, self.out_v)
        self.out_w = self._smooth(target_w, self.out_w)

        self._publish(self.out_v, self.out_w, distance, state)

    def _publish(self, v, w, distance, state):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        dist_msg = Float32()
        dist_msg.data = float(distance) if distance is not None else float('inf')
        self.dist_pub.publish(dist_msg)

        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
