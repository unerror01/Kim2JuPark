#!/usr/bin/env python3
"""ESP32-S3 조이스틱 에뮬레이터와 시리얼로 통신하는 베이스 드라이버.

ESP32-S3 는 원래 조이스틱 신호를 대신해 I2C DAC(0x60=좌우, 0x61=전후진, 중립
2397, 0~4095 카운트)로 전압을 주입하고 릴레이(IN1=GPIO4, IN2=GPIO5, LOW=자율주행)로
수동/자율 경로를 전환한다. DAC 캘리브레이션은 전부 ESP32 펌웨어 쪽 책임이므로,
이 노드는 -1.0~1.0 로 정규화한 throttle/steering 값만 시리얼로 보낸다.

★ 엔코더 미장착 ★
아직 휠 엔코더가 없어서 실제 바퀴 회전을 측정할 방법이 없다. 그래서 /wheel/odometry
와 /joint_states 는 "명령한 속도를 그대로 따라간다"고 가정하는 open-loop 적분치이다.
엔코더가 생기면 이 open-loop 적분을 실측 틱 기반 적분으로 교체할 것.
"""

import math

import rclpy
from rclpy.node import Node

import serial

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class BaseDriver(Node):

    def __init__(self):
        super().__init__('base_driver')

        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_linear', 0.6)
        self.declare_parameter('max_angular', 0.9)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('wheel_radius', 0.30)
        self.declare_parameter('wheel_separation', 0.56)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.invert_linear = self.get_parameter('invert_linear').value
        self.invert_angular = self.get_parameter('invert_angular').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.control_rate = self.get_parameter('control_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = None
        self.was_timed_out = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.last_loop_time = self.get_clock().now()

        self.ser = None
        self._open_serial()

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('base_driver started (port=%s, baud=%d)' %
                                (self.serial_port, self.baudrate))

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.2)
        except serial.SerialException as exc:
            self.get_logger().error('시리얼 포트를 열 수 없습니다: %s' % exc)
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_v = clamp(msg.linear.x, -self.max_linear, self.max_linear)
        self.cmd_w = clamp(msg.angular.z, -self.max_angular, self.max_angular)
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        if self.last_cmd_time is None:
            timed_out = True
        else:
            elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
            timed_out = elapsed > self.cmd_timeout

        if timed_out:
            if not self.was_timed_out:
                self.get_logger().warn('cmd_vel 타임아웃 -> 정지')
                self.was_timed_out = True
            v, w = 0.0, 0.0
        else:
            self.was_timed_out = False
            v, w = self.cmd_v, self.cmd_w

        self._send_motor(v, w)

        if dt > 0.0:
            self._integrate_odometry(v, w, dt, now)
            self._publish_joint_states(v, w, dt, now)

    def _send_motor(self, v, w):
        # ★★★ ESP32 펌웨어 프로토콜에 맞게 수정 ★★★
        # 여기서는 -1.0~1.0 로 정규화한 throttle(전후진)/steering(좌우) 값만 보낸다.
        # DAC count 변환(중립 2397, 0~4095, 0x60/0x61 I2C)은 ESP32 펌웨어에서 수행할 것.
        # 실제 하드웨어에서 전진/좌회전 방향이 반대로 나오면 invert_linear/invert_angular
        # 파라미터로 뒤집을 것 (DAC 전압-방향 극성은 실측 전까지 알 수 없음).
        throttle = v / self.max_linear if self.max_linear != 0.0 else 0.0
        steering = w / self.max_angular if self.max_angular != 0.0 else 0.0

        if self.invert_linear:
            throttle = -throttle
        if self.invert_angular:
            steering = -steering

        throttle = clamp(throttle, -1.0, 1.0)
        steering = clamp(steering, -1.0, 1.0)

        cmd = '<%.4f,%.4f>\n' % (throttle, steering)

        if self.ser is None or not self.ser.is_open:
            self._open_serial()
            return
        try:
            self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error('시리얼 송신 실패: %s' % exc)
            self.ser = None

    def _integrate_odometry(self, v, w, dt, stamp):
        # open-loop 적분: 엔코더가 없어 명령 속도를 그대로 따른다고 가정한다.
        dtheta = w * dt
        if abs(dtheta) > 1e-6:
            radius = v / w
            dx = radius * math.sin(dtheta)
            dy = radius * (1.0 - math.cos(dtheta))
        else:
            dx = v * dt
            dy = 0.0

        dx_global = dx * math.cos(self.theta) - dy * math.sin(self.theta)
        dy_global = dx * math.sin(self.theta) + dy * math.cos(self.theta)

        self.x += dx_global
        self.y += dy_global
        self.theta = math.atan2(
            math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # TF 는 발행하지 않는다 (map->odom->base_footprint 는 EKF/AMCL 담당)
        self.odom_pub.publish(odom)

    def _publish_joint_states(self, v, w, dt, stamp):
        v_left = v - w * self.wheel_separation / 2.0
        v_right = v + w * self.wheel_separation / 2.0

        self.left_wheel_angle += v_left / self.wheel_radius * dt
        self.right_wheel_angle += v_right / self.wheel_radius * dt

        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_angle, self.right_wheel_angle]
        js.velocity = [v_left / self.wheel_radius, v_right / self.wheel_radius]
        self.joint_pub.publish(js)

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self._send_motor(0.0, 0.0)
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
