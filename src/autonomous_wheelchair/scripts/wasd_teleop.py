#!/usr/bin/env python3
"""WASD 키보드로 /cmd_vel 을 조종하는 저속 테스트용 텔레옵.

w/s: 전/후진, a/d: 좌/우 회전, 그 외 아무 키(스페이스 포함): 즉시 정지, q: 종료.
한 번 누르면 그 속도가 유지된다 (계속 누르고 있을 필요 없음).
base_driver 의 cmd_timeout(기본 0.5s) 워치독이 있으므로 이 스크립트가 죽거나
네트워크가 끊겨도 0.5초 안에 로봇은 자동으로 멈춘다.
"""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

LINEAR_SPEED = 0.15   # m/s, 사람이 타는 저속 테스트용
ANGULAR_SPEED = 0.3   # rad/s
PUBLISH_RATE = 10.0   # Hz


class WasdTeleop(Node):

    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear = 0.0
        self.angular = 0.0
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

    def set_key(self, key):
        if key == 'w':
            self.linear, self.angular = LINEAR_SPEED, 0.0
        elif key == 's':
            self.linear, self.angular = -LINEAR_SPEED, 0.0
        elif key == 'a':
            self.linear, self.angular = 0.0, ANGULAR_SPEED
        elif key == 'd':
            self.linear, self.angular = 0.0, -ANGULAR_SPEED
        else:
            self.linear, self.angular = 0.0, 0.0


def get_key(settings, timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return ''


def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = WasdTeleop()

    print(__doc__)
    print('w/a/s/d = 이동, 그 외 아무 키 = 정지, q = 종료\n')

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings, timeout=0.1)
            if key == 'q' or key == '\x03':  # q 또는 Ctrl-C
                break
            if key:
                node.set_key(key.lower())
                state = 'STOP' if node.linear == 0.0 and node.angular == 0.0 else \
                    f'linear={node.linear:+.2f} angular={node.angular:+.2f}'
                print(f'\r{state}          ', end='', flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.linear, node.angular = 0.0, 0.0
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()
        print('\n종료 (정지 명령 전송함)')


if __name__ == '__main__':
    main()
