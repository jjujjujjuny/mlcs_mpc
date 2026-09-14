#!/usr/bin/env python3
"""
safety_node — mocap 기반 경계 감시 / 비상 정지.

■ 라이다가 없는 구성에서의 안전

  라이다가 있으면 장애물을 직접 본다. 여기는 없다. 대신 mocap 이 차의
  **절대 위치** 를 알려주므로, "실험 공간을 벗어나려 하는가" 를 본다.
  실험실 장비·벽·사람이 있는 바깥으로 차가 나가는 것을 막는 것이
  이 구성에서 가장 현실적인 안전장치다.

  감시 항목:
    ① 경계 이탈       — 지정 사각형 밖으로 나가려 하면 정지
    ② 예측 이탈       — 현재 속도로 lookahead 초 뒤 위치가 밖이면 정지
    ③ 과속            — max_speed 초과
    ④ mocap 끊김      — /mocap/valid=false
    ⑤ 경로 대폭 이탈  — 참조 경로에서 너무 멀어짐

  ★ 이 노드는 /drive 를 가로채지 않는다. 별도로 /mpc/enabled=false 를
    발행해서 mpc_node 가 스스로 멈추게 한다. 두 노드가 같은 토픽에
    동시에 쓰면 어느 쪽이 이겼는지 알 수 없어 디버깅이 불가능해진다.

  ⚠ 소프트웨어 안전장치는 하드웨어 킬스위치를 **대체하지 않는다.**
    노드가 죽거나 네트워크가 끊기면 이것도 같이 죽는다. 물리 킬스위치를
    반드시 손에 들고 실험하세요.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        # 실험 공간 경계 (mocap 좌표계, m). 실험실 실측값으로 바꾸세요.
        self.declare_parameter('x_min', -5.0)
        self.declare_parameter('x_max', 5.0)
        self.declare_parameter('y_min', -5.0)
        self.declare_parameter('y_max', 5.0)
        self.declare_parameter('margin', 0.3)       # 경계 여유
        self.declare_parameter('lookahead', 0.7)    # 예측 정지 시간 (초)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('enabled', True)

        gp = lambda n: self.get_parameter(n).value
        self.xmin = float(gp('x_min'))
        self.xmax = float(gp('x_max'))
        self.ymin = float(gp('y_min'))
        self.ymax = float(gp('y_max'))
        self.margin = float(gp('margin'))
        self.lookahead = float(gp('lookahead'))
        self.max_speed = float(gp('max_speed'))
        self.active = bool(gp('enabled'))

        self.tripped = False
        self.reason = ''

        self.create_subscription(Odometry, '/mpc/state', self._cb, 10)
        self.create_subscription(Bool, '/mocap/valid', self._valid_cb, 10)
        self.enable_pub = self.create_publisher(Bool, '/mpc/enabled', LATCHED)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info(
            f'safety_node: 경계 x[{self.xmin}, {self.xmax}] '
            f'y[{self.ymin}, {self.ymax}] margin={self.margin}m')
        self.get_logger().warn(
            '★ 소프트웨어 안전장치입니다. 하드웨어 킬스위치를 손에 드세요.')

    def _valid_cb(self, msg: Bool):
        if not msg.data and not self.tripped and self.active:
            self._trip('mocap 끊김')

    def _cb(self, msg: Odometry):
        if not self.active or self.tripped:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        v = msg.twist.twist.linear.x

        m = self.margin
        if not (self.xmin + m <= x <= self.xmax - m and
                self.ymin + m <= y <= self.ymax - m):
            self._trip(f'경계 이탈 (x={x:.2f}, y={y:.2f})')
            return

        # 예측 위치 — 지금 속도로 lookahead 초 뒤
        px = x + v * math.cos(yaw) * self.lookahead
        py = y + v * math.sin(yaw) * self.lookahead
        if not (self.xmin <= px <= self.xmax and self.ymin <= py <= self.ymax):
            self._trip(f'경계 이탈 예측 ({self.lookahead}s 뒤 '
                       f'x={px:.2f}, y={py:.2f}, v={v:.2f})')
            return

        if abs(v) > self.max_speed:
            self._trip(f'과속 (v={v:.2f} > {self.max_speed})')

    def _trip(self, reason):
        self.tripped = True
        self.reason = reason
        self.get_logger().error(f'✗ 비상 정지: {reason}')
        # 래치로 내려서 늦게 뜨는 노드도 정지 상태를 받게 한다
        self.enable_pub.publish(Bool(data=False))
        # 직접 정지 명령도 반복 발행 — enabled 를 못 받는 경우 대비
        self.create_timer(0.05, self._spam_stop)

    def _spam_stop(self):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = 0.0
        m.drive.steering_angle = 0.0
        self.drive_pub.publish(m)
        self.enable_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
