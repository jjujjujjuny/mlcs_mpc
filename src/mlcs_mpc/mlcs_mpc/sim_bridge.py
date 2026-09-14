#!/usr/bin/env python3
"""
sim_bridge — f1tenth_gym_ros 시뮬레이터 ↔ mlcs_mpc 어댑터.

시뮬의 ground-truth 오도메트리(/ego_racecar/odom)를 mocap_bridge 가 내는 것과
**똑같은 형식**(/mpc/state + /mocap/valid)으로 바꿔준다.

■ 왜 이 어댑터가 중요한가

  시뮬과 실차에서 mpc_node 가 **한 줄도 다르지 않게** 돌아간다. 컨트롤러가
  "시뮬용 코드 / 실차용 코드" 로 갈라지면 시뮬에서 검증한 것이 실차에서
  성립한다는 보장이 사라진다. 차이를 이 파일 하나에 가둬 둔다.

  시뮬에는 mocap 이 없으므로 /mocap/valid 는 항상 True 로 낸다 — 시뮬
  오도메트리는 끊기지 않기 때문이다.

■ 시뮬과 실차의 진짜 차이 (이 어댑터가 숨기지 못하는 것)

  시뮬 odom 은 **완벽하다**. 노이즈도 지연도 없다. 실차 mocap 은 1mm 노이즈와
  네트워크 지연이 있다. 그래서 시뮬에서 잘 도는 게인이 실차에서 떨릴 수 있다.
  이걸 미리 보려면 add_noise 파라미터를 켠다 — 실차 조건을 흉내 내서
  게인이 노이즈에 얼마나 민감한지 시뮬에서 먼저 확인할 수 있다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SimBridge(Node):

    def __init__(self):
        super().__init__('sim_bridge')

        self.declare_parameter('sim_odom_topic', '/ego_racecar/odom')
        self.declare_parameter('auto_enable', True)
        # 실차 mocap 조건 흉내 (게인 민감도를 시뮬에서 미리 보기 위함)
        self.declare_parameter('add_noise', False)
        self.declare_parameter('noise_pos', 0.001)   # 1mm
        self.declare_parameter('noise_yaw', 0.002)   # ~0.1도

        topic = self.get_parameter('sim_odom_topic').value
        self.add_noise = bool(self.get_parameter('add_noise').value)
        self.n_pos = float(self.get_parameter('noise_pos').value)
        self.n_yaw = float(self.get_parameter('noise_yaw').value)
        self.rng = np.random.default_rng(0)

        self.state_pub = self.create_publisher(Odometry, '/mpc/state', 10)
        self.valid_pub = self.create_publisher(Bool, '/mocap/valid', 10)
        self.enable_pub = self.create_publisher(Bool, '/mpc/enabled', LATCHED)

        self.create_subscription(Odometry, topic, self._cb, 10)
        # valid 는 주기적으로 — mpc_node 가 늦게 떠도 받는다
        self.create_timer(0.1, lambda: self.valid_pub.publish(Bool(data=True)))

        if self.get_parameter('auto_enable').value:
            # 래치라 나중에 뜨는 구독자도 받는다
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info('auto_enable — /mpc/enabled=true 발행')

        self.get_logger().info(
            f'sim_bridge: {topic} → /mpc/state'
            + (f' (노이즈 주입 {self.n_pos*1000:.1f}mm)' if self.add_noise else ''))

    def _cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'map'
        out.child_frame_id = 'base_link'
        out.pose = msg.pose
        out.twist = msg.twist

        if self.add_noise:
            out.pose.pose.position.x += self.rng.normal(0, self.n_pos)
            out.pose.pose.position.y += self.rng.normal(0, self.n_pos)
            q = out.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            yaw += self.rng.normal(0, self.n_yaw)
            out.pose.pose.orientation.z = math.sin(yaw / 2)
            out.pose.pose.orientation.w = math.cos(yaw / 2)

        self.state_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
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
