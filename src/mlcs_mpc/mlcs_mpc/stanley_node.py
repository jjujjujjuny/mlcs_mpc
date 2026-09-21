#!/usr/bin/env python3
"""
stanley_node — Stanley 경로 추종 컨트롤러.

  구독: /mpc/state (Odometry)  ← mocap_bridge / sim_bridge (MPC 와 동일)
        /mocap/valid, /mpc/enabled, /mpc/manual
  발행: /drive (AckermannDriveStamped)
        /mpc/reference_path (RViz)

■ 왜 Stanley 를 먼저 쓰는가

  MPC 는 이미 시뮬 검증이 끝났지만, 실차에서 처음 돌릴 때는 **어디가
  문제인지 분리**하는 것이 중요하다. 차가 경로를 못 따라갈 때 원인이
  ① mocap 좌표계 ② 조향 부호/게인 ③ 제어 알고리즘 중 무엇인지
  MPC 로는 구분이 어렵다 — 튜닝 파라미터가 8개나 되기 때문이다.

  Stanley 는 게인이 **사실상 2개**(k_e, k_soft)뿐이고 수식이 한 줄이라,
  배관(mocap→제어→구동)이 맞는지 확인하는 데 적합하다. 여기서 돌면
  같은 트랙에서 MPC 와 비교할 수 있는 베이스라인도 생긴다.

■ Stanley 제어법칙

      δ = θ_e + atan( k_e · e / (k_soft + v) )
          └헤딩오차┘  └────횡오차 보정────┘

  ★ 기준점이 **앞축 중심**이다 (MPC 의 뒤축 기준과 다르다).
    Stanley 가 원래 그렇게 유도됐고, 이 덕분에 횡오차와 헤딩오차를
    하나의 조향각으로 동시에 없앨 수 있다. 뒤축 기준으로 쓰면
    수렴성이 보장되지 않는다.

    /mpc/state 는 뒤축 중심이므로 여기서 앞축으로 옮겨서 쓴다:
        front = rear + L·[cos ψ, sin ψ]

■ 안전 — MPC 노드와 동일한 3중 장치

  ① mocap 무효/끊김 ② /mpc/enabled=false ③ 상태 stale
  추가로 /mpc/manual (조이스틱이 조종권을 가져감) 도 본다.
  하나라도 걸리면 speed=0 을 계속 발행한다 (VESC 는 마지막 명령을
  유지하므로 발행을 멈추는 것으로는 안 선다).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped

from mlcs_mpc.path_manager import PathManager
from mlcs_mpc.vehicle_model import VehicleParams, normalize_angle

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StanleyNode(Node):

    def __init__(self):
        super().__init__('stanley_node')

        # ── 차량 ─────────────────────────────────────────────────────
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steer', 0.36)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('min_speed', 0.3)

        # ── Stanley 게인 ─────────────────────────────────────────────
        # k_e  : 횡오차를 얼마나 세게 잡을 것인가.
        #        크면 빨리 붙지만 진동한다. 1.0~3.0 범위에서 조정.
        # k_soft: 저속에서 분모가 0 이 되는 것을 막는다. 이게 없으면
        #        v→0 일 때 atan 안이 발산해 조향이 튄다.
        self.declare_parameter('k_e', 1.5)
        self.declare_parameter('k_soft', 0.5)
        # 헤딩오차 항의 가중. 1.0 이 표준 Stanley.
        self.declare_parameter('k_heading', 1.0)
        # 조향 변화율 제한 (rad/s). 서보가 따라갈 수 있는 범위로.
        self.declare_parameter('max_steer_rate', 3.2)

        # ── 경로 / 속도 ──────────────────────────────────────────────
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('curvature_slowdown', 1.5)
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('mu', 0.35)

        # ── 안전 ─────────────────────────────────────────────────────
        self.declare_parameter('state_timeout', 0.2)
        self.declare_parameter('require_enable', True)
        self.declare_parameter('bench', False)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('debug', False)

        gp = lambda n: self.get_parameter(n).value

        self.L = float(gp('wheelbase'))
        self.max_steer = float(gp('max_steer'))
        self.max_speed = float(gp('max_speed'))
        self.min_speed = float(gp('min_speed'))
        self.k_e = float(gp('k_e'))
        self.k_soft = float(gp('k_soft'))
        self.k_head = float(gp('k_heading'))
        self.max_dsteer = float(gp('max_steer_rate'))
        self.state_timeout = float(gp('state_timeout'))
        self.require_enable = bool(gp('require_enable'))
        self.bench = bool(gp('bench'))
        self.debug = bool(gp('debug'))

        params = VehicleParams(wheelbase=self.L, max_speed=self.max_speed,
                               min_speed=self.min_speed,
                               max_steer=self.max_steer, mu=float(gp('mu')))
        wp = gp('waypoint_file')
        self.path = PathManager(
            wp, target_speed=float(gp('target_speed')),
            curvature_slowdown=float(gp('curvature_slowdown')),
            closed_loop=bool(gp('closed_loop')),
            params=params, logger=self.get_logger())
        if not self.path.ok:
            self.get_logger().error(
                f'✗ 웨이포인트를 못 읽었습니다: "{wp}" — 정지 상태로 대기합니다.')

        # ── 상태 ─────────────────────────────────────────────────────
        self.state = None
        self.last_state_t = None
        self.mocap_valid = False
        self.enabled = not self.require_enable
        self.manual = False
        self.cmd_steer = 0.0
        self.stop_reason = 'init'
        self.dt = 1.0 / float(gp('control_rate'))

        # ── 통신 ─────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/mpc/state', self._state_cb, 10)
        self.create_subscription(Bool, '/mocap/valid', self._valid_cb, 10)
        self.create_subscription(Bool, '/mpc/enabled', self._enable_cb, LATCHED)
        self.create_subscription(Bool, '/mpc/manual', self._manual_cb, LATCHED)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.ref_pub = self.create_publisher(Path, '/mpc/reference_path', 1)
        self.err_pub = self.create_publisher(Float32, '/stanley/cross_track', 1)

        self.create_timer(self.dt, self._control_loop)
        self.create_timer(2.0, self._publish_path)

        self.get_logger().info(
            f'stanley_node 시작 — k_e={self.k_e} k_soft={self.k_soft} '
            f'rate={1.0/self.dt:.0f}Hz max_speed={self.max_speed}')
        if self.bench:
            self.get_logger().warn('★ bench=true — 속도는 항상 0 으로 발행됩니다')
        if self.require_enable:
            self.get_logger().info('출발 게이트 대기: /mpc/enabled=true')

    # ────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.state = np.array([p.position.x, p.position.y,
                               yaw_from_quat(p.orientation),
                               msg.twist.twist.linear.x])
        self.last_state_t = self.get_clock().now().nanoseconds * 1e-9

    def _valid_cb(self, msg):
        self.mocap_valid = bool(msg.data)

    def _enable_cb(self, msg):
        was = self.enabled
        self.enabled = bool(msg.data)
        if self.enabled and not was:
            self.get_logger().info('▶ 출발 허가 — Stanley 주행 시작')
        elif was and not self.enabled:
            self.get_logger().info('■ 정지 명령 수신')

    def _manual_cb(self, msg):
        was = self.manual
        self.manual = bool(msg.data)
        if self.manual and not was:
            self.get_logger().warn('■ 사람이 조종을 가져갔습니다 — 컨트롤러 정지')
        elif was and not self.manual:
            self.get_logger().info('▶ 조종권 반환 — 재개')

    # ────────────────────────────────────────────────────────────────
    def _blocked(self):
        if not self.path.ok:
            return '웨이포인트 없음'
        if self.state is None or self.last_state_t is None:
            return '상태 수신 전'
        if not self.mocap_valid:
            return 'mocap 무효'
        age = self.get_clock().now().nanoseconds * 1e-9 - self.last_state_t
        if age > self.state_timeout:
            return f'상태 오래됨 ({age:.2f}s)'
        if self.manual:
            return '수동 조종 중'
        if not self.enabled:
            return '출발 대기'
        return None

    def _control_loop(self):
        reason = self._blocked()
        if reason is not None:
            if reason != self.stop_reason:
                self.get_logger().warn(f'정지: {reason}')
                self.stop_reason = reason
                self.cmd_steer = 0.0
            self._publish(0.0, 0.0)
            return
        self.stop_reason = None

        x, y, yaw, v = self.state

        # ★ Stanley 는 앞축 기준이다 (docstring 참고)
        fx = x + self.L * math.cos(yaw)
        fy = y + self.L * math.sin(yaw)

        i = self.path.nearest_index(fx, fy)
        px, py = self.path.xy[i]
        ref_yaw = self.path.heading[i]
        v_ref = float(self.path.v_ref[i])

        # ── 횡오차 (부호 포함) ──────────────────────────────────────
        #   경로 접선의 왼쪽이 +. 경로→차 벡터를 접선의 법선에 투영한다.
        dx, dy = fx - px, fy - py
        e = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy

        # ── 헤딩 오차 ───────────────────────────────────────────────
        theta_e = normalize_angle(ref_yaw - yaw)

        # ── Stanley 제어법칙 ────────────────────────────────────────
        #   부호: 차가 경로 왼쪽(e>0)이면 오른쪽으로 꺾어야 하므로 -atan
        steer = self.k_head * theta_e + math.atan2(
            -self.k_e * e, self.k_soft + abs(v))

        # 조향 변화율 제한 — 서보가 못 따라가는 명령을 내지 않는다
        max_d = self.max_dsteer * self.dt
        steer = float(np.clip(steer, self.cmd_steer - max_d,
                              self.cmd_steer + max_d))
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        self.cmd_steer = steer

        speed = float(np.clip(v_ref, self.min_speed, self.max_speed))

        self._publish(speed, steer)
        self.err_pub.publish(Float32(data=float(e)))

        if self.debug:
            self.get_logger().info(
                f'e={e*100:+6.1f}cm  θe={math.degrees(theta_e):+6.1f}°  '
                f'δ={math.degrees(steer):+6.1f}°  v={speed:.2f} (실제 {v:.2f})',
                throttle_duration_sec=0.5)

    # ────────────────────────────────────────────────────────────────
    def _publish(self, speed, steer):
        if self.bench:
            speed = 0.0
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steer)
        self.drive_pub.publish(m)

    def _publish_path(self):
        if not self.path.ok:
            return
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'map'
        for xx, yy in self.path.xy:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(xx)
            ps.pose.position.y = float(yy)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self.ref_pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
