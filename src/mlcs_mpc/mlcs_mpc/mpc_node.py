#!/usr/bin/env python3
"""
mpc_node — MPC 주행 컨트롤러 (ROS 2 노드).

  구독: /mpc/state (Odometry)   ← mocap_bridge 또는 sim_bridge
        /mocap/valid (Bool)      ← 위치추정 유효성
        /mpc/enabled (Bool)      ← 출발 게이트 (래치)
  발행: /drive (AckermannDriveStamped)
        /mpc/predicted_path, /mpc/reference_path (RViz)
        /mpc/solve_time (Float32)

■ MPC 출력 → 차량 명령 변환

  MPC 의 입력은 [가속도 a, 조향각속도 delta_dot] 인데, 차량이 받는 것은
  [속도 speed, 조향각 steering_angle] 이다. 그래서 적분해서 내보낸다:

      speed = v_now + a * dt
      steer = delta_now + delta_dot * dt

  ★ v_now/delta_now 를 어디서 가져오는가가 중요하다.
    측정된 실제 속도를 쓰면(= 폐루프 적분) mocap 속도 노이즈가 명령에
    바로 실린다. 반대로 명령값만 누적하면(= 개루프) 실제와 벌어진다.
    여기서는 **MPC 예측 궤적의 다음 스텝 값**을 쓴다 — 솔버가 이미
    "한 스텝 뒤에 이 상태가 된다" 고 계산해 둔 값이라 노이즈가 걸러져
    있으면서 모델과 일관된다.

■ 안전 — 세 겹

  ① mocap 무효/끊김      → 즉시 정지 (위치를 모르면 달리면 안 된다)
  ② /mpc/enabled = False → 정지 (출발 게이트, 기본 False)
  ③ 상태 오래됨(stale)   → 정지
  이 중 하나라도 걸리면 speed=0 을 계속 발행한다.

  ★ 정지 명령을 "한 번" 보내고 마는 게 아니라 계속 보낸다. f1tenth 의
    ackermann_mux 는 timeout 이 지나면 그 입력을 무시하는데, 정지를 한 번만
    보내고 침묵하면 mux 가 우리 채널을 죽은 것으로 보고 다른(조이스틱 등)
    입력을 통과시킬 수 있다.
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

from mlcs_mpc.vehicle_model import (
    VehicleParams, normalize_angle, X, Y, PSI, V, DELTA, A, DELTA_DOT)
from mlcs_mpc.mpc_solver import MPCSolver, MPCConfig
from mlcs_mpc.path_manager import PathManager

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MPCNode(Node):

    def __init__(self):
        super().__init__('mpc_node')

        # ── 차량 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_accel', 3.0)
        self.declare_parameter('max_decel', 4.0)
        self.declare_parameter('max_steer', 0.36)
        self.declare_parameter('max_steer_rate', 3.2)
        self.declare_parameter('mu', 0.35)
        # ↓ 운동학 모델은 안 쓰지만 config/vehicle.yaml 에 있는 값들.
        #   선언해 두지 않으면 ROS 2 가 "declared 되지 않은 파라미터" 로
        #   **노드 기동 자체를 실패시킨다.** 동역학 모델로 갈 때 쓴다.
        self.declare_parameter('mass', 3.5)
        self.declare_parameter('lf', 0.176)
        self.declare_parameter('lr', 0.144)

        # ── MPC 튜닝 ─────────────────────────────────────────────────
        self.declare_parameter('N', 20)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('w_pos', 10.0)
        self.declare_parameter('w_psi', 2.0)
        self.declare_parameter('w_v', 2.0)
        self.declare_parameter('w_a', 0.05)
        self.declare_parameter('w_ddelta', 0.5)
        self.declare_parameter('w_terminal', 20.0)
        self.declare_parameter('max_iter', 3)
        self.declare_parameter('backend', 'ilqr')

        # ── 경로 ─────────────────────────────────────────────────────
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('curvature_slowdown', 1.5)
        self.declare_parameter('closed_loop', True)

        # ── 안전 ─────────────────────────────────────────────────────
        self.declare_parameter('state_timeout', 0.2)
        self.declare_parameter('require_enable', True)
        # ★ 벤치(거치대) 모드 — 발행 직전에 속도를 0 으로 못박는다.
        #   26-jetson repo 에서 클램프에만 의존했다가 박스 위 차가 풀속도로
        #   돈 사고가 있었다(2026-08-09). 별도 스위치로 둔다.
        self.declare_parameter('bench', False)
        self.declare_parameter('control_rate', 20.0)

        gp = lambda n: self.get_parameter(n).value

        self.params = VehicleParams(
            wheelbase=float(gp('wheelbase')), max_speed=float(gp('max_speed')),
            min_speed=float(gp('min_speed')), max_accel=float(gp('max_accel')),
            max_decel=float(gp('max_decel')), max_steer=float(gp('max_steer')),
            max_steer_rate=float(gp('max_steer_rate')), mu=float(gp('mu')))

        self.cfg = MPCConfig(
            N=int(gp('N')), dt=float(gp('dt')), w_pos=float(gp('w_pos')),
            w_psi=float(gp('w_psi')), w_v=float(gp('w_v')),
            w_a=float(gp('w_a')), w_ddelta=float(gp('w_ddelta')),
            w_terminal=float(gp('w_terminal')), max_iter=int(gp('max_iter')))

        self.solver = MPCSolver(self.params, self.cfg, backend=gp('backend'))

        wp_file = gp('waypoint_file')
        self.path = PathManager(
            wp_file,
            target_speed=float(gp('target_speed')),
            curvature_slowdown=float(gp('curvature_slowdown')),
            closed_loop=bool(gp('closed_loop')),
            params=self.params,
            logger=self.get_logger())
        if not self.path.ok:
            self.get_logger().error(
                f'✗ 웨이포인트를 못 읽었습니다: "{wp_file}"\n'
                '  waypoint_file 파라미터를 확인하세요. 컨트롤러는 정지 상태로 대기합니다.')

        self.state_timeout = float(gp('state_timeout'))
        self.require_enable = bool(gp('require_enable'))
        self.bench = bool(gp('bench'))

        # ── 상태 ─────────────────────────────────────────────────────
        self.state = None           # [x, y, psi, v, delta]
        self.last_state_t = None
        self.mocap_valid = False
        self.enabled = not self.require_enable
        self.cmd_delta = 0.0        # 마지막으로 명령한 조향각
        self.stop_reason = 'init'

        # ── 통신 ─────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/mpc/state', self._state_cb, 10)
        self.create_subscription(Bool, '/mocap/valid', self._valid_cb, 10)
        self.create_subscription(Bool, '/mpc/enabled', self._enable_cb, LATCHED)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.pred_pub = self.create_publisher(Path, '/mpc/predicted_path', 1)
        self.ref_pub = self.create_publisher(Path, '/mpc/reference_path', 1)
        self.solve_pub = self.create_publisher(Float32, '/mpc/solve_time', 1)

        rate = float(gp('control_rate'))
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'mpc_node 시작 — N={self.cfg.N} dt={self.cfg.dt} '
            f'backend={gp("backend")} rate={rate}Hz')
        if self.bench:
            self.get_logger().warn('★ bench=true — 속도는 항상 0 으로 발행됩니다')
        if self.require_enable:
            self.get_logger().info('출발 게이트 대기: /mpc/enabled=true 를 기다립니다')

    # ────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: Odometry):
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        v = msg.twist.twist.linear.x
        # 조향각은 관측되지 않는다 — 마지막 명령값을 실제 조향으로 본다.
        # 서보 응답이 제어주기보다 빠르다는 가정이고, max_steer_rate 제약이
        # 그 가정을 실제로 성립시킨다.
        self.state = np.array([p.position.x, p.position.y, yaw, v,
                               self.cmd_delta])
        self.last_state_t = self.get_clock().now().nanoseconds * 1e-9

    def _valid_cb(self, msg: Bool):
        self.mocap_valid = bool(msg.data)

    def _enable_cb(self, msg: Bool):
        was = self.enabled
        self.enabled = bool(msg.data)
        if self.enabled and not was:
            self.get_logger().info('▶ 출발 허가 — MPC 주행 시작')
            self.solver.reset()
        elif was and not self.enabled:
            self.get_logger().info('■ 정지 명령 수신')

    # ────────────────────────────────────────────────────────────────
    def _blocked(self):
        """주행하면 안 되는 이유가 있으면 문자열, 없으면 None."""
        if not self.path.ok:
            return '웨이포인트 없음'
        if self.state is None:
            return '상태 수신 전'
        if not self.mocap_valid:
            return 'mocap 무효'
        if self.last_state_t is None:
            return '상태 수신 전'
        age = self.get_clock().now().nanoseconds * 1e-9 - self.last_state_t
        if age > self.state_timeout:
            return f'상태 오래됨 ({age:.2f}s)'
        if not self.enabled:
            return '출발 대기'
        return None

    def _control_loop(self):
        reason = self._blocked()
        if reason is not None:
            if reason != self.stop_reason:
                self.get_logger().warn(f'정지: {reason}')
                self.stop_reason = reason
                self.solver.reset()
                self.cmd_delta = 0.0
            self._publish_drive(0.0, 0.0)
            return

        if self.stop_reason is not None:
            self.stop_reason = None

        x0 = self.state.copy()
        ref = self.path.reference(x0, self.cfg.N, self.cfg.dt)

        try:
            u0, xs = self.solver.solve(x0, ref)
        except Exception as e:
            # 솔버가 어떤 이유로든 실패하면 달리지 않는다.
            self.get_logger().error(f'솔버 실패 — 정지: {e}')
            self._publish_drive(0.0, 0.0)
            return

        # 예측 궤적의 다음 스텝을 명령으로 (위 docstring 참고)
        speed = float(xs[1][V])
        steer = float(xs[1][DELTA])

        speed = float(np.clip(speed, self.params.min_speed,
                              self.params.max_speed))
        steer = float(np.clip(steer, -self.params.max_steer,
                              self.params.max_steer))
        self.cmd_delta = steer

        self._publish_drive(speed, steer)
        self._publish_viz(xs, ref)
        self.solve_pub.publish(Float32(data=float(self.solver.last_solve_ms)))

    # ────────────────────────────────────────────────────────────────
    def _publish_drive(self, speed, steer):
        if self.bench:
            speed = 0.0
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steer)
        self.drive_pub.publish(m)

    def _publish_viz(self, xs, ref):
        now = self.get_clock().now().to_msg()

        def mk(points):
            p = Path()
            p.header.stamp = now
            p.header.frame_id = 'map'
            for pt in points:
                ps = PoseStamped()
                ps.header = p.header
                ps.pose.position.x = float(pt[0])
                ps.pose.position.y = float(pt[1])
                ps.pose.orientation.w = 1.0
                p.poses.append(ps)
            return p

        self.pred_pub.publish(mk(xs[:, :2]))
        self.ref_pub.publish(mk(ref[:, :2]))


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료할 때 정지 명령을 남긴다 — Ctrl+C 로 죽였는데 마지막 속도
        # 명령이 mux 에 남아 차가 계속 가는 상황을 막는다.
        try:
            node._publish_drive(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
