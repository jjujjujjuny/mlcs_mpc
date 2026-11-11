#!/usr/bin/env python3
"""
lidar_bridge — Cartographer 위치추정(/tracked_pose) → 컨트롤러용 상태 (/mpc/state).

  mocap_bridge 와 같은 토픽을 같은 형식으로 낸다. 그래서 hmpc_node, data_logger 는 위치를
  어디서 받는지 모른다. 대회처럼 mocap 이 없는 곳에서 쓴다.

    /tracked_pose (PoseStamped, map 프레임, base_link)  →  /mpc/state (Odometry)
                                                        →  /mocap/valid (Bool, 이름은 호환용)

■ mocap_bridge 와 다른 점

  · TF 를 내지 않는다. map→odom 은 Cartographer 가 낸다. 같이 내면 base_link 의 부모가 둘이 된다.
  · 기준점은 Cartographer 의 tracking_frame(base_link)이다. mocap 과 맞춘 기록(1108_5)에서
    차이는 앞 2.4 cm, 왼 0.5 cm, yaw -1.1° 였다. 보정하지 않는다.
  · 라이다 위치는 mocap 보다 시끄러워서 측정 잡음을 크게 잡는다. 지연은 약 27 ms (같은 기록).
  · 멈춘 pose 검사는 끈다. Cartographer 는 새 스캔이 없으면 같은 자세를 다시 낼 수 있다.
    튄 pose 와 끊김(timeout)만 본다.
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from mlcs_mpc.mocap_bridge import PoseKF, yaw_from_quat, quat_from_yaw
from mlcs_mpc.mocap_health import PoseHealth


class LidarBridge(Node):

    def __init__(self):
        super().__init__('lidar_bridge')
        d = self.declare_parameter
        d('pose_topic', '/tracked_pose')
        d('map_frame', 'map')
        d('base_frame', 'base_link')
        d('timeout', 0.2)
        d('q_pos', 0.01)
        d('q_vel', 1.0)
        d('r_meas', 4e-4)       # 약 2 cm. mocap 은 1e-4
        d('max_step_speed', 6.0)
        d('max_step_yaw_rate', 15.0)
        d('recover_time', 0.3)
        # true 면 relocalize 가 위치를 설정(/localization/ready)하기 전에는 /mocap/valid=false.
        #   시작 위치를 안 주면 Cartographer 가 맵 전체를 헤매며 수 m 씩 튄다 (2026-11-08).
        d('require_ready', False)
        # false 면 위치 튐(프레임 간 이동이 max_step_speed 초과)을 경고만 하고 정지 신호는 내지 않는다.
        #   Cartographer 보정으로 고속에서 13 cm 튀어 멈춘 적 있음 (2026-11-09, v_max 2.5). 비상정지는 사람이.
        d('stop_on_jump', True)
        # ★ 맵 고정 (2026-11-11 실트랙): Cartographer 위치추정은 랩이 쌓일수록 저장한 맵에서 흘러간다
        #   (lidar_obstacles 맞춤 기준 10~40 cm, 5~10°, 맞춤 비율 1.0 → 0.3 — 주행 불가).
        #   lidar_obstacles 가 매 스캔 저장한 맵 (pgm) 에 맞춰 낸 보정 /loc/correction 을 조금씩 적분해 자세에 더한다.
        d('map_pin', False)
        d('pin_gain', 0.1)          # 보정 메시지 하나당 적분 비율 (라이다 40 Hz → 시정수 약 0.25 s)
        d('pin_min_fit', 0.80)      # 맞춤 비율 (벽에 붙은 스캔 점 비율) 이 이 이상일 때만
        d('pin_step_xy', 0.01)      # 메시지 하나당 최대 이동 m (튀지 않게. 0.4 m/s 까지 따라감)
        d('pin_step_yaw', 0.3)      # 메시지 하나당 최대 회전 °
        d('pin_max_xy', 1.0)        # 누적 보정 상한 m
        d('pin_max_yaw', 20.0)      # 누적 보정 상한 °
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map_frame, self.base_frame = gp('map_frame'), gp('base_frame')
        self.timeout = float(gp('timeout'))
        q, v, r = float(gp('q_pos')), float(gp('q_vel')), float(gp('r_meas'))
        self.kf = [PoseKF(q, v, r) for _ in range(3)]
        self.health = PoseHealth(freeze_frames=10 ** 9,
                                 max_speed=float(gp('max_step_speed')),
                                 max_yaw_rate=float(gp('max_step_yaw_rate')),
                                 recover_time=float(gp('recover_time')))
        self.bad = False
        self.require_ready = bool(gp('require_ready'))
        self.stop_on_jump = bool(gp('stop_on_jump'))
        self.last_jump_log = -1e9
        self.ready = not self.require_ready
        self.last_stamp = self.last_rx = None
        self.prev_yaw = None
        self.yaw_u = 0.0
        self.warned = False
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        # Cartographer 는 RELIABLE 로 낸다. BEST_EFFORT 구독은 RELIABLE 발행과 맞는다.
        self.create_subscription(PoseStamped, gp('pose_topic'), self._pose_cb, qos)
        self.state_pub = self.create_publisher(Odometry, '/mpc/state', 10)
        self.valid_pub = self.create_publisher(Bool, '/mocap/valid', 10)
        self.create_subscription(Bool, '/localization/ready', self._ready_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self._watchdog)
        self.pin = bool(gp('map_pin'))
        self.pc = [0.0, 0.0, 0.0]       # 누적 보정 (dx, dy, dyaw)
        self.pin_n = self.pin_skip = 0
        self.pin_fit = math.nan
        if self.pin:
            self.pin_k, self.pin_fit_min = float(gp('pin_gain')), float(gp('pin_min_fit'))
            self.pin_sxy, self.pin_sw = float(gp('pin_step_xy')), math.radians(float(gp('pin_step_yaw')))
            self.pin_mxy, self.pin_mw = float(gp('pin_max_xy')), math.radians(float(gp('pin_max_yaw')))
            self.create_subscription(PoseStamped, '/loc/correction', self._corr_cb, 10)
            self.create_timer(5.0, self._pin_report)
            self.get_logger().info(f'맵 고정 켬 — /loc/correction (맞춤 비율 ≥ {self.pin_fit_min}) 을 적분')
        if self.require_ready:
            self.get_logger().warn('위치 설정 전 — RViz 2D Pose Estimate 로 시작 위치를 찍을 때까지 출발하지 않습니다')
        self.get_logger().info(f'lidar_bridge: {gp("pose_topic")} 구독, timeout={self.timeout}s')

    def _corr_cb(self, m):
        fit = m.pose.position.z
        self.pin_fit = fit
        if not fit >= self.pin_fit_min:
            self.pin_skip += 1
            return
        q = m.pose.orientation
        e = (m.pose.position.x, m.pose.position.y, 2.0 * math.atan2(q.z, q.w))
        lim = (self.pin_sxy, self.pin_sxy, self.pin_sw)
        for i in range(3):
            self.pc[i] += max(-lim[i], min(lim[i], self.pin_k * e[i]))
        r = math.hypot(self.pc[0], self.pc[1])
        if r > self.pin_mxy:
            self.pc[0] *= self.pin_mxy / r
            self.pc[1] *= self.pin_mxy / r
        self.pc[2] = max(-self.pin_mw, min(self.pin_mw, self.pc[2]))
        self.pin_n += 1

    def _pin_report(self):
        self.get_logger().info(f'맵 고정: 보정 {self.pc[0] * 100:+.1f}, {self.pc[1] * 100:+.1f} cm, '
                               f'{math.degrees(self.pc[2]):+.1f}° | 맞춤 비율 {self.pin_fit:.2f}, '
                               f'반영 {self.pin_n} / 건너뜀 {self.pin_skip}')
        self.pin_n = self.pin_skip = 0

    def _ready_cb(self, m):
        if self.require_ready and bool(m.data) and not self.ready:
            self.get_logger().info('✓ 위치 설정됨 — 이제 출발할 수 있습니다')
        self.ready = bool(m.data) or not self.require_ready

    def _pose_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.015 if self.last_stamp is None else stamp - self.last_stamp
        if dt <= 0.0:
            return          # 같은 스탬프를 다시 받았다
        if dt > 0.5:
            for k in self.kf:
                k.init = False
            self.prev_yaw = None
            dt = 0.015
        self.last_stamp, self.last_rx = stamp, now

        p, o = msg.pose.position, msg.pose.orientation
        good, why = self.health.update(stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        if not self.stop_on_jump:
            if why and now - self.last_jump_log > 1.0:
                self.get_logger().warn(f'라이다 위치 튐 ({why}) — 무시하고 계속')
                self.last_jump_log = now
            why, good = '', True
            self.health.bad_until = None
        if why:
            self.get_logger().error(f'✗ 라이다 위치 불량 ({why}) — 컨트롤러 정지 신호 발행')
        if self.bad and good:
            self.get_logger().info('✓ 라이다 위치 복구')
            for k in self.kf:
                k.init = False
            self.prev_yaw = None
        self.bad = not good

        yaw = yaw_from_quat(o)
        px, py = p.x, p.y
        if self.pin:
            px, py, yaw = px + self.pc[0], py + self.pc[1], yaw + self.pc[2]
        if self.prev_yaw is None:
            self.yaw_u = yaw
        else:
            self.yaw_u += (yaw - self.prev_yaw + math.pi) % (2 * math.pi) - math.pi
        self.prev_yaw = yaw
        sx, sy, sw = (k.update(z, dt) for k, z in zip(self.kf, (px, py, self.yaw_u)))
        fyaw = sw[0]
        vx = sx[1] * math.cos(fyaw) + sy[1] * math.sin(fyaw)
        vy = -sx[1] * math.sin(fyaw) + sy[1] * math.cos(fyaw)

        od = Odometry()
        od.header.stamp = msg.header.stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = sx[0], sy[0]
        qx, qy, qz, qw = quat_from_yaw(fyaw)
        od.pose.pose.orientation.x, od.pose.pose.orientation.y = qx, qy
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y = vx, vy
        od.twist.twist.angular.z = sw[1]
        self.state_pub.publish(od)
        self.valid_pub.publish(Bool(data=good and self.ready))
        self.warned = False

    def _watchdog(self):
        if self.last_rx is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_rx > self.timeout:
            self.valid_pub.publish(Bool(data=False))
            if not self.warned:
                self.get_logger().error(f'✗ 라이다 위치 끊김 ({now - self.last_rx:.2f}s) — 정지 신호')
                self.warned = True


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
