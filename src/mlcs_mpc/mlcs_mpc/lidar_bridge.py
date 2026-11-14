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
        # ★ 요레이트는 자이로로 (2026-11-11 실트랙): KF 가 위치를 미분해 낸 r 은 자이로보다 약 110 ms 늦다
        #   (자세 yaw 는 10~20 ms). 이 늦은 r 과 조향 명령 지연 (~70 ms) 이 겹쳐 고속 직선에서 좌우로 출렁였다
        #   (offline 1111_3 v4.8: 둘 다 늦으면 직선 0.5~2 Hz 조향 11.3°, 실차 13~14° / r 만 자이로면 6.4°).
        #   wheel_odom (/odometry/filtered, 바이어스 뺀 자이로 rad/s) 이 있으면 그 값을 쓰고, 0.1 s 넘게 끊기면 KF r.
        d('gyro_rate', True)
        d('gyro_timeout', 0.1)
        d('pin_gain', 0.1)          # 보정 메시지 하나당 적분 비율 (라이다 40 Hz → 시정수 약 0.25 s)
        d('pin_min_fit', 0.80)      # 맞춤 비율 (벽에 붙은 스캔 점 비율) 이 이 이상일 때만
        d('pin_step_xy', 0.01)      # 메시지 하나당 최대 이동 m (튀지 않게. 0.4 m/s 까지 따라감)
        d('pin_step_yaw', 0.3)      # 메시지 하나당 최대 회전 °
        d('pin_max_xy', 1.5)        # 누적 보정 상한 m (11-14 대회장: 0.6 → 1.0 에도 붙은 뒤 깨짐 (122617) → 메인 PC 권장 1.5. 흡수·맞춤 나쁘면 풀기가 있음)
        d('pin_max_yaw', 10.0)      # 누적 보정 상한 ° (정상 최대 8°. 20° 였을 때 잘못 잡아 상한까지 감)
        # ★ 11-11 저속 주행: 수동 후진 중 Cartographer 가 12° 튀자 맞춤이 엉뚱한 정렬을 따라가 (−35, −66 cm, −20°)
        #   그 뒤 맞춤 비율이 떨어져 반영을 멈춘 채 틀린 보정을 들고 있었다 → 위치 60~85 cm 틀림, 벽이 장애물로.
        d('pin_bad_time', 1.0)      # 이만큼 맞춤이 계속 나쁘면 보정을 0 (Cartographer 그대로) 쪽으로 푼다 s
        d('pin_decay_tau', 1.0)     # 그때 푸는 시정수 s
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
        self.gyro_on, self.gyro_to = bool(gp('gyro_rate')), float(gp('gyro_timeout'))
        self.gyro_r, self.gyro_t = 0.0, -1e9
        self.wheel_v, self.still_since, self.pc_still = 0.0, None, None
        self.gyro_used = self.gyro_fallback = 0
        if self.gyro_on:
            self.create_subscription(Odometry, '/odometry/filtered', self._gyro_cb, 10)
            self.create_timer(10.0, self._gyro_report)
        self.pin = bool(gp('map_pin'))
        self.pc = [0.0, 0.0, 0.0]       # 누적 보정 (dx, dy, dyaw)
        self.pin_n = self.pin_skip = 0
        self.pin_fit = math.nan
        if self.pin:
            self.pin_k, self.pin_fit_min = float(gp('pin_gain')), float(gp('pin_min_fit'))
            self.pin_sxy, self.pin_sw = float(gp('pin_step_xy')), math.radians(float(gp('pin_step_yaw')))
            self.pin_mxy, self.pin_mw = float(gp('pin_max_xy')), math.radians(float(gp('pin_max_yaw')))
            self.pin_bad_t, self.pin_tau = float(gp('pin_bad_time')), float(gp('pin_decay_tau'))
            self.pin_good_t = None      # 마지막으로 맞춤 비율이 좋았던 시각
            self.pin_last_t = None
            self.pin_resets = self.pin_decays = 0
            self.pin_raw = None
            self.pin_jump_xy, self.pin_jump_yaw = 0.30, math.radians(5.0)
            # ★ 11-14 대회장: Cartographer 가 3.8 m·176° 튀어 (거울상 나선 트랙의 반대편) 30 초 동안 위치가 틀렸다.
            #   튀기 직전 맵 고정 위치가 맵과 잘 맞고 있었다면 (맞춤 좋음) 튐을 흡수해 위치를 잇는다 — 회전까지 (SE2).
            #   흡수한 뒤 맞춤이 pin_bad_time 넘게 나쁘면 (= Cartographer 쪽이 맞았다) 흡수를 버리고 Cartographer 를 따른다.
            self.pj = (0.0, 0.0, 0.0)       # 흡수 변환: 보정 전 자세 = R(th)·raw + t
            self.pin_out = None             # 직전 출력 자세 (흡수 기준)
            self.pin_absorbs = self.pin_drops = 0
            self.create_subscription(PoseStamped, '/loc/correction', self._corr_cb, 10)
            self.create_timer(5.0, self._pin_report)
            self.get_logger().info(f'맵 고정 켬 — /loc/correction (맞춤 비율 ≥ {self.pin_fit_min}) 을 적분')
        if self.require_ready:
            self.get_logger().warn('위치 설정 전 — RViz 2D Pose Estimate 로 시작 위치를 찍을 때까지 출발하지 않습니다')
        self.get_logger().info(f'lidar_bridge: {gp("pose_topic")} 구독, timeout={self.timeout}s')

    def _gyro_cb(self, m):
        self.gyro_r = m.twist.twist.angular.z
        self.gyro_t = self.get_clock().now().nanoseconds * 1e-9
        self.wheel_v = m.twist.twist.linear.x
        if abs(self.wheel_v) < 0.05:
            self.still_since = self.still_since or self.gyro_t
        else:
            self.still_since, self.pc_still = None, None

    def _apply_pj(self, x, y, yaw):
        tx, ty, th = self.pj
        c, s_ = math.cos(th), math.sin(th)
        return c * x - s_ * y + tx, s_ * x + c * y + ty, yaw + th

    def _kf_restart(self):
        for k in self.kf:
            k.init = False
        self.prev_yaw = None

    def _pin_reset(self, why):
        if any(abs(v) > 1e-6 for v in self.pc):
            self.get_logger().warn(f'맵 고정 보정 되돌림 ({why}) — {self.pc[0] * 100:+.0f}, {self.pc[1] * 100:+.0f} cm, '
                                   f'{math.degrees(self.pc[2]):+.1f}° → 0, 다시 맞춘다')
        self.pc = [0.0, 0.0, 0.0]
        self.pc_still = None
        self.pin_resets += 1

    def _corr_cb(self, m):
        fit = m.pose.position.z
        self.pin_fit = fit
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.pin_last_t is None else min(max(now - self.pin_last_t, 0.0), 0.2)
        self.pin_last_t = now
        if self.pin_good_t is None:
            self.pin_good_t = now
        if not fit >= self.pin_fit_min:
            self.pin_skip += 1
            if now - self.pin_good_t > self.pin_bad_t:
                # 맞춤이 계속 나쁘다 = 지금 보정을 믿을 수 없다 → Cartographer 쪽으로 천천히 푼다
                k = math.exp(-dt / self.pin_tau)
                self.pc = [v * k for v in self.pc]
                self.pc_still = None
                self.pin_decays += 1
                if self.pj != (0.0, 0.0, 0.0):
                    # 흡수했던 튐이 사실은 맞는 재위치였다 — 흡수를 버리고 Cartographer 를 따른다
                    self.get_logger().warn(f'맵 고정: 흡수했던 튐을 버림 (맞춤 {fit:.2f} 이 {self.pin_bad_t:.1f} s 넘게 나쁨) '
                                           f'— Cartographer 위치로')
                    self.pj = (0.0, 0.0, 0.0)
                    self.pin_drops += 1
                    self._kf_restart()
            return
        self.pin_good_t = now
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
        # ★ 11-14 170753: 출발 전 15 s 서 있는 동안 보정이 직선을 따라 −150 cm 까지 미끄러짐 (맞춤 0.8~0.9 — 직선은
        #   앞뒤로 밀어도 벽에 맞는다) → 출발하자마자 우회전 자리에서 직진해 충돌. 서 있으면 (바퀴 0, 0.5 s 넘게)
        #   멈춘 순간의 보정에서 ±15 cm · ±3° 까지만 움직이게 묶는다. 자이로 (wheel_odom) 가 끊기면 묶지 않음.
        if (self.still_since is not None and now - self.gyro_t < 0.2 and now - self.still_since > 0.5):
            if self.pc_still is None:
                self.pc_still = list(self.pc)
            for i, lim_s in ((0, 0.15), (1, 0.15), (2, math.radians(3.0))):
                self.pc[i] = max(self.pc_still[i] - lim_s, min(self.pc_still[i] + lim_s, self.pc[i]))
        self.pin_n += 1

    def _gyro_report(self):
        n = self.gyro_used + self.gyro_fallback
        if n and self.gyro_fallback:
            self.get_logger().warn(f'요레이트: 자이로 {self.gyro_used} / KF {self.gyro_fallback} (자이로 끊김 — wheel_odom 확인)')
        elif n and not getattr(self, '_gyro_said', False):
            self.get_logger().info('요레이트: 자이로 (wheel_odom) 사용 중')
            self._gyro_said = True
        self.gyro_used = self.gyro_fallback = 0

    def _pin_report(self):
        self.get_logger().info(f'맵 고정: 보정 {self.pc[0] * 100:+.1f}, {self.pc[1] * 100:+.1f} cm, '
                               f'{math.degrees(self.pc[2]):+.1f}° | 맞춤 비율 {self.pin_fit:.2f}, '
                               f'반영 {self.pin_n} / 건너뜀 {self.pin_skip}'
                               + (f' / 풀림 {self.pin_decays}' if self.pin_decays else '')
                               + (f' / 되돌림 {self.pin_resets}' if self.pin_resets else '')
                               + (f' / 튐 흡수 {self.pin_absorbs}' if self.pin_absorbs else '')
                               + (f' / 흡수 버림 {self.pin_drops}' if self.pin_drops else '')
                               + (f' | 흡수 중 {math.hypot(self.pj[0], self.pj[1]) * 100:.0f} cm·{math.degrees(self.pj[2]):+.0f}°'
                                  if self.pj != (0.0, 0.0, 0.0) else ''))
        self.pin_n = self.pin_skip = self.pin_decays = self.pin_resets = 0
        self.pin_absorbs = self.pin_drops = 0

    def _ready_cb(self, m):
        if self.require_ready and bool(m.data) and not self.ready:
            self.get_logger().info('✓ 위치 설정됨 — 이제 출발할 수 있습니다')
        self.ready = bool(m.data) or not self.require_ready
        if self.pin and bool(m.data):
            # 사람이 위치를 다시 찍었다 (RViz 2D Pose Estimate / LT → relocalize 가 맞춘 뒤 발행).
            # 그 자리를 정답으로: 맵 고정 보정·흡수·정지 묶음 기준을 모두 비우고, 궤적 재시작을 튐으로 보지 않게 한다
            if any(abs(v) > 1e-6 for v in self.pc) or self.pj != (0.0, 0.0, 0.0):
                self.get_logger().info(f'위치 다시 설정 — 맵 고정 보정 {self.pc[0] * 100:+.0f}, {self.pc[1] * 100:+.0f} cm, '
                                       f'{math.degrees(self.pc[2]):+.1f}° 와 흡수를 비움')
            self.pc = [0.0, 0.0, 0.0]
            self.pj = (0.0, 0.0, 0.0)
            self.pc_still = None
            self.pin_raw = None
            self._kf_restart()

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
        if self.pin:
            # 크게 튈 때만 (한 메시지에 5° 또는 30 cm 넘게). 평소의 10~19 cm 튐마다 되돌리면 MPC 자세가 그만큼 순간 이동한다
            yw = yaw_from_quat(o)
            if self.pin_raw is not None:
                dj = math.hypot(p.x - self.pin_raw[0], p.y - self.pin_raw[1])
                dw = abs((yw - self.pin_raw[2] + math.pi) % (2 * math.pi) - math.pi)
                if dj > self.pin_jump_xy or dw > self.pin_jump_yaw:
                    good_before = (self.pin_good_t is not None and now - self.pin_good_t < 0.5
                                   and self.pin_fit >= self.pin_fit_min and self.pin_out is not None)
                    if good_before:
                        # 흡수: 새 raw 에 pj 를 다시 잡아 '보정 전 자세 + pc = 직전 출력' 이 되게
                        ox, oy, oyaw = self.pin_out
                        txy = (ox - self.pc[0], oy - self.pc[1])
                        th = (oyaw - self.pc[2]) - yw
                        c, s_ = math.cos(th), math.sin(th)
                        self.pj = (txy[0] - (c * p.x - s_ * p.y), txy[1] - (s_ * p.x + c * p.y), th)
                        self.pin_absorbs += 1
                        self.get_logger().warn(f'맵 고정: Cartographer 가 {dj * 100:.0f} cm·{math.degrees(dw):.0f}° 튐 — '
                                               f'직전 맞춤 {self.pin_fit:.2f} 이 좋아 흡수 (위치 유지)')
                    else:
                        self._pin_reset(f'Cartographer 위치 {dj * 100:.0f} cm·{math.degrees(dw):.0f}° 튐')
                        self.pj = (0.0, 0.0, 0.0)
                        self._kf_restart()
            self.pin_raw = (p.x, p.y, yw)
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
            px, py, yaw = self._apply_pj(px, py, yaw)
            px, py, yaw = px + self.pc[0], py + self.pc[1], yaw + self.pc[2]
            self.pin_out = (px, py, yaw)
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
        r_out = sw[1]
        if self.gyro_on:
            if now - self.gyro_t < self.gyro_to:
                r_out = self.gyro_r
                self.gyro_used += 1
            else:
                self.gyro_fallback += 1
        od.twist.twist.angular.z = r_out
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
