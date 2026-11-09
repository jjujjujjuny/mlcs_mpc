#!/usr/bin/env python3
"""
sim_node — env/simulator.py (ODE) 를 ROS 로 감싼다. 컨트롤러 쪽에서는 실차와 같은 토픽만 보인다.

  구독  /drive (AckermannDriveStamped — 서보 명령, 바퀴 속도 명령)   /initialpose (RViz 로 다시 세우기)
  발행  /mpc/state   Odometry, 뒤축 pose + 뒤축 차체 속도     ← mocap_bridge / lidar_bridge 자리
        /mocap/valid Bool (항상 True)
        /sensors/core VescStateStamped (state.speed = ERPM, 차의 vesc.yaml 값)  ← vesc_driver 자리
        /mpc/enabled  autostart_delay 뒤 True (래치). 음수면 안 낸다 → ./sim.sh go
        TF map → base_link,  /sim/markers (벽·중심선),  /sim/car,  /sim/status (JSON: s, n, 랩, 벽)

■ 차 모델 — mpcc_params (mpcc.yaml + 덮어쓸 파일) 로 VehicleModel 을 만든다. mpcc_node 와 같은 파일을
  같은 순서로 받으므로 MPC 와 차가 같은 값이다. car_* 를 주면 차만 바꾼다 (모델 불일치 시험 —
  예: car_wheel_speed_tau 0.66 이 실측).

■ VESC — vesc_yaml (기본 젯슨의 f1tenth_stack/config/vesc.yaml, 없으면 패치 값 — env/vesc.py) 로 속도 ERPM
  상한·서보 한계를 차처럼 자르고 /sensors/core 를 같은 gain 으로 낸다.

■ 주기 — rate (적분) 100 Hz, state_rate (/mpc/state: mocap 100 Hz) , vesc_rate (/sensors/core 50 Hz).
  컨트롤러 주기는 mpcc.yaml 의 control_rate (30 Hz) 로 mpcc_node 가 정한다.

■ 벽 — 트랙 파일의 w_left/w_right (없으면 track_width/2). 차 반폭 car_half_width 가 벽을 넘으면 벽.
  on_wall: warn (기록만) | stop (속도 명령 무시, 정지) | reset (출발점으로)
"""
import csv
import json
import math
import os

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import TransformBroadcaster
from vesc_msgs.msg import VescStateStamped
from visualization_msgs.msg import Marker, MarkerArray

from mlcs_mpc.env.config import read_params
from mlcs_mpc.env.simulator import Simulator
from mlcs_mpc.env.track import Track
from mlcs_mpc.env.vesc import DEFAULT_PATH as VESC_PATH, Vesc
from mlcs_mpc.env.vehicle_model import VehicleModel
from mlcs_mpc.utils.sim_viz import track_markers

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CAR_KEYS = ('mass', 'friction', 'steering_tau', 'wheel_speed_tau', 'ws_acc_max', 'ws_dec_max')


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class SimNode(Node):

    def __init__(self):
        super().__init__('mlcs_sim')
        d = self.declare_parameter
        d('track_csv', '')
        d('mpcc_params', [''])          # mpcc.yaml, 그 뒤 덮어쓸 파일 (race.sh 의 extra_params)
        d('rate', 100.0)                # 적분 주기 Hz
        d('state_rate', 100.0)          # /mpc/state 주기 Hz (mocap 100, 라이다 위치추정이면 그보다 낮게)
        d('vesc_rate', 50.0)            # /sensors/core 주기 Hz
        d('vesc_yaml', '')              # 비우면 env/vesc.DEFAULT_PATH, 없으면 패치 값
        d('integrator', 'rk4')          # rk4 (고정 간격) | scipy solve_ivp 방법 (RK45, LSODA ...)
        d('start_s', 0.0)
        d('autostart_delay', 2.0)
        d('on_wall', 'warn')
        d('car_half_width', 0.135)      # vehicle.yaml vehicle_width 0.27 / 2
        d('car_length', 0.505)
        d('noise_pos', 0.0)             # m. 실차 mocap 은 약 1 mm
        d('noise_yaw', 0.0)
        d('log_csv', '')
        for k in CAR_KEYS:
            d('car_' + k, -1.0)         # < 0 이면 mpcc_params 를 따른다
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        files = [p for p in gp('mpcc_params') if p]
        cfg = read_params(*files)
        over = {k: float(gp('car_' + k)) for k in CAR_KEYS if float(gp('car_' + k)) >= 0}
        model = VehicleModel.from_yaml(*files, **{k: v for k, v in over.items() if k not in ('ws_acc_max', 'ws_dec_max')})
        if over:
            self.get_logger().warn(f'차만 다른 값: {over} (MPC 모델과 다르다)')
        path = gp('track_csv')
        if not path or not os.path.isfile(path):
            raise RuntimeError(f'track_csv 가 없습니다: {path!r}')
        self.track = Track(path)
        self.dt = 1.0 / float(gp('rate'))
        self.vesc = Vesc.load(gp('vesc_yaml') or VESC_PATH)
        self.sim = Simulator(
            model, self.track, dt=self.dt, integrator=str(gp('integrator')),
            steer_map_cmd=cfg.get('steer_map_cmd', (0.0, 0.2, 0.3, 0.4)),
            steer_map_act=cfg.get('steer_map_act', (0.0, 0.2021, 0.2972, 0.3776)),
            ws_acc=over.get('ws_acc_max', float(cfg.get('ws_acc_max', 0.0))),
            ws_dec=over.get('ws_dec_max', float(cfg.get('ws_dec_max', 0.0))),
            half_width=float(gp('car_half_width')), vesc=self.vesc)
        self.state_every = max(1, round(float(gp('rate')) / float(gp('state_rate'))))
        self.vesc_every = max(1, round(float(gp('rate')) / float(gp('vesc_rate'))))
        self.on_wall = str(gp('on_wall'))
        self.car_len = float(gp('car_length'))
        self.n_pos, self.n_yaw = float(gp('noise_pos')), float(gp('noise_yaw'))
        self.rng = np.random.default_rng(0)
        self.start = self.sim.track_pose(float(gp('start_s')))
        self.cmd = (0.0, 0.0)
        self._reset(*self.start)

        self.state_pub = self.create_publisher(Odometry, '/mpc/state', 10)
        self.valid_pub = self.create_publisher(Bool, '/mocap/valid', 10)
        self.vesc_pub = self.create_publisher(VescStateStamped, '/sensors/core', 10)
        self.enable_pub = self.create_publisher(Bool, '/mpc/enabled', LATCHED)
        self.status_pub = self.create_publisher(String, '/sim/status', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/sim/markers', LATCHED)
        self.car_pub = self.create_publisher(Marker, '/sim/car', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(AckermannDriveStamped, '/drive', self._drive_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._initialpose_cb, 1)

        self.log = None
        if gp('log_csv'):
            os.makedirs(os.path.dirname(os.path.abspath(gp('log_csv'))), exist_ok=True)
            self.log_f = open(gp('log_csv'), 'w', newline='')
            self.log = csv.writer(self.log_f)
            self.log.writerow(['t', 'x', 'y', 'yaw', 'vx', 'vy', 'r', 'wheel_speed', 'delta',
                               'cmd_steer', 'cmd_speed', 's', 'n', 'on_wall'])

        self._track_msg = None
        self._publish_track()
        self.create_timer(1.0, self._publish_track)     # RViz 가 늦게 떠도 보이게
        self.k = 0
        self.create_timer(self.dt, self._step)
        delay = float(gp('autostart_delay'))
        if delay >= 0:
            self._enable_timer = self.create_timer(max(delay, 0.01), self._autostart)
        self.get_logger().info(
            f'트랙 {os.path.basename(path)} (길이 {self.track.length:.2f} m), 출발 '
            f'({self.start[0]:+.2f}, {self.start[1]:+.2f}, {self.start[2]:+.2f}), '
            f'질량 {model.mass} kg, 마찰 {model.friction}, 적분 {gp("integrator")} {gp("rate"):.0f} Hz, '
            f'상태 {gp("state_rate"):.0f} Hz')
        lo, hi = self.vesc.steer_limits
        self.get_logger().info(f'VESC {self.vesc.source}: 모터 상한 {self.vesc.v_motor:.2f} m/s, '
                               f'조향 명령 {lo:+.3f}~{hi:+.3f} rad (서보 {self.vesc.servo_min}~{self.vesc.servo_max})')

    # ── 입력 ───────────────────────────────────────────────────────────
    def _drive_cb(self, m):
        self.cmd = (float(m.drive.steering_angle), float(m.drive.speed))

    def _initialpose_cb(self, m):
        p = m.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        self._reset(p.position.x, p.position.y, yaw)
        self.get_logger().info(f'다시 세움 ({p.position.x:+.2f}, {p.position.y:+.2f}, {yaw:+.2f})')

    def _autostart(self):
        self._enable_timer.cancel()
        self.enable_pub.publish(Bool(data=True))
        self.get_logger().info('autostart — /mpc/enabled=true')

    def _reset(self, x, y, yaw):
        self.sim.reset(x, y, yaw)
        self.cmd = (0.0, 0.0)

    # ── 시뮬 ───────────────────────────────────────────────────────────
    def _step(self):
        steer, speed = self.cmd
        if self.sim.on_wall and self.on_wall == 'stop':
            speed = 0.0
        info = self.sim.step(steer, speed)
        if info['new_hit']:
            self.get_logger().warn(f'벽 — s {info["s"]:.2f} n {info["n"]:+.3f} '
                                   f'(허용 {info["lo"]:+.3f}~{info["hi"]:+.3f})')
            if self.on_wall == 'reset':
                self._reset(*self.start)
                return
        if info['lap'] is not None:
            self.get_logger().info(f'랩 {info["laps"]}: {info["lap"]:.2f} s  '
                                   f'(최고 {min(self.sim.laps):.2f} s, 벽 {info["wall_hits"]}회)')
        x, y, yaw = self.sim.pose()
        vx, vy, r = info['vx'], info['vy'], info['r']

        stamp = self.get_clock().now().to_msg()
        px = x + (self.rng.normal(0, self.n_pos) if self.n_pos > 0 else 0.0)
        py = y + (self.rng.normal(0, self.n_pos) if self.n_pos > 0 else 0.0)
        pyaw = yaw + (self.rng.normal(0, self.n_yaw) if self.n_yaw > 0 else 0.0)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'map', 'base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = px, py
        o = od.pose.pose.orientation
        o.x, o.y, o.z, o.w = quat(pyaw)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vx, vy, r
        self.k += 1
        if self.k % self.state_every == 0:
            self.state_pub.publish(od)

        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'map', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tr = tf.transform.rotation
        tr.x, tr.y, tr.z, tr.w = quat(yaw)
        self.tf.sendTransform(tf)

        if self.k % self.vesc_every == 0:
            vs = VescStateStamped()
            vs.header.stamp = stamp
            vs.state.speed = self.vesc.erpm(self.sim.wheel_speed)
            vs.state.voltage_input = 11.4
            self.vesc_pub.publish(vs)
        if self.k % 5 == 0:
            self.valid_pub.publish(Bool(data=True))
            self._publish_car(x, y, yaw, stamp)
        if self.k % 20 == 0:
            laps = self.sim.laps
            self.status_pub.publish(String(data=json.dumps(dict(
                t=round(info['t'], 2), s=round(info['s'], 3), n=round(info['n'], 3), v=round(vx, 3),
                laps=len(laps), last_lap=round(laps[-1], 2) if laps else None,
                best_lap=round(min(laps), 2) if laps else None,
                on_wall=info['on_wall'], wall_hits=info['wall_hits']))))
        if self.log:
            self.log.writerow([f'{info["t"]:.3f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}', f'{vx:.4f}',
                               f'{vy:.4f}', f'{r:.4f}', f'{info["ws"]:.4f}', f'{info["delta"]:.4f}',
                               f'{steer:.4f}', f'{speed:.4f}', f'{info["s"]:.3f}', f'{info["n"]:.4f}',
                               int(info['on_wall'])])

    # ── 화면 ───────────────────────────────────────────────────────────
    def _publish_track(self):
        if self._track_msg is None:
            margin = read_params(*[p for p in self.get_parameter('mpcc_params').value if p]).get('wall_margin')
            self._track_msg = track_markers(self.track, margin)
        self.marker_pub.publish(self._track_msg)

    def _publish_car(self, x, y, yaw, stamp):
        m = Marker()
        m.header.frame_id, m.header.stamp, m.ns, m.id = 'map', stamp, 'car', 0
        m.type, m.action = Marker.CUBE, Marker.ADD
        c = self.sim.model.lr           # 상자 중심은 대략 무게중심
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            x + c * math.cos(yaw), y + c * math.sin(yaw), 0.05
        o = m.pose.orientation
        o.x, o.y, o.z, o.w = quat(yaw)
        m.scale.x, m.scale.y, m.scale.z = self.car_len, 2 * self.sim.half_width, 0.1
        m.color = (ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9) if self.sim.on_wall
                   else ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.9))
        self.car_pub.publish(m)

    def destroy_node(self):
        if self.log:
            self.log_f.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:       # Ctrl+C 가 두 번 오면 정리 중에도 올라온다
            pass


if __name__ == '__main__':
    main()
