#!/usr/bin/env python3
"""
lidar_obstacles — 라이다로 트랙 위 장애물을 찾아 PoseStamped (map) 로 낸다. 대회용 (mocap 없이).

  구독: /scan (LaserScan), /mpc/state (Odometry, map 프레임 base_link — lidar_bridge)
  발행: /obstacle/pose (PoseStamped, map) — hmpc_node 의 opponent_topic 에 그대로 꽂는다
        /obstacle/markers (MarkerArray) — RViz: 후보 점 (주황), 확정 장애물 (빨강 원기둥)

  계산은 obstacle_detect.py (저장한 맵과 비교해 '맵에 없는 것'). 여기서는 헛것을 거른다:
    같은 자리 (confirm_radius 안) 에서 최근 confirm_window 초 동안 confirm_hits 번 넘게 잡혀야 확정.
    위치추정 오차로 생기는 헛것은 오차를 따라 자리가 흔들려서 대부분 여기서 걸린다.
  가장 가까운 확정 장애물 하나만 낸다 (opponent.py 는 상대 하나를 다룬다).
  아무것도 없으면 내지 않는다 — hmpc_node 쪽 상대 추적이 timeout 으로 스스로 지운다.

  ros2 run mlcs_mpc lidar_obstacles --ros-args -p map_yaml:=maps/1109_3.yaml -p track_csv:=<prep.csv>
"""
import collections
import math
import os
import time
import numpy as np

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mlcs_mpc.hmpc_core import Track
from mlcs_mpc.obstacle_detect import DetectParams, TrackBand, WallField, detect_scan

BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LidarObstacles(Node):
    def __init__(self):
        super().__init__('lidar_obstacles')
        d = self.declare_parameter
        d('map_yaml', '')
        d('track_csv', '')
        d('scan_topic', '/scan')
        d('state_topic', '/mpc/state')
        d('base_frame', 'base_link')
        d('laser_xy_yaw', [0.27, 0.0, 0.0])     # TF 가 없을 때 (f1tenth bringup 의 static TF 값)
        d('max_pose_age', 0.15)                 # 이보다 오래된 자세면 쓰지 않음 s
        d('confirm_hits', 3)
        d('confirm_window', 0.3)
        d('confirm_radius', 0.3)
        d('log_dir', os.path.expanduser('~/mlcs_data'))     # '' 면 기록 안 함
        for k, v in DetectParams().__dict__.items():
            d(k, v)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = DetectParams(**{k: type(v)(gp(k)) for k, v in DetectParams().__dict__.items()})
        if not gp('map_yaml') or not gp('track_csv'):
            raise SystemExit('map_yaml 과 track_csv 가 필요합니다')
        self.walls = WallField(gp('map_yaml'))
        # 맵별 헛것 자리 (빈 트랙 주행에서 늘 잡히던 곳). 없으면 빈 목록
        gpath = os.path.splitext(gp('map_yaml'))[0] + '_ghost.csv'
        self.walls.ghost = np.loadtxt(gpath, delimiter=',', ndmin=2) if os.path.exists(gpath) else np.zeros((0, 2))
        self.band = TrackBand(Track(gp('track_csv')))
        self.base_frame = gp('base_frame')
        self.laser = tuple(float(v) for v in gp('laser_xy_yaw'))
        self.laser_from_tf = False
        self.max_age = float(gp('max_pose_age'))
        self.c_hits, self.c_win, self.c_rad = int(gp('confirm_hits')), float(gp('confirm_window')), \
            float(gp('confirm_radius'))
        self.pose = None            # (x, y, yaw, 받은 시각)
        self.pbuf = collections.deque(maxlen=100)     # (스탬프, x, y, yaw) — 스캔 시각 자세 보간용
        self.corr = (0.0, 0.0, 0.0) # 스캔 맞춤 보정 (dx, dy, dyaw) — 다음 스캔의 탐색 시작점
        self.last_fit = 0.0         # 지난 스캔의 맞춤 비율 — 낮으면 다음엔 넓게 찾는다
        self.hist = []              # [(시각, cx, cy)]
        self.n_scan = self.n_pub = 0
        self.t_cost = 0.0
        self.log = None
        if gp('log_dir'):
            os.makedirs(gp('log_dir'), exist_ok=True)
            path = os.path.join(gp('log_dir'), time.strftime('obstacles_%Y%m%d_%H%M%S.csv'))
            self.log = open(path, 'w', buffering=1)
            # 스캔마다: 자세 + 후보 수 + (확정이면) 낸 위치. 후보는 가까운 순 3개까지
            # cdx..cdyaw = 스캔 맞춤 보정, fit / fit0 = 벽에 붙은 스캔 점 비율 (보정 뒤 / 앞)
            self.log.write('stamp,x,y,yaw,n_cand,pub,ox,oy,c1x,c1y,c1n,c2x,c2y,c2n,c3x,c3y,c3n,'
                           'cdx,cdy,cdyaw,fit,fit0\n')
            self.get_logger().info(f'기록 → {path}')
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        self.create_subscription(Odometry, gp('state_topic'), self._state_cb, 10)
        self.create_subscription(LaserScan, gp('scan_topic'), self._scan_cb, BEST_EFFORT)
        self.pub = self.create_publisher(PoseStamped, '/obstacle/pose', 10)
        self.corr_pub = self.create_publisher(PoseStamped, '/loc/correction', 10)
        self.mk = self.create_publisher(MarkerArray, '/obstacle/markers', 1)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'lidar_obstacles: 맵 {gp("map_yaml")}, 벽 여유 {self.p.wall_clear}+{self.p.clear_per_m}/m, '
            f'확정 {self.c_hits}번/{self.c_win}s/{self.c_rad}m, 헛것 자리 {len(self.walls.ghost)}곳 (±{self.p.ghost_radius} m)')

    def _state_cb(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic())
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if not self.pbuf or st > self.pbuf[-1][0]:
            self.pbuf.append((st, self.pose[0], self.pose[1], self.pose[2]))

    def _pose_at(self, t):
        """스캔 시각 t 의 자세 (자세 기록을 보간, 50 ms 까지 외삽). 없으면 가장 최근 자세.
        ★ 11-11: 받은 시각의 자세를 쓰면 3 m/s 에서 스캔 지연만큼 (약 2 cm/10 ms) 맞춤 보정에 섞인다 — 맵 고정이 흔들린다."""
        b = self.pbuf
        if len(b) < 2 or t - b[-1][0] > 0.05 or t < b[0][0]:
            return self.pose[:3]
        i = len(b) - 1
        while i > 1 and b[i - 1][0] > t:
            i -= 1
        (t0, x0, y0, w0), (t1, x1, y1, w1) = b[i - 1], b[i]
        a = (t - t0) / max(t1 - t0, 1e-6)
        dw = (w1 - w0 + math.pi) % (2 * math.pi) - math.pi
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0), w0 + a * dw

    def _laser_tf(self, frame):
        if self.laser_from_tf:
            return
        try:
            t = self.tf_buf.lookup_transform(self.base_frame, frame, rclpy.time.Time())
            tr, q = t.transform.translation, t.transform.rotation
            self.laser = (tr.x, tr.y, yaw_of(q))
            self.laser_from_tf = True
            self.get_logger().info(f'라이다 위치 (TF): {self.laser[0]:.3f}, {self.laser[1]:.3f}, '
                                   f'{math.degrees(self.laser[2]):.1f}°')
        except Exception:   # noqa: BLE001 — TF 가 아직 없으면 기본값으로
            pass

    def _scan_cb(self, s):
        now = time.monotonic()
        if self.pose is None or now - self.pose[3] > self.max_age:
            return
        self._laser_tf(s.header.frame_id)
        pose = self._pose_at(s.header.stamp.sec + s.header.stamp.nanosec * 1e-9)
        t0 = time.perf_counter()
        # 넓은 탐색은 10 스캔마다 또는 맞춤이 나빠졌을 때만 (평소는 지난 보정 둘레만 — 3배 빠름)
        coarse = self.n_scan % 10 == 0 or self.last_fit < 0.7
        dets, used, self.corr, fit = detect_scan(s.ranges, s.angle_min, s.angle_increment, pose,
                                                 self.laser, self.walls, self.band, self.p, self.corr, coarse)
        self.last_fit = fit[0] if fit[0] == fit[0] else 1.0
        if fit[0] == fit[0]:
            # 맵 고정 (lidar_bridge map_pin): 이 스캔을 저장한 맵에 맞추려면 받은 자세에 더할 (dx, dy, dyaw).
            # position.z = 맞춤 비율 (벽에 붙은 점 비율). 브릿지가 비율이 좋을 때만 조금씩 적분한다.
            c = PoseStamped()
            c.header.stamp, c.header.frame_id = s.header.stamp, 'map'
            c.pose.position.x, c.pose.position.y, c.pose.position.z = self.corr[0], self.corr[1], fit[0]
            c.pose.orientation.z, c.pose.orientation.w = math.sin(self.corr[2] / 2), math.cos(self.corr[2] / 2)
            self.corr_pub.publish(c)
        self.t_cost += time.perf_counter() - t0
        self.n_scan += 1
        for c in dets:
            self.hist.append((now, c[0], c[1]))
        self.hist = [h for h in self.hist if now - h[0] < self.c_win]
        best = None
        for c in dets:      # 가까운 순
            hits = sum(1 for h in self.hist if math.hypot(h[1] - c[0], h[2] - c[1]) < self.c_rad)
            if hits >= self.c_hits:
                best = c
                break
        if best is not None:
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = s.header.stamp, 'map'
            m.pose.position.x, m.pose.position.y = best[0], best[1]
            m.pose.orientation.z, m.pose.orientation.w = math.sin(best[4] / 2), math.cos(best[4] / 2)
            self.pub.publish(m)
            self.n_pub += 1
        if self.log:
            c = [f'{o[0]:.3f},{o[1]:.3f},{o[3]}' for o in dets[:3]] + [',,'] * (3 - min(len(dets), 3))
            ob = f'1,{best[0]:.3f},{best[1]:.3f}' if best is not None else '0,,'
            self.log.write(f'{s.header.stamp.sec + s.header.stamp.nanosec * 1e-9:.4f},{pose[0]:.3f},'
                           f'{pose[1]:.3f},{pose[2]:.4f},{len(dets)},{ob},{",".join(c)},'
                           f'{self.corr[0]:.3f},{self.corr[1]:.3f},{self.corr[2]:.4f},{fit[0]:.3f},{fit[1]:.3f}\n')
        self._markers(s.header.stamp, dets, best)

    def _markers(self, stamp, dets, best):
        arr = MarkerArray()
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, 'map'
        m.ns, m.id, m.type, m.action = 'obstacle_cand', 0, Marker.POINTS, Marker.ADD
        m.scale.x = m.scale.y = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
        m.points = [Point(x=c[0], y=c[1], z=0.05) for c in dets]
        arr.markers.append(m)
        b = Marker()
        b.header.stamp, b.header.frame_id = stamp, 'map'
        b.ns, b.id, b.type = 'obstacle', 1, Marker.CYLINDER
        b.action = Marker.ADD if best is not None else Marker.DELETE
        if best is not None:
            b.pose.position.x, b.pose.position.y, b.pose.position.z = best[0], best[1], 0.1
            b.pose.orientation.w = 1.0
            b.scale.x = b.scale.y = max(best[2], 0.2)
            b.scale.z = 0.2
            b.color.r, b.color.a = 1.0, 0.8
        arr.markers.append(b)
        self.mk.publish(arr)

    def _report(self):
        if self.n_scan:
            self.get_logger().info(f'스캔 {self.n_scan}개, 장애물 발행 {self.n_pub}번, '
                                   f'계산 평균 {self.t_cost / self.n_scan * 1e3:.1f} ms')
        elif self.pose is None:
            self.get_logger().warn('자세 (/mpc/state) 가 아직 없습니다')
        self.n_scan = self.n_pub = 0
        self.t_cost = 0.0


def main(args=None):
    rclpy.init(args=args)
    n = LidarObstacles()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
