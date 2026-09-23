#!/usr/bin/env python3
"""
track_viz — 트랙·차량·주행궤적을 RViz 로 시각화한다.

  ros2 run mlcs_mpc track_viz --ros-args \
      -p track_dir:=<트랙폴더> -p waypoint_file:=<경로.csv>

■ 무엇을 그리는가

    /track/centerline    중심선 (초록)
    /track/left_bound    좌측 경계 (흰색)
    /track/right_bound   우측 경계 (흰색)
    /track/surface       트랙 면 (반투명 회색) — 좌우 경계 사이를 채운다
    /track/start         출발점 마커
    /viz/car             차량 외형 (실측 치수 박스 + 앞방향 화살표)
    /viz/trail           실제 주행 궤적 (지나온 자취)
    /viz/safety_bounds   safety_node 경계 (빨간 선)

  경로·예측은 컨트롤러가 직접 낸다:
    /mpc/reference_path   (stanley_node / mpc_node)
    /mpc/predicted_path   (mpc_node 만)

■ 좌표계

  모두 **map 프레임** = mocap 글로벌 좌표계다. 트랙 CSV 의 좌표가
  Motive 가 잡은 그 좌표계 기준이므로, 별도 변환 없이 그대로 그린다.
  mocap_bridge 가 map → base_link TF 를 내므로 차량 위치도 같은
  프레임에서 맞는다.

■ 왜 트랙 면(surface)까지 그리는가

  선만 그리면 "차가 트랙 안에 있는지" 가 눈으로 안 들어온다. 폭 0.6m
  트랙에 폭 0.27m 차량이면 좌우 여유가 16cm 뿐이라, 면으로 칠해야
  이탈이 바로 보인다.
"""

import csv
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node

from .path_manager import PathManager
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# 트랙은 안 변하므로 래치로 낸다 — RViz 를 나중에 켜도 보인다
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_xy(path):
    """CSV 에서 x,y 를 읽는다. 헤더(#/idx)와 빈 줄은 건너뛴다."""
    pts = []
    with open(path) as f:
        for row in csv.reader(f):
            if not row:
                continue
            first = row[0].strip()
            if not first or first.startswith('#'):
                continue
            try:
                # idx,x,y 형식이면 3열, x,y 형식이면 2열
                if len(row) >= 3 and first.isdigit():
                    pts.append((float(row[1]), float(row[2])))
                else:
                    pts.append((float(row[0]), float(row[1])))
            except ValueError:
                continue        # 헤더 줄
    return np.array(pts) if pts else None


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class TrackViz(Node):

    def __init__(self):
        super().__init__('track_viz')

        self.declare_parameter('track_dir', '')      # centerline/left/right 폴더
        self.declare_parameter('waypoint_file', '')  # 없으면 track_dir 사용
        self.declare_parameter('frame', 'map')
        # 차체 치수 (실측값)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('vehicle_length', 0.505)
        self.declare_parameter('vehicle_width', 0.27)
        self.declare_parameter('rear_overhang', 0.075)
        # safety 경계
        self.declare_parameter('x_min', -3.0)
        self.declare_parameter('x_max', 3.0)
        self.declare_parameter('y_min', -3.0)
        self.declare_parameter('y_max', 3.0)
        self.declare_parameter('show_safety', True)
        # ↓ safety.yaml 을 그대로 넘겨받으므로, 쓰지 않는 키도 선언해 둔다.
        #   선언하지 않으면 ROS 2 가 "not declared" 로 **노드 기동을 실패**시킨다.
        self.declare_parameter('enabled', True)
        self.declare_parameter('margin', 0.5)
        self.declare_parameter('lookahead', 0.7)
        self.declare_parameter('max_speed', 3.0)
        # 주행 궤적
        self.declare_parameter('trail_len', 2000)

        gp = lambda n: self.get_parameter(n).value
        self.frame = gp('frame')
        self.L = float(gp('wheelbase'))
        self.veh_len = float(gp('vehicle_length'))
        self.veh_w = float(gp('vehicle_width'))
        self.rear_oh = float(gp('rear_overhang'))
        self.trail_len = int(gp('trail_len'))
        self.trail = []

        # ── 트랙 로드 ────────────────────────────────────────────────
        tdir = os.path.expanduser(gp('track_dir'))
        wp = os.path.expanduser(gp('waypoint_file'))
        self.center = self.left = self.right = None

        if tdir and os.path.isdir(tdir):
            for name, attr in (('centerline.csv', 'center'),
                               ('left_boundary.csv', 'left'),
                               ('right_boundary.csv', 'right')):
                f = os.path.join(tdir, name)
                if os.path.isfile(f):
                    setattr(self, attr, load_xy(f))
        # ★ 파일명만 줘도 찾는다 — config/track.yaml 이 'track_5.csv' 처럼
        #   파일명만 적기 때문이다 (절대경로는 장비마다 달라 공유가 안 된다).
        #   stanley_node 가 쓰는 PathManager.resolve() 와 같은 규칙.
        if wp:
            wp = PathManager.resolve(wp)
        if self.center is None and wp and os.path.isfile(wp):
            self.center = load_xy(wp)

        if self.center is None:
            self.get_logger().error(
                '✗ 트랙을 못 읽었습니다. track_dir 또는 waypoint_file 을 주세요.')
        else:
            n = len(self.center)
            bnd = '경계 있음' if self.left is not None else '경계 없음 (중심선만)'
            self.get_logger().info(
                f'트랙 로드: 중심선 {n}점, {bnd}\n'
                f'  x[{self.center[:,0].min():+.2f}, {self.center[:,0].max():+.2f}] '
                f'y[{self.center[:,1].min():+.2f}, {self.center[:,1].max():+.2f}]  '
                f'(frame={self.frame})')

        # ── 발행 ─────────────────────────────────────────────────────
        self.pub_center = self.create_publisher(Path, '/track/centerline', LATCHED)
        self.pub_left = self.create_publisher(Path, '/track/left_bound', LATCHED)
        self.pub_right = self.create_publisher(Path, '/track/right_bound', LATCHED)
        self.pub_static = self.create_publisher(
            MarkerArray, '/track/markers', LATCHED)
        self.pub_car = self.create_publisher(MarkerArray, '/viz/car', 10)
        self.pub_trail = self.create_publisher(Path, '/viz/trail', 10)

        self.create_subscription(Odometry, '/mpc/state', self._state_cb, 10)

        # 트랙은 한 번만 그리면 되지만, RViz 재시작 대비로 몇 초마다 다시 낸다
        self._publish_static()
        self.create_timer(5.0, self._publish_static)

    # ────────────────────────────────────────────────────────────────
    def _mk_path(self, xy, close=True):
        p = Path()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self.frame
        seq = list(xy) + ([xy[0]] if close and len(xy) > 2 else [])
        for x, y in seq:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        return p

    def _publish_static(self):
        if self.center is None:
            return
        now = self.get_clock().now().to_msg()
        self.pub_center.publish(self._mk_path(self.center))
        if self.left is not None:
            self.pub_left.publish(self._mk_path(self.left))
        if self.right is not None:
            self.pub_right.publish(self._mk_path(self.right))

        arr = MarkerArray()
        mid = 0

        # ── 트랙 면 — 좌우 경계 사이를 삼각형으로 채운다 ─────────────
        if self.left is not None and self.right is not None:
            n = min(len(self.left), len(self.right))
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame
            m.ns = 'track'
            m.id = mid; mid += 1
            m.type = Marker.TRIANGLE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color = rgba(0.35, 0.35, 0.38, 0.45)
            m.pose.orientation.w = 1.0
            for i in range(n):
                j = (i + 1) % n
                l1, r1 = self.left[i], self.right[i]
                l2, r2 = self.left[j], self.right[j]
                for a, b, c in ((l1, r1, r2), (l1, r2, l2)):
                    for pt in (a, b, c):
                        m.points.append(Point(x=float(pt[0]),
                                              y=float(pt[1]), z=-0.01))
            arr.markers.append(m)

        # ── 출발점 ──────────────────────────────────────────────────
        s = Marker()
        s.header.stamp = now
        s.header.frame_id = self.frame
        s.ns = 'start'
        s.id = mid; mid += 1
        s.type = Marker.CYLINDER
        s.action = Marker.ADD
        s.pose.position.x = float(self.center[0][0])
        s.pose.position.y = float(self.center[0][1])
        s.pose.position.z = 0.0
        s.pose.orientation.w = 1.0
        s.scale.x = s.scale.y = 0.15
        s.scale.z = 0.02
        s.color = rgba(1.0, 0.85, 0.1, 0.9)
        arr.markers.append(s)

        # ── safety 경계 ─────────────────────────────────────────────
        if self.get_parameter('show_safety').value:
            gp = lambda n: float(self.get_parameter(n).value)
            x0, x1 = gp('x_min'), gp('x_max')
            y0, y1 = gp('y_min'), gp('y_max')
            b = Marker()
            b.header.stamp = now
            b.header.frame_id = self.frame
            b.ns = 'safety'
            b.id = mid; mid += 1
            b.type = Marker.LINE_STRIP
            b.action = Marker.ADD
            b.scale.x = 0.02
            b.color = rgba(1.0, 0.2, 0.2, 0.8)
            b.pose.orientation.w = 1.0
            for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)):
                b.points.append(Point(x=x, y=y, z=0.0))
            arr.markers.append(b)

        self.pub_static.publish(arr)

    # ────────────────────────────────────────────────────────────────
    def _state_cb(self, msg: Odometry):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y = p.position.x, p.position.y

        # ── 차량 외형 ───────────────────────────────────────────────
        #   /mpc/state 는 뒤축 중심이다. 박스 중심은 거기서
        #   (전장/2 − 뒤오버행) 만큼 앞이다.
        cx_off = self.veh_len / 2.0 - self.rear_oh
        arr = MarkerArray()

        body = Marker()
        body.header = msg.header
        body.header.frame_id = self.frame
        body.ns = 'car'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = x + math.cos(yaw) * cx_off
        body.pose.position.y = y + math.sin(yaw) * cx_off
        body.pose.position.z = 0.05
        body.pose.orientation.z = math.sin(yaw / 2.0)
        body.pose.orientation.w = math.cos(yaw / 2.0)
        body.scale.x = self.veh_len
        body.scale.y = self.veh_w
        body.scale.z = 0.10
        body.color = rgba(0.2, 0.6, 1.0, 0.8)
        arr.markers.append(body)

        # 앞 방향 화살표 — 뒤축에서 앞으로
        ar = Marker()
        ar.header = body.header
        ar.ns = 'car'
        ar.id = 1
        ar.type = Marker.ARROW
        ar.action = Marker.ADD
        ar.scale.x = 0.04
        ar.scale.y = 0.08
        ar.scale.z = 0.08
        ar.color = rgba(1.0, 0.3, 0.0, 1.0)
        ar.pose.orientation.w = 1.0
        ar.points.append(Point(x=x, y=y, z=0.12))
        ar.points.append(Point(x=x + math.cos(yaw) * 0.35,
                               y=y + math.sin(yaw) * 0.35, z=0.12))
        arr.markers.append(ar)
        self.pub_car.publish(arr)

        # ── 주행 자취 ───────────────────────────────────────────────
        self.trail.append((x, y))
        if len(self.trail) > self.trail_len:
            self.trail.pop(0)
        if len(self.trail) > 1:
            self.pub_trail.publish(self._mk_path(self.trail, close=False))


def main(args=None):
    rclpy.init(args=args)
    node = TrackViz()
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
