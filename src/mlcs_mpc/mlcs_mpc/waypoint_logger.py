#!/usr/bin/env python3
"""
waypoint_logger — 차를 손으로/조이스틱으로 몰면서 주행 경로를 CSV 로 남긴다.

라이다 SLAM 이 없으므로 "맵" 개념이 없다. mocap 좌표계 자체가 전역 맵이고,
경로는 그 위의 점들이다. 그래서 맵 만들기 단계 없이 바로 경로를 딴다 —
이 장비 구성의 이점이다.

  ros2 run mlcs_mpc waypoint_logger --ros-args -p output:=track1.csv

  Ctrl+C 로 종료하면 저장된다.

■ 점 사이 간격

  min_dist(기본 0.1m) 이상 움직였을 때만 기록한다. mocap 100Hz 를 전부 쓰면
  정지 중에도 노이즈만큼의 점이 수천 개 쌓이고, 곡률 계산이 그 노이즈를
  증폭한다 (곡률은 2차 미분이라 특히 민감하다).

■ 저장 후 할 일

  이 파일은 **사람이 운전한 날것의 경로**다. 레이싱 라인이 아니다.
  MPC 가 이걸 그대로 추종하면 사람의 실수까지 따라한다. 실제로 쓰기 전에
  smooth_path 로 한 번 다듬는 것을 권한다:

    ros2 run mlcs_mpc smooth_path --ros-args -p input:=track1.csv -p output:=track1_smooth.csv
"""

import csv
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WaypointLogger(Node):

    def __init__(self):
        super().__init__('waypoint_logger')

        self.declare_parameter('output', 'waypoints.csv')
        self.declare_parameter('min_dist', 0.1)
        self.declare_parameter('log_speed', True)

        self.out = os.path.expanduser(self.get_parameter('output').value)
        self.min_dist = float(self.get_parameter('min_dist').value)
        self.log_speed = bool(self.get_parameter('log_speed').value)

        self.pts = []
        self.last = None

        self.create_subscription(Odometry, '/mpc/state', self._cb, 10)
        self.create_timer(2.0, self._status)
        self.get_logger().info(
            f'웨이포인트 기록 시작 → {self.out} (간격 {self.min_dist}m)\n'
            '  차를 트랙 한 바퀴 몰고 Ctrl+C 로 종료하세요.')

    def _cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x

        if self.last is not None:
            if math.hypot(x - self.last[0], y - self.last[1]) < self.min_dist:
                return
        self.last = (x, y)
        self.pts.append((x, y, v))

    def _status(self):
        if self.pts:
            d = 0.0
            a = np.array(self.pts)
            if len(a) > 1:
                d = float(np.sum(np.hypot(*np.diff(a[:, :2], axis=0).T)))
            self.get_logger().info(f'  {len(self.pts)}점, {d:.1f}m')

    def save(self):
        if len(self.pts) < 3:
            self.get_logger().error(
                f'점이 {len(self.pts)}개뿐이라 저장하지 않습니다.')
            return
        d = os.path.dirname(self.out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# x', 'y'] + (['speed'] if self.log_speed else []))
            for x, y, v in self.pts:
                w.writerow([f'{x:.4f}', f'{y:.4f}'] +
                           ([f'{v:.3f}'] if self.log_speed else []))

        a = np.array([(p[0], p[1]) for p in self.pts])
        length = float(np.sum(np.hypot(*np.diff(a, axis=0).T)))
        gap = float(np.hypot(*(a[0] - a[-1])))
        print(f'\n저장 완료: {self.out}')
        print(f'  {len(self.pts)}점, 경로길이 {length:.2f}m')
        print(f'  시작점-끝점 거리 {gap:.2f}m '
              f'{"(폐곡선으로 보임)" if gap < 1.0 else "(개곡선 — closed_loop:=false 를 쓰세요)"}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
