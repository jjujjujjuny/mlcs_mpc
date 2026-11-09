"""
sim_rviz — main_sim.py 의 RViz 출력 전부 (노드·퍼블리셔·TF·마커·RViz 프로세스). 루프는 main_sim.py 에 있다.

  viz = SimRviz(track, margin=cfg.wall_margin, lr=model.lr, car_width=2 * sim.half_width)
  viz.spin()                → RViz '2D Pose Estimate' 로 찍은 (x, y, yaw) 들 (다시 세우기 요청)
  viz.draw(sim, info, tick, pred_xy)   제어 주기마다
  viz.close()

  토픽 (frame map)
    /sim/markers   트랙 면·벽·기준 라인·MPC 경계·출발선 (1 초마다 다시 — Foxy RViz 는 래치를 못 받는다)
    /sim/car       차 (벽에 닿으면 빨강)          /sim/predicted  MPC 예측 궤적 (주황)
    /sim/trail     지나온 자취 (노랑, 기본 전부)   /sim/text       범례 판 (트랙 옆 고정) — 속도·풀이 시간·랩·벽 + 색 범례
    /mpc/state     뒤축 Odometry                  TF map → base_link
"""
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from mlcs_mpc.utils.sim_viz import LEGEND, car_marker, legend_anchor, line, text_marker, track_markers

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
RVIZ_CONFIG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'launch', 'sim.rviz'))


class SimRviz:

    def __init__(self, track, margin, lr, car_width, car_length=0.505, trail=0.0, period=1.0 / 30.0,
                 rviz=True, node_name='mlcs_main_sim'):
        rclpy.init()
        self.node = n = rclpy.create_node(node_name)
        self.pub_track = n.create_publisher(MarkerArray, '/sim/markers', LATCHED)
        self.pub_car = n.create_publisher(Marker, '/sim/car', 1)
        self.pub_pred = n.create_publisher(Marker, '/sim/predicted', 1)
        self.pub_trail = n.create_publisher(Marker, '/sim/trail', 1)
        self.pub_text = n.create_publisher(Marker, '/sim/text', 1)
        self.pub_state = n.create_publisher(Odometry, '/mpc/state', 10)
        self.tf = TransformBroadcaster(n)
        self._resets = []
        n.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._initialpose, 1)
        self.track_msg = track_markers(track, margin)
        self.anchor = legend_anchor(track)
        self.solve_ms = []
        self.pub_track.publish(self.track_msg)
        self.t_track = time.monotonic()
        self.lr, self.car_w, self.car_l = lr, car_width, car_length
        self.keep = int(trail / period) if trail > 0 else 0      # 0 = 처음부터 전부
        self.trail = []
        self.proc = None
        if rviz:
            self.proc = subprocess.Popen(['rviz2', '-d', RVIZ_CONFIG],
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _initialpose(self, m):
        p = m.pose.pose
        self._resets.append((p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w)))

    def ok(self):
        return rclpy.ok()

    def spin(self):
        """콜백 처리 + 트랙 다시 보내기. → 다시 세우기 요청 [(x, y, yaw), ...]."""
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if time.monotonic() - self.t_track > 1.0:
            self.pub_track.publish(self.track_msg)
            self.t_track = time.monotonic()
        out, self._resets = self._resets, []
        return out

    def clear_trail(self):
        self.trail = []

    def draw(self, sim, info, tick, pred_xy=None):
        """제어 주기마다 — 차·예측·자취·글·상태·TF."""
        stamp = self.node.get_clock().now().to_msg()
        x, y, yaw = sim.pose()
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        self.trail.append((x, y))
        if self.keep:
            self.trail = self.trail[-self.keep:]
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'map', 'base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y = info['vx'], info['vy']
        od.twist.twist.angular.z = info['r']
        self.pub_state.publish(od)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'map', 'base_link'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)
        self.pub_car.publish(car_marker(x, y, yaw, self.lr, self.car_l, self.car_w, info['on_wall'], stamp))
        if pred_xy is not None:
            self.pub_pred.publish(line('predicted', 0, *pred_xy, (1.0, 0.55, 0.0, 0.9), 0.03, stamp))
        tx, ty = zip(*self.trail)
        self.pub_trail.publish(line('trail', 0, tx, ty, (1.0, 0.9, 0.1, 0.8), 0.02, stamp))
        if tick['status'] >= 0:
            self.solve_ms = (self.solve_ms + [tick['ms']])[-300:]
        st = {-1: 'start (straight, no solve)', 0: 'ok'}.get(tick['status'], f'FAIL status {tick["status"]}')
        laps = ' '.join(f'{t:.2f}' for t in sim.laps[-4:]) or '-'
        txt = (f'time   {sim.t:6.1f} s\n'
               f'speed  {info["vx"]:4.2f} m/s    n {info["n"]:+.3f} m\n'
               f'solve  {tick["ms"]:4.1f} ms (recent max {max(self.solve_ms or [0]):.1f})  {st}\n'
               f'laps   {len(sim.laps)}: {laps} s\n'
               f'walls  {sim.wall_hits}\n'
               f'------------------------\n' + LEGEND)
        ax, ay = self.anchor
        self.pub_text.publish(text_marker(ax, ay, txt, stamp))

    def close(self):
        if self.proc is not None:
            self.proc.terminate()
        try:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
