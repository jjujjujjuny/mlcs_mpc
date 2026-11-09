"""
sim_viz — 시뮬레이션 RViz 마커 (트랙 면·벽·기준 라인·MPC 경계·출발선, 차, MPC 예측 궤적, 자취, 상태 글).
main_sim.py 와 io/sim_node.py 가 쓴다. 프레임은 map (트랙 좌표계).
"""
import math

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _color(rgba):
    return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))


def line(ns, i, xs, ys, rgba, width, stamp=None):
    m = Marker()
    m.header.frame_id, m.ns, m.id = 'map', ns, i
    if stamp is not None:
        m.header.stamp = stamp
    m.type, m.action = Marker.LINE_STRIP, Marker.ADD
    m.scale.x = width
    m.pose.orientation.w = 1.0
    m.color = _color(rgba)
    m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in zip(xs, ys)]
    return m


def track_markers(track, margin=None, ds=0.03):
    """트랙 면 (회색 삼각형), 좌·우 벽 (흰색), 기준 라인 (초록 점선), MPC 경계 = 벽 − margin (빨강)."""
    ss = np.linspace(0.0, track.length, max(200, int(track.length / ds)) + 1)
    S = np.r_[track.s, track.length]
    wl = np.interp(ss, S, np.r_[track.w_left, track.w_left[0]])
    wr = np.interp(ss, S, np.r_[track.w_right, track.w_right[0]])
    lx, ly = track.xy(ss, wl)
    rx, ry = track.xy(ss, -wr)
    arr = MarkerArray()
    surf = Marker()
    surf.header.frame_id, surf.ns, surf.id = 'map', 'track_surface', 0
    surf.type, surf.action = Marker.TRIANGLE_LIST, Marker.ADD
    surf.scale.x = surf.scale.y = surf.scale.z = 1.0
    surf.pose.orientation.w = 1.0
    surf.color = _color((0.35, 0.35, 0.38, 0.6))
    for i in range(len(ss) - 1):
        a0, a1 = Point(x=float(lx[i]), y=float(ly[i])), Point(x=float(lx[i + 1]), y=float(ly[i + 1]))
        b0, b1 = Point(x=float(rx[i]), y=float(ry[i])), Point(x=float(rx[i + 1]), y=float(ry[i + 1]))
        surf.points += [a0, b0, a1, a1, b0, b1]
    arr.markers.append(surf)
    arr.markers.append(line('track', 1, lx, ly, (1, 1, 1, 1), 0.04))          # 왼쪽 벽
    arr.markers.append(line('track', 2, rx, ry, (1, 1, 1, 1), 0.04))          # 오른쪽 벽
    cx, cy = track.xy(ss, 0 * ss)
    dash = Marker()                                                          # 기준 라인 (점선)
    dash.header.frame_id, dash.ns, dash.id = 'map', 'track', 0
    dash.type, dash.action = Marker.LINE_LIST, Marker.ADD
    dash.scale.x = 0.02
    dash.pose.orientation.w = 1.0
    dash.color = _color((0.2, 0.95, 0.3, 0.9))
    for i in range(0, len(ss) - 1, 2):
        dash.points += [Point(x=float(cx[i]), y=float(cy[i]), z=0.02), Point(x=float(cx[i + 1]), y=float(cy[i + 1]), z=0.02)]
    arr.markers.append(dash)
    if margin is not None:
        for j, n in ((3, wl - margin), (4, -(wr - margin))):
            mx, my = track.xy(ss, n)
            arr.markers.append(line('track', j, mx, my, (1.0, 0.25, 0.25, 0.7), 0.015))
    sm = Marker()                                                            # 출발선 (s = 0)
    sm.header.frame_id, sm.ns, sm.id = 'map', 'track', 5
    sm.type, sm.action = Marker.LINE_STRIP, Marker.ADD
    sm.scale.x = 0.05
    sm.pose.orientation.w = 1.0
    sm.color = _color((1.0, 1.0, 0.2, 1.0))
    sm.points = [Point(x=float(lx[0]), y=float(ly[0]), z=0.03), Point(x=float(rx[0]), y=float(ry[0]), z=0.03)]
    arr.markers.append(sm)
    return arr


def car_marker(x, y, yaw, lr, length, width, on_wall, stamp):
    m = Marker()
    m.header.frame_id, m.header.stamp, m.ns, m.id = 'map', stamp, 'car', 0
    m.type, m.action = Marker.CUBE, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x + lr * math.cos(yaw), y + lr * math.sin(yaw), 0.05
    m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.scale.x, m.scale.y, m.scale.z = length, width, 0.1
    m.color = _color((1.0, 0.2, 0.2, 0.9) if on_wall else (0.2, 0.4, 1.0, 0.9))
    return m


def text_marker(x, y, text, stamp, size=0.16, ns='status', rgba=(1, 1, 1, 1)):
    """map 의 고정 위치 (x, y) 에 여러 줄 글 — 범례 판처럼 쓴다 (RViz2 에는 화면 고정 글이 없다)."""
    m = Marker()
    m.header.frame_id, m.header.stamp, m.ns, m.id = 'map', stamp, ns, 0
    m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.3
    m.pose.orientation.w = 1.0
    m.scale.z = size
    m.color = _color(rgba)
    m.text = text
    return m


# RViz2 글꼴에는 한글이 없어 영어로 쓴다
LEGEND = """track: gray surface, white walls
reference line: green dashed
MPC bound: red (wall - margin)
start line: yellow
car: blue (red = wall contact)
MPC prediction: orange   trail: yellow"""


def legend_anchor(track):
    """트랙 오른쪽 바깥 (범례 판 자리)."""
    ss = np.linspace(0.0, track.length, 200)
    S = np.r_[track.s, track.length]
    xs, ys = [], []
    for w, sg in ((track.w_left, 1), (track.w_right, -1)):
        x, y = track.xy(ss, sg * np.interp(ss, S, np.r_[w, w[0]]))
        xs.append(x)
        ys.append(y)
    xs, ys = np.concatenate(xs), np.concatenate(ys)
    return float(xs.max()) + 0.4, float(ys.max())
