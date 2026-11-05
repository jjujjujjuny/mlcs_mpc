#!/usr/bin/env python3
"""
hmpc_core — hmpc_node 의 ROS 없는 계산. test/test_hmpc_core.py 로 시험한다.

  OCP 상태 x = [s, n, mu, vx, vy, r, friction, wheel_speed, delta]
  모델은 무게중심 기준이고 /mpc/state 는 뒤축 기준이다.
"""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Track:
    """prep_*.csv (s,x,y,heading,curvature,track_width). 닫힌 트랙."""

    def __init__(self, path):
        d = np.genfromtxt(path, delimiter=',', names=True)
        self.s, self.x, self.y = d['s'], d['x'], d['y']
        self.heading, self.kappa = d['heading'], d['curvature']
        n = len(self.s)
        close = math.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1])
        self.length = float(self.s[-1] + close)
        self.n = n
        self.last = None
        self.s_cont = None

    def kappa_at(self, s):
        q = np.mod(s, self.length)
        return np.interp(q, np.r_[self.s, self.length], np.r_[self.kappa, self.kappa[0]])

    def _nearest(self, x, y):
        if self.last is None:
            i = int(np.argmin((self.x - x) ** 2 + (self.y - y) ** 2))
        else:
            # 전 구간을 찾으면 트랙이 가까이 지나는 곳으로 점프할 수 있다.
            idx = (self.last + np.arange(-5, 15)) % self.n
            i = int(idx[np.argmin((self.x[idx] - x) ** 2 + (self.y[idx] - y) ** 2)])
        self.last = i
        return i

    def frenet(self, x, y, yaw):
        """(s_연속, n, mu). n 은 왼쪽이 +. s 는 랩을 넘어도 끊기지 않는다."""
        i = self._nearest(x, y)
        h = self.heading[i]
        dx, dy = x - self.x[i], y - self.y[i]
        s = self.s[i] + math.cos(h) * dx + math.sin(h) * dy
        n = -math.sin(h) * dx + math.cos(h) * dy
        if self.s_cont is None:
            self.s_cont = float(s)
        else:
            self.s_cont += wrap((s - self.s_cont) * 2 * math.pi / self.length) \
                * self.length / (2 * math.pi)
        return self.s_cont, float(n), wrap(yaw - h)

    def reset(self):
        self.last = None
        self.s_cont = None


def make_state(track, pose, twist, wheel_speed, delta, lr, v_floor, friction):
    """/mpc/state (뒤축) → OCP x (무게중심).

    v_x 는 v_floor 아래로 안 내린다. 저자 비용의 atan(vy/vx) 와 슬립 분모는 v_x = 0 에서
    발산한다. 서 있어도 조향은 계산되고, 출발 속도 명령은 ws_ref 하한이 아니라 비용이 정한다.
    """
    x, y, yaw = pose
    vx, vy, r = twist
    cx = x + lr * math.cos(yaw)
    cy = y + lr * math.sin(yaw)
    s, n, mu = track.frenet(cx, cy, yaw)
    return np.array([s, n, mu, max(vx, v_floor), vy + lr * r, r, friction,
                     max(wheel_speed, v_floor), delta])


def solution_ok(status, u, x1):
    """내보내도 되는 해인가. status 0 이고 전부 유한해야 한다."""
    return int(status) == 0 and bool(np.all(np.isfinite(u))) and bool(np.all(np.isfinite(x1)))


def to_command(u, v_max, max_steer, prev_steer, max_dsteer):
    """해 → (속도, 조향). 솔버 경계와 별개로 여기서 다시 묶는다."""
    speed = float(np.clip(u[0], 0.0, v_max))
    steer = float(np.clip(u[1], prev_steer - max_dsteer, prev_steer + max_dsteer))
    return speed, float(np.clip(steer, -max_steer, max_steer))
