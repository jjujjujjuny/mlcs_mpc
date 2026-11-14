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
        # 지점별 벽 폭 (MPCC 경계). maptrack --walls 가 쓴다. 없으면 track_width 의 반씩.
        self.has_walls = 'w_left' in d.dtype.names
        if self.has_walls:
            self.w_left, self.w_right = d['w_left'], d['w_right']
        else:
            self.w_left = self.w_right = d['track_width'] / 2.0
        close = math.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1])
        self.length = float(self.s[-1] + close)
        if close < 1e-6:    # 저자 전처리는 끝점을 시작점과 겹쳐 둔다
            self.s, self.x, self.y = self.s[:-1], self.x[:-1], self.y[:-1]
            self.heading, self.kappa = self.heading[:-1], self.kappa[:-1]
            self.w_left, self.w_right = self.w_left[:-1], self.w_right[:-1]
        self.n = len(self.s)
        self.last = None
        self.s_cont = None

    def kappa_at(self, s):
        q = np.mod(s, self.length)
        return np.interp(q, np.r_[self.s, self.length], np.r_[self.kappa, self.kappa[0]])

    def bounds_at(self, s, margin):
        """n 의 [하한, 상한] = [-(오른쪽 폭 - margin), 왼쪽 폭 - margin]. n 은 왼쪽 +."""
        q = np.mod(s, self.length)
        S = np.r_[self.s, self.length]
        wl = np.interp(q, S, np.r_[self.w_left, self.w_left[0]])
        wr = np.interp(q, S, np.r_[self.w_right, self.w_right[0]])
        return -(wr - margin), wl - margin

    def bounds_asym(self, s, margin, inner=None, outer=None, k_full=1.0):
        """bounds_at 과 같은데 코너 안쪽·바깥쪽 여유를 따로 (11-14 대회장: 차 앞모서리가 안쪽 벽에 닿음).
        여유 = margin + (inner 또는 outer − margin) × min(|κ| / k_full, 1). κ > 0 (왼쪽 커브) 이면 왼쪽이 안쪽.
        inner·outer 가 None 이면 margin 그대로 (= bounds_at)."""
        if inner is None and outer is None:
            return self.bounds_at(s, margin)
        inner = margin if inner is None else inner
        outer = margin if outer is None else outer
        q = np.mod(s, self.length)
        S = np.r_[self.s, self.length]
        wl = np.interp(q, S, np.r_[self.w_left, self.w_left[0]])
        wr = np.interp(q, S, np.r_[self.w_right, self.w_right[0]])
        k = self.kappa_at(q)
        w = np.clip(np.abs(k) / k_full, 0.0, 1.0)
        m_in, m_out = margin + (inner - margin) * w, margin + (outer - margin) * w
        m_l = np.where(k > 0, m_in, m_out)
        m_r = np.where(k > 0, m_out, m_in)
        return -(wr - m_r), wl - m_l

    def xy(self, s, n):
        """(s, n) → (x, y). 선 위 점에서 왼쪽 법선 방향으로 n. 시각화용 (점 사이는 선형 보간)."""
        q = np.mod(s, self.length)
        S = np.r_[self.s, self.length]
        x = np.interp(q, S, np.r_[self.x, self.x[0]])
        y = np.interp(q, S, np.r_[self.y, self.y[0]])
        h = np.interp(q, S, np.unwrap(np.r_[self.heading, self.heading[0]]))
        return x - np.sin(h) * n, y + np.cos(h) * n

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
        # ★ 점 간격이 20 cm 라 가장 가까운 점의 heading 을 그대로 쓰면 곡선에서 mu 가 점을
        #   지날 때마다 약 0.2 rad 씩 튄다. 이웃 점과 보간한다.
        i = self._nearest(x, y)
        dx, dy = x - self.x[i], y - self.y[i]
        h = self.heading[i]
        along = math.cos(h) * dx + math.sin(h) * dy
        j = (i + 1) % self.n if along >= 0 else (i - 1) % self.n
        gap = abs(self.length - abs(self.s[j] - self.s[i])) if abs(j - i) > 1 \
            else abs(self.s[j] - self.s[i])
        frac = min(abs(along) / max(gap, 1e-6), 1.0)
        h = h + frac * wrap(self.heading[j] - h)
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


def steer_to_command(actual, map_cmd, map_act):
    """실제 조향 → 서보 명령. 맵은 원선회로 잰 좌우 평균이다 (0 에서 시작)."""
    return math.copysign(float(np.interp(abs(actual), map_act, map_cmd)), actual)


def to_command(u, v_max, max_steer, prev_steer, max_dsteer, map_cmd, map_act):
    """해 → (속도, 조향 명령). 솔버 경계와 별개로 여기서 다시 묶는다.

    u[1] 은 모델의 실제 조향이다. max_steer, 변화율, prev_steer 는 명령 단위다.
    """
    speed = float(np.clip(u[0], 0.0, v_max))
    cmd = steer_to_command(u[1], map_cmd, map_act)
    steer = float(np.clip(cmd, prev_steer - max_dsteer, prev_steer + max_dsteer))
    return speed, float(np.clip(steer, -max_steer, max_steer))
