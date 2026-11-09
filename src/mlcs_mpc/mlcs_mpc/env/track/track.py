"""
track — 닫힌 트랙 (prep_*.csv: s,x,y,heading,curvature,track_width[,w_right,w_left]).

  데이터 prep_<이름>.csv 대신 같은 폴더의 ref_<이름>.csv (센터라인 스플라인 하나에서 x·y·heading·κ, reference.py)
  MPC  kappa_grid() 를 OCP 빌드 때 넣는다 → OCP 가 상태 s 로 곡률을 스스로 찾는다 (env/mpc/acados_ocp.py)
  노드 frenet() 으로 (x, y, yaw) → (s, n, mu), bounds_at() 으로 지점별 벽
  시뮬 같은 frenet()·bounds_at() 으로 벽·랩
"""
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Track:
    """prep_*.csv (s,x,y,heading,curvature,track_width). 닫힌 트랙."""

    def __init__(self, path):
        # prep_<이름>.csv 를 받으면 센터라인에서 다시 계산한 ref_<이름>.csv 를 쓴다 (reference.py)
        from mlcs_mpc.env.track.reference import resolve
        self.path = resolve(path)
        d = np.genfromtxt(self.path, delimiter=',', names=True)
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

    def heading_at(self, s):
        q = np.mod(s, self.length)
        return float(np.interp(q, np.r_[self.s, self.length], np.unwrap(np.r_[self.heading, self.heading[0]])))

    def kappa_grid(self):
        """닫힌 트랙의 κ(s) 격자 (s 0 … length, 끝값 = 첫 값). OCP 가 B-스플라인으로 생성 코드에 넣는다."""
        return np.r_[self.s, self.length], np.r_[self.kappa, self.kappa[0]]
