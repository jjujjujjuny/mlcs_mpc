#!/usr/bin/env python3
"""
path_manager — 웨이포인트 → MPC 참조 궤적.

MPC 는 "예측 구간의 매 스텝마다 어디에 어떤 속도로 있어야 하는가" 를
필요로 한다. 웨이포인트 CSV 는 그냥 점들의 나열이므로 그 사이를 메워야 한다.

■ 참조 궤적을 만드는 방법

  1. 현재 위치에서 가장 가까운 경로 지점을 찾는다
  2. 거기서부터 **예측 속도로 진행한 거리만큼** 앞으로 나아가며 샘플링
        s_k = s_now + sum(v_ref * dt)
  3. 각 지점의 (x, y, heading, v_ref) 를 낸다

  ★ 시간 기반(속도로 진행)이 핵심이다. 고정 거리 간격으로 샘플링하면
    빠를 때는 예측이 짧아 코너를 늦게 보고, 느릴 때는 너무 멀리 본다.
    MPC 는 "dt 뒤에 여기" 를 기대하므로 반드시 시간으로 떠야 한다.

■ 속도 프로파일 — 곡률 기반

    v_max(kappa) = sqrt(mu * g / |kappa|)

  횡가속도 a_lat = v^2 * kappa 가 마찰한계 mu*g 를 넘지 않는 속도다.
  여기에 grip_util(<1) 을 곱해 여유를 둔다. 26-jetson repo 의
  speed_profile.py 가 하는 것과 같은 계산이되, 여기서는 MPC 가 가감속
  자체를 최적화하므로 전방 예견 감속(전체 경로 역방향 전파)까지는 하지
  않는다 — MPC 의 예측 구간이 그 역할을 일부 대신한다.

  ⚠ 단, 예측 구간(1초)보다 먼 코너는 MPC 도 못 본다. 고속에서는 N 을
    늘리거나 여기에 역방향 전파를 넣어야 한다. docs/ROADMAP.md 참고.
"""

import csv
import math
import os

import numpy as np

G = 9.81


class PathManager:

    def __init__(self, waypoint_file, target_speed=2.0,
                 curvature_slowdown=1.5, closed_loop=True,
                 params=None, grip_util=0.6, logger=None):
        self.target_speed = float(target_speed)
        self.curvature_slowdown = float(curvature_slowdown)
        self.closed_loop = bool(closed_loop)
        self.params = params
        self.grip_util = float(grip_util)
        self.log = logger
        self.ok = False

        self.xy = None        # (M, 2)
        self.s = None         # 누적거리 (M,)
        self.heading = None   # (M,)
        self.kappa = None     # (M,)
        self.v_ref = None     # (M,)
        self.total_s = 0.0
        self._last_idx = 0

        if waypoint_file:
            self.load(waypoint_file)

    # ────────────────────────────────────────────────────────────────
    def load(self, path):
        if not os.path.isfile(path):
            self._err(f'파일이 없습니다: {path}')
            return False

        pts = []
        speeds = []
        with open(path, 'r') as f:
            for row in csv.reader(f):
                if not row:
                    continue
                first = row[0].strip()
                if not first or first.startswith('#'):
                    continue
                try:
                    x, y = float(row[0]), float(row[1])
                except ValueError:
                    continue    # 헤더 줄
                pts.append((x, y))
                speeds.append(float(row[2]) if len(row) > 2 else None)

        if len(pts) < 3:
            self._err(f'웨이포인트가 {len(pts)}개뿐입니다 (최소 3개 필요): {path}')
            return False

        self.xy = np.array(pts, dtype=float)

        # 중복점 제거 — 같은 점이 연속되면 heading 계산이 0/0 이 된다
        keep = [0]
        for i in range(1, len(self.xy)):
            if np.hypot(*(self.xy[i] - self.xy[keep[-1]])) > 1e-6:
                keep.append(i)
        if len(keep) != len(self.xy):
            self._info(f'중복 웨이포인트 {len(self.xy) - len(keep)}개 제거')
            self.xy = self.xy[keep]
            speeds = [speeds[i] for i in keep]

        self._compute_geometry()
        self._compute_speed(speeds)
        self.ok = True
        self._info(
            f'웨이포인트 {len(self.xy)}개 로드, 길이 {self.total_s:.2f}m, '
            f'{"폐곡선" if self.closed_loop else "개곡선"}, '
            f'v_ref {self.v_ref.min():.2f}~{self.v_ref.max():.2f} m/s')
        return True

    def _compute_geometry(self):
        xy = self.xy
        M = len(xy)

        d = np.zeros(M)
        d[1:] = np.hypot(*(xy[1:] - xy[:-1]).T)
        self.s = np.cumsum(d)
        if self.closed_loop:
            self.total_s = self.s[-1] + np.hypot(*(xy[0] - xy[-1]))
        else:
            self.total_s = self.s[-1]

        # heading: 중앙차분 (폐곡선이면 감아서)
        nxt = np.roll(xy, -1, axis=0)
        prv = np.roll(xy, 1, axis=0)
        if not self.closed_loop:
            nxt[-1] = xy[-1]
            prv[0] = xy[0]
        dxy = nxt - prv
        self.heading = np.arctan2(dxy[:, 1], dxy[:, 0])

        # 곡률: 세 점을 지나는 원의 역반경 (Menger 곡률)
        self.kappa = np.zeros(M)
        a = prv
        b = xy
        c = nxt
        ab = np.hypot(*(b - a).T)
        bc = np.hypot(*(c - b).T)
        ca = np.hypot(*(a - c).T)
        # 외적 → 삼각형 넓이의 2배
        cross = ((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) -
                 (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
        denom = ab * bc * ca
        good = denom > 1e-9
        self.kappa[good] = 2.0 * cross[good] / denom[good]

        # 곡률은 노이즈에 민감하다 — 웨이포인트가 손으로 찍힌 것이면 특히.
        # 이동평균으로 눌러준다. 안 하면 속도 프로파일이 들쭉날쭉해진다.
        self.kappa = self._smooth(self.kappa, 5)

    def _smooth(self, arr, w):
        if len(arr) < w * 2:
            return arr
        k = np.ones(w) / w
        if self.closed_loop:
            pad = np.concatenate([arr[-w:], arr, arr[:w]])
            return np.convolve(pad, k, mode='same')[w:-w]
        return np.convolve(arr, k, mode='same')

    def _compute_speed(self, csv_speeds):
        """CSV 3열에 속도가 있으면 그걸, 없으면 곡률로 계산."""
        if all(sp is not None for sp in csv_speeds):
            self.v_ref = np.array(csv_speeds, dtype=float)
            self._info('속도 프로파일: CSV 3열 사용')
            return

        mu = self.params.mu if self.params else 0.35
        vmax = self.params.max_speed if self.params else self.target_speed

        ak = np.abs(self.kappa)
        with np.errstate(divide='ignore'):
            v_grip = np.sqrt(self.grip_util * mu * G / np.maximum(ak, 1e-6))

        v = np.minimum(self.target_speed, v_grip)
        v = np.minimum(v, vmax)
        # curvature_slowdown: 곡률 구간을 추가로 더 줄이고 싶을 때의 손잡이
        v = v / (1.0 + self.curvature_slowdown * ak)
        self.v_ref = np.maximum(v, 0.3)
        self._info('속도 프로파일: 곡률 기반 계산')

    # ────────────────────────────────────────────────────────────────
    def nearest_index(self, x, y):
        """가장 가까운 웨이포인트 인덱스.

        ★ 전역 탐색이 아니라 지역 탐색이다. 폐곡선 경로는 스스로와
          가까워지는 구간(8자, 좁은 헤어핀)이 있어서 전역 최근접점이
          **경로 반대편** 으로 튈 수 있다. 직전 인덱스 주변만 보면
          그 사고가 구조적으로 막힌다.
        """
        M = len(self.xy)
        if M == 0:
            return 0

        win = 40
        idxs = np.arange(self._last_idx - 5, self._last_idx + win)
        if self.closed_loop:
            idxs = idxs % M
        else:
            idxs = idxs[(idxs >= 0) & (idxs < M)]

        d = np.hypot(self.xy[idxs, 0] - x, self.xy[idxs, 1] - y)
        best = idxs[int(np.argmin(d))]

        # 지역 탐색이 너무 멀면(경로를 놓쳤으면) 전역으로 한 번 복구
        if d.min() > 2.0:
            dg = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
            best = int(np.argmin(dg))
            if self.log:
                self.log.warn(
                    f'경로에서 {d.min():.1f}m 벗어남 — 전역 재탐색 (idx={best})')

        self._last_idx = int(best)
        return int(best)

    def reference(self, state, N, dt):
        """MPC 참조 궤적 (N+1, 4) = [x, y, psi, v_ref].

        경로를 따라 시간으로 전진하며 뜬다.
        """
        if not self.ok:
            # 경로가 없으면 제자리 정지를 참조로 준다
            return np.tile([state[0], state[1], state[2], 0.0], (N + 1, 1))

        i0 = self.nearest_index(state[0], state[1])
        s0 = self.s[i0]

        ref = np.zeros((N + 1, 4))
        s = s0
        for k in range(N + 1):
            x, y, psi, v = self._at_s(s)
            ref[k] = (x, y, psi, v)
            s += v * dt      # ← 시간 기반 전진
        return ref

    def _at_s(self, s):
        """경로 길이 s 에서의 (x, y, heading, v_ref). 선형보간."""
        if self.closed_loop:
            s = s % self.total_s
        else:
            s = min(max(s, 0.0), self.total_s)

        i = int(np.searchsorted(self.s, s, side='right') - 1)
        i = max(0, min(i, len(self.s) - 1))
        j = (i + 1) % len(self.xy) if self.closed_loop else min(i + 1, len(self.xy) - 1)

        seg = self.s[j] - self.s[i] if j > i else self.total_s - self.s[i]
        t = 0.0 if seg <= 1e-9 else (s - self.s[i]) / seg
        t = min(max(t, 0.0), 1.0)

        x = self.xy[i, 0] + t * (self.xy[j, 0] - self.xy[i, 0])
        y = self.xy[i, 1] + t * (self.xy[j, 1] - self.xy[i, 1])

        # heading 은 각도라 선형보간 전에 감아야 한다
        dh = (self.heading[j] - self.heading[i] + math.pi) % (2 * math.pi) - math.pi
        psi = self.heading[i] + t * dh
        v = self.v_ref[i] + t * (self.v_ref[j] - self.v_ref[i])
        return x, y, psi, v

    # ────────────────────────────────────────────────────────────────
    def _info(self, m):
        if self.log:
            self.log.info(f'[path] {m}')

    def _err(self, m):
        if self.log:
            self.log.error(f'[path] {m}')
