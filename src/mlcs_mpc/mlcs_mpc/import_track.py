#!/usr/bin/env python3
"""
import_track — DeepRacer 트랙 폴더를 우리 웨이포인트 형식으로 가져온다.

  ros2 run mlcs_mpc import_track --ros-args \
      -p src:=/home/user/Desktop/deepracer_mpc_low_level/tracks/track_5 \
      -p out:=<repo>/src/mlcs_mpc/waypoints/track_5.csv

■ 하는 일

  ① centerline.csv (idx,x,y) → 우리 형식 (x,y[,speed])
  ② **차량 주행 가능성 검사** — 이게 핵심이다
  ③ 곡률 기반 속도 프로파일을 3열로 넣는다

■ ★ 왜 주행 가능성 검사가 필요한가

  이 트랙들은 **DeepRacer 기준**으로 만들어졌다. DeepRacer 는 조향각이
  우리보다 크므로, 그 차가 돌 수 있던 코너를 우리 차는 못 돈다.

  우리 차 한계:  R_min = L/tan(δ_max) = 0.33/tan(0.36) = 0.877 m

  실측 결과 (2026-09-21):

      track_1  최소반경 0.294 m → 35.0% 구간 주행 불가
      track_2  최소반경 0.191 m → 33.2% 불가
      track_3  최소반경 0.355 m → 41.7% 불가
      track_4  최소반경 0.220 m → 34.1% 불가
      track_5  최소반경 1.300 m → ✓ 전 구간 주행 가능

  track_5(스타디움) 만 그대로 쓸 수 있다. 나머지는 곡률을 완화하지
  않으면 코너에서 반드시 경로를 벗어난다 — 조향을 끝까지 줘도 못 도니
  게인을 아무리 튜닝해도 소용없다.
"""

import csv
import json
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node


def load_centerline(path):
    xy = []
    with open(path) as f:
        for row in csv.DictReader(f):
            xy.append((float(row['x']), float(row['y'])))
    return np.array(xy)


def curvature(xy, closed=True):
    """Menger 곡률 — 연속한 세 점을 지나는 원의 역반경."""
    nxt = np.roll(xy, -1, axis=0)
    prv = np.roll(xy, 1, axis=0)
    if not closed:
        nxt[-1] = xy[-1]
        prv[0] = xy[0]
    ab = np.hypot(*(xy - prv).T)
    bc = np.hypot(*(nxt - xy).T)
    ca = np.hypot(*(prv - nxt).T)
    cross = ((xy[:, 0] - prv[:, 0]) * (nxt[:, 1] - prv[:, 1]) -
             (xy[:, 1] - prv[:, 1]) * (nxt[:, 0] - prv[:, 0]))
    den = ab * bc * ca
    k = np.zeros(len(xy))
    g = den > 1e-12
    k[g] = 2.0 * cross[g] / den[g]
    return k


class ImportTrack(Node):

    def __init__(self):
        super().__init__('import_track')
        self.declare_parameter('src', '')
        self.declare_parameter('out', '')
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('max_steer', 0.36)
        self.declare_parameter('mu', 0.35)
        self.declare_parameter('grip_util', 0.6)
        self.declare_parameter('target_speed', 1.0)
        # ★ 기본은 속도 열을 **쓰지 않는다**.
        #   PathManager 는 CSV 3열에 speed 가 있으면 그걸 우선하고
        #   런타임의 target_speed 파라미터를 무시한다. 그러면
        #       ros2 launch ... target_speed:=0.5
        #   로 저속 시험을 하려 해도 CSV 에 박힌 속도로 달린다 —
        #   첫 주행에서 위험하다. 속도는 런타임에 정하게 둔다.
        #   곡률 기반 프로파일을 CSV 에 굽고 싶으면 write_speed:=true.
        self.declare_parameter('write_speed', False)
        self.declare_parameter('spacing', 0.05)   # 재샘플 간격 (m)
        self.declare_parameter('smooth_window', 5)

        src = os.path.expanduser(self.get_parameter('src').value)
        out = os.path.expanduser(self.get_parameter('out').value)
        L = float(self.get_parameter('wheelbase').value)
        dmax = float(self.get_parameter('max_steer').value)
        mu = float(self.get_parameter('mu').value)
        grip = float(self.get_parameter('grip_util').value)
        vmax = float(self.get_parameter('target_speed').value)
        write_speed = bool(self.get_parameter('write_speed').value)
        spacing = float(self.get_parameter('spacing').value)
        win = int(self.get_parameter('smooth_window').value)

        if not src:
            print('✗ src 파라미터가 필요합니다 (트랙 폴더 경로)')
            sys.exit(1)
        cl = os.path.join(src, 'centerline.csv')
        if not os.path.isfile(cl):
            print(f'✗ centerline.csv 가 없습니다: {cl}')
            sys.exit(1)
        if not out:
            out = os.path.join(os.getcwd(),
                               os.path.basename(src.rstrip('/')) + '.csv')

        xy = load_centerline(cl)
        meta = {}
        mj = os.path.join(src, 'track.json')
        if os.path.isfile(mj):
            meta = json.load(open(mj))

        # ── 등간격 재샘플 ────────────────────────────────────────────
        loop = np.vstack([xy, xy[:1]])
        d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(loop, axis=0).T))])
        total = d[-1]
        n = max(int(total / spacing), 16)
        t = np.linspace(0.0, total, n, endpoint=False)
        rs = np.c_[np.interp(t, d, loop[:, 0]), np.interp(t, d, loop[:, 1])]

        # ── 가벼운 평활화 (폐곡선은 감아서) ──────────────────────────
        if win >= 3:
            k = np.ones(win) / win
            pad = np.vstack([rs[-win:], rs, rs[:win]])
            rs = np.c_[np.convolve(pad[:, 0], k, 'same'),
                       np.convolve(pad[:, 1], k, 'same')][win:-win]

        kap = curvature(rs)
        ak = np.abs(kap)

        # ── ★ 주행 가능성 검사 ──────────────────────────────────────
        R_car = L / math.tan(dmax)
        k_lim = 1.0 / R_car
        bad = ak > k_lim
        frac = bad.mean()

        print()
        print('=' * 66)
        print(f'  트랙 가져오기 — {meta.get("label", os.path.basename(src))}')
        print('=' * 66)
        print(f'  원본 점 {len(xy)} → 재샘플 {len(rs)} (간격 {spacing*100:.0f}cm)')
        print(f'  길이 {total:.2f} m   x[{rs[:,0].min():+.2f},{rs[:,0].max():+.2f}] '
              f'y[{rs[:,1].min():+.2f},{rs[:,1].max():+.2f}]')
        print()
        print(f'  차량 최소 선회반경 : {R_car:.3f} m  (L={L}, δmax={dmax})')
        print(f'  트랙 최소 반경     : {1.0/ak.max():.3f} m')
        print()
        if frac > 0:
            need = math.atan(L * ak.max())
            print(f'  ✗ 주행 불가 구간 {bad.sum()}/{len(rs)} ({frac*100:.1f}%)')
            print(f'    가장 급한 코너는 조향 {math.degrees(need):.1f}° 가 필요한데')
            print(f'    차량 한계는 {math.degrees(dmax):.1f}° 입니다.')
            print()
            print('    ★ 이 트랙은 조향을 끝까지 줘도 못 돕니다.')
            print('      게인 튜닝으로 해결되지 않습니다 — 트랙을 바꾸거나')
            print('      곡률을 완화해야 합니다. (track_5 는 주행 가능합니다)')
        else:
            print(f'  ✓ 전 구간 주행 가능 (여유 {(1.0/ak.max())/R_car:.2f}배)')

        # ── 속도 프로파일 ───────────────────────────────────────────
        with np.errstate(divide='ignore'):
            v_grip = np.sqrt(grip * mu * 9.81 / np.maximum(ak, 1e-6))
        v = np.minimum(vmax, v_grip)
        v = np.maximum(v, 0.3)

        print()
        if write_speed:
            print(f'  속도 프로파일을 CSV 에 굽습니다 (μ={mu}, grip_util={grip}) : '
                  f'{v.min():.2f} ~ {v.max():.2f} m/s')
            print('  ⚠ 이러면 런타임 target_speed 가 무시됩니다.')
        else:
            print(f'  속도 열 없음 — 런타임 target_speed 로 정합니다.')
            print(f'  (이 트랙의 곡률 한계는 {v.min():.2f} ~ {v.max():.2f} m/s)')

        # ── 저장 ────────────────────────────────────────────────────
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            if write_speed:
                w.writerow(['# x', 'y', 'speed'])
                for (xx, yy), vv in zip(rs, v):
                    w.writerow([f'{xx:.4f}', f'{yy:.4f}', f'{vv:.3f}'])
            else:
                w.writerow(['# x', 'y'])
                for xx, yy in rs:
                    w.writerow([f'{xx:.4f}', f'{yy:.4f}'])
        print()
        print(f'  저장: {out}')
        print('=' * 66)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = ImportTrack()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
