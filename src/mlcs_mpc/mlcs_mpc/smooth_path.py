#!/usr/bin/env python3
"""
smooth_path — 손으로 몬 경로를 MPC 가 쓸 만한 경로로 다듬는다.

  ros2 run mlcs_mpc smooth_path --ros-args \
      -p input:=track1.csv -p output:=track1_smooth.csv

■ 왜 다듬어야 하는가

  waypoint_logger 로 딴 경로는 사람이 운전한 궤적이라 흔들린다. MPC 는
  참조 궤적의 **곡률** 로 속도를 정하는데, 곡률은 위치의 2차 미분이라
  작은 흔들림도 크게 증폭된다:

      위치 1cm 흔들림 (간격 10cm) → 곡률 최대 2 m⁻¹ 변동
      → v_ref = sqrt(mu·g/kappa) 가 4 m/s ↔ 1 m/s 로 튄다

  그러면 MPC 가 직선에서도 가감속을 반복한다. 다듬기는 선택이 아니다.

■ 방법

  ① 등간격 재샘플링 — 사람 주행은 속도가 달라 점 간격이 제각각이다
  ② 이동평균 평활화 (폐곡선이면 감아서)
  ③ 곡률 리포트 — 다듬은 결과가 실제로 나아졌는지 숫자로 보여준다

  스플라인 대신 이동평균을 쓰는 이유: 스플라인은 노이즈가 있는 점을
  **정확히 통과** 하려 해서 오히려 진동(Runge 현상)이 생긴다. 근사 평활화가
  이 용도에는 맞다.
"""

import csv
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node


def resample(xy, spacing, closed):
    """경로를 등간격으로 다시 뜬다."""
    if closed:
        xy = np.vstack([xy, xy[:1]])
    d = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    total = d[-1]
    n = max(int(total / spacing), 8)
    targets = np.linspace(0.0, total, n, endpoint=not closed)
    out = np.c_[np.interp(targets, d, xy[:, 0]),
                np.interp(targets, d, xy[:, 1])]
    return out


def smooth(xy, window, closed, passes=2):
    """이동평균. 폐곡선이면 양끝을 감아서 이음매가 안 생기게 한다."""
    if window < 3:
        return xy
    k = np.ones(window) / window
    out = xy.copy()
    for _ in range(passes):
        if closed:
            pad = np.vstack([out[-window:], out, out[:window]])
            sm = np.c_[np.convolve(pad[:, 0], k, mode='same'),
                       np.convolve(pad[:, 1], k, mode='same')]
            out = sm[window:-window]
        else:
            sm = np.c_[np.convolve(out[:, 0], k, mode='same'),
                       np.convolve(out[:, 1], k, mode='same')]
            # 개곡선은 양끝이 평활화로 안쪽으로 말린다 — 원본 유지
            sm[:window] = out[:window]
            sm[-window:] = out[-window:]
            out = sm
    return out


def curvature(xy, closed):
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
    g = den > 1e-9
    k[g] = 2.0 * cross[g] / den[g]
    return k


class SmoothPath(Node):

    def __init__(self):
        super().__init__('smooth_path')
        self.declare_parameter('input', '')
        self.declare_parameter('output', '')
        self.declare_parameter('spacing', 0.1)
        self.declare_parameter('window', 9)
        self.declare_parameter('passes', 2)
        self.declare_parameter('closed_loop', True)

        inp = os.path.expanduser(self.get_parameter('input').value)
        out = os.path.expanduser(self.get_parameter('output').value)
        spacing = float(self.get_parameter('spacing').value)
        window = int(self.get_parameter('window').value)
        passes = int(self.get_parameter('passes').value)
        closed = bool(self.get_parameter('closed_loop').value)

        if not inp or not os.path.isfile(inp):
            self.get_logger().error(f'input 파일이 없습니다: "{inp}"')
            sys.exit(1)
        if not out:
            out = inp.replace('.csv', '_smooth.csv')

        pts = []
        with open(inp) as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith('#'):
                    continue
                try:
                    pts.append((float(row[0]), float(row[1])))
                except (ValueError, IndexError):
                    continue
        if len(pts) < 8:
            self.get_logger().error(f'점이 {len(pts)}개뿐입니다')
            sys.exit(1)

        xy = np.array(pts)
        k_before = curvature(xy, closed)

        rs = resample(xy, spacing, closed)
        sm = smooth(rs, window, closed, passes)
        k_after = curvature(sm, closed)

        d = os.path.dirname(out)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['# x', 'y'])
            for x, y in sm:
                w.writerow([f'{x:.4f}', f'{y:.4f}'])

        length = float(np.sum(np.hypot(*np.diff(sm, axis=0).T)))
        print('\n' + '=' * 58)
        print('  경로 평활화 완료')
        print('=' * 58)
        print(f'  입력   : {inp}  ({len(xy)}점)')
        print(f'  출력   : {out}  ({len(sm)}점, 간격 {spacing}m)')
        print(f'  길이   : {length:.2f} m')
        print()
        print('  곡률 |kappa|        평활화 전 → 후')
        print(f'    평균   : {np.abs(k_before).mean():8.3f} → {np.abs(k_after).mean():.3f} m⁻¹')
        print(f'    최대   : {np.abs(k_before).max():8.3f} → {np.abs(k_after).max():.3f} m⁻¹')
        rms_b = float(np.sqrt(np.mean(np.diff(k_before) ** 2)))
        rms_a = float(np.sqrt(np.mean(np.diff(k_after) ** 2)))
        print(f'    변동   : {rms_b:8.3f} → {rms_a:.3f}   ← 이게 작아야 속도가 안정된다')
        if np.abs(k_after).max() > 1e-6:
            print()
            print(f'  최소 선회반경 : {1.0/np.abs(k_after).max():.2f} m')
            print(f'  (차의 최소반경보다 작으면 MPC 가 못 따라갑니다)')
        print('=' * 58 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = SmoothPath()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
