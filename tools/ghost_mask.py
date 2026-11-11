#!/usr/bin/env python3
"""
ghost_mask.py — 빈 트랙 주행 기록에서 '늘 잡히는 헛것 자리' 를 뽑아 maps/<맵>_ghost.csv 로 쓴다.

  빈 트랙 (상대 차·사람 없음) 에서 obstacles: watch 로 2~3 바퀴 → 그 기록으로
    python3 tools/ghost_mask.py --map 1111_2 ~/mlcs_data/obstacles_<시각>.csv [...]
  lidar_obstacles 가 다음 실행부터 그 자리 ±ghost_radius (0.20 m) 의 점을 버린다.

  쓰는 것: 확정 발행 (ox, oy) 와 확정 전 후보 (c1..c3, min_points 이상). 0.1 m 칸에 min-hits 번 넘게 잡힌 칸.
  움직이는 것 (사람이 지나감 등) 은 한 칸에 오래 안 머물러 빠진다.
  ★ 상대 차를 세워 둔 기록은 넣지 말 것 — 그 자리가 안 보이게 된다.
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--min-hits', type=int, default=20, help='한 칸에 이만큼 넘게 잡혀야 헛것 자리')
    ap.add_argument('--cell', type=float, default=0.10)
    ap.add_argument('--append', action='store_true', help='있는 목록에 더한다 (기본: 새로 쓴다)')
    ap.add_argument('--max-wall', type=float, default=0.30,
                    help='벽에서 이보다 먼 칸은 넣지 않는다 m (트랙 한가운데를 가리면 상대를 못 본다)')
    ap.add_argument('--moving-only', action='store_true', default=True,
                    help='우리 차가 움직일 때 (0.2 m/s 넘게) 잡힌 것만 (서 있을 때는 옆에 사람이 많다)')
    ap.add_argument('csv', nargs='+')
    a = ap.parse_args()
    xs = []
    for f in a.csv:
        o = pd.read_csv(f)
        if a.moving_only:
            dt = np.diff(o.stamp.to_numpy(), prepend=np.nan)
            v = np.hypot(np.diff(o.x.to_numpy(), prepend=np.nan), np.diff(o.y.to_numpy(), prepend=np.nan)) / dt
            o = o[(dt > 0.005) & (dt < 0.1) & (v > 0.2)]
        p = o[o.pub == 1]
        xs.append(p[['ox', 'oy']].to_numpy())
        for k in (1, 2, 3):
            c = o[o[f'c{k}n'] >= 3][[f'c{k}x', f'c{k}y']].dropna().to_numpy()
            xs.append(c)
        print(f'{os.path.basename(f)}: 스캔 {len(o)}, 발행 {len(p)}')
    X = np.vstack([x for x in xs if len(x)]) if xs else np.zeros((0, 2))
    out = os.path.join(ROOT, 'maps', f'{a.map}_ghost.csv')
    old = np.loadtxt(out, delimiter=',', ndmin=2) if (a.append and os.path.exists(out)) else np.zeros((0, 2))
    if len(X):
        g = np.round(X / a.cell).astype(int)
        cells, cnt = np.unique(g, axis=0, return_counts=True)
        sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
        from mlcs_mpc.obstacle_detect import WallField
        W = WallField(os.path.join(ROOT, 'maps', f'{a.map}.yaml'))
        xy = cells * a.cell
        keep = (cnt > a.min_hits) & (W.dist(xy[:, 0], xy[:, 1]) <= a.max_wall)
        new, cnt = xy[keep], cnt[keep]
        order = np.argsort(-cnt)
        for (x, y), c in zip(new[order], cnt[order]):
            print(f'  헛것 칸 ({x:+.2f}, {y:+.2f})  {c}번')
    else:
        new = np.zeros((0, 2))
    allg = np.unique(np.round(np.vstack([old, new]), 3), axis=0)
    np.savetxt(out, allg, delimiter=',', fmt='%.3f')
    print(f'→ {out}  ({len(allg)}칸, 반경 0.20 m 안의 점을 버림). lidar_obstacles 를 다시 띄우면 적용')
    return 0


if __name__ == '__main__':
    sys.exit(main())
