#!/usr/bin/env python3
"""
test_obstacle_detect.py — 라이다 장애물 검출 (obstacle_detect.py) 합성 시험. ROS·차 불필요.

  실제 맵 (기본 1109_3) 에 상자를 놓고 레이캐스팅으로 가짜 스캔 (UST-10LX: 270°, 0.25°) 을 만든다.
  ① 상자 위치·거리별로 찾는지, 중심 오차   ② 상자 없을 때 헛것   ③ 위치추정 오차 (위치·yaw) 에서 헛것

  python3 test/test_obstacle_detect.py [--map 1109_3]
"""
import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.hmpc_core import Track  # noqa: E402
from mlcs_mpc.obstacle_detect import DetectParams, TrackBand, WallField, detect, scan_to_map  # noqa: E402

LASER = (0.27, 0.0, 0.0)
ANG_MIN, ANG_INC, N_BEAM = -math.radians(135), math.radians(0.25), 1081


def raycast(walls, pose, boxes, noise=0.01, rng=None):
    """pose = base_link 의 참 자세. boxes = [(cx, cy, 반폭, heading)]."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = x + c * LASER[0], y + s * LASER[0]
    a = ANG_MIN + ANG_INC * np.arange(N_BEAM)
    ang = yaw + a
    t = np.arange(0.05, 10.0, 0.01)
    X = ox + np.cos(ang)[:, None] * t[None]
    Y = oy + np.sin(ang)[:, None] * t[None]
    hit = walls.dist(X.ravel(), Y.ravel()).reshape(X.shape) <= 0.0
    for bx, by, hw, bh in boxes:
        u = np.cos(bh) * (X - bx) + np.sin(bh) * (Y - by)
        v = -np.sin(bh) * (X - bx) + np.cos(bh) * (Y - by)
        hit |= (np.abs(u) < hw) & (np.abs(v) < hw)
    first = np.where(hit.any(1), hit.argmax(1), len(t) - 1)
    r = t[first]
    if rng is not None:
        r = r + rng.normal(0, noise, r.shape)
    r[~hit.any(1)] = np.inf
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='1109_3')
    a = ap.parse_args()
    walls = WallField(os.path.join(ROOT, 'maps', a.map + '.yaml'))
    tr = Track(os.path.join(ROOT, f'lab_map/data/tracks/{a.map}_track/prep_{a.map}_track.csv'))
    band = TrackBand(tr)
    p = DetectParams()
    rng = np.random.default_rng(0)
    ok_all = True

    def pose_at(s, n=0.0):
        i = int(np.argmin(np.abs(tr.s - np.mod(s, tr.length))))
        h = tr.heading[i]
        return tr.x[i] - math.sin(h) * n, tr.y[i] + math.cos(h) * n, h

    # ① 상자 (한 변 0.30 m) — 트랙 여러 곳 × 앞 거리 × 옆 위치
    print('■ ① 상자 찾기 (한 변 0.30, 트랙 8곳 × 거리 1.0/2.0/3.0 m × 옆 −0.2/0/+0.2)')
    errs, miss, extra = [], 0, 0
    for s0 in np.linspace(0, tr.length, 8, endpoint=False):
        car = pose_at(s0)
        for ahead in (1.0, 2.0, 3.0):
            for n in (-0.2, 0.0, 0.2):
                bx, by, bh = pose_at(s0 + ahead, n)
                if walls.dist(bx, by) < 0.2:
                    continue
                r = raycast(walls, car, [(bx, by, 0.15, bh)], rng=rng)
                pts, org = scan_to_map(r, ANG_MIN, ANG_INC, car, LASER, p)
                det = detect(pts, org, walls, band, p)
                if not det:
                    miss += 1
                    continue
                e = math.hypot(det[0][0] - bx, det[0][1] - by)
                errs.append(e)
                extra += len(det) - 1
    errs = np.array(errs)
    found = len(errs) / (len(errs) + miss)
    print(f'   찾음 {len(errs)}/{len(errs) + miss} ({found * 100:.0f}%), 중심 오차 평균 {errs.mean() * 100:.1f} '
          f'p95 {np.percentile(errs, 95) * 100:.1f} cm, 같이 잡힌 헛것 {extra}')
    ok = found > 0.9 and np.percentile(errs, 95) < 0.12
    print('   ' + ('✓' if ok else '✗'))
    ok_all &= ok

    # ② ③ 상자 없음 — 위치추정 오차별 헛것 (스캔 프레임 중 하나라도 잡힌 비율)
    print('■ ②③ 상자 없음 — 위치추정 오차별 헛것 (트랙 40곳)')
    rows = [(0.0, 0.0), (0.05, 0.0), (0.10, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0), (0.10, 2.0), (0.0, 5.0)]
    fp_ok = {}
    for dxy, dyaw in rows:
        bad = 0
        for s0 in np.linspace(0, tr.length, 40, endpoint=False):
            car = pose_at(s0)
            r = raycast(walls, car, [], rng=rng)
            k = rng.uniform(0, 2 * math.pi)
            est = (car[0] + dxy * math.cos(k), car[1] + dxy * math.sin(k),
                   car[2] + math.radians(dyaw) * rng.choice((-1, 1)))
            pts, org = scan_to_map(r, ANG_MIN, ANG_INC, est, LASER, p)
            bad += bool(detect(pts, org, walls, band, p))
        fp_ok[(dxy, dyaw)] = bad / 40
        print(f'   위치 {dxy * 100:4.0f} cm  yaw {dyaw:3.0f}°  → 헛것 있는 프레임 {bad}/40')
    ok = fp_ok[(0.0, 0.0)] == 0 and fp_ok[(0.05, 0.0)] <= 0.05
    print('   ' + ('✓' if ok else '✗') + ' (오차 0 / 5 cm 에서 헛것 없음 — 그 이상은 참고값)')
    ok_all &= ok
    print('모두 통과' if ok_all else '실패')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
