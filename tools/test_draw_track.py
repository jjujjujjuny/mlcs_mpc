#!/usr/bin/env python3
"""
test_draw_track.py — draw_track.py 회귀 시험 (차·화면 불필요)

  python3 tools/test_draw_track.py

■ 왜 있나

  2026-10-01 에 "클릭해도 점이 안 생긴다" 를 겪었다. 원인은 matplotlib 의
  CallbackRegistry 가 bound method 를 WeakMethod 로 들고 있어서, main() 이
  TrackDrawer 의 반환값을 버리면 인스턴스가 GC 되고 콜백만 조용히 죽는
  것이었다. 창과 경계는 __init__ 의 첫 redraw() 로 이미 그려져 있으니
  **화면만 보고는 멀쩡해 보인다.** 그래서 눈으로 하는 확인이 통하지 않는다.

  아래 ①이 그 회귀를 잡는다. 나머지는 손으로 확인하기 번거로운 기하 검산이다.
"""
import gc
import math
import sys
import types
import warnings
import weakref

import numpy as np

warnings.filterwarnings('ignore')
sys.path.insert(0, __file__.rsplit('/', 1)[0])

import matplotlib                                           # noqa: E402
matplotlib.use('Agg')                                       # 창을 띄우지 않는다
import matplotlib.pyplot as plt                             # noqa: E402
from matplotlib.backend_bases import (KeyEvent, MouseButton,  # noqa: E402
                                      MouseEvent)
from matplotlib.path import Path as MplPath                 # noqa: E402

import draw_track as dt                                     # noqa: E402

FAIL = []


def check(cond, name, detail=''):
    print(f'  {"✓" if cond else "✗"} {name}{"   " + detail if detail else ""}')
    if not cond:
        FAIL.append(name)


def args(**kw):
    a = dict(name='_test', boundary=dt.BOUNDARY, half_width=0.25, spacing=0.05,
             raw_spacing=0.01, smooth=0.03, wheelbase=0.33, max_steer=0.40,
             kappa_target=1.0, hypermpc='~/hypermpc_code',
             allow_infeasible=False)
    a.update(kw)
    return types.SimpleNamespace(**a)


def oval(rx=1.45, ry=1.75, n=8):
    th = np.linspace(0, 2 * np.pi, n + 1)[:-1]
    return np.c_[-0.20 + rx * np.cos(th), -0.20 + ry * np.sin(th)]


# ─────────────────────────────────────────────────────────────────────
print('① 콜백 생존 — 반환값을 버려도 클릭이 먹어야 한다')
plt.close('all')
ref = weakref.ref(dt.TrackDrawer(args()))      # main() 과 똑같이 버린다
gc.collect()
check(ref() is not None, 'GC 후 인스턴스 생존',
      '(죽으면 창은 떠 있는데 클릭만 안 먹는다)')

d = ref()
if d is not None:
    def click(x, y, btn=MouseButton.LEFT):
        px, py = d.ax.transData.transform((x, y))
        d.fig.canvas.callbacks.process(
            'button_press_event',
            MouseEvent('button_press_event', d.fig.canvas, px, py, button=btn))

    def key(k):
        d.fig.canvas.callbacks.process(
            'key_press_event', KeyEvent('key_press_event', d.fig.canvas, k))

    for q in oval():
        click(*q)
    check(len(d.pts) == 8, '실제 MouseEvent 8회 → 점 8개', f'점 {len(d.pts)}개')
    click(5.0, 5.0)
    check(len(d.pts) == 8, '경계 밖 클릭 거부', f'점 {len(d.pts)}개')
    click(1.2, -0.2, MouseButton.RIGHT)
    check(len(d.pts) == 7, '우클릭 삭제', f'점 {len(d.pts)}개')
    before = (d.hw, d.dev, d.show_ref, d.show_prep)
    for k in ('[', ']', '=', '-', 'r', 't', 'h'):
        key(k)
    check((d.show_ref, d.show_prep) == (not before[2], not before[3]),
          'KeyEvent 경유 토글')
    key('u')
    check(len(d.pts) == 6, 'u 취소', f'점 {len(d.pts)}개')
    key('c')
    check(len(d.pts) == 0, 'c 비우기', f'점 {len(d.pts)}개')

# ─────────────────────────────────────────────────────────────────────
print('\n② 스플라인 기하')
r = dt.fit_closed_spline(oval(), 0.03, 0.05)
check(r is not None, 'fit 성공')
c, kap, total, tck = r
check(abs((kap * 0.05).sum() - 2 * math.pi) < 0.02,
      '∮κ ds = 2π  (폐곡선·곡률 부호)', f'{(kap*0.05).sum():.4f}')
check(np.linalg.norm(c[0] - c[-1]) <= 0.05 + 1e-9,
      '끝점이 간격 안에서 닫힘', f'{np.linalg.norm(c[0]-c[-1]):.4f} m')
for hw in (0.25, 0.30):
    l, rg = dt.offset_walls(c, hw)
    w = np.r_[np.linalg.norm(c - l, axis=1), np.linalg.norm(c - rg, axis=1)]
    check(np.allclose(w, hw, atol=1e-9), f'벽 거리가 정확히 {hw}',
          f'{w.min():.6f}~{w.max():.6f}')

print('\n③ 거부되어야 하는 입력')
check(dt.fit_closed_spline(np.array([[0, 0], [1, 0], [.5, 1]]), 0, 0.05) is None,
      '점 3개')
check(dt.fit_closed_spline(
    np.array([[0, 0], [0, 0], [1, 0], [1, 1], [0, 1]]), 0, 0.05) is None,
    '중복점')
th = np.linspace(0, 2 * np.pi, 9)[:-1]
c2, k2, _, _ = dt.fit_closed_spline(np.c_[.25 * np.cos(th), .25 * np.sin(th)],
                                   0, 0.05)
check(0.30 * np.abs(k2).max() >= 1.0, '폭이 코너보다 크면 벽 접힘 감지',
      f'0.30·κ = {0.30*np.abs(k2).max():.3f}')

print('\n④ 경계')
V, meta = dt.load_boundary(dt.BOUNDARY)
check(len(V) == 19, '19각형', f'{len(V)}정점')
check(meta['coordinate_frame']['unit'] == 'meter', '단위가 meter')
x, y = V[:, 0], V[:, 1]
A = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
check(abs(A - meta['geometry']['area_m2']) < 1e-6, '면적이 메타와 일치',
      f'{A:.4f} m²')
poly = MplPath(np.vstack([V, V[:1]]))
t5 = np.loadtxt(f'{dt.WAYPOINTS}/track_5.csv', delimiter=',', comments='#',
                usecols=(0, 1))
check(poly.contains_points(t5).all(), 'track_5 가 경계 안',
      f'최소 여유 {dt.dist_to_polygon(t5, V).min():.3f} m')

print('\n⑤ HyperMPC 전처리 미리보기')
TR = dt.load_track_reader('~/hypermpc_code')
if TR is None:
    print('  – 원저장소 없음 — 건너뜀 (도구는 미리보기만 빠지고 동작해야 한다)')
else:
    seg, uu, T = dt.arclen_table(tck)
    craw, _ = dt.resample(tck, seg, uu, T, 0.01)
    check(abs(len(craw) * 0.01 - T) < 0.02, '1 cm 재샘플 점 개수',
          f'{len(craw)}점 길이 {T:.3f} m')
    pr = dt.run_prep(TR, craw, 0.25, 0.01)
    check('err' not in pr, '저자 TrackReader 동작',
          pr.get('err', f'R_min {pr.get("r_min", 0):.3f} m'))
    if 'err' not in pr:
        check(abs(pr['spacing'] - 0.2) < 0.01, 'points_per_meter=5 → 20 cm',
              f'{pr["spacing"]*100:.1f} cm')
        check(pr['r_min'] != 1 / np.abs(kap).max(),
              '저자 곡률이 우리와 다르다 (그래서 둘 다 봐야 한다)',
              f'우리 {1/np.abs(kap).max():.3f} vs 저자 {pr["r_min"]:.3f} m')

print()
if FAIL:
    print(f'✗ {len(FAIL)}건 실패: {", ".join(FAIL)}')
    sys.exit(1)
print('✓ 전부 통과')
