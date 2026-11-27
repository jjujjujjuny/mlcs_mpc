#!/usr/bin/env python3
"""
track_from_sketch.py — 연구실 지도 그림 위에 손으로 그린 레이싱 라인 → 트랙 (2026-11-27).

  1) 지도 만들기:  python3 tools/track_from_sketch.py --map 연구실_트랙_그리기_지도.png
  2) 그 그림 위에 순수 파란색 (RGB 0,0,255 근처) 펜으로 닫힌 선 한 바퀴를 그린다
  3) 트랙 만들기:  python3 tools/track_from_sketch.py <이름> --sketch 그린그림.png [--ccw] [--start X,Y]

■ 하는 일

  ① 그림에서 파란 픽셀을 뽑아 지도와 같은 축 변환으로 mocap 좌표로 바꾼다
  ② 2 cm 칸으로 솎아 가장 가까운 점끼리 이어 한 바퀴 순서를 만든다
  ③ 닫힌 3 차 스플라인 (제어점 K 개) 을 맞춘다. 비용 =
        그림과의 거리²  +  곡률 > 1/r_min 벌점  +  중심선~경계 < 반폭 + clr_min 벌점  +  매끄러움
     → 손 그림을 최대한 따르되 차가 돌 수 있고 벽이 경계 안에 들어오는 선
  ④ draw_track.write_track 으로 저장 (make_o_track 과 같은 형식: centerline / 벽 / 저자 형식 / prep 검산 / track.json)
     + 그림과 결과를 겹친 비교 그림 sketch_fit.png

  방향은 기본 시계 (o_track 과 같음), --ccw 면 반시계. 출발점은 --start 에 가장 가까운 점 (기본: 가장 긴 직선 가운데).
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib                                           # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                             # noqa: E402
from matplotlib.path import Path as MplPath                 # noqa: E402

import draw_track as dt                                     # noqa: E402

DPI = 150


def font():
    from matplotlib import font_manager as fm
    fs = [f for f in fm.findSystemFonts() if 'Nanum' in f or ('Noto' in f and 'Sans' in f and 'CJK' in f)]
    if fs:
        fm.fontManager.addfont(fs[0])
        plt.rcParams['font.family'] = fm.FontProperties(fname=fs[0]).get_name()
    plt.rcParams['axes.unicode_minus'] = False


def boundary_dist(pts, V):
    """경계 다각형까지 부호 거리 (안 +, 밖 −)."""
    P = np.vstack([V, V[:1]])
    d = np.full(len(pts), np.inf)
    for a, b in zip(P[:-1], P[1:]):
        ab = b - a
        t = np.clip(((pts - a) @ ab) / (ab @ ab), 0, 1)
        d = np.minimum(d, np.hypot(*(pts - (a + t[:, None] * ab)).T))
    inside = MplPath(P).contains_points(pts)
    return np.where(inside, d, -d)


def make_map(V, hw, clr, r_min, o_track=None):
    """그릴 지도. 축 변환을 되살리려고 (fig, ax) 를 돌려준다 — 지도와 읽기가 같은 함수를 쓴다."""
    font()
    P = np.vstack([V, V[:1]])
    fig, ax = plt.subplots(figsize=(11.7, 8.3))
    fig.set_dpi(DPI)          # ★ 만든 뒤 dpi 를 바꾸고 tight_layout — 처음 지도 (11-27) 와 같은 축 배치. subplots(dpi=) 로 주면 배치가 다르다
    gx, gy = np.meshgrid(np.linspace(P[:, 0].min() - .3, P[:, 0].max() + .3, 500),
                         np.linspace(P[:, 1].min() - .3, P[:, 1].max() + .3, 500))
    ok = (boundary_dist(np.c_[gx.ravel(), gy.ravel()], V) >= hw + clr).reshape(gx.shape)
    ax.contourf(gx, gy, (~ok).astype(float), levels=[0.5, 1.5], colors=['#f6d5d5'], zorder=0)
    ax.plot(P[:, 0], P[:, 1], 'k', lw=2.2, zorder=3)
    ax.set_xticks(np.arange(-3, 3.01, 0.5)); ax.set_yticks(np.arange(-3, 3.01, 0.5))
    ax.set_xticks(np.arange(-3, 3.01, 0.25), minor=True); ax.set_yticks(np.arange(-3, 3.01, 0.25), minor=True)
    ax.grid(which='major', c='0.55', lw=0.7); ax.grid(which='minor', c='0.85', lw=0.4)
    ax.set_xlim(P[:, 0].min() - .25, P[:, 0].max() + .25); ax.set_ylim(P[:, 1].min() - .25, P[:, 1].max() + .25)
    ax.set_aspect('equal')
    if o_track is not None:
        ax.plot(o_track[:, 0], o_track[:, 1], ls=(0, (4, 4)), c='tab:blue', lw=1, alpha=0.5, label='지금 O자 트랙 중심선 (참고)')
    th = np.linspace(0, 2 * np.pi, 100); c = P[:-1].mean(0)
    ax.plot(c[0] + r_min * np.cos(th), c[1] + r_min * np.sin(th), c='tab:green', lw=1.2, alpha=0.7,
            label=f'반경 {r_min} m 견본 (이보다 급한 코너 X)')
    ax.set_xlabel('x [m] (mocap 월드)'); ax.set_ylabel('y [m]')
    ax.set_title('연구실 mocap 영역 — 레이싱 라인을 손으로 그려 주세요 (한 바퀴 닫힌 선 + 방향 화살표)', fontsize=12)
    ax.legend(loc='upper right', fontsize=9)
    # ★ ax.text + fig 좌표 (fig.text 가 아님) — tight_layout 이 이 글상자까지 넣어 축을 줄인다. 처음 지도와 같은 배치
    ax.text(0.01, 0.02, f'흰 곳 = 중심선이 갈 수 있는 곳 (길 반폭 {hw} + 벽~경계 여유 {clr})\n'
             f'분홍 = 중심선 금지   가는 칸 0.25 m, 굵은 칸 0.5 m\n코너는 초록 원 (반경 {r_min} m) 보다 완만하게',
             transform=fig.transFigure, fontsize=9, va='bottom', bbox=dict(fc='white', ec='0.6'))
    fig.tight_layout()
    return fig, ax


def read_sketch(path, fig, ax):
    from PIL import Image
    fig.canvas.draw()
    im = np.asarray(Image.open(path).convert('RGB')).astype(int)
    H, W, _ = im.shape
    w0, h0 = fig.canvas.get_width_height()
    if (W, H) != (w0, h0):
        sys.exit(f'✗ 그림 크기 {W}×{H} 가 지도 {w0}×{h0} 와 다릅니다 (지도 파일 위에 그대로 그려 주세요)')
    r, g, b = im[..., 0], im[..., 1], im[..., 2]
    yy, xx = np.nonzero((b > 180) & (r < 90) & (g < 90))
    return ax.transData.inverted().transform(np.c_[xx + 0.5, H - (yy + 0.5)])


def order_loop(pts, cell=0.02):
    """픽셀 점 → 칸으로 솎아 가장 가까운 점끼리 이은 한 바퀴 순서."""
    key = np.round(pts / cell).astype(int)
    _, idx = np.unique(key, axis=0, return_index=True)
    q = pts[np.sort(idx)]
    tree = cKDTree(q)
    used = np.zeros(len(q), bool)
    cur = int(np.argmin(q[:, 1])); order = [cur]; used[cur] = True
    for _ in range(len(q) - 1):
        for k in (8, 32, len(q)):
            d, j = tree.query(q[cur], k=min(k, len(q)))
            j = np.atleast_1d(j)[~used[np.atleast_1d(j)]]
            if len(j):
                break
        cur = int(j[0]); used[cur] = True; order.append(cur)
    L = q[order]
    gaps = np.linalg.norm(np.diff(np.vstack([L, L[:1]]), axis=0), axis=1)
    return L, gaps


def spline_from_ctrl(C, n=1200):
    P = np.vstack([C, C[:1]])
    tck, _ = splprep([P[:, 0], P[:, 1]], s=0, per=1, k=3)
    u = np.linspace(0, 1, n, endpoint=False)
    xy = np.array(splev(u, tck)).T
    d1 = np.array(splev(u, tck, der=1)).T; d2 = np.array(splev(u, tck, der=2)).T
    sp = np.linalg.norm(d1, axis=1)
    k = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(sp ** 3, 1e-12)
    return xy, k, tck


def clr_at(xy, clr, local):
    """점마다 벽~경계 여유. local = [(X, Y, 반경, 여유)] 안의 점은 그 여유 (가장 작은 것)."""
    out = np.full(len(xy), float(clr))
    for x0, y0, rad, c0 in local:
        # 경계에서 0.1 m 에 걸쳐 부드럽게 바꾼다 (계단이면 최적화가 첫 걸음에서 멈춘다)
        w = 1.0 / (1.0 + np.exp((np.hypot(xy[:, 0] - x0, xy[:, 1] - y0) - rad) / 0.03))
        out = out + w * np.minimum(c0 - out, 0.0)
    return out


def fit(loop, V, hw, clr, r_min, K, w_fit=1.0, rounds=4, apex=(), local=()):
    """제어점 K 개 닫힌 스플라인을 그림·곡률·경계 비용으로 맞춘다."""
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(loop, axis=0), axis=1))]
    C0 = np.array([np.interp(np.linspace(0, seg[-1], K, endpoint=False), seg, loop[:, i]) for i in (0, 1)]).T
    tree = cKDTree(loop)
    k_lim = 1.0 / r_min; need = hw + clr

    W = dict(k=20.0, b=200.0)

    def cost(z):
        C = z.reshape(-1, 2)
        try:
            xy, k, _ = spline_from_ctrl(C, 600)
        except Exception:
            return 1e6
        d_out = tree.query(xy)[0]                               # 선 → 그림
        d_in = cKDTree(xy).query(loop[::3])[0]                  # 그림 → 선 (모양을 통째로 줄이지 않게)
        bd = boundary_dist(xy, V)
        need = hw + clr_at(xy, clr, local)
        f = w_fit * (np.mean(d_out ** 2) + np.mean(d_in ** 2))
        f += W['k'] * np.mean(np.maximum(np.abs(k) - k_lim, 0) ** 2)
        f += W['b'] * np.mean(np.maximum(need - bd, 0) ** 2)
        f += 1e-4 * np.mean(np.diff(k) ** 2) * 100
        for ax_, ay_, w in apex:          # 이 점까지 최대한 들어가게 (제약이 허락하는 만큼)
            f += w * np.min((xy[:, 0] - ax_) ** 2 + (xy[:, 1] - ay_) ** 2)
        return f

    z = C0.ravel()
    for _ in range(rounds):              # 벌점을 키워 가며 (그림 → 제약 쪽으로 차츰)
        r = minimize(cost, z, method='L-BFGS-B', options=dict(maxiter=400, maxfun=200000))
        z = r.x; W['k'] *= 10; W['b'] *= 10
    return z.reshape(-1, 2), r


def main():
    p = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('name', nargs='?')
    p.add_argument('--map', help='그릴 지도 png 를 이 경로에 만들고 끝')
    p.add_argument('--sketch', help='파란 선을 그린 지도 png')
    p.add_argument('--boundary', default=dt.BOUNDARY)
    p.add_argument('--half-width', type=float, default=0.25)
    p.add_argument('--clr-min', type=float, default=0.30, help='벽~경계 여유 [m]')
    p.add_argument('--r-min', type=float, default=0.90, help='코너 반경 목표 하한 [m] (차 한계 0.78)')
    p.add_argument('--ctrl', type=int, default=36, help='스플라인 제어점 수 (많을수록 그림을 잘 따른다)')
    p.add_argument('--ccw', action='store_true', help='반시계 방향 (기본 시계)')
    p.add_argument('--start', help='출발점 근처 X,Y (기본: 가장 긴 직선 가운데)')
    p.add_argument('--apex', action='append', default=[], metavar='X,Y[,W]',
                   help='코너가 이 점까지 최대한 들어가게 끌어당김 (여러 번 가능, W 기본 1.0)')
    p.add_argument('--local-clr', action='append', default=[], metavar='X,Y,R,CLR',
                   help='(X,Y) 반경 R 안에서만 벽~경계 여유를 CLR 로 (여러 번 가능)')
    p.add_argument('--spacing', type=float, default=0.05)
    p.add_argument('--raw-spacing', type=float, default=0.01)
    p.add_argument('--wheelbase', type=float, default=0.33)
    p.add_argument('--max-steer', type=float, default=0.40)
    p.add_argument('--max-bad-run', type=float, default=0.15)
    p.add_argument('--hypermpc', default='~/hypermpc_code')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()

    V, _ = dt.load_boundary(a.boundary)
    hw = a.half_width
    ot = os.path.join(dt.OUTDIR, 'o_track', 'centerline.csv')
    o_track = np.genfromtxt(ot, delimiter=',', skip_header=1)[:, 1:] if os.path.exists(ot) else None
    fig, ax = make_map(V, hw, a.clr_min, a.r_min, o_track)
    if a.map:
        fig.savefig(a.map, dpi=DPI); print(f'✓ 지도 {a.map}'); return
    if not (a.name and a.sketch):
        p.error('<이름> 과 --sketch 가 필요합니다 (지도만 만들려면 --map)')

    raw = read_sketch(a.sketch, fig, ax)
    print(f'■ 그림 파란 픽셀 {len(raw)} 개  x {raw[:,0].min():.2f}~{raw[:,0].max():.2f}  y {raw[:,1].min():.2f}~{raw[:,1].max():.2f}')
    loop, gaps = order_loop(raw)
    print(f'■ 한 바퀴 순서: {len(loop)} 점, 최대 빈틈 {gaps.max()*100:.0f} cm (그림 끊김이 크면 이어 그려 주세요)')
    if gaps.max() > 0.3:
        sys.exit('✗ 선이 끊겼거나 겹쳐 순서를 못 정했습니다')

    apex = [tuple(map(float, q.split(','))) + ((1.0,) if q.count(',') == 1 else ()) for q in a.apex]
    local = [tuple(map(float, q.split(','))) for q in a.local_clr]
    C, res = fit(loop, V, hw, a.clr_min, a.r_min, a.ctrl, apex=apex, local=local)
    # 방향·출발점
    xy, k, tck = spline_from_ctrl(C, 4000)
    area2 = np.dot(xy[:, 0], np.roll(xy[:, 1], -1)) - np.dot(xy[:, 1], np.roll(xy[:, 0], -1))
    if (area2 > 0) != a.ccw:            # 양수 = 반시계
        C = C[::-1]
    xy, k, tck = spline_from_ctrl(C, 4000)
    if a.start:
        sx, sy = map(float, a.start.split(','))
        i0 = int(np.argmin(np.hypot(xy[:, 0] - sx, xy[:, 1] - sy)))
    else:                               # 가장 긴 |κ| < 0.2 구간의 가운데
        m = np.abs(k) < 0.2; best, cur, bi = 0, 0, 0
        for i in range(2 * len(m)):
            cur = cur + 1 if m[i % len(m)] else 0
            if cur > best:
                best, bi = cur, i
        i0 = (bi - best // 2) % len(m)
    u = (np.linspace(0, 1, a.ctrl * 4, endpoint=False) + i0 / len(xy)) % 1.0
    C = np.array(splev(u, tck)).T          # 출발점부터 다시 깐 제어점 (같은 곡선)
    P = np.vstack([C, C[:1]])
    tck, _ = splprep([P[:, 0], P[:, 1]], s=0, per=1, k=3)
    seg, uu, total = dt.arclen_table(tck)
    c, kap = dt.resample(tck, seg, uu, total, a.spacing)
    craw, _ = dt.resample(tck, seg, uu, total, a.raw_spacing)
    left, right = dt.offset_walls(c, hw)

    kappa_lim = math.tan(a.max_steer) / a.wheelbase
    ak = np.abs(kap)
    poly = MplPath(np.vstack([V, V[:1]]))
    inside = poly.contains_points(c).all() and poly.contains_points(left).all() and poly.contains_points(right).all()
    clr = float(min(boundary_dist(left, V).min(), boundary_dist(right, V).min()))
    # 지점별 요구 여유와 비교 (local 구역은 작게)
    clr_short = float(min((boundary_dist(w, V) - clr_at(w, a.clr_min, local)).min() for w in (left, right)))
    bad_m = ak > kappa_lim; bad_run = dt.longest_run(bad_m, a.spacing)
    fold_m = hw * ak >= 1.0
    wind = dt.winding(kap, a.spacing)
    dev = cKDTree(loop).query(c)[0]
    mk = lambda b: '✓' if b else '✗'
    print(f'\n■ 맞춤 (제어점 {a.ctrl}, 반복 {res.nit})')
    print(f'   그림에서 벗어남  중앙 {np.median(dev)*100:.1f} cm, 최대 {dev.max()*100:.1f} cm')
    print(f'   {mk(abs(abs(wind)-1) < 0.02)} ∮κ ds / 2π  {wind:+.4f}   방향 {"반시계" if a.ccw else "시계"}')
    print(f'   {mk(bad_run == 0)} R_min {1/ak.max():.3f} m (목표 {a.r_min}, 차 한계 {1/kappa_lim:.3f}), 1/κ<0.9 인 곳 {(ak > 1/a.r_min).mean()*100:.0f} %')
    print(f'   {mk(not fold_m.any())} 벽 접힘 {fold_m.sum()} 점')
    print(f'   {mk(inside)} 경계 포함   {mk(clr_short >= -0.02)} 벽~경계 최소 {clr:.3f} m (요구보다 {clr_short*100:+.0f} cm)')
    print(f'     길이 {total:.2f} m')
    for ax_, ay_, _ in apex:
        j = int(np.argmin(np.hypot(c[:, 0] - ax_, c[:, 1] - ay_)))
        print(f'     꼭짓점 ({ax_:+.2f}, {ay_:+.2f}) 까지 {np.hypot(c[j,0]-ax_, c[j,1]-ay_)*100:.0f} cm  (도달 ({c[j,0]:+.2f}, {c[j,1]:+.2f}))')

    TR = dt.load_track_reader(a.hypermpc)
    prep = None
    if TR is not None:
        prep = dt.run_prep(TR, craw, hw, a.raw_spacing)
        if 'err' in prep:
            print(f'   ✗ 저자 전처리 실패: {prep["err"]}')
        else:
            pk = np.abs(prep['kappa'])
            prep['bad'] = float((pk > kappa_lim).mean()); prep['warn'] = float((pk > 1.0).mean())
            prep['bad_run'] = dt.longest_run(pk > kappa_lim, prep['spacing'])
            print(f'   {mk(prep["bad_run"] <= a.max_bad_run)} 전처리 후 R_min {prep["r_min"]:.3f} m  RMSE {prep["rmse"]:.4f} m')

    # 비교 그림
    out_dir = os.path.join(dt.OUTDIR, a.name)
    fig2, ax2 = make_map(V, hw, a.clr_min, a.r_min, None)
    ax2.set_title(f'{a.name} — 손 그림 (파랑) vs 만든 트랙 (검정 중심선·회색 벽, 빨강 = R < {a.r_min} m)', fontsize=12)
    ax2.plot(loop[:, 0], loop[:, 1], '.', c='blue', ms=1.5, alpha=0.6)
    ax2.plot(left[:, 0], left[:, 1], c='0.35', lw=1.2); ax2.plot(right[:, 0], right[:, 1], c='0.35', lw=1.2)
    ax2.plot(c[:, 0], c[:, 1], c='k', lw=1.4)
    tight = ak > 1 / a.r_min
    ax2.plot(np.where(tight, c[:, 0], np.nan), np.where(tight, c[:, 1], np.nan), c='red', lw=3)
    ax2.annotate('', xy=c[12], xytext=c[0], arrowprops=dict(arrowstyle='-|>', color='tab:red', lw=2.5, mutation_scale=20))
    ax2.text(*c[0], '  출발', color='tab:red', fontsize=10)
    for ax_, ay_, _ in apex:
        ax2.plot(ax_, ay_, 'x', c='tab:orange', ms=12, mew=3)
    for x0, y0, rad, c0 in local:
        th = np.linspace(0, 2 * np.pi, 80)
        ax2.plot(x0 + rad * np.cos(th), y0 + rad * np.sin(th), ':', c='tab:orange', lw=1.2)
        ax2.text(x0, y0 + rad, f'여유 {c0} m', color='tab:orange', ha='center', va='bottom', fontsize=9)
    ok = inside and clr_short >= -0.02 and bad_run <= a.max_bad_run and abs(abs(wind) - 1) < 0.02
    if prep and 'err' not in prep:
        ok = ok and prep['bad_run'] <= a.max_bad_run
    print(f'\n{"✓ 저장 가능" if ok else "✗ 조건 미달"}')
    if a.dry_run:
        f = os.path.splitext(a.sketch)[0] + '_fit.png'
        fig2.savefig(f, dpi=DPI); print(f'  (--dry-run) 비교 그림 {f}')
        return
    if not ok:
        sys.exit(1)
    stats = dict(total=total, n=len(c), k_max=float(ak.max()), r_min=float(1 / ak.max()),
                 bad=float(bad_m.mean()), bad_n=int(bad_m.sum()), bad_run=bad_run,
                 warn=float((ak > 1.0).mean()), inside=inside, clear=clr,
                 fold=bool(fold_m.any()), fold_n=int(fold_m.sum()), fold_run=dt.longest_run(fold_m, a.spacing),
                 wind=wind, area=dt.shoelace(c), area_click=dt.shoelace(loop), degenerate=False)
    dt.write_track(a.name, c, kap, left, right, craw, hw, stats, prep,
                   dict(source=f'tools/track_from_sketch.py ({os.path.basename(a.sketch)} 손 그림)',
                        spline=f'닫힌 3 차 스플라인 제어점 {a.ctrl} — 그림 거리 + 곡률(R≥{a.r_min}) + 경계(≥{hw + a.clr_min}) 비용 최소화'
                               + (f', 부분 여유 {a.local_clr}' if local else '') + (f', 꼭짓점 {a.apex}' if apex else ''),
                        smooth_dev=float(np.median(dev)), clicked=C.tolist(), boundary=a.boundary,
                        wheelbase=a.wheelbase, max_steer=a.max_steer, kappa_lim=kappa_lim, max_bad_run=a.max_bad_run),
                   spacing=a.spacing, raw_spacing=a.raw_spacing)
    fig2.savefig(os.path.join(out_dir, 'sketch_fit.png'), dpi=DPI)
    print(f'    비교 그림 {os.path.relpath(os.path.join(out_dir, "sketch_fit.png"), dt.REPO)}')


if __name__ == '__main__':
    main()
