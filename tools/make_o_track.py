#!/usr/bin/env python3
"""
make_o_track.py — 실험실 경계 안에 최대한 큰 O자(모서리 둥근 사각형) 트랙을 해석적으로 만든다.

  python3 tools/make_o_track.py [이름] [--fit] [--dry-run]

■ 모양 — 직선 4개 + 코너 4개 (2회 회전대칭)

      ╭──────────╮     ① 긴 직선   2a
      │          │     ② 코너      90°, 반경 R (클로소이드 진입·탈출 Lt)
      │          │     ③ 짧은 직선 2b
      │          │     ④ 코너      90°
      ╰──────────╯     ⑤~⑧ ①~④ 를 180° 돌린 것

  반주기 회전각이 정확히 π 라서 180° 돌려 붙이면 반드시 닫힌다 (make_s_track 과 같은 원리).
  코너 한 개의 회전 = 클로소이드 2개 (Lt/R) + 원호 θ = π/2  →  θ = π/2 − Lt/R.

■ 치수 — --fit 으로 최적화

  19각형 안에서 **둘레(중심선 길이)를 최대화**. 제약은
      벽~경계 여유 ≥ clr_min (기본 0.30 m, s_track 과 같다)
      R           ≥ r_min   (기본 0.90 m, 차 한계 0.781 m 보다 여유)
  최적화 뒤 HyperMPC 저자 전처리 결과도 make_s_track 과 같은 방식으로 검산한다.

■ 출력 — draw_track.write_track 과 같은 형식 + pure pursuit 용 파일 하나

  lab_map/data/tracks/<이름>/       centerline / left / right / prep_<이름>.csv / track.json
  src/mlcs_mpc/waypoints/<이름>.csv  path_manager 형식 (# x,y)
  lab_map/data/tracks/<이름>/<이름>_pp.csv   pure pursuit 용 x,y,speed (센터라인)
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib                                           # noqa: E402
matplotlib.use('Agg')
from matplotlib.path import Path as MplPath                 # noqa: E402

import draw_track as dt                                     # noqa: E402
from make_s_track import resample, place, signed_clear      # noqa: E402

DS_FINE = 0.001

# 2026-11-26 --fit --clr-min 0.40 결과를 반올림 (19각형, 반폭 0.25). 여유 0.406 m, 길이 12.32 m.
#   여유 0.30 으로 맞추면 13.04 m (3.41 × 4.87) — 벽 여유를 두려고 조금 줄였다.
#   비대칭(볼록 다각형 둥글리기)으로 해도 13.35 m 라 이득이 2 % 뿐이어서 대칭형을 쓴다.
DEFAULTS = dict(a=0.06, b=1.21, R=1.0, Lt=0.24, ang=0.32591, cx=-0.160, cy=0.154)


def half_profile(a, b, R, Lt):
    """반주기 곡률 프로파일 [(길이, κ시작, κ끝), ...]. 회전각이 정확히 π."""
    k = 1.0 / R
    th = math.pi / 2 - Lt / R               # 코너 원호 각
    if th <= 0.0 or a < 0.0 or b < 0.0:
        return None
    corner = [(Lt, 0.0, k), (th * R, k, k), (Lt, k, 0.0)]
    return [(a, 0.0, 0.0)] + corner + [(2 * b, 0.0, 0.0)] + corner + [(a, 0.0, 0.0)]


def build_fine(a, b, R, Lt, ds=DS_FINE):
    segs = half_profile(a, b, R, Lt)
    if segs is None:
        return None, None
    kap = np.concatenate([np.linspace(k0, k1, max(int(round(L / ds)), 1))
                          for L, k0, k1 in segs if L > 0])
    th = np.r_[0.0, np.cumsum(kap)[:-1] * ds]
    H = np.c_[np.cumsum(np.cos(th)) * ds, np.cumsum(np.sin(th)) * ds]
    ctr = (H[0] + H[-1]) / 2.0
    full = np.vstack([H[:-1], (2 * ctr - H)[:-1]])      # 중복 끝점 제거 (make_s_track 참고)
    return full, np.r_[kap[:-1], kap[:-1]]


def build(a, b, R, Lt, spacing):
    cf, kf = build_fine(a, b, R, Lt)
    if cf is None:
        return None, None, None
    return resample(cf, kf, spacing)


def fit(V, poly, hw, clr_min, r_min, ds=0.02):
    from scipy.optimize import differential_evolution, minimize

    def cost(p):
        a, b, R, Lt, ang, cx, cy = p
        c, k, tot = build(a, b, R, Lt, ds)
        if c is None:
            return 1e3
        c = place(c, ang, cx, cy)
        l, r = dt.offset_walls(c, hw)
        clr = min(signed_clear(l, V, poly), signed_clear(r, V, poly))
        pen = 0.0
        if clr < clr_min:
            pen += 200.0 * (clr_min - clr)
        if R < r_min:
            pen += 200.0 * (r_min - R)
        return -tot + pen

    B = [(0.0, 2.5),        # a   긴 직선 반길이
         (0.0, 2.5),        # b   짧은 직선 반길이
         (0.90, 2.0),       # R   코너 반경
         (0.15, 0.60),      # Lt  클로소이드 길이
         (0.0, math.pi / 2),  # ang 회전 (2회 대칭 + a/b 교환 → 90° 면 충분)
         (-1.0, 1.0), (-1.0, 1.0)]
    print(f'■ 치수 최적화 — 둘레 최대화, 여유 ≥ {clr_min} m, R ≥ {r_min} m')
    res = differential_evolution(cost, B, seed=3, maxiter=200, popsize=24,
                                 tol=1e-10, polish=False, workers=1,
                                 updating='deferred', init='sobol')
    res = minimize(cost, res.x, method='Nelder-Mead',
                   options=dict(maxiter=4000, xatol=1e-6, fatol=1e-9))
    a, b, R, Lt, ang, cx, cy = res.x
    print(f'   a={a:.4f} b={b:.4f} R={R:.4f} Lt={Lt:.4f} '
          f'ang={math.degrees(ang):.3f}deg 중심=({cx:+.4f},{cy:+.4f})\n')
    return dict(a=a, b=b, R=R, Lt=Lt, ang=ang, cx=cx, cy=cy)


def write_pp(path, c, kap, spacing, v_max, a_lat, v_min):
    """pure pursuit 용 x,y,speed. speed = min(v_max, √(a_lat/|κ|)), 앞뒤로 가감속 0.8 m/s² 제한."""
    ak = np.abs(kap)
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(ak, 1e-6)))
    acc = 0.8
    n = len(v)
    for _ in range(2):                       # 닫힌 루프라 두 바퀴 돌려 이음매도 맞춘다
        for i in range(n):                   # 가속 제한 (앞으로)
            j = (i - 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * acc * spacing))
        for i in range(n - 1, -1, -1):       # 감속 제한 (뒤로)
            j = (i + 1) % n
            v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * acc * spacing))
    v = np.maximum(v, v_min)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('x,y,speed\n')
        for (x, y), s in zip(c, v):
            f.write(f'{x:.4f},{y:.4f},{s:.3f}\n')
    return v


def main():
    p = argparse.ArgumentParser(description='O자 트랙을 해석적으로 생성',
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=__doc__)
    p.add_argument('name', nargs='?', default='o_track')
    p.add_argument('--boundary', default=dt.BOUNDARY)
    p.add_argument('--half-width', type=float, default=0.25)
    p.add_argument('--spacing', type=float, default=0.05)
    p.add_argument('--raw-spacing', type=float, default=0.01)
    p.add_argument('--wheelbase', type=float, default=0.33)
    p.add_argument('--max-steer', type=float, default=0.40)
    p.add_argument('--max-bad-run', type=float, default=0.15)
    p.add_argument('--hypermpc', default='~/hypermpc_code')
    for k, v in DEFAULTS.items():
        p.add_argument(f'--{k}', type=float, default=v)
    p.add_argument('--fit', action='store_true', help='치수를 다시 최적화')
    p.add_argument('--clr-min', type=float, default=0.30, help='벽~경계 여유 하한 [m]')
    p.add_argument('--r-min', type=float, default=0.90, help='코너 반경 하한 [m]')
    p.add_argument('--pp-vmax', type=float, default=1.5, help='pure pursuit 속도 상한 [m/s]')
    p.add_argument('--pp-alat', type=float, default=1.5, help='pure pursuit 횡가속 한계 [m/s²]')
    p.add_argument('--pp-vmin', type=float, default=0.5, help='pure pursuit 속도 하한 [m/s]')
    p.add_argument('--dry-run', action='store_true', help='검사만, 저장 안 함')
    a = p.parse_args()

    V, _ = dt.load_boundary(a.boundary)
    poly = MplPath(np.vstack([V, V[:1]]))
    kappa_lim = math.tan(a.max_steer) / a.wheelbase
    hw = a.half_width
    print(f'■ 실험실 경계 {len(V)}각형   차 한계 R_min {1/kappa_lim:.3f} m '
          f'(max_steer {a.max_steer})\n')

    g = fit(V, poly, hw, a.clr_min, a.r_min) if a.fit else {k: getattr(a, k) for k in DEFAULTS}

    c, kap, total = build(g['a'], g['b'], g['R'], g['Lt'], a.spacing)
    if c is None:
        sys.exit('✗ 치수가 잘못됐습니다 (Lt/R ≥ π/2 이거나 직선 길이가 음수)')
    c = place(c, g['ang'], g['cx'], g['cy'])
    left, right = dt.offset_walls(c, hw)
    ak = np.abs(kap)

    wind = dt.winding(kap, a.spacing)
    gap = float(np.linalg.norm(c[0] - c[-1]))
    inside = (poly.contains_points(c).all() and poly.contains_points(left).all()
              and poly.contains_points(right).all())
    clr = min(signed_clear(left, V, poly), signed_clear(right, V, poly))
    bad_m = ak > kappa_lim
    bad_run = dt.longest_run(bad_m, a.spacing)
    fold_m = hw * ak >= 1.0

    def mk(b):
        return '✓' if b else '✗'
    print('■ 설계')
    print(f'   긴 직선      {2*g["a"]:.3f} m × 2')
    print(f'   짧은 직선    {2*g["b"]:.3f} m × 2')
    print(f'   코너         R = {g["R"]:.3f} m × 4,  클로소이드 {g["Lt"]:.3f} m × 8')
    print(f'   배치         회전 {math.degrees(g["ang"]):.2f}°  중심 ({g["cx"]:+.3f}, {g["cy"]:+.3f})')
    print('\n■ 검산')
    print(f'   {mk(1e-6 < gap <= a.spacing*1.05)} 닫힘 간격   {gap*1000:.1f} mm')
    print(f'   {mk(abs(abs(wind)-1) < 0.02)} ∮κ ds / 2π  {wind:+.5f}')
    print(f'   {mk(bad_run == 0)} 주행 가능   R_min {1/ak.max():.4f} m, 한계초과 {bad_m.sum()}점')
    print(f'   {mk(not fold_m.any())} 벽 접힘     {fold_m.sum()}점')
    print(f'   {mk(inside)} 경계 포함   중심선·좌벽·우벽 전부')
    print(f'   {mk(clr >= a.clr_min - 1e-3)} 벽~경계     {clr:.4f} m')
    print(f'     길이 {total:.3f} m,  {len(c)}점,  크기 {np.ptp(c[:,0]):.2f} × {np.ptp(c[:,1]):.2f} m')

    craw, _, _ = build(g['a'], g['b'], g['R'], g['Lt'], a.raw_spacing)
    craw = place(craw, g['ang'], g['cx'], g['cy'])
    TR = dt.load_track_reader(a.hypermpc)
    prep = None
    if TR is None:
        print('\n   – HyperMPC 전처리 확인 건너뜀 (원저장소 없음)')
    else:
        prep = dt.run_prep(TR, craw, hw, a.raw_spacing)
        if 'err' in prep:
            print(f'\n   ✗ 저자 전처리 실패: {prep["err"]}')
        else:
            pk = np.abs(prep['kappa'])
            prep['bad'] = float((pk > kappa_lim).mean())
            prep['warn'] = float((pk > 1.0).mean())
            prep['bad_run'] = dt.longest_run(pk > kappa_lim, prep['spacing'])
            print(f'\n■ HyperMPC 전처리 후 ({prep["n"]}점 {prep["spacing"]*100:.0f}cm)')
            print(f'   {mk(prep["bad_run"] <= a.max_bad_run)} R_min {prep["r_min"]:.4f} m   '
                  f'최장불가 {prep["bad_run"]*100:.0f} cm   RMSE {prep["rmse"]:.4f} m')

    ok = inside and bad_run <= a.max_bad_run and abs(abs(wind) - 1) < 0.02
    if prep and 'err' not in prep:
        ok = ok and prep['bad_run'] <= a.max_bad_run
    print(f'\n{"✓ 저장 가능" if ok else "✗ 조건 미달"}')
    if a.dry_run:
        print('  (--dry-run — 저장하지 않습니다)')
        return
    if not ok:
        sys.exit(1)

    stats = dict(
        total=total, n=len(c), k_max=float(ak.max()), r_min=float(1 / ak.max()),
        bad=float(bad_m.mean()), bad_n=int(bad_m.sum()), bad_run=bad_run,
        warn=float((ak > 1.0).mean()), inside=inside, clear=clr,
        fold=bool(fold_m.any()), fold_n=int(fold_m.sum()),
        fold_run=dt.longest_run(fold_m, a.spacing),
        wind=wind, area=dt.shoelace(c), area_click=dt.shoelace(c), degenerate=False)
    dt.write_track(
        a.name, c, kap, left, right, craw, hw, stats, prep,
        dict(source='tools/make_o_track.py (O자: 직선 4 + 클로소이드 코너 4, 2회 회전대칭)',
             spline='해석적 — 곡률 프로파일 적분 (스플라인 아님)',
             smooth_dev=0.0, clicked=[], boundary=a.boundary, wheelbase=a.wheelbase,
             max_steer=a.max_steer, kappa_lim=kappa_lim, max_bad_run=a.max_bad_run),
        spacing=a.spacing, raw_spacing=a.raw_spacing)

    pp = os.path.join(dt.OUTDIR, a.name, f'{a.name}_pp.csv')
    v = write_pp(pp, c, kap, a.spacing, a.pp_vmax, a.pp_alat, a.pp_vmin)
    print(f'    pure pursuit {os.path.relpath(pp, dt.REPO)}   '
          f'(x,y,speed — 직선 {v.max():.2f} / 코너 {v.min():.2f} m/s)')


if __name__ == '__main__':
    main()
