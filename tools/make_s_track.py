#!/usr/bin/env python3
"""
make_s_track.py — 직선 구간 + S자 구간 트랙을 해석적으로 만든다.

  ./drive.sh stracks [이름]
  ./drive.sh stracks s1 --fit        실험실에 맞게 치수를 다시 최적화

■ 왜 손으로 찍지 않고 수식으로 만드나

  `docs/TRACKS.md` 가 track_5 를 두고 권한 방법이다 — 반경을 상수로 두면
  **곡률이 설계로 보장**되므로 "요청한 한계를 못 지키는" 일이 없다.
  손클릭(draw_track.py)은 자유롭지만 코너를 정확히 R=0.90 m 로 찍을 수 없다.

■ 모양 — 직선 2개 + S자 2개

      ╭───────╮        ① 직선     (2a)
      │       │        ② 좌회전   90°+ψ,  반경 R
   ╭──╯       ╰──╮     ③ 우회전   2ψ,     반경 Rs   ← S자
   │   S자        │     ④ 좌회전   90°+ψ,  반경 R
   ╰──╮       ╭──╯     ⑤ 직선     (2a)
      │       │        ⑥~⑩ ①~⑤ 를 180° 돌린 것
      ╰───────╯

■ ★ 닫힘을 어떻게 보장하나

  반주기의 **회전각이 정확히 π** 면, 그것을 중점 기준으로 180° 돌려 붙인
  곡선은 **반드시 닫힌다.** 시작·끝 자세가 180° 회전으로 맞물리기 때문이다.

      회전 = (90°+ψ) − 2ψ + (90°+ψ) = 180°      ← ψ 와 무관하게 성립

  그래서 ψ(S자 깊이)를 아무렇게 바꿔도 트랙이 열리지 않는다. 수치 해를
  구하지 않으므로 "수렴 실패" 가 있을 수 없다 — track_1 이 겪은 문제다.

  실측 닫힘 간격: 0.00 mm (모든 ψ)

■ 치수는 최적화로 정했다 (--fit 으로 다시 돌릴 수 있다)

  19각형 안에서 **직선 길이를 최대화**, 제약은

      벽~경계 여유 ≥ 0.40 m     (track_5 가 같은 폭에서 0.336 m)
      R_min        ≥ 0.90 m     (차 한계 0.781 m 보다 여유)
      S자 깊이 ψ   ≥ 0.45 rad   (26° — S 로 보이려면)

  ⚠ 여유만 최대화하면 최적화가 직선을 0.62 m 로, S 를 15° 로 줄여 버린다
    (실제로 그랬다). 요청한 성격은 목적함수가 아니라 제약으로 박아야 한다.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib                                           # noqa: E402
matplotlib.use('Agg')                                       # 창을 띄우지 않는다
from matplotlib.path import Path as MplPath                 # noqa: E402

import draw_track as dt                                     # noqa: E402

# 2026-10-01 최적화 결과 (19각형 기준)
# 2026-10-01 최적화 결과 (19각형, 반폭 0.25, 저자 전처리를 제약에 포함)
DEFAULTS = dict(a=0.4585, R=0.9793, Rs=1.0493, psi=0.7438, Lt=0.1675,
                ang=math.radians(22.189), cx=-0.3001, cy=0.0808)
#  psi 는 이제 S자 **호 각 u** [rad] 다 (전에는 깊이 ψ, u = 2ψ 였다)


# ─────────────────────────────────────────────────────────────────────
#  기하
# ─────────────────────────────────────────────────────────────────────
def half_profile(a, R, Rs, u, Lt):
    """반주기 곡률 프로파일 [(길이, κ시작, κ끝), ...]. 회전각이 정확히 π 다.

    ★ 클로소이드(곡률 1차 램프)를 넣는다. 구간상수 κ 로 두면 접합부에서
      곡률이 점프하는데,

        ① 차가 그렇게 못 돈다 — max_steer_rate 3.2 rad/s 한계가 있다
        ② 저자 전처리기(k=5, s=2.0)가 그 점프에서 **오버슈트해서 코너를
           더 조인다.** 실측: R_min 0.900 → 0.772 로 한계 아래로 떨어졌다

    회전각을 π 로 맞추려면 끝 선회 각 t 를 풀어 준다. 램프가 기여하는
    회전은 (κ시작+κ끝)/2 · 길이 이므로

        π = Lt·(1/R) + 2t + Lt·(1/R − 1/Rs) − u
        t = [π + u − Lt·(2/R − 1/Rs)] / 2

    u(S자 호 각)·Lt 를 아무렇게 줘도 닫힘이 유지된다.
    """
    kR, kS = 1.0 / R, -1.0 / Rs
    t = (math.pi + u - Lt * (2.0 / R - 1.0 / Rs)) / 2.0
    if t <= 0.0:
        return None
    return [(a, 0.0, 0.0),            # 직선
            (Lt, 0.0, kR),            # 전이
            (t * R, kR, kR),          # 좌회전
            (Lt, kR, kS),             # 전이 (S자 진입)
            (u * Rs, kS, kS),         # 우회전  ← S자
            (Lt, kS, kR),             # 전이 (S자 탈출)
            (t * R, kR, kR),          # 좌회전
            (Lt, kR, 0.0),            # 전이
            (a, 0.0, 0.0)]            # 직선


DS_FINE = 0.001       # 적분 간격. 회전각 오차를 작게 하려면 촘촘해야 한다


def build_fine(a, R, Rs, psi, Lt, ds=DS_FINE):
    """곡률을 적분해 반주기를 만들고 180° 돌려 붙인다.

    ★ 중복점을 반드시 떼야 한다. 회전한 사본은 H[-1] 에서 시작해 H[0] 에서
      끝나므로, 그냥 vstack 하면 두 점이 각각 **두 번** 들어간다. 그러면
      닫힘 간격이 0.000 mm 로 보이지만(같은 점이니까) 실제로는 중복이고,
      저자 TrackReader 의 splprep 이 'Invalid inputs' 로 죽는다.
      (2026-10-01 에 실제로 겪었다)
    """
    segs = half_profile(a, R, Rs, psi, Lt)
    if segs is None:
        return None, None
    kap = np.concatenate([np.linspace(k0, k1, max(int(round(L / ds)), 1))
                          for L, k0, k1 in segs])
    th = np.r_[0.0, np.cumsum(kap)[:-1] * ds]
    H = np.c_[np.cumsum(np.cos(th)) * ds, np.cumsum(np.sin(th)) * ds]
    ctr = (H[0] + H[-1]) / 2.0
    full = np.vstack([H[:-1], (2 * ctr - H)[:-1]])      # ← 중복 끝점 제거
    return full, np.r_[kap[:-1], kap[:-1]]


def resample(cf, kf, spacing):
    """호길이 등간격으로 다시 깔고, 곡률은 그 구간의 값을 그대로 준다.

    κ 가 구간상수이므로 선형보간하면 경계에서 없는 값이 생긴다. 왼쪽
    값(searchsorted)을 쓰면 설계한 κ 가 정확히 보존된다.
    """
    loop = np.vstack([cf, cf[:1]])
    sf = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(loop, axis=0), axis=1))]
    total = sf[-1]
    st = np.arange(0.0, total, spacing)
    c = np.c_[np.interp(st, sf, loop[:, 0]), np.interp(st, sf, loop[:, 1])]
    idx = np.clip(np.searchsorted(sf[:-1], st, side='right') - 1,
                  0, len(kf) - 1)
    return c, kf[idx], total


def build(a, R, Rs, psi, Lt, spacing):
    cf, kf = build_fine(a, R, Rs, psi, Lt)
    if cf is None:
        return None, None, None
    return resample(cf, kf, spacing)


def place(c, ang, cx, cy):
    M = np.array([[math.cos(ang), -math.sin(ang)],
                  [math.sin(ang), math.cos(ang)]])
    c = c @ M.T
    return c - c.mean(axis=0) + np.array([cx, cy])


def dist_vec(P, V):
    """점 N개 → 다각형 변까지 최소거리. 벡터화 (루프판은 200배 느리다)."""
    A = V
    AB = np.roll(V, -1, axis=0) - A
    AB2 = np.einsum('ij,ij->i', AB, AB)
    AP = P[:, None, :] - A[None, :, :]
    t = np.clip(np.einsum('nij,ij->ni', AP, AB) / AB2, 0.0, 1.0)
    C = A[None, :, :] + t[:, :, None] * AB[None, :, :]
    return np.linalg.norm(P[:, None, :] - C, axis=2).min(axis=1)


def signed_clear(P, V, poly):
    d = dist_vec(P, V)
    return float(np.min(np.where(poly.contains_points(P), d, -d)))


# ─────────────────────────────────────────────────────────────────────
#  치수 최적화
# ─────────────────────────────────────────────────────────────────────
def fit(V, poly, hw, kappa_lim, TR, r_prep_min, clr_min, u_min,
        straight_min, ds=0.02, raw=0.01):
    """치수를 전역 최적화한다.

    ★ 저자 전처리를 **최적화 안에** 돌린다. 설계 R_min 만 보고 고르면
      전처리 후 0.72~0.85 배로 줄어 한계 아래로 떨어진다 (실측). 그린
      쪽에서는 통과로 보이는데 MPC 가 받는 트랙은 불가인 상황이 된다.

      여유만 최대화하면 직선과 S 를 없애 버리므로, 직선 길이와 S 깊이는
      목적함수가 아니라 **제약**으로 박는다.
    """
    from scipy.optimize import differential_evolution, minimize
    r_car = 1.0 / kappa_lim

    def ev(p):
        a, R, Rs, u, Lt, ang, cx, cy = p
        c, k, tot = build(a, R, Rs, u, Lt, ds)
        if c is None:
            return None
        c = place(c, ang, cx, cy)
        l, r = dt.offset_walls(c, hw)
        clr = min(signed_clear(l, V, poly), signed_clear(r, V, poly))
        rmin = 1.0 / max(np.abs(k).max(), 1e-9)
        if clr < -0.5 or rmin < r_car or TR is None:
            return dict(clr=clr, rmin=rmin, tot=tot, prep=None)
        craw, _, _ = build(a, R, Rs, u, Lt, raw)
        craw = place(craw, ang, cx, cy)
        pr = dt.run_prep(TR, craw, hw, raw)
        return dict(clr=clr, rmin=rmin, tot=tot,
                    prep=None if 'err' in pr else pr)

    def cost(p):
        a, u = p[0], p[3]
        e = ev(p)
        if e is None:
            return 1e3
        pen = 0.0
        if e['clr'] < 0.0:
            pen += 500.0 * (-e['clr'])
        if e['rmin'] < r_car:
            pen += 600.0 * (r_car - e['rmin'])
        if e['prep'] is None and TR is not None:
            pen += 90.0
        elif e['prep'] is not None:
            pm = e['prep']['r_min']
            if pm < r_prep_min:
                pen += 60.0 * (r_prep_min - pm)
            if pm < r_car:
                pen += 600.0 * (r_car - pm)
        if 2 * a < straight_min:
            pen += 30.0 * (straight_min - 2 * a)
        if u < u_min:
            pen += 30.0 * (u_min - u)
        return -(min(e['clr'], 0.45) + 0.08 * 2 * a) + pen

    B = [(0.45, 1.60),      # a   직선 반길이
         (0.90, 1.90),      # R   끝 선회 반경
         (0.90, 2.40),      # Rs  S자 반경
         (0.70, 1.60),      # u   S자 호 각 [rad]
         (0.15, 0.70),      # Lt  클로소이드 전이 길이
         (0.0, math.pi),    # ang 회전
         (-1.0, 1.0), (-1.5, 1.0)]
    print(f'■ 치수 최적화 — 여유 최대화, 제약: 전처리 R_min ≥ {r_prep_min}, '
          f'직선 ≥ {straight_min}, S자 호 ≥ {u_min} rad')
    res = differential_evolution(cost, B, seed=3, maxiter=70, popsize=16,
                                 tol=1e-9, polish=False, workers=-1,
                                 init='sobol')
    res = minimize(cost, res.x, method='Nelder-Mead',
                   options=dict(maxiter=3000, xatol=1e-5, fatol=1e-8))
    a, R, Rs, u, Lt, ang, cx, cy = res.x
    print(f'   a={a:.4f} R={R:.4f} Rs={Rs:.4f} u={u:.4f} Lt={Lt:.4f} '
          f'ang={math.degrees(ang):.3f}deg 중심=({cx:+.4f},{cy:+.4f})\n')
    return dict(a=a, R=R, Rs=Rs, psi=u, Lt=Lt, ang=ang, cx=cx, cy=cy)


# ─────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description='직선 + S자 트랙을 해석적으로 생성',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('name', nargs='?', default='s_track')
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
    p.add_argument('--r-prep-min', type=float, default=0.85,
                   help='저자 전처리 **후** R_min 하한 (한계 0.781 + 여유)')
    p.add_argument('--clr-min', type=float, default=0.30,
                   help='벽~경계 여유 하한 (track_5 는 같은 폭에서 0.336)')
    p.add_argument('--psi-min', type=float, default=0.70,
                   help='S자 호 각 하한 [rad] — S 로 보이려면')
    p.add_argument('--straight-min', type=float, default=0.90,
                   help='직선 구간 길이 하한 [m]')
    p.add_argument('--dry-run', action='store_true', help='검사만, 저장 안 함')
    a = p.parse_args()

    V, _ = dt.load_boundary(a.boundary)
    poly = MplPath(np.vstack([V, V[:1]]))
    kappa_lim = math.tan(a.max_steer) / a.wheelbase
    hw = a.half_width

    print(f'■ 실험실 경계 {len(V)}각형   차 한계 R_min '
          f'{1/kappa_lim:.3f} m (max_steer {a.max_steer})\n')

    TR = dt.load_track_reader(a.hypermpc)
    g = (fit(V, poly, hw, kappa_lim, TR, a.r_prep_min, a.clr_min, a.psi_min,
             a.straight_min)
         if a.fit else {k: getattr(a, k) for k in DEFAULTS})

    # 주행용 간격으로 본체 생성
    c, kap, total = build(g['a'], g['R'], g['Rs'], g['psi'], g['Lt'],
                          a.spacing)
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

    print('■ 설계')
    print(f'   직선 구간    {2*g["a"]:.3f} m  × 2')
    print(f'   끝 선회      R = {g["R"]:.3f} m × 4')
    print(f'   S자          Rs = {g["Rs"]:.3f} m,  '
          f'호 각 u = {math.degrees(g["psi"]):.1f}° × 2')
    print(f'   클로소이드   전이 {g["Lt"]:.3f} m × 8  (곡률 1차 램프)')
    print(f'   배치         회전 {math.degrees(g["ang"]):.2f}°  '
          f'중심 ({g["cx"]:+.3f}, {g["cy"]:+.3f})')
    print('\n■ 검산')
    def mk(b):
        return '✓' if b else '✗'
    print(f'   {mk(1e-6 < gap <= a.spacing*1.05)} 닫힘 간격   {gap*1000:.1f} mm '
          f'(간격 {a.spacing*1000:.0f} mm 과 같아야 — 0 이면 중복점)')
    print(f'   {mk(abs(abs(wind)-1) < 0.02)} ∮κ ds / 2π  {wind:+.5f}  '
          f'(단순 폐곡선이면 ±1)')
    print(f'   {mk(ak.max() <= 1/g["R"]+1e-6)} R_min       {1/ak.max():.4f} m  '
          f'(설계값 min(R, Rs) = {min(g["R"], g["Rs"]):.3f})')
    print(f'   {mk(bad_run == 0)} 주행 가능   한계초과 {bad_m.sum()}점, '
          f'최장 연속 {bad_run*100:.0f} cm')
    print(f'   {mk(not fold_m.any())} 벽 접힘     {fold_m.sum()}점')
    print(f'   {mk(inside)} 경계 포함   중심선·좌벽·우벽 전부')
    print(f'   {mk(clr >= a.clr_min)} 벽~경계     {clr:.4f} m  '
          f'(track_5 는 같은 폭에서 0.336)')
    print(f'     길이 {total:.3f} m,  {len(c)}점,  '
          f'크기 {np.ptp(c[:,0]):.2f} × {np.ptp(c[:,1]):.2f} m')
    print(f'     곡률 {kap.min():+.4f} ~ {kap.max():+.4f} /m  '
          f'(부호가 바뀌어야 S 자다: {mk(kap.min() < 0 < kap.max())})')

    # HyperMPC 원저자 전처리까지 확인
    craw, _, _ = build(g['a'], g['R'], g['Rs'], g['psi'], g['Lt'],
                       a.raw_spacing)
    craw = place(craw, g['ang'], g['cx'], g['cy'])
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
            print(f'\n■ HyperMPC 전처리 후 (저자 k=5, s=2.0, '
                  f'{prep["n"]}점 {prep["spacing"]*100:.0f}cm)')
            print(f'   {mk(prep["bad_run"] <= a.max_bad_run)} R_min '
                  f'{prep["r_min"]:.4f} m   최장불가 {prep["bad_run"]*100:.0f} cm'
                  f'   RMSE {prep["rmse"]:.4f} m')

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
        total=total, n=len(c), k_max=float(ak.max()),
        r_min=float(1 / ak.max()), bad=float(bad_m.mean()),
        bad_n=int(bad_m.sum()), bad_run=bad_run,
        warn=float((ak > 1.0).mean()), inside=inside, clear=clr,
        fold=bool(fold_m.any()), fold_n=int(fold_m.sum()),
        fold_run=dt.longest_run(fold_m, a.spacing),
        wind=wind, area=dt.shoelace(c), area_click=dt.shoelace(c),
        degenerate=False)
    dt.write_track(
        a.name, c, kap, left, right, craw, hw, stats, prep,
        dict(source='tools/make_s_track.py (직선 + S자, 2회 회전대칭)',
             spline='해석적 — 곡률 프로파일 적분 (스플라인 아님)',
             smooth_dev=0.0, clicked=[],
             boundary=a.boundary, wheelbase=a.wheelbase,
             max_steer=a.max_steer, kappa_lim=kappa_lim,
             max_bad_run=a.max_bad_run),
        spacing=a.spacing, raw_spacing=a.raw_spacing)


if __name__ == '__main__':
    main()
