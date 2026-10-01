#!/usr/bin/env python3
"""
track_from_lap.py — 수동으로 한 바퀴 돈 기록을 트랙으로 만든다.

  ./drive.sh record mylap          조이스틱으로 한 바퀴 (Ctrl+C 로 저장)
  ./drive.sh laptrack mylap        그 기록 → 트랙 (벽·검사·HyperMPC 형식까지)

■ 왜 이 방법이 제일 낫나

  수식으로 만들면 **실험실에 안 들어가거나 차가 못 도는** 모양이 쉽게 나온다
  (레이아웃 A/B 를 풀어 봤지만 R≥1.05 에서는 닫히는 해가 없었다). 반면
  직접 몰아서 딴 라인은

      ① 차가 실제로 돌았으니 선회반경이 자동으로 지켜진다
      ② 실제로 그 공간을 지났으니 경계 안이 보장된다
      ③ 원하는 모양을 손이 아는 대로 그릴 수 있다

  남는 일은 "사람이 흔든 것" 을 걷어내는 것뿐이다.

■ 하는 일

  ① 한 바퀴만 잘라낸다 — 중심 기준 회전각이 2π 를 도는 구간을 찾는다.
     진입/이탈 구간과 여러 바퀴가 섞여 있어도 된다.
  ② 닫힌 주기 스플라인으로 평활화 (근사 — 점을 정확히 통과하지 않는다)
  ③ 곡률·경계·HyperMPC 전처리가 통과할 때까지 평활화를 자동으로 올린다
  ④ 중심선 ±half_width 로 좌/우 벽
  ⑤ draw_track / make_s_track 과 **같은 형식**으로 저장

■ 주의

  ★ 기록은 **뒤축 중심**(/mpc/state)이다. 사람이 안쪽으로 붙여 몰았으면
    그 라인이 그대로 중심선이 된다 — 레이싱 라인이지 트랙 중앙이 아니다.
    벽은 그 라인에서 좌우로 같은 거리에 생기므로, 한쪽이 실제 벽에
    가까울 수 있다. 저장 시 경계까지 여유를 숫자로 보여준다.

  ★ 평활화를 많이 올리면 코너가 뭉개져 실제로 돈 라인에서 멀어진다.
    화면/로그의 "원본에서 벗어난 거리(RMSE)" 를 보고 정하라.
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

REPO = dt.REPO


# ─────────────────────────────────────────────────────────────────────
def dist_to_polyline(P, C):
    """점 N개 → 닫힌 폴리라인 C 까지 최소거리. 벡터화.

    평활화를 올려 가며 매번 부르므로 파이썬 루프판(dt.dist_to_polygon)은
    너무 느리다 — 200x240 에 수 초가 걸린다.
    """
    A = C
    AB = np.roll(C, -1, axis=0) - A
    AB2 = np.maximum(np.einsum('ij,ij->i', AB, AB), 1e-12)
    AP = P[:, None, :] - A[None, :, :]
    t = np.clip(np.einsum('nij,ij->ni', AP, AB) / AB2, 0.0, 1.0)
    Q = A[None, :, :] + t[:, :, None] * AB[None, :, :]
    return np.linalg.norm(P[:, None, :] - Q, axis=2).min(axis=1)


def load_lap_csv(path):
    """waypoint_logger 출력 (# x,y[,speed]) 을 읽는다."""
    pts = []
    import csv as _csv
    with open(path, encoding='utf-8') as f:
        for row in _csv.reader(f):
            if not row or row[0].strip().startswith('#') or not row[0].strip():
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return np.array(pts, dtype=float)


def split_laps(xy, tol=0.30, min_len=4.0, min_pts=30):
    """출발점으로 **되돌아오는** 구간을 한 바퀴로 센다.

    ★ 예전에는 중심 둘레의 누적 회전각이 2π 를 지날 때마다 한 바퀴로
      셌다. 그건 트랙이 중심에서 봤을 때 단조롭게 돌 때만 맞는다.
      안쪽으로 파고드는 S 나 헤어핀이 있으면 각이 되돌아가서 깨진다 —
      실제로 사용자가 U턴 있는 라인을 몰았을 때 75 m 를 달렸는데도
      "누적 회전 1.03 바퀴" 로 읽고 바퀴를 못 찾았다 (2026-10-01).

      '되돌아옴' 은 모양에 무관하다. 호길이로 min_len 이상 간 뒤
      출발점 tol 안으로 들어오는 첫 지점을 한 바퀴의 끝으로 본다.
    """
    if len(xy) < min_pts:
        return []
    s_ = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    laps, i, n = [], 0, len(xy)
    while i < n - min_pts:
        j0 = int(np.searchsorted(s_, s_[i] + min_len))
        if j0 >= n:
            break
        hit = np.flatnonzero(
            np.linalg.norm(xy[j0:] - xy[i], axis=1) < tol)
        if len(hit) == 0:
            i += 5
            continue
        j = j0 + int(hit[0])
        if j - i >= min_pts:
            laps.append((i, j))
        i = j
    return laps


def detect_interleaved(xy, min_dist=0.1):
    """퍼블리셔가 둘 이상 섞였는지 본다.

    ★ waypoint_logger 는 min_dist(0.1 m) 이상 움직였을 때만 점을 쌓는다.
      그런데 /mpc/state 에 퍼블리셔가 둘이면 점이 두 궤적을 **번갈아**
      찍어서, 점간 거리가 min_dist 가 아니라 '두 궤적 사이 거리' 가 된다.

      2026-10-01 실측: 점간 거리 중앙 2.49 m (min_dist 의 25배),
      1 m 넘는 점프가 88%. 그 상태로는 어떤 평활화로도 복구가 안 된다.

    반환: (의심됨, 설명)
    """
    if len(xy) < 20:
        return False, ''
    d = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    med = float(np.median(d))
    big = float(np.mean(d > max(1.0, 8 * min_dist)))
    if med > 4 * min_dist and big > 0.3:
        # 한 칸 건너뛰면 매끄러워지는지 — 그러면 소스가 둘이다
        alt = float(np.median(np.linalg.norm(
            np.diff(xy[0::2], axis=0), axis=1)))
        return True, (f'점간 거리 중앙 {med:.2f} m (로거 min_dist {min_dist} 의 '
                      f'{med/min_dist:.0f}배), {100*big:.0f}% 가 1 m 넘는 점프. '
                      f'한 칸 건너뛰면 {alt:.3f} m 로 매끄러워집니다 '
                      f'→ /mpc/state 에 퍼블리셔가 둘입니다.')
    return False, ''


def lap_quality(xy, poly):
    """바퀴 고르기용 점수 — 매끄럽고 경계 안쪽일수록 좋다."""
    if len(xy) < 10:
        return 1e9
    d = np.diff(xy, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    jerk = float(np.abs(np.diff(np.unwrap(h))).sum())
    out = 0 if poly is None else int((~poly.contains_points(xy)).sum())
    return jerk + 50.0 * out


# ─────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description='수동 주행 기록 → 트랙',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('source', help='기록 CSV 경로 또는 waypoints/ 안의 이름')
    p.add_argument('name', nargs='?', default=None, help='새 트랙 이름')
    p.add_argument('--boundary', default=dt.BOUNDARY)
    p.add_argument('--half-width', type=float, default=0.25,
                   help='중심선에서 벽까지 (기본 0.25 — HyperMPC 원저자 값)')
    p.add_argument('--spacing', type=float, default=0.05)
    p.add_argument('--raw-spacing', type=float, default=0.01)
    p.add_argument('--smooth', type=float, default=0.06,
                   help='초기 평활화 m. 사람 주행은 손클릭보다 흔들려서 '
                        '기본값이 더 크다 (draw_track 은 0.03)')
    p.add_argument('--lap', type=int, default=None,
                   help='몇 번째 바퀴를 쓸지 (1부터). 기본은 가장 매끄러운 것')
    p.add_argument('--wheelbase', type=float, default=0.33)
    p.add_argument('--max-steer', type=float, default=0.40)
    p.add_argument('--max-bad-run', type=float, default=0.15)
    p.add_argument('--hypermpc', default='~/hypermpc_code')
    p.add_argument('--allow-infeasible', action='store_true')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()

    src = a.source
    if not os.path.isfile(src):
        cand = os.path.join(dt.WAYPOINTS, src if src.endswith('.csv')
                            else f'{src}.csv')
        if os.path.isfile(cand):
            src = cand
        else:
            print(f'✗ 기록을 못 찾았습니다: {a.source}  (또는 {cand})')
            sys.exit(1)
    name = a.name or (os.path.splitext(os.path.basename(src))[0] + '_track')

    V, _ = dt.load_boundary(a.boundary)
    poly = MplPath(np.vstack([V, V[:1]]))
    kappa_lim = math.tan(a.max_steer) / a.wheelbase
    hw = a.half_width

    raw = load_lap_csv(src)
    if len(raw) < 20:
        print(f'✗ 점이 {len(raw)}개뿐입니다: {src}')
        sys.exit(1)
    L_raw = float(np.sum(np.linalg.norm(np.diff(raw, axis=0), axis=1)))
    print(f'■ 기록  {os.path.relpath(src, REPO)}')
    print(f'   {len(raw)}점  주행거리 {L_raw:.2f} m  '
          f'x [{raw[:,0].min():+.2f},{raw[:,0].max():+.2f}] '
          f'y [{raw[:,1].min():+.2f},{raw[:,1].max():+.2f}]')
    n_out = int((~poly.contains_points(raw)).sum())
    print(f'   경계 밖 점 {n_out}개'
          + ('  ⚠ mocap 튐이거나 경계가 실제보다 좁습니다' if n_out else ''))

    _d = np.linalg.norm(np.diff(raw, axis=0), axis=1)
    print(f'   점간 거리 중앙 {np.median(_d):.3f} m')

    bad, why = detect_interleaved(raw)
    if bad:
        print(f'\n✗ 기록이 오염됐습니다 — {why}')
        print('  어떤 평활화로도 복구되지 않습니다. 원인을 없애고 다시 찍으세요:')
        print('     ./drive.sh stop            떠 있는 노드를 전부 내린다')
        print('     ros2 topic info /mpc/state 퍼블리셔가 1 인지 확인')
        print('     ./drive.sh record <이름>   다시 기록')
        print('  (mocap_bridge 가 둘 떠 있거나 sim_bridge 가 같이 돌면 이렇게 됩니다)')
        sys.exit(1)

    laps = split_laps(raw)
    if not laps:
        print('✗ 한 바퀴를 못 찾았습니다 — 출발점으로 되돌아온 구간이 없습니다.')
        print('  출발한 자리로 완전히 돌아오게 한 바퀴를 더 도세요.')
        print('  (--lap 으로 고를 수 있게 두세 바퀴 도는 편이 낫습니다)')
        sys.exit(1)
    print(f'\n■ 바퀴 {len(laps)}개')
    for i, (s0, s1) in enumerate(laps, 1):
        seg = raw[s0:s1 + 1]
        d = float(np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1)))
        print(f'   {i}: 점 {s1-s0+1:4d}  길이 {d:6.2f} m  '
              f'매끄러움 {lap_quality(seg, poly):7.2f} (작을수록 좋음)')
    if a.lap:
        if not 1 <= a.lap <= len(laps):
            print(f'✗ --lap 은 1~{len(laps)} 입니다')
            sys.exit(1)
        pick = a.lap - 1
    else:
        pick = int(np.argmin([lap_quality(raw[s:e + 1], poly)
                              for s, e in laps]))
    s0, s1 = laps[pick]
    lap = raw[s0:s1 + 1]
    print(f'   → {pick+1}번 사용')

    # 중복점 제거 — 정지 중 노이즈가 스플라인을 죽인다
    keep = [0]
    for i in range(1, len(lap)):
        if np.hypot(*(lap[i] - lap[keep[-1]])) > 0.02:
            keep.append(i)
    lap = lap[keep]
    print(f'   중복/정지 점 제거 후 {len(lap)}점')

    TR = dt.load_track_reader(a.hypermpc)
    if TR is None:
        print('   ⚠ 원저장소가 없어 HyperMPC 전처리 검사를 건너뜁니다')

    # ── 평활화를 올려 가며 통과하는 최소값 찾기 ──────────────────────
    print(f'\n■ 평활화 — 통과하는 최소값을 찾습니다 '
          f'(차 한계 R {1/kappa_lim:.3f} m, 허용 연속 {a.max_bad_run*100:.0f} cm)')
    best = None
    for dev in np.arange(a.smooth, 0.401, 0.01):
        r = dt.fit_closed_spline(lap, float(dev), a.spacing)
        if r is None:
            continue
        c, kap, total, tck = r
        ak = np.abs(kap)
        wind = dt.winding(kap, a.spacing)
        if abs(abs(wind) - 1.0) > 0.15:
            print(f'   붕괴 평활 {dev:.2f}  ∮κds/2π {wind:+.2f}')
            continue
        left, right = dt.offset_walls(c, hw)
        inside = (poly.contains_points(c).all()
                  and poly.contains_points(left).all()
                  and poly.contains_points(right).all())
        bad_run = dt.longest_run(ak > kappa_lim, a.spacing)
        # 원본에서 얼마나 멀어졌나
        rmse = float(np.sqrt(np.mean(dist_to_polyline(lap, c) ** 2)))
        prep = None
        prun = 0.0
        if TR is not None and bad_run <= a.max_bad_run and inside:
            seg, uu, T = dt.arclen_table(tck)
            craw, _ = dt.resample(tck, seg, uu, T, a.raw_spacing)
            prep = dt.run_prep(TR, craw, hw, a.raw_spacing)
            if 'err' in prep:
                prep = None
            else:
                pk = np.abs(prep['kappa'])
                prep['bad'] = float((pk > kappa_lim).mean())
                prep['warn'] = float((pk > 1.0).mean())
                prep['bad_run'] = dt.longest_run(pk > kappa_lim,
                                                 prep['spacing'])
                prun = prep['bad_run']
        ok = (inside and bad_run <= a.max_bad_run
              and (prep is None or prun <= a.max_bad_run))
        print(f'   {"✓" if ok else "  "} 평활 {dev:.2f}  R_min {1/ak.max():6.3f}  '
              f'최장불가 {bad_run*100:3.0f}cm  경계 {"O" if inside else "X"}  '
              f'원본편차 {rmse:.3f} m'
              + (f'  전처리 {prep["r_min"]:.3f}/{prun*100:.0f}cm'
                 if prep else ''))
        if ok:
            best = (float(dev), c, kap, total, tck, left, right, prep, rmse)
            break
    if best is None:
        print('\n✗ 0.40 m 까지 올려도 통과하지 못했습니다.')
        print('  차가 실제로 돈 라인이므로 보통 통과합니다. 안 되면:')
        print('   · 다른 바퀴를 써 보세요 (--lap N)')
        print('   · 그 바퀴에서 벽에 너무 붙었을 수 있습니다 '
              '(--half-width 를 줄여 보세요)')
        print('   · mocap 이 튄 구간이 있는지 기록을 확인하세요')
        sys.exit(1)

    dev, c, kap, total, tck, left, right, prep, rmse = best
    ak = np.abs(kap)
    clr = min(dist_to_polyline(left, V).min(),
              dist_to_polyline(right, V).min())
    bad_m = ak > kappa_lim
    fold_m = hw * ak >= 1.0
    print(f'\n■ 결과   평활 {dev:.2f} m')
    print(f'   길이 {total:.2f} m · {len(c)}점 · 폭 {2*hw:.2f} m')
    print(f'   R_min {1/ak.max():.3f} m  (차 한계 {1/kappa_lim:.3f})')
    print(f'   벽~경계 여유 {clr:.3f} m   원본에서 벗어남 {rmse:.3f} m')
    print(f'   ∮κ ds / 2π {dt.winding(kap, a.spacing):+.4f}')
    if prep:
        print(f'   HyperMPC 전처리 후 R_min {prep["r_min"]:.3f} m '
              f'({prep["n"]}점, RMSE {prep["rmse"]:.4f} m)')
    if fold_m.any():
        print(f'   ⚠ 벽 접힘 {fold_m.sum()}점 — left/right_boundary.csv 만 '
              f'영향, MPC 는 벽 좌표를 안 씁니다')

    if a.dry_run:
        print('\n(--dry-run — 저장하지 않습니다)')
        return

    seg, uu, T = dt.arclen_table(tck)
    craw, _ = dt.resample(tck, seg, uu, T, a.raw_spacing)
    stats = dict(
        total=total, n=len(c), k_max=float(ak.max()),
        r_min=float(1 / ak.max()), bad=float(bad_m.mean()),
        bad_n=int(bad_m.sum()),
        bad_run=dt.longest_run(bad_m, a.spacing),
        warn=float((ak > 1.0).mean()), inside=True, clear=float(clr),
        fold=bool(fold_m.any()), fold_n=int(fold_m.sum()),
        fold_run=dt.longest_run(fold_m, a.spacing),
        wind=dt.winding(kap, a.spacing), area=dt.shoelace(c),
        area_click=dt.shoelace(c), degenerate=False)
    dt.write_track(
        name, c, kap, left, right, craw, hw, stats, prep,
        dict(source=f'tools/track_from_lap.py (수동 주행 기록 '
                    f'{os.path.relpath(src, REPO)} 의 {pick+1}번째 바퀴)',
             spline='scipy.interpolate.splprep(k=3, per=1) — 근사 평활',
             smooth_dev=dev, clicked=[],
             boundary=a.boundary, wheelbase=a.wheelbase,
             max_steer=a.max_steer, kappa_lim=kappa_lim,
             max_bad_run=a.max_bad_run),
        spacing=a.spacing, raw_spacing=a.raw_spacing)


if __name__ == '__main__':
    main()
