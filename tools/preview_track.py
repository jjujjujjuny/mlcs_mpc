#!/usr/bin/env python3
"""
preview_track.py — 트랙을 그림 한 장으로 본다.

  ./drive.sh preview s_track          lab_map/data/tracks/s_track/preview.png
  ./drive.sh preview track_5          웨이포인트 CSV 만 있어도 된다
  ./drive.sh preview s_track --show   창으로 띄우기 (NoMachine 필요)

■ 무엇을 그리나

  왼쪽  배치 — 실험실 19각형, 중심선(곡률로 색), 좌/우 벽,
         HyperMPC 전처리 결과(주황 파선), 직선·S자 구간 표시
  오른쪽 곡률 프로파일 — 설계값과 전처리 후를 겹쳐 그린다.
         차 한계선(κ=1.281)과 권장선(κ=1.0)을 같이 긋는다.

  ★ 두 곡선을 겹쳐 보는 것이 요점이다. 저자 전처리(k=5, s=2.0)가 코너를
    조여서 설계 R_min 의 0.72~0.85 만 남기므로, 설계만 보고 판단할 수 없다.
    프로파일에서 주황 파선이 빨간 선을 넘으면 MPC 가 받는 트랙이 불가다.

■ 색

  초록 κ ≤ 1.0 (권장 안쪽) · 주황 1.0 < κ ≤ 1.281 · 빨강 κ > 1.281 (주행 불가)
  검정 점 직선 구간(κ≈0) · 보라 점 곡률 부호가 뒤집힌 구간(S자)
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib                                           # noqa: E402

import draw_track as dt                                     # noqa: E402

REPO = dt.REPO
TRACKS = os.path.join(REPO, 'lab_map/data/tracks')


def load(name, spacing):
    """트랙 폴더가 있으면 그것을, 없으면 웨이포인트 CSV 를 읽는다."""
    d = os.path.join(TRACKS, name)
    cl = os.path.join(d, 'centerline.csv')
    if os.path.isfile(cl):
        c = np.loadtxt(cl, delimiter=',', skiprows=1, usecols=(1, 2))
        kp = os.path.join(d, 'curvature_profile.csv')
        kap = (np.loadtxt(kp, delimiter=',', skiprows=1, usecols=(2,))
               if os.path.isfile(kp) else None)
        return c, kap, d
    wp = dt.PathLike = os.path.join(dt.WAYPOINTS, f'{name}.csv')
    if not os.path.isfile(wp):
        print(f'✗ 트랙을 못 찾았습니다: {d}/ 도 {wp} 도 없습니다')
        sys.exit(1)
    return np.loadtxt(wp, delimiter=',', comments='#', usecols=(0, 1)), None, None


def curvature(c):
    """폐곡선 중심선에서 곡률 [1/m] — curvature_profile.csv 가 없을 때만.

    ★ 간격으로 다시 나누지 말 것. κ = (x'y'' − y'x'') / |r'|³ 는
      **매개변수화에 무관**하므로 인덱스로 미분해도 1/m 이 나온다.
      나눴다가 track_5 의 R_min 이 1.13 → 0.06 m 로 20배 틀렸다.
    """
    P = np.vstack([c[-1:], c, c[:1]])
    d1 = (P[2:] - P[:-2]) / 2.0
    d2 = P[2:] - 2 * P[1:-1] + P[:-2]
    sp = np.linalg.norm(d1, axis=1)
    return ((d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])
            / np.maximum(sp ** 3, 1e-12))


def main():
    p = argparse.ArgumentParser(
        description='트랙 프리뷰 그림 생성',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('name', help='트랙 이름')
    p.add_argument('-o', '--out', default=None, help='저장 경로 (.png)')
    p.add_argument('--show', action='store_true', help='창으로 띄운다')
    p.add_argument('--half-width', type=float, default=0.25)
    p.add_argument('--spacing', type=float, default=0.05)
    p.add_argument('--raw-spacing', type=float, default=0.01)
    p.add_argument('--wheelbase', type=float, default=0.33)
    p.add_argument('--max-steer', type=float, default=0.40)
    p.add_argument('--kappa-target', type=float, default=1.0)
    p.add_argument('--hypermpc', default='~/hypermpc_code')
    p.add_argument('--ref', default='track_5', help='참고로 겹칠 트랙 (none 이면 끔)')
    p.add_argument('--dpi', type=int, default=105)
    a = p.parse_args()

    if not a.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    dt.setup_font()

    V, _ = dt.load_boundary(dt.BOUNDARY)
    c, kap, tdir = load(a.name, a.spacing)
    if kap is None or len(kap) != len(c):
        kap = curvature(c)
    left, right = dt.offset_walls(c, a.half_width)
    ak = np.abs(kap)
    klim = math.tan(a.max_steer) / a.wheelbase
    ds = float(np.median(np.linalg.norm(
        np.diff(np.vstack([c, c[:1]]), axis=0), axis=1)))
    total = ds * len(c)

    # 저자 전처리 — 원본 CSV 가 있으면 그대로, 없으면 중심선으로 만든다
    prep = None
    TR = dt.load_track_reader(a.hypermpc)
    if TR is not None:
        raw = (os.path.join(tdir, f'{a.name}.csv') if tdir else None)
        if raw and os.path.isfile(raw):
            craw = np.loadtxt(raw, delimiter=',', skiprows=1, usecols=(0, 1))
        else:
            craw = c
        pr = dt.run_prep(TR, craw, a.half_width, a.raw_spacing)
        if 'err' not in pr:
            prep = pr

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 7.5),
                                  gridspec_kw=dict(width_ratios=[1.15, 1]))

    # ── 왼쪽: 배치 ──────────────────────────────────────────────────
    B = np.vstack([V, V[:1]])
    ax.plot(B[:, 0], B[:, 1], '-', color='#333', lw=2.4,
            label=f'실험실 경계 {len(V)}각형')
    if a.ref != 'none' and a.ref != a.name:
        rp = os.path.join(dt.WAYPOINTS, f'{a.ref}.csv')
        if os.path.isfile(rp):
            t = np.loadtxt(rp, delimiter=',', comments='#', usecols=(0, 1))
            ax.plot(np.r_[t[:, 0], t[0, 0]], np.r_[t[:, 1], t[0, 1]], ':',
                    color='#999', lw=1.2, label=f'{a.ref} (참고)')

    segs = np.stack([c, np.roll(c, -1, axis=0)], axis=1)
    col = np.where(ak > klim, '#c0392b',
                   np.where(ak > a.kappa_target, '#d9a24e', '#1B6F4A'))
    ax.add_collection(LineCollection(segs, colors=col, lw=3.0, zorder=4))
    for w, lb in ((left, f'좌/우 벽 (반폭 {a.half_width})'), (right, None)):
        ww = np.vstack([w, w[:1]])
        ax.plot(ww[:, 0], ww[:, 1], '-', color='#2B45C4', alpha=0.6, lw=1.3,
                label=lb)

    st = ak < 0.10
    if st.any():
        ax.plot(c[st, 0], c[st, 1], '.', color='#111', ms=6, zorder=8,
                label=f'직선 구간 (|κ|<0.1, {100*st.mean():.0f}%)')
    # ★ 문턱을 넉넉히 둔다. 0 근처로 두면 직선 구간의 수치 잡음이 0 을
    #   넘나드는 것까지 "S자" 로 세어 버린다 (track_5 가 6% 로 나왔다).
    S_K = 0.15
    main_sign = 1.0 if kap.sum() >= 0 else -1.0
    sw = (kap * main_sign) < -S_K
    if sw.any():
        ax.plot(c[sw, 0], c[sw, 1], '.', color='#8E24AA', ms=6, zorder=8,
                label=f'곡률 역방향 = S자 ({100*sw.mean():.0f}%)')
    if (ak > klim).any():
        ax.plot(c[ak > klim, 0], c[ak > klim, 1], 'x', color='#c0392b',
                ms=8, mew=2, zorder=9, label='주행 불가')

    if prep is not None:
        pc = prep['c']
        ax.plot(np.r_[pc[:, 0], pc[0, 0]], np.r_[pc[:, 1], pc[0, 1]], '--',
                color='#8E5A05', lw=1.6, zorder=6,
                label=f'HyperMPC 전처리 ({prep["n"]}점)')
    ax.plot(c[0, 0], c[0, 1], 'o', color='#2B45C4', ms=9, zorder=10)
    ax.annotate('시작', (c[0, 0], c[0, 1]), xytext=(10, 10),
                textcoords='offset points', fontsize=10)

    clr = min(dt.dist_to_polygon(left, V).min(),
              dt.dist_to_polygon(right, V).min())
    ax.set_aspect('equal')
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_xlabel('x [m]  (mocap world)')
    ax.set_ylabel('y [m]')
    ax.legend(loc='upper right', fontsize=8.5, framealpha=0.92)
    ax.set_title(f'{a.name}   길이 {total:.2f} m · {len(c)}점 · '
                 f'폭 {2*a.half_width:.2f} m\n'
                 f'R_min {1/ak.max():.3f} m · 벽~경계 {clr:.3f} m · '
                 f'크기 {np.ptp(c[:,0]):.2f} × {np.ptp(c[:,1]):.2f} m',
                 fontsize=11, loc='left')

    # ── 오른쪽: 곡률 ────────────────────────────────────────────────
    s_ax = np.arange(len(kap)) * ds
    ax2.axhline(0, color='#999', lw=0.8)
    ax2.axhline(klim, color='#c0392b', ls='--', lw=1.2,
                label=f'차 한계 κ={klim:.3f} (R {1/klim:.3f} m)')
    ax2.axhline(-klim, color='#c0392b', ls='--', lw=1.2)
    ax2.axhline(a.kappa_target, color='#d9a24e', ls='--', lw=1.2,
                label=f'권장 κ={a.kappa_target:.1f}')
    ax2.axhline(-a.kappa_target, color='#d9a24e', ls='--', lw=1.2)
    ax2.plot(s_ax, kap, '-', color='#1B6F4A', lw=2.2, label='설계 곡률')
    if st.any():
        ax2.fill_between(s_ax, -0.03, 0.03, where=st, color='#111', alpha=0.3,
                         label='직선 구간')
    if prep is not None:
        pk = prep['kappa']
        ps = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(prep['c'], axis=0),
                                                 axis=1))]
        # 저자 전처리는 진행 방향을 뒤집으므로 부호를 맞춰 겹친다
        sign = -1.0 if np.sign(pk.sum()) != np.sign(kap.sum()) else 1.0
        ax2.plot(ps, sign * pk, '--', color='#8E5A05', lw=1.6,
                 label=f'HyperMPC 전처리 후 (R_min {prep["r_min"]:.3f} m)')
    ax2.set_xlabel('s [m]')
    ax2.set_ylabel('κ [1/m]')
    ax2.grid(alpha=0.25, lw=0.5)
    ax2.legend(fontsize=8.5, loc='lower right', framealpha=0.92)
    ax2.set_title('곡률 프로파일 — 부호가 바뀌는 곳이 S자다',
                  fontsize=11, loc='left')

    plt.tight_layout()
    out = a.out or (os.path.join(tdir, 'preview.png') if tdir
                    else os.path.join(TRACKS, f'{a.name}_preview.png'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    plt.savefig(out, dpi=a.dpi)
    print(f'✓ {os.path.relpath(out, REPO)}')
    print(f'   길이 {total:.2f} m · R_min {1/ak.max():.3f} m · '
          f'벽~경계 {clr:.3f} m · 한계초과 {100*(ak>klim).mean():.1f}%')
    if prep is not None:
        print(f'   전처리 후 R_min {prep["r_min"]:.3f} m '
              f'({prep["n"]}점, RMSE {prep["rmse"]:.4f} m)')
    if a.show:
        plt.show()


if __name__ == '__main__':
    main()
