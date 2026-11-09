#!/usr/bin/env python3
"""
reference — 주행 기준 트랙. 센터라인 하나에서 위치·heading·곡률을 같은 스플라인으로 미리 계산해 둔다.

  python3 -m mlcs_mpc.env.track.reference [이름 ...]      (없으면 data/ 의 전부)
  → data/<이름>/ref_<이름>.csv   s,x,y,heading,curvature,track_width,w_right,w_left,cos_heading,sin_heading

■ 왜 — prep_*.csv 의 curvature 열은 센터라인 (x, y) 과 따로 계산돼 있어 조금씩 어긋나고, 특히 시작·끝
  이음매 (s = 0) 에서 끊긴다 (comp1_track: dκ/ds 7.6 /m², ∮κds 6.21 ≠ 2π). OCP 는 κ(s) 로 차의 진행을 적분하고
  노드는 (x, y) 로 Frenet 을 계산하므로, 둘이 어긋나면 MPC 가 실제 트랙과 다른 길을 예측한다.

■ 어떻게 (race_mpc env/track/data 처럼)
  ① (x, y) 를 닫힌 5차 평활 스플라인으로 맞춘다 (점당 허용 편차 smooth_m, 기본 5 mm)
  ② 호 길이로 ds 간격 재샘플 → heading = atan2(y', x'), κ = (x'y'' − y'x'') / |r'|³ (같은 스플라인의 미분)
  ③ 벽 폭은 원래 점에서 옮겨 온다 (센터라인이 옮겨 간 만큼 보정)
  닫힌 곡선이라 ∮κds = ±2π 가 저절로 맞고 이음매도 매끄럽다.

  Track(path) 는 prep_<이름>.csv 를 받으면 같은 폴더의 ref_<이름>.csv 를 쓴다 (없거나 prep 보다 오래되면 만든다).
"""
import os
import sys

import numpy as np

COLS = ('s', 'x', 'y', 'heading', 'curvature', 'track_width', 'w_right', 'w_left', 'cos_heading', 'sin_heading')


def build(prep_csv, ds=0.05, smooth_m=0.005):
    from scipy.interpolate import splev, splprep
    d = np.genfromtxt(prep_csv, delimiter=',', names=True)
    x, y = d['x'], d['y']
    if 'w_left' in d.dtype.names:
        wl, wr = d['w_left'], d['w_right']
    else:
        wl = wr = d['track_width'] / 2.0
    if np.hypot(x[0] - x[-1], y[0] - y[-1]) < 1e-6:     # 끝점이 시작점과 겹친 형식
        x, y, wl, wr = x[:-1], y[:-1], wl[:-1], wr[:-1]
    n = len(x)
    # ① 닫힌 평활 스플라인 (k=5 → κ 가 C³). splprep 의 s 는 잔차 제곱합 → 점당 smooth_m
    tck, u = splprep([np.r_[x, x[0]], np.r_[y, y[0]]], s=(n + 1) * smooth_m ** 2, per=1, k=5)
    # ② 호 길이 재샘플
    uf = np.linspace(0, 1, 40 * n + 1)
    dxf, dyf = splev(uf, tck, der=1)
    sf = np.r_[0, np.cumsum(0.5 * (np.hypot(dxf[1:], dyf[1:]) + np.hypot(dxf[:-1], dyf[:-1])) * np.diff(uf))]
    L = float(sf[-1])
    m = max(int(round(L / ds)), 8)
    s = np.arange(m) * (L / m)
    us = np.interp(s, sf, uf)
    xs, ys = splev(us, tck)
    d1x, d1y = splev(us, tck, der=1)
    d2x, d2y = splev(us, tck, der=2)
    sp = np.hypot(d1x, d1y)
    h = np.arctan2(d1y, d1x)
    k = (d1x * d2y - d1y * d2x) / sp ** 3
    # ③ 벽 폭 — 원래 점 i 의 매개 u_i 위치로 옮기고, 센터가 왼쪽으로 e 만큼 옮겨 갔으면 왼쪽 폭 −e, 오른쪽 +e
    ui = u[:-1]
    xi, yi = splev(ui, tck)
    hi = np.arctan2(*splev(ui, tck, der=1)[::-1])
    e = -np.sin(hi) * (xi - x) + np.cos(hi) * (yi - y)      # 새 센터의 원래 점 기준 왼쪽 변위
    si = np.interp(ui, uf, sf)
    order = np.argsort(si)
    wrap = lambda v: np.interp(s, np.r_[si[order], L], np.r_[v[order], v[order][0]])  # noqa: E731
    w_left, w_right = wrap(wl - e), wrap(wr + e)
    info = dict(length=L, points=m, shift_max=float(np.abs(e).max()), kappa_max=float(np.abs(k).max()),
                turn=float(np.sum(k) * L / m))
    return np.c_[s, xs, ys, h, k, w_left + w_right, w_right, w_left, np.cos(h), np.sin(h)], info


def ref_path(prep_csv):
    d, f = os.path.split(os.path.abspath(prep_csv))
    return os.path.join(d, 'ref_' + f[len('prep_'):]) if f.startswith('prep_') else None


def write(prep_csv, out=None, **kw):
    data, info = build(prep_csv, **kw)
    out = out or ref_path(prep_csv)
    np.savetxt(out, data, delimiter=',', fmt='%.9f', comments='', header=','.join(COLS))
    return out, info


def resolve(path):
    """prep_<이름>.csv → ref_<이름>.csv (없거나 prep 보다 오래되면 만든다). 다른 파일은 그대로."""
    ref = ref_path(path)
    if ref is None or not os.path.isfile(path):
        return path
    if not os.path.isfile(ref) or os.path.getmtime(ref) < os.path.getmtime(path):
        try:
            write(path)
        except OSError:
            return path         # 쓸 수 없는 곳이면 원래 파일
    return ref


def main(argv):
    from mlcs_mpc.env.track import DATA_DIR
    names = argv or sorted(n for n in os.listdir(DATA_DIR) if os.path.isfile(os.path.join(DATA_DIR, n, f'prep_{n}.csv')))
    for n in names:
        prep = n if os.path.isfile(n) else os.path.join(DATA_DIR, n, f'prep_{n}.csv')
        out, i = write(prep)
        print(f'✓ {os.path.relpath(out)}  길이 {i["length"]:.2f} m, {i["points"]}점, |κ|max {i["kappa_max"]:.2f}, '
              f'∮κds {i["turn"]:+.3f} (2π {2 * np.pi:.3f}), 센터 이동 최대 {i["shift_max"] * 1000:.1f} mm')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
