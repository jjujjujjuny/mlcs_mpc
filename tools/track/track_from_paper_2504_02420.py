#!/usr/bin/env python3
"""
track_from_paper_2504_02420.py — 논문 "On learning racing policies with reinforcement learning"
(Czechmanowski et al., IROS 2025, arXiv:2504.02420v2) 의 실험 트랙 (L자, 길이 17 m, 폭 1 m) 을 Fig. 6 에서 뽑는다.

  python3 tools/track/track_from_paper_2504_02420.py docs/references/2504.02420v2.pdf [--name rl17_track] [--plot out.png]
  → src/mlcs_mpc/mlcs_mpc/env/track/data/<이름>/prep_<이름>.csv  (s,x,y,heading,curvature,track_width,w_right,w_left)
    Track() 가 처음 읽을 때 같은 폴더에 ref_<이름>.csv 를 만든다 (env/track/reference.py).

■ 어떻게 (Fig. 6 왼쪽 그림은 래스터 이미지라 벡터 좌표가 없다 → 픽셀에서 잰다)
  ① PDF 7쪽의 그림을 원본 해상도로 꺼낸다 (2613×909, 왼쪽 36 % 가 트랙 그림)
  ② 축척 — 격자선 (회색 176) 의 픽셀 위치: x = −2, 0, 2, 4 m / y = 0 … 6 m → 103.83 px/m (가로·세로 같다)
  ③ 트랙 영역 = 회색 (128) + 그 위의 주행 점 (주황·파랑). 격자선·숫자 틈은 닫기 + 작은 구멍 메우기
  ④ ★ 그림의 축이 트랙 위 (y > 6.13) 와 아래 (y < −0.36) 를 잘라 바깥 경계가 일부 안 보인다. 안쪽 경계 (트랙 안의 구멍)
     는 전부 보이므로, 센터라인 = 안쪽 경계에서 0.5 m 떨어진 등거리선 (거리 변환의 0.5 m 등고선 — 오목한 곳에서도 꼬이지 않는다)
     으로 잡는다. 검증: 바깥 경계가 보이는 곳에서 센터 → 바깥 경계 거리 중앙값 0.53 m (전폭 1.03 m, 논문 1 m).
  ⑤ 방향 — 논문 진행 표시 (0 → 17) 대로 시계 방향 (왼쪽 변을 위로). s = 0 은 표시 "0" 근처 (왼쪽 변 y = 2.0 m).
  ⑥ 픽셀 잡음 (1 px = 9.6 mm) 은 닫힌 5차 평활 스플라인 (점당 1 cm) 으로 지우고 0.1 m 간격으로 재샘플.

  결과 길이 ≈ 17.4 m (논문 "17 m", Fig. 6 진행 축 ≈ 17.5 m). 폭은 논문대로 1.0 m (반폭 0.5 m 양쪽).
  ★ 그림에서 잰 것이라 cm 단위 오차가 있다. 실제 트랙을 깔 때는 이 파일로 테이프 위치를 정한 뒤 다시 재서 (maptrack) 쓴다.
"""
import argparse
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA = os.path.join(ROOT, 'src/mlcs_mpc/mlcs_mpc/env/track/data')
WIDTH = 1.0             # 논문: "measuring 17 meters in length and 1 meter in width"


def load_figure(pdf):
    import pymupdf
    doc = pymupdf.open(pdf)
    pg = doc[6]                                   # 7쪽 (Fig. 6)
    pix = pymupdf.Pixmap(doc, pg.get_images()[0][0])
    a = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)[:, :, :3]
    return a[:, :int(pix.width * 0.36)].astype(int)


def grid_lines(a):
    """격자선 픽셀 위치 → (x0 px, y0 px, px/m)."""
    g = np.abs(a - 176).sum(2) < 30

    def peaks(v):
        idx = np.where(v > 150)[0]
        return [grp.mean() for grp in np.split(idx, np.where(np.diff(idx) > 2)[0] + 1) if len(grp)]
    cols, rows = peaks(g.sum(0)), peaks(g.sum(1))
    if len(cols) != 4 or len(rows) != 7:
        raise RuntimeError(f'격자선을 못 찾았습니다: 세로 {len(cols)} (4 기대), 가로 {len(rows)} (7 기대)')
    kx = np.polyfit([-2, 0, 2, 4], cols, 1)          # px = kx0·x + kx1
    ky = np.polyfit([6, 5, 4, 3, 2, 1, 0], rows, 1)  # px = ky0·y + ky1 (ky0 < 0)
    return kx, ky


def track_mask(a):
    from scipy import ndimage as ndi
    near = lambda c, t: np.abs(a - np.array(c)).sum(2) < t      # noqa: E731
    band = near((128, 128, 128), 40) | near((248, 120, 8), 90) | near((0, 0, 248), 90)
    yy, xx = np.mgrid[:a.shape[0], :a.shape[1]]
    band &= (xx > 120) & (xx < 840) & (yy > 8) & (yy < 680)      # 축 틀 안쪽만
    band = ndi.binary_closing(band, structure=np.ones((7, 7)))  # 격자선 틈
    lab, n = ndi.label(~band)
    sizes = ndi.sum(np.ones_like(lab), lab, range(1, n + 1))
    for i, sz in enumerate(sizes, 1):
        if sz < 20000:                                            # 숫자·점 사이 작은 구멍
            band[lab == i] = True
    lab, n = ndi.label(~band)
    sizes = ndi.sum(np.ones_like(lab), lab, range(1, n + 1))
    if n != 2:
        raise RuntimeError(f'트랙 영역이 고리 모양이 아닙니다 (배경 조각 {n} 개)')
    hole = lab == 1 + int(np.argmin(sizes))
    outer = lab == 1 + int(np.argmax(sizes))
    return band, hole, outer


def centerline(a):
    from scipy import ndimage as ndi
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    kx, ky = grid_lines(a)
    ppm = kx[0]
    if abs(abs(ky[0]) - ppm) / ppm > 0.01:
        raise RuntimeError(f'가로·세로 축척이 다릅니다: {kx[0]:.2f} / {-ky[0]:.2f} px/m')
    _, hole, outer = track_mask(a)
    d_hole = ndi.distance_transform_edt(~hole) / ppm
    d_out = ndi.distance_transform_edt(~outer) / ppm
    fig = plt.figure()
    seg = max(plt.contour(d_hole, levels=[WIDTH / 2]).allsegs[0], key=len)
    plt.close(fig)
    c, r = seg[:, 0], seg[:, 1]
    x, y = (c - kx[1]) / kx[0], (r - ky[1]) / ky[0]
    # 검증용: 바깥 경계까지 거리 (축에 잘리지 않은 곳)
    ri = np.clip(np.round(r).astype(int), 0, a.shape[0] - 1)
    ci = np.clip(np.round(c).astype(int), 0, a.shape[1] - 1)
    y_top, y_bot = (8 - ky[1]) / ky[0], (680 - ky[1]) / ky[0]
    vis = (y + WIDTH / 2 < y_top - 0.05) & (y - WIDTH / 2 > y_bot + 0.05)
    half_out = d_out[ri, ci][vis]
    return x, y, ppm, half_out


def resample(x, y, ds=0.1, smooth_m=0.01, start=(-1.25, 2.0)):
    from scipy.interpolate import splev, splprep
    if np.hypot(x[0] - x[-1], y[0] - y[-1]) < 1e-9:
        x, y = x[:-1], y[:-1]
    # 시계 방향으로 (신발끈 넓이 < 0)
    if 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0:
        x, y = x[::-1], y[::-1]
    i0 = int(np.argmin(np.hypot(x - start[0], y - start[1]) + 10.0 * (x > 0)))
    x, y = np.roll(x, -i0), np.roll(y, -i0)
    n = len(x)
    tck, _ = splprep([np.r_[x, x[0]], np.r_[y, y[0]]], s=(n + 1) * smooth_m ** 2, per=1, k=5)
    uf = np.linspace(0, 1, 20 * n + 1)
    dx, dy = splev(uf, tck, der=1)
    sf = np.r_[0, np.cumsum(0.5 * (np.hypot(dx[1:], dy[1:]) + np.hypot(dx[:-1], dy[:-1])) * np.diff(uf))]
    L = float(sf[-1])
    m = int(round(L / ds))
    s = np.arange(m) * (L / m)
    us = np.interp(s, sf, uf)
    xs, ys = splev(us, tck)
    d1x, d1y = splev(us, tck, der=1)
    d2x, d2y = splev(us, tck, der=2)
    h = np.arctan2(d1y, d1x)
    k = (d1x * d2y - d1y * d2x) / np.hypot(d1x, d1y) ** 3
    return s, np.asarray(xs), np.asarray(ys), h, k, L


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('pdf', help='docs/references/2504.02420v2.pdf')
    ap.add_argument('--name', default='rl17_track')
    ap.add_argument('--plot', default='', help='확인용 그림 (기본 data/<이름>/preview.png)')
    a = ap.parse_args(argv)
    img = load_figure(a.pdf)
    x, y, ppm, half_out = centerline(img)
    s, xs, ys, h, k, L = resample(x, y)
    out_dir = os.path.join(DATA, a.name)
    os.makedirs(out_dir, exist_ok=True)
    w = np.full_like(s, WIDTH / 2)
    prep = os.path.join(out_dir, f'prep_{a.name}.csv')
    np.savetxt(prep, np.c_[s, xs, ys, h, k, 2 * w, w, w], delimiter=',', fmt='%.6f', comments='',
               header='s,x,y,heading,curvature,track_width,w_right,w_left')
    turn = float(np.sum(k) * L / len(s))
    print(f'✓ {os.path.relpath(prep, ROOT)}')
    print(f'  축척 {ppm:.2f} px/m (1 px = {1000 / ppm:.1f} mm)   길이 {L:.2f} m (논문 17 m)   {len(s)} 점')
    print(f'  |κ| 최대 {np.abs(k).max():.2f} /m (반경 {1 / np.abs(k).max():.2f} m)   ∮κds {turn:+.3f} (−2π = {-2 * np.pi:.3f}, 시계 방향)')
    print(f'  검증: 바깥 경계가 보이는 곳 센터→경계 중앙값 {np.median(half_out):.3f} m '
          f'(p10 {np.percentile(half_out, 10):.3f}, p90 {np.percentile(half_out, 90):.3f}) — 반폭 {WIDTH / 2} m 로 둔다')
    png = a.plot or os.path.join(out_dir, 'preview.png')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 5))
    nx, ny = -np.sin(h), np.cos(h)
    ax[0].fill(np.r_[xs + nx * w, (xs - nx * w)[::-1]], np.r_[ys + ny * w, (ys - ny * w)[::-1]], color='0.85')
    ax[0].plot(xs + nx * w, ys + ny * w, 'k-', lw=0.8)
    ax[0].plot(xs - nx * w, ys - ny * w, 'k-', lw=0.8)
    ax[0].plot(xs, ys, 'b-', lw=1)
    for sm in range(0, int(L) + 1):
        i = int(np.argmin(np.abs(s - sm)))
        ax[0].text(xs[i], ys[i], str(sm), fontsize=8)
    ax[0].set_aspect('equal')
    ax[0].grid(True)
    ax[0].set_title(f'{a.name} (arXiv:2504.02420 Fig. 6) — {L:.2f} m')
    ax[1].plot(s, k)
    ax[1].set_xlabel('s [m]')
    ax[1].set_ylabel('κ [1/m]')
    ax[1].grid(True)
    fig.tight_layout()
    fig.savefig(png, dpi=120)
    print(f'  그림 {os.path.relpath(png, ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
