#!/usr/bin/env python3
"""
track_from_map.py — 라이다 맵(pgm/yaml)에서 트랙을 만든다. 대회장용 (mocap 없음).

  보통은  ./drive.sh maptrack [맵] [트랙]  — 이 도구 + 전처리 + 프리뷰를 한 번에

  ./drive.sh mapping                       조이스틱으로 트랙을 1~2바퀴
  ./drive.sh savemap comp1                 → maps/comp1.pbstream/.pgm/.yaml
  python3 tools/track_from_map.py comp1 comp1_track
  ./drive.sh preptrack comp1_track         HyperMPC 용 prep_*.csv

■ 하는 일

  ① 맵에서 달릴 수 있는 칸을 고른다. 알 수 없는 칸(회색)은 벽으로 본다.
  ② 벽의 작은 틈을 막는다 (--seal). 라이다가 덜 본 틈으로 바깥이 이어지면
     트랙이 고리가 아니게 된다.
  ③ 출발점(기본: 맵핑을 시작한 자리 = map 원점)이 속한 영역을 고른다.
     그 영역이 가운데 섬을 감싸는 **고리**여야 트랙이다.
  ④ 안쪽 벽과 바깥 벽에서 같은 거리인 선을 중심선으로 뽑는다.
     (거리장 두 개의 차가 0 인 등고선 — 골격선과 달리 가지가 안 생긴다)
  ⑤ 진행 방향은 출발 방향(기본: map +x = 맵핑을 시작할 때 차가 본 방향)에
     맞추고, 출발점에서 가장 가까운 점을 0 번으로 둔다.
  ⑥ 평활화를 올려 가며 곡률·벽 여유·HyperMPC 전처리가 통과하는 최소값을 고른다
     (track_from_lap 과 같은 방식).
  ⑦ 지점마다 좌/우 벽까지 실제 폭을 재서 draw_track 과 **같은 형식**으로 저장한다.

■ 좌표계

  결과는 Cartographer **map 프레임**이다. 위치추정(/tracked_pose, map→base_link)
  과 바로 비교할 수 있다. 연구실에서 mocap 좌표로 옮기려면
  tools/map_mocap_align.py apply 를 쓴다.

■ 주의

  ★ 중심선은 트랙 중앙이지 레이싱 라인이 아니다. 첫 주행용이다.
  ★ 벽 모양이 들쭉날쭉하면 가운데 선이 꺾여 차가 못 돈다 (2026-11-08 연구실 맵 세 장).
    폭이 넉넉하면 --mincurv 로 폭 안에서 덜 굽은 선을 찾는다. 이때 결과는 가운데가
    아니므로 --line-margin 으로 벽에서 띄운다 (MPC 가 선에서 ±0.25 m 벗어날 수 있다).
  ★ 트랙 안의 작은 장애물(기둥 등)은 중심선 계산에서 빼고 경고만 한다.
    폭과 벽 여유 검사에는 들어간다.
  ★ 맵이 휘었으면(루프가 안 닫힘) 트랙도 휜다. 맵을 먼저 RViz 로 확인한다.
  ★ 직선이 원호로 바뀌는 곳에서 R_min 이 실제보다 작게 나온다 (평활 스플라인이
    곡률 계단을 넘친다. 반지름 1.5 m 경기장 합성 맵에서 1.29 m). 다른 트랙
    도구와 같은 성질이고, 실제보다 급하게 보므로 안전한 쪽이다.
"""
import argparse
import math
import os
import sys

import numpy as np
from scipy import ndimage, sparse
from scipy.optimize import lsq_linear

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib                                           # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                             # noqa: E402

import draw_track as dt                                     # noqa: E402
from track_from_lap import dist_to_polyline                 # noqa: E402

REPO = dt.REPO
_MAP = {}       # write_own_prep 가 폭을 다시 잴 때 쓴다
MAPS = os.path.join(REPO, 'maps')


# ─────────────────────────────────────────────────────────────────────
#  맵 읽기
# ─────────────────────────────────────────────────────────────────────
def read_pgm(path):
    """P5(이진)/P2(텍스트) pgm → uint8 배열. PIL 없이 읽는다."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        # 주석(#...)과 공백을 건너뛰며 헤더 토큰 4개 (매직, 폭, 높이, 최대값)
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, mx = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b'P5':
        i += 1                               # 헤더 뒤 공백 한 칸
        if mx > 255:
            img = np.frombuffer(data, '>u2', w * h, i).reshape(h, w)
            return (img.astype(float) * 255.0 / mx).astype(np.uint8)
        return np.frombuffer(data, np.uint8, w * h, i).reshape(h, w).copy()
    if magic == b'P2':
        vals = np.array(data[i:].split(), dtype=float)[:w * h]
        return (vals.reshape(h, w) * 255.0 / mx).astype(np.uint8)
    raise ValueError(f'pgm 이 아닙니다 ({magic!r}): {path}')


def load_map(path):
    """map_server 형식 yaml → 칸 분류와 좌표 변환 정보.

    p = (255 - v)/255 (negate 0) 를 점유 확률로 보고
    p > occupied_thresh 는 벽, p < free_thresh 는 빈칸, 나머지는 알 수 없음.
    """
    import yaml
    if not path.endswith(('.yaml', '.yml')):
        cand = os.path.join(MAPS, f'{path}.yaml')
        path = cand if os.path.isfile(cand) else path + '.yaml'
    with open(path, encoding='utf-8') as f:
        y = yaml.safe_load(f)
    img_path = y['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(path)),
                                img_path)
    if img_path.lower().endswith('.pgm'):
        img = read_pgm(img_path).astype(float)
    else:
        import matplotlib.image as mpimg
        img = mpimg.imread(img_path)
        if img.ndim == 3:
            img = img[..., :3].mean(axis=2)
        if img.max() <= 1.0:
            img = img * 255.0
    p = img / 255.0 if y.get('negate', 0) else (255.0 - img) / 255.0
    occ_t = float(y.get('occupied_thresh', 0.65))
    free_t = float(y.get('free_thresh', 0.196))
    origin = [float(v) for v in y.get('origin', [0.0, 0.0, 0.0])]
    return dict(
        path=path, image=img_path, img=img,
        free=p < free_t, occ=p > occ_t,
        res=float(y['resolution']),
        ox=origin[0], oy=origin[1], oyaw=origin[2] if len(origin) > 2 else 0.0,
        H=img.shape[0], W=img.shape[1])


def pix_to_world(m, rows, cols):
    """픽셀 (행, 열; 실수 허용) → map 좌표. 0 행이 이미지 맨 위다."""
    u = (np.asarray(cols, float) + 0.5) * m['res']
    v = (m['H'] - 0.5 - np.asarray(rows, float)) * m['res']
    c, s = math.cos(m['oyaw']), math.sin(m['oyaw'])
    return np.c_[m['ox'] + c * u - s * v, m['oy'] + s * u + c * v]


def world_to_pix(m, xy):
    """map 좌표 → 픽셀 (행, 열) 실수."""
    xy = np.atleast_2d(xy)
    dx, dy = xy[:, 0] - m['ox'], xy[:, 1] - m['oy']
    c, s = math.cos(m['oyaw']), math.sin(m['oyaw'])
    u, v = c * dx + s * dy, -s * dx + c * dy
    return m['H'] - 0.5 - v / m['res'], u / m['res'] - 0.5


# ─────────────────────────────────────────────────────────────────────
#  트랙 영역과 중심선
# ─────────────────────────────────────────────────────────────────────
def disk(r_px):
    r = max(int(math.ceil(r_px)), 0)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return xx * xx + yy * yy <= r_px * r_px + 1e-9


def extract_ring(m, start_xy, seal_m, speck_m2=0.01):
    """출발점이 속한 빈 영역과 그 안쪽 섬을 고른다.

    ■ 틈 막기 (seal)
      빈 공간을 seal 만큼 깎은 상태에서 연결을 보고, 고른 영역을 다시 seal
      만큼 불려 원래 빈칸과 겹치는 곳만 살린다. 그러면 폭 2·seal 미만의 틈
      으로는 영역이 새지 않는다.
      ★ 닫힘(팽창→침식)으로 벽을 메우는 방법은 안 된다. 벽이 2픽셀(10 cm)
        처럼 얇으면 침식이 틈을 다시 연다 (합성 시험에서 겪었다).
      ★ 벽에 아주 가까운(대략 seal 안쪽) 장애물은 벽의 일부로 볼 수 있다.
        그때는 중심선이 그쪽에서 조금 밀린다. 폭·여유 검사에는 그대로 들어간다.

    반환 dict: region(빈 영역), inner(안쪽 섬), outer(그 밖 전부),
    islands(트랙 안 장애물 [(넓이 m², 중심 xy)]), start_px, err
    """
    blocked = ~m['free']
    k = disk(seal_m / m['res']) if seal_m > 0 else None
    if k is not None:
        free = ~ndimage.binary_dilation(blocked, structure=k, border_value=1)
    else:
        free = ~blocked
    lab, n = ndimage.label(free)            # 4-연결 — 대각선으로 새지 않게
    r, c = world_to_pix(m, start_xy)
    r0, c0 = int(round(r[0])), int(round(c[0]))
    out = dict(start_px=(r0, c0), err=None, free_sealed=free)
    if not (0 <= r0 < m['H'] and 0 <= c0 < m['W']):
        out['err'] = f'출발점 {start_xy} 이 맵 밖입니다'
        return out
    if lab[r0, c0] == 0:
        # 출발점이 벽/미지 칸이면 가장 가까운 빈칸으로
        d, (ir, ic) = ndimage.distance_transform_edt(lab == 0,
                                                     return_indices=True)
        if d[r0, c0] * m['res'] > 0.5:
            out['err'] = (f'출발점에서 0.5 m 안에 빈칸이 없습니다 '
                          f'({d[r0, c0] * m["res"]:.2f} m)')
            return out
        r0, c0 = int(ir[r0, c0]), int(ic[r0, c0])
        out['start_px'] = (r0, c0)
    region = lab == lab[r0, c0]
    if k is not None:
        grown = m['free'] & ndimage.binary_dilation(region, structure=k)
        lab2, _ = ndimage.label(grown | region)
        region = lab2 == lab2[r0, c0]

    # 영역 밖에서 이미지 테두리에 안 닿는 덩어리 = 영역이 감싼 구멍
    hole_lab, nh = ndimage.label(~region)
    border = set(np.unique(np.r_[hole_lab[0], hole_lab[-1],
                                 hole_lab[:, 0], hole_lab[:, -1]]))
    holes = [k for k in range(1, nh + 1) if k not in border]
    if not holes:
        out['err'] = ('출발점 영역이 고리가 아닙니다 — 감싼 섬이 없습니다. '
                      '벽 틈으로 바깥과 이어졌을 수 있습니다 (--seal 을 키우세요)')
        out['region'] = region
        return out
    sizes = ndimage.sum(np.ones_like(hole_lab), hole_lab, holes)
    inner_k = holes[int(np.argmax(sizes))]
    inner = hole_lab == inner_k
    islands = []
    filled = region.copy()
    for k, sz in zip(holes, sizes):
        if k == inner_k:
            continue
        mask = hole_lab == k
        filled |= mask                       # 중심선 계산에서는 빈칸 취급
        if float(sz) * m['res'] ** 2 < speck_m2 and not m['occ'][mask].any():
            continue                         # 미지 잡음 점 — 알리지 않는다
        rr, cc = np.nonzero(mask)
        islands.append((float(sz) * m['res'] ** 2,
                        pix_to_world(m, [rr.mean()], [cc.mean()])[0]))
    # 거리장은 실제 장애물 픽셀로 잰다. 영역 밖이라도 복원되지 않은 빈칸
    # (벽 옆 좁은 틈 등)까지 벽으로 치면 중심선이 그쪽으로 끌린다.
    walls = wall_mask(m, speck_m2)
    out.update(region=region, filled=filled,
               inner=inner & walls if (inner & walls).any() else inner,
               outer=~(filled | inner) & walls, islands=islands)
    return out


def center_contour(ring):
    """안쪽 섬과 바깥 벽까지 거리가 같은 닫힌 선 (픽셀 행, 열)."""
    d_in = ndimage.distance_transform_edt(~ring['inner'])
    d_out = ndimage.distance_transform_edt(~ring['outer'])
    f = np.where(ring['filled'], d_in - d_out, np.nan)
    fig = plt.figure()
    try:
        cs = fig.add_subplot(111).contour(f, levels=[0.0])
        segs = [s for s in cs.allsegs[0] if len(s) > 10]
    finally:
        plt.close(fig)
    if not segs:
        return None
    closed = [s for s in segs if np.hypot(*(s[0] - s[-1])) < 2.0]
    pool = closed or segs
    L = [np.sum(np.linalg.norm(np.diff(s, axis=0), axis=1)) for s in pool]
    s = pool[int(np.argmax(L))]
    if np.hypot(*(s[0] - s[-1])) < 1e-6:
        s = s[:-1]
    return np.c_[s[:, 1], s[:, 0]]           # (x=열, y=행) → (행, 열)


def orient(c, start_xy, start_yaw, reverse=False):
    """출발점에서 가장 가까운 점을 0 번으로, 출발 방향과 같은 쪽으로 돈다."""
    i0 = int(np.argmin(np.linalg.norm(c - np.asarray(start_xy), axis=1)))
    c = np.roll(c, -i0, axis=0)
    t = c[1] - c[-1]
    if t @ np.array([math.cos(start_yaw), math.sin(start_yaw)]) < 0:
        c = np.r_[c[:1], c[1:][::-1]]
    if reverse:
        c = np.r_[c[:1], c[1:][::-1]]
    return c


def resample_poly(c, step):
    """닫힌 폴리라인을 호길이 등간격으로."""
    P = np.vstack([c, c[:1]])
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    # ★ 길이를 정확히 n 등분한다. arange(0, L, step) 이면 마지막 점→첫 점 간격이 남는 자투리
    #   (1108_6 에서 1.1 cm, 나머지 5 cm)가 되어 최소 곡률선이 이음매에서 꺾였다 (R 0.54 m, 2026-11-08).
    n = max(3, int(round(s[-1] / step)))
    q = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.c_[np.interp(q, s, P[:, 0]), np.interp(q, s, P[:, 1])]


def normals(c):
    t = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-12)
    return np.c_[-t[:, 1], t[:, 0]]          # 왼쪽 법선


def min_curvature_line(m, blocked, c, margin, max_w, iters=4, reg=1e-3, smooth_w=0.0):
    """폭 안에서 가장 덜 굽은 닫힌 선.

    c + α·n (n 은 왼쪽 법선) 에서 α 를 [-(오른쪽 폭 - margin), 왼쪽 폭 - margin] 안에서
    골라 2차 차분 제곱합을 최소로 한다 (등간격이면 곡률 제곱합과 같다). 법선과 폭을 새
    선에서 다시 재며 몇 번 되풀이한다. 좁아서 범위가 없는 곳은 그 자리에 둔다.

    smooth_w 는 α 의 1차 차분에 거는 무게다. 0.3 으로 해 봤더니 1108_2 맵이 오히려
    실패해서 (선이 경계에서 덜 떨어짐) 기본은 끈다 (2026-11-08).
    """
    for _ in range(iters):
        N = len(c)
        n = normals(c)
        W = ray_widths(m, blocked, c, max_w)
        lo, hi = -(W[:, 0] - margin), W[:, 1] - margin
        narrow = lo > hi
        lo[narrow] = hi[narrow] = 0.0
        hi = np.maximum(hi, lo + 1e-9)
        D2 = sparse.diags([np.ones(N), -2 * np.ones(N), np.ones(N)], [-1, 0, 1],
                          shape=(N, N), format='lil')
        D2[0, N - 1] = D2[N - 1, 0] = 1.0
        D2 = D2.tocsr()
        D1 = sparse.diags([-np.ones(N), np.ones(N)], [0, 1], shape=(N, N), format='lil')
        D1[N - 1, 0] = 1.0
        A = sparse.vstack([D2 @ sparse.diags(n[:, 0]), D2 @ sparse.diags(n[:, 1]),
                           reg * sparse.identity(N), smooth_w * D1.tocsr()]).tocsr()
        b = -np.r_[D2 @ c[:, 0], D2 @ c[:, 1], np.zeros(N), np.zeros(N)]
        al = lsq_linear(A, b, bounds=(lo, hi), lsmr_tol='auto').x
        c = resample_poly(c + al[:, None] * n, 0.05)
    return c


def ray_widths(m, blocked, c, max_w):
    """각 점에서 좌/우 법선 방향으로 벽(blocked)까지 거리. (N,2) [오른쪽, 왼쪽]."""
    n = normals(c)
    step = m['res'] * 0.5
    ds = np.arange(step, max_w + step, step)
    out = np.empty((len(c), 2))
    for j, sgn in enumerate((-1.0, 1.0)):            # 오른쪽, 왼쪽
        P = c[:, None, :] + sgn * ds[None, :, None] * n[:, None, :]
        r, cc = world_to_pix(m, P.reshape(-1, 2))
        r = np.clip(np.round(r).astype(int), 0, m['H'] - 1)
        cc = np.clip(np.round(cc).astype(int), 0, m['W'] - 1)
        hit = blocked[r, cc].reshape(len(c), len(ds))
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(ds) - 1)
        out[:, j] = ds[first]
    return out


def wall_mask(m, speck_m2):
    """폭·여유 계산용 벽. 알 수 없는 칸 중 작은 덩어리는 빈칸으로 본다.

    ★ Cartographer 맵은 트랙 한가운데에도 회색(미지) 점이 흩어진다. 그걸
      벽으로 치면 여유가 몇 cm 로 잡혀 멀쩡한 트랙이 떨어진다 (ForzaETH
      GLC 맵에서 0.194 m 로 실패). 실제 점유 픽셀이 하나라도 있는 덩어리는
      크기와 상관없이 벽으로 둔다 — 가는 기둥을 지우면 안 된다.
    """
    blocked = ~m['free']
    lab, n = ndimage.label(blocked)
    if n == 0:
        return blocked
    idx = np.arange(1, n + 1)
    size = ndimage.sum(np.ones_like(lab), lab, idx) * m['res'] ** 2
    has_occ = ndimage.maximum(m['occ'], lab, idx) > 0
    drop = idx[(size < speck_m2) & ~has_occ]
    return blocked & ~np.isin(lab, drop)


def clearance(m, blocked, c):
    """각 점에서 가장 가까운 벽까지 거리 [m]."""
    d = ndimage.distance_transform_edt(~blocked) * m['res']
    r, cc = world_to_pix(m, c)
    return ndimage.map_coordinates(d, [r, cc], order=1, mode='nearest')


# ─────────────────────────────────────────────────────────────────────
def plot_result(m, ring, c, left, right, clr, margin, start_xy, start_yaw,
                path, title):
    dt.setup_font()
    fig, ax = plt.subplots(figsize=(9, 9))
    ext_xy = pix_to_world(m, [m['H'] - 0.5, -0.5], [-0.5, m['W'] - 0.5])
    if abs(m['oyaw']) < 1e-9:
        ax.imshow(m['img'], cmap='gray', vmin=0, vmax=255, origin='upper',
                  extent=[ext_xy[0, 0], ext_xy[1, 0], ext_xy[0, 1],
                          ext_xy[1, 1]])
    if ring.get('region') is not None:
        rr, cc = np.nonzero(ring['region'][::2, ::2])
        P = pix_to_world(m, rr * 2, cc * 2)
        ax.plot(P[:, 0], P[:, 1], ',', color='tab:cyan', alpha=0.3)
    if c is not None:
        ax.plot(*np.vstack([c, c[:1]]).T, '-', color='tab:blue', lw=1.5,
                label='중심선')
        ax.plot(*np.vstack([left, left[:1]]).T, '-', color='tab:green',
                lw=0.8, label='왼쪽 벽')
        ax.plot(*np.vstack([right, right[:1]]).T, '-', color='tab:orange',
                lw=0.8, label='오른쪽 벽')
        bad = clr < margin
        if bad.any():
            ax.plot(c[bad, 0], c[bad, 1], 'o', color='red', ms=3,
                    label=f'벽 여유 < {margin:.2f} m')
    ax.annotate('', xy=(start_xy[0] + 0.5 * math.cos(start_yaw),
                        start_xy[1] + 0.5 * math.sin(start_yaw)),
                xytext=start_xy,
                arrowprops=dict(arrowstyle='->', color='magenta', lw=2))
    for area, xy in ring.get('islands', []):
        ax.plot(*xy, 'x', color='red', ms=8)
    ax.set_aspect('equal')
    ax.set_title(title)
    ax.legend(loc='upper right', fontsize=8)
    if c is not None:
        lo, hi = c.min(axis=0) - 1.0, c.max(axis=0) + 1.0
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
    fig.savefig(path, dpi=110, bbox_inches='tight')
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description='라이다 맵 → 트랙 (map 프레임)',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument('map', help='맵 yaml 경로 또는 maps/ 안의 이름')
    p.add_argument('name', nargs='?', default=None,
                   help='새 트랙 이름 (기본 <맵>_track)')
    p.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0],
                   metavar=('X', 'Y'),
                   help='출발점 (map 좌표). 기본 0 0 = 맵핑을 시작한 자리')
    p.add_argument('--start-yaw', type=float, default=0.0,
                   help='출발 방향 [rad]. 기본 0 = 맵핑을 시작할 때 차의 방향')
    p.add_argument('--reverse', action='store_true', help='반대 방향으로 돈다')
    p.add_argument('--seal', type=float, default=0.10,
                   help='벽 틈 막기 m — 폭 2·seal 미만 틈으로는 새지 않는다. '
                        '벽에 아주 붙은 장애물은 벽으로 볼 수 있다')
    p.add_argument('--speck', type=float, default=0.01,
                   help='이보다 작은 미지(회색) 덩어리는 빈칸으로 본다 m² '
                        '(점유 픽셀이 있으면 크기와 무관하게 벽)')
    p.add_argument('--margin', type=float, default=0.20,
                   help='중심선에서 벽까지 최소 여유 m (차 반폭 + 여유)')
    p.add_argument('--max-width', type=float, default=2.0,
                   help='한쪽 폭 상한 m. 넘으면 벽이 덜 보인 곳이다')
    p.add_argument('--spacing', type=float, default=0.05)
    p.add_argument('--raw-spacing', type=float, default=0.01)
    p.add_argument('--smooth', type=float, default=0.01,
                   help='초기 평활화 m. 맵 중심선은 사람 주행보다 깨끗해서 작게 '
                        '시작한다 (합성 시험: 0.01 → 정답에서 최대 2.4 cm, 0.03 → 5.3 cm)')
    p.add_argument('--wheelbase', type=float, default=0.33)
    p.add_argument('--max-steer', type=float, default=0.378,
                   help='실측 최대 조향 (11-07 확인 0.378 rad)')
    p.add_argument('--max-bad-run', type=float, default=0.15)
    p.add_argument('--hypermpc', default='~/hypermpc_code')
    p.add_argument('--allow-infeasible', action='store_true',
                   help='중심선 곡률이 차 한계를 넘어도 저장한다 (경고만). '
                        '좁은 헤어핀은 MPC 가 폭을 써서 돌아야 한다')
    p.add_argument('--mincurv', action='store_true',
                   help='가운데 선 대신 폭 안에서 가장 덜 굽은 선을 쓴다 (주석 참고)')
    p.add_argument('--line-margin', type=float, default=0.40,
                   help='--mincurv 선에서 벽까지 최소 여유 m. 차 반폭 0.135 + MPC 가 '
                        '선에서 벗어나는 0.25 정도')
    p.add_argument('--no-author-prep', action='store_true',
                   help='저자 전처리를 안 거친다. 그 전처리는 코너 반경을 0.72~0.85 배로 줄인다 '
                        '(hypermpc/README.md). 대신 우리 선을 같은 형식으로 prep_<이름>.csv 에 쓴다')
    p.add_argument('--walls', action='store_true',
                   help='MPCC 용 (기본, maptrack). 선은 좌표계일 뿐이라 차 한계 곡률 검사를 안 하고, '
                        '매끄러운 가운데 선 + 지점별 좌/우 벽 폭을 prep 에 쓴다. MPC 가 벽 안에서 선을 고른다')
    p.add_argument('--ref-radius', type=float, default=0.5,
                   help='--walls: 기준선 최소 반경 m. 이보다 급하면 평활을 더 한다 (잡음 꺾임 방지)')
    p.add_argument('--frenet-frac', type=float, default=0.9,
                   help='--walls: 코너 안쪽 폭을 이 비율 × 반경으로 자른다 (1 - n·κ > 0 특이점)')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    if a.walls:
        a.no_author_prep, a.mincurv = True, False

    m = load_map(a.map)
    base = os.path.splitext(os.path.basename(m['path']))[0]
    name = a.name or f'{base}_track'
    kappa_lim = math.tan(a.max_steer) / a.wheelbase
    if a.walls:
        # ★ MPCC 에서 선은 좌표계다. 차가 그 선을 그대로 돌 필요가 없다 — 매끄럽기만 하면 된다.
        kappa_lim = 1.0 / a.ref_radius
    print(f'■ 맵  {m["path"]}')
    print(f'   {m["W"]}x{m["H"]} 칸, {m["res"]*100:.1f} cm, 원점 '
          f'({m["ox"]:+.2f}, {m["oy"]:+.2f}, {m["oyaw"]:+.3f})  '
          f'빈칸 {m["free"].mean()*100:.1f}%  벽 {m["occ"].mean()*100:.1f}%')

    ring = extract_ring(m, a.start, a.seal, a.speck)
    png = (os.path.join('/tmp', f'track_from_map_{name}.png') if a.dry_run
           else os.path.join(dt.OUTDIR, name, 'map_track.png'))
    os.makedirs(os.path.dirname(png), exist_ok=True)
    if ring['err']:
        print(f'\n✗ {ring["err"]}')
        plot_result(m, ring, None, None, None, None, a.margin, a.start,
                    a.start_yaw, png, f'{name} — 실패')
        print(f'  그림: {png}')
        sys.exit(1)
    area = ring['region'].sum() * m['res'] ** 2
    print(f'\n■ 트랙 영역  {area:.1f} m²  안쪽 섬 '
          f'{ring["inner"].sum() * m["res"]**2:.1f} m²')
    for ar, xy in ring['islands']:
        print(f'   ⚠ 트랙 안 장애물 {ar*1e4:.0f} cm² @ ({xy[0]:+.2f}, '
              f'{xy[1]:+.2f}) — 중심선은 무시, 폭·여유 검사에는 넣음')

    cpx = center_contour(ring)
    if cpx is None:
        print('\n✗ 중심선을 못 뽑았습니다')
        sys.exit(1)
    c0 = pix_to_world(m, cpx[:, 0], cpx[:, 1])
    c0 = orient(resample_poly(c0, 0.05), a.start, a.start_yaw, a.reverse)
    L0 = float(np.sum(np.linalg.norm(np.diff(np.vstack([c0, c0[:1]]),
                                             axis=0), axis=1)))
    print(f'   중심선 원본 {len(c0)}점, 길이 {L0:.2f} m')

    blocked = wall_mask(m, a.speck)         # 폭·여유는 메우기 전 실제 벽으로
    if a.mincurv:
        c0 = orient(min_curvature_line(m, blocked, c0, a.line_margin, a.max_width),
                    a.start, a.start_yaw, a.reverse)
        L1 = float(np.sum(np.linalg.norm(np.diff(np.vstack([c0, c0[:1]]),
                                                 axis=0), axis=1)))
        k1 = np.abs(signed_curvature(c0, 0.05))
        print(f'   최소 곡률선 {len(c0)}점, 길이 {L1:.2f} m, 벽 여유 최소 '
              f'{clearance(m, blocked, c0).min():.3f} m, 평활 전 R_min {1/k1.max():.3f} m '
              f'(벽 여유 목표 {a.line_margin:.2f} m)')
    TR = None if a.no_author_prep else dt.load_track_reader(a.hypermpc)
    if a.no_author_prep:
        print('   ★ 저자 전처리를 안 거칩니다 — prep_*.csv 를 이 선으로 직접 씁니다')
    elif TR is None:
        print('   ⚠ 원저장소가 없어 HyperMPC 전처리 검사를 건너뜁니다')

    print(f'\n■ 평활화 — 통과하는 최소값을 찾습니다 '
          f'(차 한계 R {1/kappa_lim:.3f} m, 벽 여유 {a.margin:.2f} m)')
    best = None
    for dev in np.arange(a.smooth, 0.401, 0.01):
        r = dt.fit_closed_spline(c0, float(dev), a.spacing)
        if r is None:
            continue
        c, kap, total, tck = r
        wind = dt.winding(kap, a.spacing)
        if abs(abs(wind) - 1.0) > 0.15:
            print(f'   붕괴 평활 {dev:.2f}  ∮κds/2π {wind:+.2f}')
            continue
        ak = np.abs(kap)
        clr = clearance(m, blocked, c)
        bad_run = dt.longest_run(ak > kappa_lim, a.spacing)
        rmse = float(np.sqrt(np.mean(dist_to_polyline(c0, c) ** 2)))
        prep, prun = None, 0.0
        if TR is not None and clr.min() >= a.margin and \
                (a.allow_infeasible or bad_run <= a.max_bad_run):
            seg, uu, T = dt.arclen_table(tck)
            craw, _ = dt.resample(tck, seg, uu, T, a.raw_spacing)
            craw = orient(craw, a.start, a.start_yaw, a.reverse)
            Wr = ray_widths(m, blocked, craw, a.max_width)
            prep = dt.run_prep(TR, craw, Wr, a.raw_spacing)
            if 'err' in prep:
                prep = None
            else:
                pk = np.abs(prep['kappa'])
                prep['bad'] = float((pk > kappa_lim).mean())
                prep['bad_run'] = dt.longest_run(pk > kappa_lim,
                                                 prep['spacing'])
                prun = prep['bad_run']
        curv_ok = a.allow_infeasible or (
            bad_run <= a.max_bad_run
            and (prep is None or prun <= a.max_bad_run))
        ok = curv_ok and clr.min() >= a.margin
        print(f'   {"✓" if ok else "  "} 평활 {dev:.2f}  R_min {1/ak.max():6.3f}  '
              f'최장불가 {bad_run*100:3.0f}cm  벽여유 최소 {clr.min():.3f} m  '
              f'원본편차 {rmse:.3f} m'
              + (f'  전처리 {prep["r_min"]:.3f}/{prun*100:.0f}cm'
                 if prep else ''))
        if ok:
            best = (float(dev), tck, rmse, prep)
            break
        if clr.min() < a.margin and dev >= a.smooth + 0.05:
            # 평활을 올려도 좁은 곳은 안 넓어진다 — 더 돌 이유가 없다
            q = c[int(np.argmin(clr))]
            print(f'   ✗ 벽 여유 {clr.min():.3f} m < {a.margin:.2f} m '
                  f'@ ({q[0]:+.2f}, {q[1]:+.2f}) — 평활을 올리자 선이 벽 쪽으로 '
                  '당겨졌습니다. 그 근처가 차 한계보다 급한 코너거나 맵 잡음입니다')
            break
    if best is None:
        print('\n✗ 통과하는 평활화를 못 찾았습니다. 그림을 보세요.')
        print('  곡률만 걸렸다면 트랙이 차 한계보다 급하게 굽은 것입니다.')
        print('  폭이 넉넉하면 --allow-infeasible 로 저장하고 MPC 가 폭을 쓰게 하세요.')
        r = dt.fit_closed_spline(c0, a.smooth, a.spacing)
        if r is not None:
            c = orient(r[0], a.start, a.start_yaw, a.reverse)
            W = ray_widths(m, blocked, c, a.max_width)
            n = normals(c)
            plot_result(m, ring, c, c + W[:, 1:] * n, c - W[:, :1] * n,
                        clearance(m, blocked, c), a.margin, a.start,
                        a.start_yaw, png, f'{name} — 실패')
            print(f'  그림: {png}')
        sys.exit(1)

    dev, tck, rmse, prep = best
    seg, uu, T = dt.arclen_table(tck)
    c, _ = dt.resample(tck, seg, uu, T, a.spacing)
    craw, _ = dt.resample(tck, seg, uu, T, a.raw_spacing)
    # 시작점·방향을 맞춘 뒤 곡률을 그 순서로 다시 낸다 (방향이 바뀌면 부호도 바뀐다)
    c = orient(c, a.start, a.start_yaw, a.reverse)
    craw = orient(craw, a.start, a.start_yaw, a.reverse)
    kap = signed_curvature(c, a.spacing)

    ak = np.abs(kap)
    W = ray_widths(m, blocked, c, a.max_width)
    Wraw = ray_widths(m, blocked, craw, a.max_width)
    nrm = normals(c)
    left, right = c + W[:, 1:] * nrm, c - W[:, :1] * nrm
    clr = clearance(m, blocked, c)
    total = float(T)
    wide = (W >= a.max_width - m['res']).any(axis=1)
    bad_m = ak > kappa_lim
    print(f'\n■ 결과   평활 {dev:.2f} m')
    print(f'   길이 {total:.2f} m · {len(c)}점 · 방향 '
          f'{"CCW (왼쪽으로 돈다)" if dt.winding(kap, a.spacing) > 0 else "CW (오른쪽으로 돈다)"}')
    print(f'   R_min {1/ak.max():.3f} m  (차 한계 {1/kappa_lim:.3f})')
    if bad_m.any():
        print(f'   ⚠ 중심선이 차 한계를 넘는 곳 {bad_m.mean()*100:.1f}%, '
              f'최장 {dt.longest_run(bad_m, a.spacing):.2f} m — 중심선 그대로는 '
              '못 돈다. MPC 가 폭을 써서 돌 수 있는지 확인하라')
    print(f'   폭 오른쪽 {W[:,0].min():.2f}~{W[:,0].max():.2f} m, '
          f'왼쪽 {W[:,1].min():.2f}~{W[:,1].max():.2f} m, '
          f'벽 여유 최소 {clr.min():.3f} m')
    print(f'   원본 중심선에서 벗어남 {rmse:.3f} m')
    if wide.any():
        print(f'   ⚠ 폭이 상한 {a.max_width} m 에 걸린 점 {wide.sum()}개 — '
              '벽이 덜 보였거나 트랙 밖이 열려 있습니다. 그림을 확인하세요')
    if prep:
        print(f'   HyperMPC 전처리 후 R_min {prep["r_min"]:.3f} m '
              f'({prep["n"]}점, RMSE {prep["rmse"]:.4f} m)')

    plot_result(m, ring, c, left, right, clr, a.margin, a.start,
                a.start_yaw, png, f'{name} — 평활 {dev:.2f} m, '
                f'R_min {1/ak.max():.2f} m')
    print(f'   그림: {png}')
    if a.dry_run:
        print('\n(--dry-run — 저장하지 않습니다)')
        return

    hw = float(np.median(W))
    stats = dict(
        total=total, n=len(c), k_max=float(ak.max()),
        r_min=float(1 / ak.max()), bad=float(bad_m.mean()),
        bad_n=int(bad_m.sum()),
        bad_run=dt.longest_run(bad_m, a.spacing),
        warn=float((ak > 1.0).mean()), inside=True, clear=float(clr.min()),
        fold=False, fold_n=0, fold_run=0.0,
        wind=dt.winding(kap, a.spacing), area=dt.shoelace(c),
        area_click=dt.shoelace(c), degenerate=False)
    frame = {
        'name': 'map', 'unit': 'meter',
        'note': 'Cartographer map 프레임 (맵핑 시작 자세가 원점). '
                'mocap 과 비교하려면 tools/map_mocap_align.py',
        'map_yaml': os.path.abspath(m['path']),
    }
    dt.write_track(
        name, c, kap, left, right, craw, hw, stats, prep,
        dict(source=f'tools/track_from_map.py (맵 {os.path.abspath(m["path"])}'
                    f', 출발 {a.start} yaw {a.start_yaw})',
             spline='scipy.interpolate.splprep(k=3, per=1) — 근사 평활',
             smooth_dev=dev, clicked=[], boundary=None,
             wheelbase=a.wheelbase, max_steer=a.max_steer,
             kappa_lim=kappa_lim, max_bad_run=a.max_bad_run),
        spacing=a.spacing, raw_spacing=a.raw_spacing,
        widths=Wraw, frame=frame)
    if a.no_author_prep:
        _MAP.update(m=m, blocked=blocked, max_w=a.max_width)
        write_own_prep(name, c, kap, W, a.spacing,
                       frenet_frac=a.frenet_frac if a.walls else None)
    print('   ★ 위 "벽~경계" 는 여기서 중심선에서 가장 가까운 벽까지 거리입니다')
    print(f'   ★ map 프레임 트랙입니다. hmpc 는 위치추정(map→base_link)으로 '
          '달려야 맞습니다')


def write_own_prep(name, c, kap, W, spacing, frenet_frac=None):
    """저자 prep 형식 (s,x,y,heading,curvature,track_width) 을 우리 선으로 쓴다.

    ★ 시작점을 옮기면서 이음매에 아주 가까운 점이 하나 생긴다 (간격 0.4 cm 를 봤다).
      그대로 쓰면 그 자리 곡률이 엉뚱하게 나와서, 등간격으로 다시 뽑고 곡률도 다시 낸다.
    """
    keep = np.r_[True, np.linalg.norm(np.diff(c, axis=0), axis=1) > 0.5 * spacing]
    if np.linalg.norm(c[-1] - c[0]) < 0.5 * spacing:
        keep[-1] = False
    c = resample_poly(c[keep], spacing)
    kap = signed_curvature(c, spacing)
    W = ray_widths(_MAP['m'], _MAP['blocked'], c, _MAP['max_w'])
    t = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    head = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))]
    # 벽 폭 [오른쪽, 왼쪽]. ★ 이웃 ±15 cm 중 최소로 깎는다 — 벽 틈으로 레이가 새면 그 점만
    #   폭이 크게 나오는데, MPC 에는 그게 "벽이 멀다" 로 보인다. 깎는 쪽이 안전하다.
    win = max(1, int(round(0.15 / spacing)))
    Wc = np.stack([ndimage.minimum_filter1d(W[:, j], 2 * win + 1, mode='wrap') for j in (0, 1)], axis=1)
    if frenet_frac is not None:
        # 1 - n·κ > 0: 왼쪽으로 도는 곳(κ>0)은 왼쪽 폭, 오른쪽으로 도는 곳은 오른쪽 폭을 반경 이내로
        lim = frenet_frac / np.maximum(np.abs(kap), 1e-6)
        cut_l = (kap > 0) & (Wc[:, 1] > lim)
        cut_r = (kap < 0) & (Wc[:, 0] > lim)
        Wc[cut_l, 1] = lim[cut_l]
        Wc[cut_r, 0] = lim[cut_r]
        if cut_l.any() or cut_r.any():
            print(f'   코너 안쪽 폭을 {frenet_frac:.1f}·R 로 자름: {int(cut_l.sum() + cut_r.sum())}점 '
                  f'(최소 {min(Wc[cut_l, 1].min() if cut_l.any() else 9, Wc[cut_r, 0].min() if cut_r.any() else 9):.2f} m)')
    path = os.path.join(dt.OUTDIR, name, f'prep_{name}.csv')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('s,x,y,heading,curvature,track_width,w_right,w_left\n')
        for i in range(len(c)):
            f.write(f'{s[i]},{c[i, 0]},{c[i, 1]},{head[i]},{kap[i]},{W[i, 0] + W[i, 1]},'
                    f'{Wc[i, 0]},{Wc[i, 1]}\n')
    print(f'   벽 폭 (깎은 값) 오른쪽 {Wc[:, 0].min():.2f}~{Wc[:, 0].max():.2f} m, '
          f'왼쪽 {Wc[:, 1].min():.2f}~{Wc[:, 1].max():.2f} m')
    print(f'   ✓ {path}  (저자 전처리 없이, {spacing*100:.0f} cm 간격, '
          f'R_min {1/np.abs(kap).max():.3f} m)')


def signed_curvature(c, spacing):
    """닫힌 등간격 점열의 부호 있는 곡률 (중앙 차분)."""
    d1 = (np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)) / (2 * spacing)
    d2 = (np.roll(c, -1, axis=0) - 2 * c + np.roll(c, 1, axis=0)) / spacing ** 2
    sp = np.maximum(np.linalg.norm(d1, axis=1), 1e-12)
    return (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / sp ** 3


if __name__ == '__main__':
    main()
