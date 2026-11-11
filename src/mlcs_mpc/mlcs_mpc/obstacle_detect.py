"""
obstacle_detect.py — 라이다 스캔에서 '맵에 없는 것' 을 찾아 장애물 위치로 (ROS 없음, 시험용으로 분리).

  1. 저장한 맵 (maps/<맵>.pgm) 에서 벽(점유 + 미탐색) 까지 거리장을 만든다.
     ★ Cartographer 의 /map 은 쓰지 않는다 — 주행 중 본 장애물이 지도에 그려져 들어갈 수 있다.
  2. 스캔 점을 map 좌표로 옮겨, 벽에서 wall_clear + clear_per_m × 거리 넘게 떨어진 점만 남긴다
     (= 맵에 없던 것). 거리 항은 yaw 오차 몫 — 먼 벽일수록 yaw 오차만큼 옆으로 밀려 보인다.
  3. 트랙 안 (지점별 벽 폭 안쪽) 에 있는 점만 남긴다 — 트랙 밖 사람·의자는 무시.
  4. 스캔 순서로 이웃 점을 묶고 (간격 cluster_gap), 점 수·크기로 거른다.
  5. 보이는 면의 중심에서 라이다 반대쪽으로 depth_shift 만큼 민 곳을 장애물 중심으로 본다
     (라이다는 앞면만 본다).
  0. (refine) 그 전에 스캔 전체를 맵 벽 거리장에 맞춰 자세를 국소 보정한다 (refine_pose).
     Cartographer 가 30 cm / 8° 틀려도 (1109_4, 2026-11-09) 벽은 맵 벽에 겹치고 장애물만 남는다.
     장애물 점은 소수라 맞춤을 끌고 가지 못한다 (점수는 벽에 붙은 점 수).

  위치추정이 틀리면 진짜 벽이 지도와 어긋나 '맵에 없는 것' 으로 보인다 — wall_clear 가 그 여유다.
"""
import math
import os
from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class DetectParams:
    wall_clear: float = 0.12    # 맵 벽에서 이만큼 넘게 떨어져야 장애물 후보 m (위치추정 오차 여유)
    clear_per_m: float = 0.04   # + 라이다에서 1 m 멀어질 때마다 이만큼 더 (yaw 오차 약 2.3° 몫) m/m
    track_inset: float = 0.05   # 트랙 벽 폭에서 이만큼 안쪽만 m
    max_range: float = 4.0      # 이보다 먼 점은 무시 m
    fov: float = math.radians(120.0)    # 앞쪽 ± 이 각도 안만 (라이다 기준) rad
    cluster_gap: float = 0.12   # 이웃 점 간격이 이보다 크면 다른 물체 m
    min_points: int = 3
    ghost_radius: float = 0.20  # 맵별 헛것 자리 (maps/<맵>_ghost.csv) 에서 이 안의 점은 버린다 m
    max_size: float = 0.8       # 묶음의 대각 크기가 이보다 크면 버림 (벽 조각 등) m
    depth_shift: float = 0.10   # 보이는 면 → 중심까지 m (상자 깊이의 절반)
    refine: bool = True         # 스캔을 맵 벽에 맞춰 자세 국소 보정
    refine_xy: float = 0.30     # 보정 탐색 범위 ± m (지난 보정값 둘레)
    refine_yaw: float = math.radians(8.0)   # ± rad
    refine_sigma: float = 0.05  # 점수 exp(−(d/σ)²) 의 σ m
    refine_points: int = 240    # 맞춤에 쓰는 스캔 점 수 (전 시야에서 고르게)
    refine_range: float = 8.0   # 맞춤에 쓰는 점의 최대 거리 m
    refine_prior: float = 0.05  # 탐색 시작점에서 refine_xy (또는 refine_yaw) 만큼 가면 점의 이 비율만큼 깎는다.
                                #   긴 직선 통로는 앞뒤로 밀어도 점수가 같다 — 같으면 제자리에 머물게


class WallField:
    """map_server 형식 yaml + pgm → 벽까지 거리 (m) 를 좌표로 조회."""

    def __init__(self, yaml_path):
        import yaml
        from PIL import Image
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        img = m['image'] if os.path.isabs(m['image']) else \
            os.path.join(os.path.dirname(yaml_path), m['image'])
        a = np.asarray(Image.open(img), dtype=np.float32) / 255.0
        if int(m.get('negate', 0)):
            a = 1.0 - a
        occ_p = 1.0 - a                         # map_server: 검을수록 점유
        free = occ_p < float(m.get('free_thresh', 0.196))
        wall = ~free                            # 점유 + 미탐색 = 벽으로 본다
        self.res = float(m['resolution'])
        self.ox, self.oy = float(m['origin'][0]), float(m['origin'][1])
        # 이미지 위쪽이 +y 끝 → 행을 뒤집어 [row=y 증가, col=x 증가]
        self.d = (distance_transform_edt(~wall[::-1]) * self.res).astype(np.float32)
        # 맞춤용: 점유 칸 (검은 벽선) 까지 거리. 미탐색까지 벽으로 보면 점을 벽 바깥 미탐색 쪽으로
        #   밀어 넣어도 만점이라 맞춤이 참 자세에서 달아난다 (합성 시험, 2026-11-09).
        occ = occ_p > float(m.get('occupied_thresh', 0.65))
        #   칸 중심이 아니라 칸 가장자리까지 (반 칸 뺌) — 스캔은 벽 칸의 가장자리에 닿는다. 중심 거리면
        #   모든 점이 한 칸 (5 cm) 떨어져 보여 맞춤이 점을 벽 안으로 밀어 넣으려 엉뚱하게 움직인다.
        self.d_occ = np.maximum(distance_transform_edt(~occ[::-1]) * self.res - 0.5 * self.res,
                                0.0).astype(np.float32)
        self.h, self.w = self.d.shape

    def dist(self, x, y, occ_only=False):
        ix = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        iy = np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        if occ_only:
            out = np.full(ix.shape, 9.9, np.float32)   # 지도 밖 = 먼 곳 (맞춤 점수 0)
            out[ok] = self.d_occ[iy[ok], ix[ok]]
            return out
        out = np.zeros(ix.shape, np.float32)       # 지도 밖 = 벽
        out[ok] = self.d[iy[ok], ix[ok]]
        return out


class TrackBand:
    """prep 트랙 (Track 객체) → 점이 트랙 안쪽인지, 그 지점의 heading.

    가장 가까운 트랙 점은 격자표 (res 간격) 로 미리 구해 둔다 — 점마다 전 트랙과 거리를 재면
    실차 스캔 (후보 수백 점) 에서 7~18 ms 걸렸다 (2026-11-09)."""

    def __init__(self, track, res=0.05, pad=1.5):
        from scipy.spatial import cKDTree
        self.x, self.y, self.h = track.x, track.y, track.heading
        self.wl, self.wr = track.w_left, track.w_right
        self.res = res
        self.ox, self.oy = self.x.min() - pad, self.y.min() - pad
        nx = int((self.x.max() + pad - self.ox) / res) + 1
        ny = int((self.y.max() + pad - self.oy) / res) + 1
        gx, gy = np.meshgrid(self.ox + (np.arange(nx) + 0.5) * res, self.oy + (np.arange(ny) + 0.5) * res)
        _, idx = cKDTree(np.c_[self.x, self.y]).query(np.c_[gx.ravel(), gy.ravel()])
        self.grid = idx.reshape(ny, nx).astype(np.int32)
        self.tree = cKDTree(np.c_[self.x, self.y])

    def nearest(self, px, py):
        ix = np.floor((px - self.ox) / self.res).astype(int)
        iy = np.floor((py - self.oy) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.grid.shape[1]) & (iy >= 0) & (iy < self.grid.shape[0])
        out = np.empty(len(px), np.int64)
        out[ok] = self.grid[iy[ok], ix[ok]]
        if (~ok).any():
            out[~ok] = self.tree.query(np.c_[px[~ok], py[~ok]])[1]
        return out

    def inside(self, px, py, inset):
        i = self.nearest(px, py)
        h = self.h[i]
        n = -np.sin(h) * (px - self.x[i]) + np.cos(h) * (py - self.y[i])
        return (n < self.wl[i] - inset) & (n > -(self.wr[i] - inset)), i


def scan_base(ranges, angle_min, angle_inc, laser, max_range, fov=None):
    """LaserScan → base_link 좌표 점 (P,2) 와 각 점의 각도 (스캔 순서 유지)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.05) & (r < max_range)
    if fov is not None:
        ok &= np.abs(a) < fov
    r, a = r[ok], a[ok]
    lx, ly, lw = laser
    return np.stack([lx + r * np.cos(a + lw), ly + r * np.sin(a + lw)], axis=1)


def _score(walls, pb, x, y, yaws, dxy, sig):
    """yaws (Y,) × dxy (G,2) 자세마다 점수. pb = base 점 (P,2). → (Y, G)."""
    out = np.empty((len(yaws), len(dxy)))
    for i, w in enumerate(yaws):
        c, s = math.cos(w), math.sin(w)
        px = x + c * pb[:, 0] - s * pb[:, 1]
        py = y + s * pb[:, 0] + c * pb[:, 1]
        d = walls.dist((px[None, :] + dxy[:, 0:1]).ravel(), (py[None, :] + dxy[:, 1:2]).ravel(), occ_only=True)
        out[i] = np.exp(-(d.reshape(len(dxy), -1) / sig) ** 2).sum(1)
    return out


def refine_pose(pb, pose, walls, p, corr=(0.0, 0.0, 0.0), coarse=True):
    """base 점 pb 를 맵 벽에 맞춰 자세 보정. corr = 지난 보정 (dx, dy, dyaw) — 그 둘레를 찾는다.
    → (보정한 자세, 새 corr, 벽에 붙은 점 비율 (보정 뒤), 보정 전 비율)."""
    if len(pb) > p.refine_points:
        pb = pb[np.linspace(0, len(pb) - 1, p.refine_points).astype(int)]
    x0, y0, w0 = pose[0] + corr[0], pose[1] + corr[1], pose[2] + corr[2]
    best = (0.0, 0.0, 0.0)
    # coarse = 넓게 (±refine_xy/±refine_yaw, 0.1 m/2°) 찾은 뒤 좁게. 아니면 지난 보정 둘레만 좁게 (빠름).
    stages = [(0.075, 0.025, math.radians(1.5), math.radians(0.5))]
    if coarse:
        stages.insert(0, (p.refine_xy, 0.10, p.refine_yaw, math.radians(2.0)))
    for span_xy, step_xy, span_w, step_w in stages:
        g = np.arange(-span_xy, span_xy + 1e-9, step_xy)
        dxy = np.stack(np.meshgrid(g, g), -1).reshape(-1, 2) + np.array(best[:2])
        yaws = w0 + best[2] + np.arange(-span_w, span_w + 1e-9, step_w)
        sc = _score(walls, pb, x0, y0, yaws, dxy, p.refine_sigma)
        pen = p.refine_prior * len(pb) * (((dxy ** 2).sum(1)[None, :] / p.refine_xy ** 2)
                                         + ((yaws - w0)[:, None] / p.refine_yaw) ** 2)
        i, j = np.unravel_index(np.argmax(sc - pen), sc.shape)
        best = (float(dxy[j, 0]), float(dxy[j, 1]), float(yaws[i] - w0))
    new = (corr[0] + best[0], corr[1] + best[1], corr[2] + best[2])
    # 보정이 범위 끝에 붙으면 (못 맞춤) 지난 값을 줄여 간다 — 엉뚱한 곳으로 달아나지 않게
    lim = 2.0 * p.refine_xy
    if math.hypot(new[0], new[1]) > lim or abs(new[2]) > 2.0 * p.refine_yaw:
        new = (corr[0] * 0.5, corr[1] * 0.5, corr[2] * 0.5)
    fit = lambda q: float(np.mean(walls.dist(*_xy(pb, q), occ_only=True) < 2 * p.refine_sigma))  # noqa: E731
    q = (pose[0] + new[0], pose[1] + new[1], pose[2] + new[2])
    return q, new, fit(q), fit(pose)


def _xy(pb, q):
    c, s = math.cos(q[2]), math.sin(q[2])
    return q[0] + c * pb[:, 0] - s * pb[:, 1], q[1] + s * pb[:, 0] + c * pb[:, 1]


def scan_to_map(ranges, angle_min, angle_inc, pose, laser, p):
    """LaserScan 값 → (map 점 (P,2), 라이다 map 위치). pose = base_link (x, y, yaw), laser = (lx, ly, lyaw)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.05) & (r < p.max_range) & (np.abs(a) < p.fov)
    r, a = r[ok], a[ok]
    x, y, yaw = pose
    lx, ly, lw = laser
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy = x + c * lx - s * ly, y + s * lx + c * ly      # 라이다 원점 (map)
    ang = yaw + lw + a
    return np.stack([ox + r * np.cos(ang), oy + r * np.sin(ang)], axis=1), (ox, oy)


def detect_scan(ranges, angle_min, angle_inc, pose, laser, walls, band, p, corr=(0.0, 0.0, 0.0), coarse=True):
    """스캔 하나 → (장애물 목록, 쓴 자세, 새 corr, 맞춤 비율 (뒤, 앞)). refine 이 꺼져 있으면 자세 그대로."""
    fit = (math.nan, math.nan)
    if p.refine:
        pb = scan_base(ranges, angle_min, angle_inc, laser, p.refine_range)
        if len(pb) >= 30:
            pose, corr, f1, f0 = refine_pose(pb, pose, walls, p, corr, coarse)
            fit = (f1, f0)
    pts, org = scan_to_map(ranges, angle_min, angle_inc, pose, laser, p)
    return detect(pts, org, walls, band, p), pose, corr, fit


def detect(pts, origin, walls, band, p):
    """→ [(cx, cy, 크기, 점 수, heading)] 가까운 순. pts 는 스캔 순서 그대로여야 한다."""
    if len(pts) == 0:
        return []
    rng = np.hypot(pts[:, 0] - origin[0], pts[:, 1] - origin[1])
    cand = walls.dist(pts[:, 0], pts[:, 1]) > p.wall_clear + p.clear_per_m * rng
    # ★ 2026-11-11 실트랙: 빈 트랙에서도 같은 자리 (벽 10~40 cm 옆) 가 주행마다 장애물로 잡혀 길을 막았다.
    #   빈 트랙 기록에서 뽑은 헛것 자리 (tools/ghost_mask.py) 근처 점은 후보에서 뺀다. 상대는 움직이니 몇 군데쯤은 괜찮다.
    gh = getattr(walls, 'ghost', None)
    if gh is not None and len(gh):
        dg = np.min(np.hypot(pts[:, 0:1] - gh[None, :, 0], pts[:, 1:2] - gh[None, :, 1]), axis=1)
        cand &= dg > p.ghost_radius
    idx = np.where(cand)[0]
    if len(idx) == 0:
        return []
    ins, near = band.inside(pts[idx, 0], pts[idx, 1], p.track_inset)
    idx, near = idx[ins], near[ins]
    if len(idx) == 0:
        return []
    # 스캔 순서로 묶기 — 이웃 '후보' 점끼리 거리로 (사이에 벽 점이 끼면 끊긴다)
    groups, cur = [], [0]
    for k in range(1, len(idx)):
        a, b = pts[idx[k - 1]], pts[idx[k]]
        if idx[k] - idx[k - 1] <= 2 and math.hypot(*(b - a)) < p.cluster_gap:
            cur.append(k)
        else:
            groups.append(cur)
            cur = [k]
    groups.append(cur)
    out = []
    for g in groups:
        if len(g) < p.min_points:
            continue
        q = pts[idx[g]]
        size = float(np.hypot(*(q.max(0) - q.min(0))))
        if size > p.max_size:
            continue
        c = q.mean(0)
        v = c - np.asarray(origin)
        dist = float(np.hypot(*v))
        c = c + p.depth_shift * v / max(dist, 1e-6)
        out.append((float(c[0]), float(c[1]), size, len(g), float(band.h[near[g[len(g) // 2]]]), dist))
    out.sort(key=lambda o: o[5])
    return [o[:5] for o in out]
