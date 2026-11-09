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
    max_size: float = 0.8       # 묶음의 대각 크기가 이보다 크면 버림 (벽 조각 등) m
    depth_shift: float = 0.10   # 보이는 면 → 중심까지 m (상자 깊이의 절반)


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
        self.h, self.w = self.d.shape

    def dist(self, x, y):
        ix = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        iy = np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        out = np.zeros(ix.shape, np.float32)       # 지도 밖 = 벽
        out[ok] = self.d[iy[ok], ix[ok]]
        return out


class TrackBand:
    """prep 트랙 (Track 객체) → 점이 트랙 안쪽인지, 그 지점의 heading."""

    def __init__(self, track):
        self.x, self.y, self.h = track.x, track.y, track.heading
        self.wl, self.wr = track.w_left, track.w_right

    def nearest(self, px, py):
        d2 = (px[:, None] - self.x[None, :]) ** 2 + (py[:, None] - self.y[None, :]) ** 2
        return np.argmin(d2, axis=1)

    def inside(self, px, py, inset):
        i = self.nearest(px, py)
        h = self.h[i]
        n = -np.sin(h) * (px - self.x[i]) + np.cos(h) * (py - self.y[i])
        return (n < self.wl[i] - inset) & (n > -(self.wr[i] - inset)), i


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


def detect(pts, origin, walls, band, p):
    """→ [(cx, cy, 크기, 점 수, heading)] 가까운 순. pts 는 스캔 순서 그대로여야 한다."""
    if len(pts) == 0:
        return []
    rng = np.hypot(pts[:, 0] - origin[0], pts[:, 1] - origin[1])
    cand = walls.dist(pts[:, 0], pts[:, 1]) > p.wall_clear + p.clear_per_m * rng
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
