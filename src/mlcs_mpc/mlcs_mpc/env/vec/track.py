"""
vec.track — env/track/track.Track 을 배치 torch 로. 환경 B 개의 Frenet (s, n, mu) 와 앞쪽 트랙 정보 (곡률·벽 폭) 를 한 번에.

  vt = VecTrack(Track(find('rl17_track')), device='cpu')
  s, n, mu, idx = vt.frenet(x, y, yaw, idx_hint)       (B,) 텐서들. idx 는 다음 호출의 힌트 (가까운 점 번호)
  kappa, w_left, w_right = vt.lookahead(s, offsets)    (B, K)

■ Track.frenet 과 같은 규칙 — n 은 왼쪽이 +, 가장 가까운 점과 이웃 점 사이에서 heading 을 보간.
  가장 가까운 점은 힌트 주변 창 (−window_back … +window_fwd) 에서만 찾는다 (트랙이 가까이 지나는 곳으로 건너뛰지 않게,
  Track._nearest 와 같다). 힌트가 없으면 (-1) 전 구간에서 찾는다.
  점 간격은 Track 의 ref 파일 간격 (0.05 m) 이다.
"""
import math

import numpy as np
import torch


def wrap(a):
    return torch.remainder(a + math.pi, 2 * math.pi) - math.pi


class VecTrack:

    def __init__(self, track, device='cpu', dtype=torch.float32, window_back=10, window_fwd=40):
        t = lambda a: torch.as_tensor(np.asarray(a, dtype=float), device=device, dtype=dtype)  # noqa: E731
        self.track = track
        self.device, self.dtype = torch.device(device), dtype
        self.s, self.x, self.y = t(track.s), t(track.x), t(track.y)
        self.heading, self.kappa = t(track.heading), t(track.kappa)
        self.w_left, self.w_right = t(track.w_left), t(track.w_right)
        self.n = len(track.s)
        self.length = float(track.length)
        self.ds = self.length / self.n
        self.win = torch.arange(-window_back, window_fwd + 1, device=device)

    def nearest(self, x, y, idx_hint):
        """(B,) 가장 가까운 점 번호. idx_hint < 0 이면 전 구간."""
        full = idx_hint < 0
        idx = torch.empty_like(idx_hint)
        if full.any():
            d = (self.x[None, :] - x[full, None]) ** 2 + (self.y[None, :] - y[full, None]) ** 2
            idx[full] = d.argmin(dim=1)
        loc = ~full
        if loc.any():
            cand = (idx_hint[loc, None] + self.win[None, :]) % self.n
            d = (self.x[cand] - x[loc, None]) ** 2 + (self.y[cand] - y[loc, None]) ** 2
            idx[loc] = cand.gather(1, d.argmin(dim=1, keepdim=True)).squeeze(1)
        return idx

    def frenet(self, x, y, yaw, idx_hint):
        """→ s (0 ≤ s < L, 감기 전), n, mu, idx."""
        i = self.nearest(x, y, idx_hint)
        dx, dy = x - self.x[i], y - self.y[i]
        h = self.heading[i]
        along = torch.cos(h) * dx + torch.sin(h) * dy
        j = torch.where(along >= 0, (i + 1) % self.n, (i - 1) % self.n)
        frac = torch.clamp(along.abs() / self.ds, max=1.0)
        h = h + frac * wrap(self.heading[j] - h)
        s = self.s[i] + torch.cos(h) * dx + torch.sin(h) * dy
        n = -torch.sin(h) * dx + torch.cos(h) * dy
        return torch.remainder(s, self.length), n, wrap(yaw - h), i

    def interp(self, table, s):
        """주기 선형 보간 (s 아무 모양)."""
        q = torch.remainder(s, self.length) / self.ds
        i0 = torch.floor(q).long() % self.n
        f = q - torch.floor(q)
        return table[i0] * (1 - f) + table[(i0 + 1) % self.n] * f

    def walls(self, s):
        return self.interp(self.w_left, s), self.interp(self.w_right, s)

    def lookahead(self, s, offsets):
        """s (B,), offsets (K,) → 곡률·왼쪽 벽·오른쪽 벽 (B, K)."""
        q = s[:, None] + offsets[None, :]
        return self.interp(self.kappa, q), self.interp(self.w_left, q), self.interp(self.w_right, q)

    def pose_at(self, s, n=None):
        """s 지점 (센터라인 + n) 의 (x, y, heading)."""
        h = self.interp_angle(s)
        x, y = self.interp(self.x, s), self.interp(self.y, s)
        if n is not None:
            x, y = x - torch.sin(h) * n, y + torch.cos(h) * n
        return x, y, h

    def interp_angle(self, s):
        q = torch.remainder(s, self.length) / self.ds
        i0 = torch.floor(q).long() % self.n
        f = q - torch.floor(q)
        h0 = self.heading[i0]
        return wrap(h0 + f * wrap(self.heading[(i0 + 1) % self.n] - h0))
