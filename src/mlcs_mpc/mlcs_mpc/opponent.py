#!/usr/bin/env python3
"""
opponent — 상대 차 추적·예측과 추월 통로 (ROS 없음). test/test_opponent.py 로 시험한다.

■ 무엇을 하나

  MPCC 의 스테이지별 횡방향 경계 [n_lo, n_hi] 를 상대 차 옆 통로로 좁힌다. 솔버는 그대로이고
  경계 값만 바뀐다. 상대를 지나치면 통로가 저절로 사라져 MPCC 가 알아서 라인으로 돌아온다.
  양쪽 통로가 다 막히면 뒤따른다 (스테이지별 바퀴 속도 상한).

    상대 pose → OpponentTracker   Frenet (s, d, vs, vd), 등속 칼만
             → OpponentModel     스테이지 시각 t_k 의 s, d, 속도와 불확실성
             → plan()            겹치는 스테이지, 양쪽 남는 폭
             → SideSelector      FREE / LEFT / RIGHT / TRAIL (히스테리시스)
             → n_lo, n_hi, v_cap

  ForzaETH·UNICORN 은 MPC 밖에서 회피 경로를 만들어 추종기에 넘긴다. 여기서는 측면만 밖에서
  정하고 그 통로 안의 선과 속도는 MPCC 가 고른다 (Liniger 2015 의 통로 방식).

■ 예측 — 학습 없이 시작해서 관측이 쌓이면 GP 로

  처음 (제로샷): 횡은 지금 d 에서 횡속도를 감쇠시켜 외삽, 종은 min(지금 속도, 코너 한계 √(a_lat/|κ|)).
  상대를 한 바퀴 거의 다 보면 (구간 채움 ≥ gp_coverage): 상대 라인 d(s) 와 속도 vs(s) 를 주기 커널
  GP 로 맞추고, 지금 값에서 그 선으로 지수로 수렴한다고 본다 (UNICORN 의 '라인 복귀' 와 같은 생각).
  불확실성은 상대가 물리적으로 움직일 수 있는 범위 (½ a t², 상한 있음) 와 GP 표준편차를 합친다.
  상한이 없으면 1 초 안에 트랙 폭을 다 덮어 추월이 아예 안 된다.

■ 좌표

  s 는 각자 연속이라 (랩 수가 다를 수 있다) 차이는 항상 트랙 길이로 감아서 본다. n, d 는 왼쪽 +.
  우리 차 n 은 무게중심 기준이고 상대는 받은 pose 그대로다 (차 중심 근처라면 clear 여유로 덮는다).
"""
import copy
import math
from dataclasses import dataclass

import numpy as np

FREE, LEFT, RIGHT, TRAIL = 'free', 'left', 'right', 'trail'


@dataclass
class OppParams:
    """기본값은 출발점이다. 우리 차 실측이 아니다."""
    clear: float = 0.40         # 두 차 중심 사이 최소 횡거리 m (차 폭 약 0.30 + 여유 0.10)
    length: float = 0.70        # 앞뒤로 이 안이면 겹친다고 본다 m (차 길이 약 0.55 + 여유)
    a_y: float = 1.0            # 횡 불확실성 ½ a_y t² 의 a_y m/s²
    d_cap: float = 0.15         # 그 상한 m
    a_x: float = 1.5            # 종 불확실성 ½ a_x t² 의 a_x m/s²
    a_lat: float = 4.0          # 제로샷 코너 속도 한계 √(a_lat/|κ|) (마찰 0.41 × g, hmpc.yaml friction)
    tau_d: float = 0.8          # 횡속도 감쇠·GP 선 수렴 시정수 s
    tau_v: float = 1.0          # 속도가 GP 속도로 수렴하는 시정수 s
    opp_half: float = 0.15      # 상대 반폭. 상대 예측 d 를 벽 안으로 자른다 m
    gap_start: float = 0.05     # 이 폭 이상 남아야 그 쪽으로 추월을 시작한다 m
    gap_switch: float = 0.10    # 반대쪽이 이만큼 더 넓어야 바꿀 후보가 된다 m
    switch_frames: int = 10     # 후보가 이만큼 이어져야 바꾼다 (TRAIL 에서 나올 때도)
    trail_gap: float = 0.80     # 뒤따를 때 목표 간격 (s 차이) m
    trail_gain: float = 1.0     # 간격 오차 → 속도 1/s
    trail_hold: float = 2.0     # 뒤따르는 중 상대가 이 안에 앞서 있으면 겹침이 없어도 계속 뒤따른다 m
    gp: bool = True
    gp_coverage: float = 0.9    # 상대를 본 구간 비율이 이 이상이면 GP 를 쓴다
    bin: float = 0.1            # 상대 라인 기록 구간 m
    timeout: float = 0.3        # 상대 pose 가 이보다 오래되면 없는 것으로 본다 s
    meas_noise: float = 0.01    # 상대 위치 측정 표준편차 m (mocap 은 mm 급, 라이다면 키울 것)
    accel_noise: float = 2.0    # 추적 칼만 가속도 잡음 m/s²


def wrap_s(ds, length):
    """s 차이를 (−L/2, L/2] 로."""
    return (np.asarray(ds) + length / 2.0) % length - length / 2.0


# ── 추적 ─────────────────────────────────────────────────────────────

class _CV1:
    """1축 등속 칼만. 상태 [p, v]."""

    def __init__(self, q, r):
        self.q, self.r = q, r
        self.x = None
        self.P = None

    def update(self, z, dt):
        if self.x is None:
            self.x = np.array([z, 0.0])
            self.P = np.diag([self.r, 4.0])
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = self.q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        x = F @ self.x
        P = F @ self.P @ F.T + Q
        k = P[:, 0] / (P[0, 0] + self.r)
        self.x = x + k * (z - x[0])
        self.P = P - np.outer(k, P[0, :])


class OpponentTracker:
    """상대 pose → Frenet (s, d, vs, vd). 트랙 객체는 복사해서 쓴다 (frenet() 이 상태를 가진다)."""

    def __init__(self, track, p):
        self.track = copy.copy(track)
        self.track.reset()
        self.p = p
        self.t = None

    def reset(self):
        self.track.reset()
        self.t = None

    def update(self, t, x, y, yaw):
        if self.t is not None and (t - self.t > self.p.timeout or t <= self.t):
            if t - self.t > self.p.timeout:
                self.reset()
            else:
                return
        s, d, _ = self.track.frenet(x, y, yaw)
        if self.t is None:
            q, r = self.p.accel_noise ** 2, self.p.meas_noise ** 2
            self.fs, self.fd = _CV1(q, r), _CV1(q, r)
            dt = 0.0
        else:
            dt = t - self.t
        self.fs.update(s, dt)
        self.fd.update(d, dt)
        self.t = t

    def state(self, t):
        """(s, d, vs, vd) 를 시각 t 로 외삽. 없거나 오래됐으면 None."""
        if self.t is None or t - self.t > self.p.timeout:
            return None
        age = max(t - self.t, 0.0)
        s, vs = self.fs.x
        d, vd = self.fd.x
        return s + vs * age, d + vd * age, vs, vd


# ── 상대 라인 (GP) ───────────────────────────────────────────────────

class PeriodicGP:
    """1차원 주기 커널 GP (닫힌 트랙). 하이퍼파라미터는 작은 격자에서 주변 우도로 고른다."""

    LENGTHS = (0.25, 0.5, 1.0, 2.0)     # 상관 거리 m
    NOISE = (0.01, 0.05, 0.2)           # 신호 분산 대비 잡음 분산

    def __init__(self, period):
        self.period = period

    def _k(self, a, b, ell, sf2):
        u = np.sin(np.pi * (a[:, None] - b[None, :]) / self.period)
        # 작은 차이에서 exp(−Δ²/2ℓ²) 가 되도록 ℓ(m) 를 주기 커널 꼴로 바꾼다
        e = 2.0 * np.pi * ell / self.period
        return sf2 * np.exp(-2.0 * u ** 2 / e ** 2)

    def fit(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        self.mean = float(y.mean())
        yc = y - self.mean
        sf2 = max(float(yc.var()), 1e-6)
        best = None
        for ell in self.LENGTHS:
            K0 = self._k(x, x, ell, sf2)
            for rn in self.NOISE:
                sn2 = sf2 * rn + 1e-8
                try:
                    Lc = np.linalg.cholesky(K0 + sn2 * np.eye(len(x)))
                except np.linalg.LinAlgError:
                    continue
                a = np.linalg.solve(Lc.T, np.linalg.solve(Lc, yc))
                ll = -0.5 * yc @ a - np.log(np.diag(Lc)).sum()
                if best is None or ll > best[0]:
                    best = (ll, ell, sn2, Lc, a)
        _, self.ell, self.sn2, self.L, self.alpha = best
        self.sf2, self.x = sf2, x
        return self

    def predict(self, xq):
        xq = np.asarray(xq, float)
        Ks = self._k(xq, self.x, self.ell, self.sf2)
        mu = self.mean + Ks @ self.alpha
        v = np.linalg.solve(self.L, Ks.T)
        var = np.maximum(self.sf2 - (v ** 2).sum(0), 0.0)
        return mu, np.sqrt(var)


class LineMemory:
    """상대의 d, vs 를 s 구간별로 모은다 (구간마다 평균)."""

    def __init__(self, length, bin_m):
        self.length = length
        self.nb = max(int(round(length / bin_m)), 4)
        self.bw = length / self.nb
        self.d = np.zeros(self.nb)
        self.v = np.zeros(self.nb)
        self.cnt = np.zeros(self.nb)

    def add(self, s, d, vs):
        i = int(np.mod(s, self.length) / self.bw) % self.nb
        self.cnt[i] += 1
        a = max(1.0 / self.cnt[i], 0.2)     # 처음엔 평균, 나중엔 최근 쪽으로
        self.d[i] += a * (d - self.d[i])
        self.v[i] += a * (vs - self.v[i])

    def coverage(self):
        return float((self.cnt > 0).mean())

    def centers(self):
        return (np.arange(self.nb) + 0.5) * self.bw


@dataclass
class Prediction:
    t: np.ndarray       # 스테이지 시각 s
    s: np.ndarray       # 상대 s (상대 기준 연속)
    d: np.ndarray
    v: np.ndarray       # 상대 진행 속도
    sig_d: np.ndarray   # 횡 불확실성 m
    sig_s: np.ndarray   # 종 불확실성 m
    learned: bool


class OpponentModel:
    """제로샷 예측 + 관측이 쌓이면 GP 선."""

    def __init__(self, track, p):
        self.track, self.p = track, p
        self.mem = LineMemory(track.length, p.bin)
        self.tab = None     # GP 를 구간 중심에서 미리 계산해 둔 표 (d, v, sd, sv)

    def observe(self, s, d, vs):
        if vs > 0.2:        # 서 있는 상대는 라인 정보가 아니다
            self.mem.add(s, d, vs)

    def refit(self):
        """GP 를 다시 맞춘다. 제어 루프 밖에서 가끔 (1 Hz) 부른다. 쓰면 True."""
        if not self.p.gp or self.mem.coverage() < self.p.gp_coverage:
            self.tab = None
            return False
        m = self.mem.cnt > 0
        xs, xq = self.mem.centers()[m], self.mem.centers()
        d, sd = PeriodicGP(self.track.length).fit(xs, self.mem.d[m]).predict(xq)
        v, sv = PeriodicGP(self.track.length).fit(xs, self.mem.v[m]).predict(xq)
        v = np.maximum(v, 0.0)
        close = lambda a: np.r_[a, a[0]]  # noqa: E731  닫힌 트랙: 끝에 첫 값을 붙여 보간
        self.tab = (np.r_[xq, xq[0] + self.track.length], close(d), close(v), close(sd), close(sv))
        return True

    def _lookup(self, s):
        S, d, v, sd, sv = self.tab
        q = np.mod(s - S[0], self.track.length) + S[0]
        return (np.interp(q, S, d), np.interp(q, S, v), np.interp(q, S, sd), np.interp(q, S, sv))

    def predict(self, s0, d0, vs0, vd0, N, dt):
        p = self.p
        t = dt * np.arange(N + 1)
        s = np.empty(N + 1)
        v = np.empty(N + 1)
        s[0] = s0
        learned = self.tab is not None
        vs0 = max(vs0, 0.0)
        if learned:
            _, v_line0, _, _ = self._lookup(s0)
        for k in range(N + 1):
            if learned:
                vk = self._lookup(s[k])[1] + (vs0 - v_line0) * math.exp(-t[k] / p.tau_v)
            else:
                kap = abs(float(self.track.kappa_at(s[k])))
                vk = min(vs0, math.sqrt(p.a_lat / max(kap, 1e-3)))
            v[k] = max(vk, 0.0)
            if k < N:
                s[k + 1] = s[k] + v[k] * dt
        decay = np.exp(-t / p.tau_d)
        reach = np.minimum(0.5 * p.a_y * t ** 2, p.d_cap)
        sig_s = 0.5 * p.a_x * t ** 2
        if learned:
            d_line, _, sd, sv = self._lookup(s)
            d_line0 = self._lookup(s0)[0]
            d = d_line + (d0 - d_line0) * decay
            sig_d = np.sqrt(reach ** 2 + sd ** 2)
            sig_s = sig_s + sv * t
        else:
            d = d0 + vd0 * p.tau_d * (1.0 - decay)
            sig_d = reach
        lo, hi = self.track.bounds_at(s, p.opp_half)
        d = np.clip(d, np.minimum(lo, hi), np.maximum(lo, hi))
        return Prediction(t, s, d, v, sig_d, sig_s, learned)


# ── 통로와 측면 ───────────────────────────────────────────────────────

@dataclass
class Plan:
    hit: np.ndarray         # 앞뒤로 겹치는 스테이지
    gap_left: float         # 왼쪽 통로에 남는 폭의 최소 (겹치는 스테이지) m. 겹침 없으면 inf
    gap_right: float
    ds0: float              # 지금 (우리 s − 상대 s), 음수 = 상대가 앞
    alongside: bool         # 지금 나란히 (앞뒤로 겹침)
    ego_left: bool          # 우리가 상대보다 왼쪽


def plan(s_ego, n0, lo, hi, pred, length, p):
    """겹침과 양쪽 남는 폭. s_ego 는 스테이지별 우리 s, lo/hi 는 벽 경계."""
    ds = wrap_s(np.asarray(s_ego) - pred.s, length)
    hit = np.abs(ds) < p.length + pred.sig_s
    c = p.clear + pred.sig_d
    if hit.any():
        gl = float(np.min(hi[hit] - (pred.d[hit] + c[hit])))
        gr = float(np.min((pred.d[hit] - c[hit]) - lo[hit]))
    else:
        gl = gr = math.inf
    ds0 = float(wrap_s(s_ego[0] - pred.s[0], length))
    return Plan(hit, gl, gr, ds0, abs(ds0) < p.length, n0 > pred.d[0])


def apply_corridor(lo, hi, pred, pl, side, p):
    """side 쪽 통로로 경계를 좁힌 새 (lo, hi)."""
    lo, hi = np.array(lo, float), np.array(hi, float)
    h = pl.hit
    c = p.clear + pred.sig_d
    if side == LEFT:
        lo[h] = np.maximum(lo[h], pred.d[h] + c[h])
    elif side == RIGHT:
        hi[h] = np.minimum(hi[h], pred.d[h] - c[h])
    return lo, hi


def trail_cap(pl, pred, v_max, p):
    """뒤따를 때 스테이지 0..N−1 의 바퀴 속도 상한. 상대가 앞에 있을 때만 의미가 있다."""
    gap = -pl.ds0
    cap = pred.v[:-1] + p.trail_gain * (gap - p.trail_gap)
    return np.clip(cap, 0.0, v_max)


class SideSelector:
    """FREE / LEFT / RIGHT / TRAIL. 나란히 달리는 중에는 측면을 절대 바꾸지 않는다."""

    def __init__(self, p):
        self.p = p
        self.mode = FREE
        self.cand = None
        self.count = 0

    def _gap(self, pl, side):
        return pl.gap_left if side == LEFT else pl.gap_right

    def _pending(self, side):
        """같은 후보가 switch_frames 번 이어지면 True."""
        if side != self.cand:
            self.cand, self.count = side, 0
        self.count += 1
        return self.count >= self.p.switch_frames

    def update(self, pl):
        p = self.p
        if self.mode == TRAIL and not pl.hit.any() and -p.trail_hold < pl.ds0 < 0:
            # 속도를 줄이면 예측이 겹치지 않게 되는데, 그때 풀어 주면 다시 붙는다 (진동)
            return self.mode
        if not pl.hit.any():
            self.mode, self.cand, self.count = FREE, None, 0
            return self.mode
        if self.mode in (LEFT, RIGHT):
            cur = self.mode
            other = RIGHT if cur == LEFT else LEFT
            if pl.alongside:
                return cur          # 나란히 달리는 중: 슬랙이 최소한으로만 넘게 둔다
            g_cur, g_oth = self._gap(pl, cur), self._gap(pl, other)
            if g_cur >= 0.0:
                if g_oth >= max(g_cur + p.gap_switch, p.gap_start) and self._pending(other):
                    self.mode, self.cand, self.count = other, None, 0
                elif g_oth < max(g_cur + p.gap_switch, p.gap_start):
                    self.cand, self.count = None, 0
                return self.mode
            # 지금 쪽이 막혔다 (상대가 옮겨옴) → 아래에서 다시 고른다. 기다리지 않는다
        ok = [s for s in (LEFT, RIGHT) if self._gap(pl, s) >= p.gap_start]
        if not ok:
            if self.mode != TRAIL:
                self.cand, self.count = None, 0
            self.mode = TRAIL
            return self.mode
        # 지금 있는 쪽을 조금 더 좋게 본다 (건너가지 않아도 되니까)
        pick = max(ok, key=lambda s: self._gap(pl, s) + (0.05 if (s == LEFT) == pl.ego_left else 0.0))
        if self.mode == TRAIL and not self._pending(pick):
            return self.mode        # 뒤따르다 나올 때는 잠깐 지켜본다
        self.mode, self.cand, self.count = pick, None, 0
        return self.mode


class Opponent:
    """노드와 오프라인 시험이 같이 쓰는 묶음. 매 주기 step() 한 번."""

    def __init__(self, track, p=None):
        self.p = p or OppParams()
        self.track = track
        self.tracker = OpponentTracker(track, self.p)
        self.model = OpponentModel(track, self.p)
        self.sel = SideSelector(self.p)
        self.last = None    # 마지막 (Prediction, Plan, mode) — 시각화·기록용

    def measure(self, t, x, y, yaw):
        self.tracker.update(t, x, y, yaw)
        st = self.tracker.state(t)
        if st is not None:
            self.model.observe(st[0], st[1], st[2])

    def step(self, t, s_ego, n0, lo, hi, v_max, N, dt):
        """→ (lo, hi, v_cap 또는 None, mode). 상대가 없으면 경계 그대로, mode None."""
        st = self.tracker.state(t)
        if st is None:
            self.sel = SideSelector(self.p)
            self.last = None
            return lo, hi, None, None
        pred = self.model.predict(*st, N, dt)
        pl = plan(s_ego, n0, lo, hi, pred, self.track.length, self.p)
        mode = self.sel.update(pl)
        cap = None
        if mode in (LEFT, RIGHT):
            lo, hi = apply_corridor(lo, hi, pred, pl, mode, self.p)
        elif mode == TRAIL and pl.ds0 < 0:
            cap = trail_cap(pl, pred, v_max, self.p)
        self.last = (pred, pl, mode)
        return lo, hi, cap, mode
