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
  ★ 앞뒤는 따로 본다. 상대가 뒤로 밀리는 것은 제동으로 서는 만큼까지다 (서 있는 상대는 0).
    옆으로 움직이는 것도 달린 거리만큼만이다 (차는 제자리에서 옆으로 못 간다). 2026-11-08 젯슨 실제
    솔버 시험에서 정지 장애물의 ½ a t² 가 앞뒤 3 m 까지 커져 모든 스테이지가 막혔다 (status 4).

■ 도달 가능성 (2026-11-08)

  통로는 우리 차가 그 스테이지까지 달린 거리 Δs 안에 옆으로 갈 수 있어야 의미가 있다. 차는 최대
  곡률 κ 로 꺾어 헤딩 θ 까지 틀고 그대로 가는 만큼만 옆으로 간다:
      y(Δs) = ½ κ Δs²                       (Δs ≤ θ/κ)
            = θ²/(2κ) + (Δs − θ/κ) sin θ     (그 뒤)
  시간이 아니라 거리라서 저속에서도 같다. 못 가는 쪽은 고르지 않고, 둘 다 못 가면 뒤따른다.
  뒤따를 때는 옆으로 빠질 수 있는 간격을 두고 서고, 우리 진행을 pass_speed 이상으로 가정해
  다시 판단한다 (멈춰 있으면 스테이지가 상대에 닿지 않아 영영 다시 고르지 않는다).

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
    sig_s_cap: float = 1.0      # 앞쪽 종 불확실성 상한 m (상대가 예측보다 더 가는 쪽)
    opp_turn: float = 0.5       # 상대 횡 이동 ≤ 달린 거리 × sin(opp_turn) rad
    reach_kappa: float = 1.0    # 우리 차 회피 곡률 1/m (최대 조향 곡률 약 1.28 보다 작게). 고속은 a_lat/v² 로 줄인다
    reach_heading: float = 0.5  # 회피 때 트는 최대 헤딩 rad
    reach_margin: float = 0.02  # 도달 판정 여유 m
    pass_speed: float = 0.8     # 판단할 때 우리 진행 속도 하한 m/s (서 있어도 다시 고르게)
    gap_start: float = 0.05     # 이 폭 이상 남아야 그 쪽으로 추월을 시작한다 m
    gap_switch: float = 0.10    # 반대쪽이 이만큼 더 넓어야 바꿀 후보가 된다 m
    switch_frames: int = 10     # 후보가 이만큼 이어져야 바꾼다 (TRAIL 에서 나올 때도)
    trail_gap: float = 0.80     # 뒤따를 때 목표 간격 (s 차이) m
    trail_gain: float = 1.0     # 목표 간격보다 가까울 때 간격 오차 → 속도 1/s (물러나는 쪽)
    trail_brake: float = 0.6    # 상대 앞에 서는 속도 곡선 √(2 a 남은 거리) 의 a m/s² (바퀴 지연까지 덮게 낮게)
    standoff_rate: float = 0.5  # 뒤따를 때 목표 간격이 늘어나는 최대 빠르기 m/s (훅 늘면 상한이 0 으로 떨어진다)
    trail_headway: float = 0.4  # 간격에 더하는 우리 속도 × 이 시간 s (바퀴 시정수 0.4 s 동안 굴러가는 거리)
    pass_guard: bool = True     # 추월 중 옆으로 아직 못 비켰으면 상대 앞에 설 수 있는 속도까지만
    pass_standoff: float = 0.70 # 추월 중 옆으로 아직 못 비켰으면 이 간격 앞에서 설 수 있는 속도까지만 m
    trail_hold: float = 2.0     # 뒤따르는 중 상대가 이 안에 앞서 있으면 겹침이 없어도 계속 뒤따른다 m
    hold_side: bool = True      # 측면을 정하면 첫 겹침 앞 스테이지도 상대 쪽으로 못 가게 (apply_corridor)
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
    sig_s: np.ndarray   # 종 불확실성 m — 앞쪽 (상대가 예측보다 더 감)
    learned: bool
    sig_b: np.ndarray = None   # 종 불확실성 m — 뒤쪽 (상대가 제동). 없으면 sig_s 와 같다


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
        # 옆으로는 달린 거리만큼만 간다 (서 있는 차는 0)
        reach = np.minimum(np.minimum(0.5 * p.a_y * t ** 2, p.d_cap),
                           (s - s[0]) * math.sin(p.opp_turn))
        sig_s = np.minimum(0.5 * p.a_x * t ** 2, p.sig_s_cap)
        # 뒤로는 최대 제동으로 서는 만큼까지: v t − (제동 거리). 서 있으면 0
        stop_t = vs0 / p.a_x
        sig_b = np.where(t < stop_t, 0.5 * p.a_x * t ** 2, vs0 * t - 0.5 * vs0 * stop_t)
        if learned:
            d_line, _, sd, sv = self._lookup(s)
            d_line0 = self._lookup(s0)[0]
            d = d_line + (d0 - d_line0) * decay
            sig_d = np.sqrt(reach ** 2 + sd ** 2)
            sig_s = sig_s + sv * t
            sig_b = sig_b + np.minimum(sv * t, vs0 * t)
        else:
            d = d0 + vd0 * p.tau_d * (1.0 - decay)
            sig_d = reach
        lo, hi = self.track.bounds_at(s, p.opp_half)
        d = np.clip(d, np.minimum(lo, hi), np.maximum(lo, hi))
        return Prediction(t, s, d, v, sig_d, sig_s, learned, sig_b)


# ── 통로와 측면 ───────────────────────────────────────────────────────

@dataclass
class Plan:
    hit: np.ndarray         # 앞뒤로 겹치는 스테이지
    gap_left: float         # 왼쪽 통로에 남는 폭의 최소 (겹치는 스테이지) m. 겹침 없으면 inf
    gap_right: float
    ds0: float              # 지금 (우리 s − 상대 s), 음수 = 상대가 앞
    alongside: bool         # 지금 나란히 (앞뒤로 겹침)
    ego_left: bool          # 우리가 상대보다 왼쪽
    reach_left: bool = True     # 왼쪽 통로에 달린 거리 안에 갈 수 있나
    reach_right: bool = True
    need_left: float = 0.0      # 그쪽 통로까지 옆으로 가야 할 최대 거리 m
    need_right: float = 0.0


def lateral_reach(ds, kappa, heading):
    """달린 거리 ds 동안 옆으로 갈 수 있는 거리 (곡률 kappa 로 heading 까지 틀고 직진)."""
    ds = np.maximum(np.asarray(ds, float), 0.0)
    a0 = heading / kappa
    return np.where(ds <= a0, 0.5 * kappa * ds ** 2,
                    0.5 * kappa * a0 ** 2 + (ds - a0) * math.sin(heading))


def reach_distance(need, kappa, heading):
    """lateral_reach 의 역: 옆으로 need 만큼 가는 데 필요한 거리."""
    need = max(float(need), 0.0)
    a0 = heading / kappa
    y0 = 0.5 * kappa * a0 ** 2
    return math.sqrt(2.0 * need / kappa) if need <= y0 else a0 + (need - y0) / math.sin(heading)


def eval_s(s_ego, t, p):
    """판단에 쓰는 우리 s: 예측이 pass_speed 보다 느리면 그만큼은 간다고 본다."""
    s_ego = np.asarray(s_ego, float)
    return np.maximum(s_ego, s_ego[0] + p.pass_speed * np.asarray(t))


def plan(s_ego, n0, lo, hi, pred, length, p, kappa=None):
    """겹침과 양쪽 남는 폭. s_ego 는 스테이지별 우리 s, lo/hi 는 벽 경계, kappa 는 그 s 의 트랙 곡률."""
    s_ego = np.asarray(s_ego, float)
    ds = wrap_s(s_ego - pred.s, length)
    sig_b = pred.sig_s if pred.sig_b is None else pred.sig_b
    # 우리가 뒤 (ds < 0): 상대가 제동하면 다가온다 / 우리가 앞: 상대가 가속하면 다가온다
    hit = (ds > -(p.length + sig_b)) & (ds < p.length + pred.sig_s)
    c = p.clear + pred.sig_d
    reach_l = reach_r = True
    need_l = need_r = 0.0
    if hit.any():
        edge_l = pred.d[hit] + c[hit]           # 왼쪽 통로 하한
        edge_r = pred.d[hit] - c[hit]           # 오른쪽 통로 상한
        gl = float(np.min(hi[hit] - edge_l))
        gr = float(np.min(edge_r - lo[hit]))
        # 도달 가능성: 그 스테이지까지 달린 거리로 옆으로 갈 수 있는 만큼
        v0 = max((s_ego[1] - s_ego[0]) / max(pred.t[1], 1e-6), 0.1) if len(s_ego) > 1 else 0.1
        kap = min(p.reach_kappa, p.a_lat / v0 ** 2)
        # ★ 2026-11-10 (offline 1108_7 급커브 안쪽 상자): 커브를 도는 데 이미 곡률 κ_트랙 을 쓰고 있어서
        #   안쪽으로 비키는 데는 그만큼 덜 남는다 (조향 포화 0.38 인 채로 못 비키고 그 자리에 섰다).
        #   바깥쪽은 덜 꺾으면 되니 그대로 둔다. n 은 왼쪽 +, κ > 0 은 왼쪽 커브.
        kap_l = kap_r = kap
        if kappa is not None:
            k_end = int(np.nonzero(hit)[0].max()) + 1
            kb = float(np.mean(np.asarray(kappa, float)[:k_end]))
            kap_l = max(kap - max(kb, 0.0), 0.05)
            kap_r = max(kap - max(-kb, 0.0), 0.05)
        dsh = s_ego[hit] - s_ego[0]
        r_l = lateral_reach(dsh, kap_l, p.reach_heading) + p.reach_margin
        r_r = lateral_reach(dsh, kap_r, p.reach_heading) + p.reach_margin
        nl, nr = edge_l - n0, n0 - edge_r
        reach_l, reach_r = bool(np.all(nl <= r_l)), bool(np.all(nr <= r_r))
        need_l, need_r = float(max(nl.max(), 0.0)), float(max(nr.max(), 0.0))
    else:
        gl = gr = math.inf
    ds0 = float(wrap_s(s_ego[0] - pred.s[0], length))
    return Plan(hit, gl, gr, ds0, abs(ds0) < p.length, n0 > pred.d[0], reach_l, reach_r, need_l, need_r)


def apply_corridor(lo, hi, pred, pl, side, p, n0=None):
    """side 쪽 통로로 경계를 좁힌 새 (lo, hi).

    n0 를 주면 첫 겹침 스테이지 앞쪽도 상대 쪽으로 못 가게 묶는다 (지금 n 과 통로 끝 중 덜 좁은 쪽).
    통로가 먼 스테이지에만 걸리면 그 앞은 자유라 MPCC 가 레이싱 라인을 따라 상대 쪽으로 가 버리고,
    가까워졌을 땐 옆으로 갈 거리가 모자라 뒤따름으로 떨어진다 (1109_3 왼쪽 상자, 2026-11-09)."""
    lo, hi = np.array(lo, float), np.array(hi, float)
    lo_w, hi_w = lo.copy(), hi.copy()
    h = pl.hit
    c = p.clear + pred.sig_d
    if side == LEFT:
        lo[h] = np.maximum(lo[h], pred.d[h] + c[h])
    elif side == RIGHT:
        hi[h] = np.minimum(hi[h], pred.d[h] - c[h])
    # ★ 2026-11-11: 통로가 반대쪽 벽을 넘으면 (나란히 가는 중 상대가 붙어 옴 등) 솔버 쪽 wall_bounds 가
    #   lo > hi 를 가운데 한 점으로 두는데, 그 점이 벽 밖이라 MPC 가 차를 벽 밖으로 밀었다 (offline 벽 밖
    #   60~87%). 통로는 벽을 넘지 않게 벽에 붙여 둔다. 상대와의 간격은 속도 상한 (pass_guard) 이 맡는다.
    lo, hi = np.minimum(lo, hi_w), np.maximum(hi, lo_w)
    if n0 is not None and h.any() and side in (LEFT, RIGHT):
        k0 = int(np.argmax(h))
        if k0 > 0:
            pre = np.arange(len(lo)) < k0
            if side == LEFT:
                lo[pre] = np.maximum(lo[pre], min(n0, float(lo[k0])))
            else:
                hi[pre] = np.minimum(hi[pre], max(n0, float(hi[k0])))
    return lo, hi


def trail_standoff(pl, p):
    """뒤따를 때 목표 간격. 옆에 자리는 있는데 못 가서 뒤따르는 거면 옆으로 빠질 거리를 둔다."""
    needs = [n for n, g in ((pl.need_left, pl.gap_left), (pl.need_right, pl.gap_right))
             if g >= p.gap_start]
    if not needs:
        return p.trail_gap
    return max(p.trail_gap, p.length + reach_distance(min(needs), p.reach_kappa, p.reach_heading))


def trail_cap(pl, pred, v_max, p, s_ego, length, n0=0.0, kappa=None, standoff=None):
    """상대 앞에 설 수 있는 스테이지 0..N−1 의 바퀴 속도 상한. 상대가 앞에 있을 때만 의미가 있다.

    ★ 2026-11-10 (offline 1108_7 정지 장애물 241 주기 충돌, 가운데 상대 14 주기): 예전에는 지금 간격
      하나로 v = v_상대 + 1.0 × (간격 − 목표) 를 모든 스테이지에 같게 줬다. MPC 는 앞에서 서야 한다는
      것을 못 보고, 바퀴 지연 0.4 s 와 맞물려 0.8 m 를 지나쳐 0.2 m 까지 붙었다. 이제는
        · 스테이지마다 예측 위치의 간격 gap_k 로 (MPC 가 미리 감속한다)
        · 남은 거리 e_k = gap_k − 목표 − headway × 우리 속도_k 에서 서는 속도 √(2 a e_k)
          (가까우면 trail_gain × e_k 로 상대보다 느리게)
        · s 진행 속도를 바퀴 속도로: × (1 − n κ). 코너 안쪽이면 같은 바퀴 속도로 s 가 더 빨리 간다
    s_ego 는 스테이지별 우리 s (eval_s 로 늘리지 않은 것), kappa 는 그 s 의 곡률.
    """
    s_ego = np.asarray(s_ego, float)
    gap = -wrap_s(s_ego - pred.s, length)
    dt = max(float(pred.t[1] - pred.t[0]), 1e-6) if len(pred.t) > 1 else 0.033
    v_e = np.maximum(np.r_[np.diff(s_ego), s_ego[-1] - s_ego[-2]] / dt, 0.0) if len(s_ego) > 1 else 0.0
    goal = trail_standoff(pl, p) if standoff is None else standoff
    e = gap - goal - p.trail_headway * v_e
    dv = np.where(e >= 0.0, np.sqrt(2.0 * p.trail_brake * np.maximum(e, 0.0)), p.trail_gain * e)
    sd = np.maximum(pred.v + dv, 0.0)
    if kappa is not None:
        sd = sd * np.clip(1.0 - n0 * np.asarray(kappa, float), 0.5, 1.5)
    return np.clip(sd, 0.0, v_max)[:-1]


class SideSelector:
    """FREE / LEFT / RIGHT / TRAIL. 나란히 달리는 중에는 측면을 절대 바꾸지 않는다."""

    def __init__(self, p):
        self.p = p
        self.mode = FREE
        self.cand = None
        self.count = 0

    def _gap(self, pl, side):
        """남는 폭. 달린 거리 안에 못 가는 쪽은 막힌 것으로 본다."""
        if side == LEFT:
            return pl.gap_left if pl.reach_left else -math.inf
        return pl.gap_right if pl.reach_right else -math.inf

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
        self.standoff = None    # 뒤따를 때 쓰는 목표 간격 (늘 때는 천천히)
        self.t_last = None

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
        s_eval = eval_s(s_ego, pred.t, self.p)
        pl = plan(s_eval, n0, lo, hi, pred, self.track.length, self.p, self.track.kappa_at(s_eval))
        mode = self.sel.update(pl)
        cap = None
        kap = self.track.kappa_at(np.asarray(s_ego, float))
        if mode in (LEFT, RIGHT):
            lo, hi = apply_corridor(lo, hi, pred, pl, mode, self.p, n0 if self.p.hold_side else None)
            # ★ 2026-11-10 (offline 1109_3 가운데 정지 장애물 441 주기 충돌): 오른쪽 통로를 골랐는데 옆으로
            #   못 비킨 채 2 m/s 로 다가가다 1.4 m 앞에서 뒤따르기로 바뀌어 못 섰다. 옆으로 아직 못 비켰으면
            #   pass_standoff 앞에서 설 수 있는 속도까지만 간다. 비키고 나면 (need ≤ 여유) 풀린다.
            need = pl.need_left if mode == LEFT else pl.need_right
            if self.p.pass_guard and pl.ds0 < 0 and need > self.p.reach_margin:
                cap = trail_cap(pl, pred, v_max, self.p, s_ego, self.track.length, n0, kap,
                                standoff=self.p.pass_standoff)
        elif mode == TRAIL and pl.ds0 < 0:
            # ★ 2026-11-11 (offline 1109_3 1.4 m/s 상대): 목표 간격이 0.8 → 1.3 m 로 훅 바뀌면 상한이 1.3 → 0 으로
            #   떨어지고, 빠를 때 그렇게 서면 차가 커브에서 밀려 벽 밖으로 갔다. 늘 때는 standoff_rate 로 천천히.
            goal = trail_standoff(pl, self.p)
            dtc = 0.0 if self.t_last is None else max(t - self.t_last, 0.0)
            if self.standoff is None or goal <= self.standoff:
                self.standoff = goal if self.standoff is not None else min(goal, max(self.p.trail_gap, -pl.ds0))
            else:
                self.standoff = min(goal, self.standoff + self.p.standoff_rate * dtc)
            cap = trail_cap(pl, pred, v_max, self.p, s_ego, self.track.length, n0, kap, standoff=self.standoff)
        if mode != TRAIL:
            self.standoff = None
        self.t_last = t
        self.last = (pred, pl, mode)
        return lo, hi, cap, mode
