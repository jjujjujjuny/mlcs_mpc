#!/usr/bin/env python3
"""
hyper_params.py — HyperPM (시변 동역학 파라미터) 를 numpy 로 돌려 MPC 스테이지별 파라미터를 만든다.

  torch 가 없는 차에서 쓴다. 가중치는 learned/hyperpm/export_hyper.py 가 만든 .npz 다.
  env/mpc/param_model.py 의 인터페이스 (observe, stage_params, set_plan, clear_plan) 를 따른다.

■ 무엇을 하나 (저자 run_mpc_car.py 와 같은 흐름)

    M  지난 0.25 s 관측 [vx, vy, r, ws, delta] 25개 (0.01 s 격자, 지금 바로 전 칸까지)
    P  앞으로 1 s 의 [ws, delta] 100개 — 직전 MPC 해의 예측 궤적
       → GRU + 인과 MLP → 제어점 8개 × 파라미터 30개 → B-스플라인 → 0.01 s 격자 100개
       → MPC 스테이지 시각 k·dt 로 보간. 1 s 뒤 스테이지는 마지막 값을 유지한다.

  학습 데이터 (prep_hpm_dataset) 와 같게 vx, vy, r 은 무게중심 기준, ws 는 바퀴 선속도 [m/s],
  delta 는 실제 조향이다 — OCP 상태 x[3], x[4], x[5], x[7], x[8] 그대로.

■ 안전

  · 파라미터는 모두 양수형이라 네트워크 기준값 × [1 − s, 1 + s] (s = max_param_scaler 0.5) 안에만 있다.
  · v_min 아래 (서 있거나 출발할 때) 와 출력이 유한하지 않을 때는 fallback (mpcc.yaml 상수) 을 쓴다.
    학습 데이터는 vx ≥ 0.25 만 있고, 정지 근처에서는 어차피 동역학 차이가 작다.
  · v_min ~ v_min + blend 사이는 섞는다 (출발할 때 파라미터가 계단으로 바뀌지 않게).
  · 타이어 Cx 는 CX_MAX (1) 이하로 묶는다 (clamp_cx). Cx > 1 이면 종방향 힘이 정점 뒤 떨어지고 그 구간에서
    v_x 가 불안정해져 저속에서 QP 가 깨진다 (mpcc.yaml 주석). J 는 뒤 Cx 가 늘 1 을 넘지만 묶어도 손실은 같다.
"""
import numpy as np

CX_MAX = 1.0
CX = (17, 27)       # 앞·뒤 타이어 Cx 위치 (바로 앞이 Bx)


def clamp_cx(p, cx_max=CX_MAX):
    """(..., 30) 파라미터의 Cx 를 cx_max 이하로. Bx·Cx 를 유지해 원점 기울기 Bx·Cx·Dx 가 그대로다."""
    p = np.array(p, dtype=np.float64)
    for i in CX:
        c = np.minimum(p[..., i], cx_max)
        p[..., i - 1] *= p[..., i] / c
        p[..., i] = c
    return p


def _sig(x):
    return 1.0 / (1.0 + np.exp(-x))


class HyperParams:
    """네트워크 하나. predict(M, P) → (horizon, 30) 파라미터, 시각 k·tp."""

    def __init__(self, path):
        z = np.load(path)
        self.w = {k: z[k] for k in z.files}
        w = self.w
        self.W = int(w['obs_window'])
        self.H = int(w['horizon'])
        self.tp = float(w['tp'])
        self.nc = int(w['n_ctrl'])
        self.p_const = w['p_const']
        self.n_par = self.p_const.size
        self.scaler = float(w['max_param_scaler'])
        self.Ns = w['interp_Ns'][::2]        # rk4 반 칸 격자 → 정수 칸 (시각 k·tp)
        self.hid = w['gru_w_hh'].shape[1]

    def predict(self, M, P):
        w = self.w
        M = np.asarray(M, dtype=np.float64) / w['obs_scale']
        assert M.shape == (self.W, 5) and np.shape(P) == (self.H, 2)
        h = np.zeros(self.hid)
        Wi, Wh, bi, bh = w['gru_w_ih'], w['gru_w_hh'], w['gru_b_ih'], w['gru_b_hh']
        n = self.hid
        gi_all = M @ Wi.T + bi
        for t in range(self.W):     # torch.nn.GRU — 게이트 순서 r, z, n
            gi = gi_all[t]
            gh = Wh @ h + bh
            r = _sig(gi[:n] + gh[:n])
            zg = _sig(gi[n:2 * n] + gh[n:2 * n])
            c = np.tanh(gi[2 * n:] + r * gh[2 * n:])
            h = (1.0 - zg) * c + zg * h
        a1 = np.tanh(w['up_w'] @ h + w['up_b'])
        ctrl = w['pred_pinv'] @ (np.asarray(P, dtype=np.float64) / w['pred_div'])     # (nc, 2)
        m1 = np.tanh(w['c1_w'] @ ctrl.reshape(-1) + w['h1m_w'] @ a1 + w['h1m_b'])
        a2 = np.tanh(w['h12_w'] @ a1 + w['h12_b'])
        fc = (w['c2_w'] @ m1 + w['h2p_w'] @ a2 + w['h2p_b']).reshape(self.nc, self.n_par)
        p_ctrl = (1.0 + np.tanh(fc) * self.scaler) * self.p_const
        return self.Ns @ p_ctrl


class HyperStage:
    """MPC 와 붙이는 쪽. 관측 기록과 직전 해를 들고 있다가 스테이지별 파라미터를 만든다.

        observe(t, x)          매 주기 지금 상태 (OCP x, 9개) 를 넣는다
        stage_params(t, x)     → (N+1, 30), 쓴 비율 (0 = fallback 만, 1 = 네트워크만)
        set_plan(t, xs)        풀이가 끝나면 예측 궤적 (N+1, 9) 을 넣는다 (다음 P 가 된다)
        reset()                정지·재시작 때
    """

    def __init__(self, path, N, dt, fallback, v_min=0.5, blend=0.3, cx_max=CX_MAX):
        self.net = HyperParams(path)
        self.N, self.dt = N, dt
        self.fallback = np.asarray(fallback, dtype=np.float64)
        assert self.fallback.size == self.net.n_par
        self.v_min, self.blend, self.cx_max = v_min, blend, cx_max
        self.t_stage = dt * np.arange(N + 1)
        self.t_nn = self.net.tp * np.arange(self.net.H)
        # 관측은 0.25 s 만 필요하다. 주기가 늦어도 끊기지 않게 넉넉히 둔다.
        self.keep = 1.0
        self.reset()

    def reset(self):
        self.hist_t, self.hist_m = [], []
        self.plan = None

    def observe(self, t, x):
        if self.hist_t and t <= self.hist_t[-1]:
            return
        self.hist_t.append(float(t))
        self.hist_m.append(np.array([x[3], x[4], x[5], x[7], x[8]], dtype=np.float64))
        while self.hist_t[-1] - self.hist_t[0] > self.keep:
            self.hist_t.pop(0)
            self.hist_m.pop(0)

    name = 'hyperpm'

    def clear_plan(self):
        self.plan = None

    def set_plan(self, t, xs):
        xs = np.asarray(xs)
        self.plan = (float(t), xs[:, 7].copy(), xs[:, 8].copy())

    def M(self, t):
        """학습과 같게 t−W·tp … t−tp. 기록 앞은 첫 값, 뒤는 지금 값을 유지 (np.interp)."""
        tg = t - self.net.tp * np.arange(self.net.W, 0, -1)
        T = np.array(self.hist_t)
        A = np.array(self.hist_m)
        return np.stack([np.interp(tg, T, A[:, i]) for i in range(5)], axis=1)

    def P(self, t, x):
        if self.plan is None:
            return np.tile([x[7], x[8]], (self.net.H, 1))
        t0, ws, de = self.plan
        tq = (t - t0) + self.t_nn
        return np.stack([np.interp(tq, self.t_stage, ws), np.interp(tq, self.t_stage, de)], axis=1)

    def weight(self, vx):
        if self.blend <= 0:
            return float(vx >= self.v_min)
        return float(np.clip((vx - self.v_min) / self.blend, 0.0, 1.0))

    def stage_params(self, t, x):
        fb = np.tile(self.fallback, (self.N + 1, 1))
        a = self.weight(x[3])
        if a == 0.0 or not self.hist_t:
            return fb, 0.0
        p_nn = self.net.predict(self.M(t), self.P(t, x))
        if not np.all(np.isfinite(p_nn)):
            return fb, 0.0
        p = np.stack([np.interp(self.t_stage, self.t_nn, p_nn[:, i]) for i in range(p_nn.shape[1])], axis=1)
        if self.cx_max:
            p = clamp_cx(p, self.cx_max)
        return (1.0 - a) * fb + a * p, a
