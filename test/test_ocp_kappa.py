#!/usr/bin/env python3
"""
test_ocp_kappa.py — OCP 가 곡률 κ(s) 를 제대로 쓰는가 (casadi + acados_template, 빌드 불필요).

  python3 test/test_ocp_kappa.py [트랙 이름]

  ① OCP 동역학 식 (build_ocp 의 f_expl) 에서 κ 를 거꾸로 꺼내 (μ' = r − κ·ṡ) 트랙 κ(s) 와 맞는지.
     s 는 랩을 넘어 늘어나므로 −L … 3L 에서 본다 (감기).
  ② 예측 구간 전체에서 맞는지 — 같은 입력열 u_k 로
       Frenet 모델 (OCP 와 같은 식, κ(s) 를 상태 s 로 매번 찾음) 을 적분해 (s, n) → (x, y) 로 바꾼 경로와
       Cartesian 모델 (곡률 없음, x·y 로 바로 적분) 의 경로를 비교한다. κ 가 맞으면 같은 길이어야 한다.
     비교용으로 κ 를 출발점 값 하나로 고정한 Frenet 적분도 같이 계산한다 (κ 를 한 값만 넣으면 이만큼 어긋난다).
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
import casadi as cs  # noqa: E402
from mlcs_mpc.env.config import MpccConfig  # noqa: E402
from mlcs_mpc.env.mpc.acados_ocp import build_ocp, kappa_function  # noqa: E402
from mlcs_mpc.env.track import Track, find  # noqa: E402
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402

CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')
NAME = sys.argv[1] if len(sys.argv) > 1 else 'comp1_track'


def setup():
    track = Track(find(NAME))
    model = VehicleModel.from_yaml(CFG)
    cfg = MpccConfig.from_yaml(CFG)
    return track, model, cfg


def test_ocp_kappa_matches_track():
    track, model, cfg = setup()
    grid = track.kappa_grid()
    ocp = build_ocp(model, grid, cfg)
    am = ocp.model
    f = cs.Function('f', [am.x, am.u, am.p], [am.f_expl_expr])
    L = track.length
    rng = np.random.default_rng(0)
    S, K = grid
    worst_node, worst_mid = 0.0, 0.0
    for lap in (-1, 0, 1, 2):
        for i in range(0, len(S) - 1, 3):
            for frac, which in ((0.0, 'node'), (0.5, 'mid')):
                s = S[i] + frac * (S[i + 1] - S[i]) + lap * L
                x = [s, rng.uniform(-0.1, 0.1), rng.uniform(-0.2, 0.2), 1.5, 0.05, 0.8,
                     model.friction, 1.5, 0.1]
                d = np.array(f(x, [1.5, 0.1], model.p)).ravel()
                k_ocp = (x[5] - d[2]) / d[0]                            # μ' = r − κ ṡ
                k_ref = K[i] if which == 'node' else 0.5 * (K[i] + K[i + 1])
                err = abs(k_ocp - k_ref)
                if which == 'node':
                    worst_node = max(worst_node, err)
                else:
                    worst_mid = max(worst_mid, err)
    print(f'  OCP κ − 트랙 κ: 격자점 최대 {worst_node:.2e}, 사이 최대 {worst_mid:.3f} /m (스플라인 ↔ 선형 보간 차)  '
          f'— s ∈ [−L, 3L] (L {L:.2f} m)')
    assert worst_node < 1e-6, worst_node
    assert worst_mid < 0.05, worst_mid


def rk4(fun, x, h):
    k1 = fun(x)
    k2 = fun(x + h / 2 * k1)
    k3 = fun(x + h / 2 * k2)
    k4 = fun(x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def test_horizon_frenet_equals_cartesian():
    track, model, cfg = setup()
    kfn = kappa_function(*track.kappa_grid())
    ff, fc = model.function('frenet'), model.function('cartesian')
    N, dt, sub = cfg.N, cfg.dt, 8
    h = dt / sub
    p = model.p
    rows = []
    for s0 in np.linspace(0, track.length, 12, endpoint=False):
        # 기준 라인 위, 헤딩 맞춤, 1.5 m/s. 입력: 그 곡률을 따라가는 조향 + 조금 흔들림
        x0, y0 = track.xy(s0, 0.0)
        h0 = track.heading_at(s0)
        v = 1.5
        u_seq = []
        for k in range(N):
            sk = s0 + v * k * dt
            u_seq.append([v, float(np.clip(math.atan(model.L * track.kappa_at(sk)), -0.37, 0.37))
                          + 0.05 * math.sin(0.3 * k)])
        xf = np.array([s0, 0.0, 0.0, v, 0.0, 0.0, model.friction, v, u_seq[0][1]])
        xfc = xf.copy()                                                  # κ 고정 비교용
        k_fixed = float(kfn(s0))
        xc = np.array([float(x0), float(y0), h0, v, 0.0, 0.0, v, u_seq[0][1]])
        dev, dev_fixed = 0.0, 0.0
        for k in range(N):
            u = u_seq[k]
            for _ in range(sub):
                xf = rk4(lambda z: np.array(ff(z, u, p, float(kfn(z[0])))).ravel(), xf, h)
                xfc = rk4(lambda z: np.array(ff(z, u, p, k_fixed)).ravel(), xfc, h)
                xc = rk4(lambda z: np.array(fc(z, u, p)).ravel(), xc, h)
            a = np.array(track.xy(xf[0], xf[1]), float)
            b = np.array(track.xy(xfc[0], xfc[1]), float)
            dev = max(dev, float(np.hypot(*(a - xc[:2]))))
            dev_fixed = max(dev_fixed, float(np.hypot(*(b - xc[:2]))))
        rows.append((s0, float(kfn(s0)), xf[0] - s0, dev, dev_fixed))
    print(f'  예측 구간 {N} × {dt} s ({N * dt:.2f} s) 끝까지, 12 곳 출발:')
    for s0, k0, ds, d, dfx in rows:
        print(f'    s0 {s0:5.2f}  κ0 {k0:+.2f}  진행 {ds:4.2f} m   Frenet(κ(s)) − Cartesian {d * 100:5.2f} cm   '
              f'κ 고정이면 {dfx * 100:6.1f} cm')
    worst = max(r[3] for r in rows)
    worst_fixed = max(r[4] for r in rows)
    print(f'  최대: κ(s) {worst * 100:.2f} cm  /  κ 고정 {worst_fixed * 100:.1f} cm')
    assert worst < 0.03, worst              # 3 cm — 기준 라인 (x, y) 의 선형 보간 오차 수준


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        print(f'■ {f.__name__}')
        f()
    print(f'{len(tests)}개 통과')
