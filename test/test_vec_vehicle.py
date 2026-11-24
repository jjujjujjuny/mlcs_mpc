#!/usr/bin/env python3
"""
test_vec_vehicle.py — 벡터화 (torch) 차 모델·트랙·RL 환경이 원래 (casadi·numpy) 것과 같은가.

  python3 test/test_vec_vehicle.py          torch 가 있는 파이썬 (macOS conda 환경 mlcs_ros)

  ① 미분식 — VecVehicle.deriv == VehicleModel.cartesian (임의 상태 2000 개, float64)
  ② 적분 — 1 초 동안 VecVehicle.rk4 == Simulator._rhs (기본 저수준 = 모델에 그대로, 인자 δ_cmd·ω_cmd) 로 같은 RK4 (float64, 같은 1 ms)
  ③ float32 + 5 ms (학습 기본) 가 float64 + 0.5 ms 와 3 초 위치 0.1 mm 안
  ④ Frenet — VecTrack.frenet == Track.frenet (rl17_track)
  ⑤ 환경 — 관측 모양·유한, 벽을 넘으면 보상 −1·종료, 스스로 리셋하지 않음, 랜덤화가 환경마다 다름, actuator='none' 은 지연 없음
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))

import torch  # noqa: E402

from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402
from mlcs_mpc.env.vec.vehicle import VecVehicle  # noqa: E402

YAML = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')


def model():
    return VehicleModel.from_yaml(YAML)


def random_states(B, seed=0):
    rng = np.random.default_rng(seed)
    x = np.c_[rng.normal(size=(B, 3)), rng.uniform(-0.5, 4, B), rng.normal(0, 0.3, B), rng.normal(0, 1.5, B),
              rng.uniform(0, 4.5, B), rng.uniform(-0.38, 0.38, B)]
    u = np.c_[rng.uniform(0, 4.5, B), rng.uniform(-0.38, 0.38, B)]
    return x, u


def test_deriv_matches_casadi():
    m = model()
    f = m.function('cartesian', v_lo=0.2, v_hi=0.4)
    x, u = random_states(2000)
    ref = np.array([np.array(f(x[i], u[i], m.p)).ravel() for i in range(len(x))])
    vv = VecVehicle.from_model(m, len(x), dtype=torch.float64)
    got = vv.deriv(torch.tensor(x), torch.tensor(u), vv.base).numpy()
    err = float(np.max(np.abs(got - ref) / (1 + np.abs(ref))))
    print(f'  ① 미분식 상대 오차 최대 {err:.1e}')
    assert err < 1e-9


def test_rk4_matches_simulator():
    from mlcs_mpc.env.simulator import Simulator
    m = model()
    sim = Simulator(m, None, dt=0.05, substep=0.001)
    x, u = random_states(20, seed=3)
    x[:, 3] = np.abs(x[:, 3]) + 0.5
    vv = VecVehicle.from_model(m, len(x), dtype=torch.float64)
    xt = torch.tensor(x)
    for _ in range(20):                         # 1 초
        xt = vv.rk4(xt, torch.tensor(u), vv.base, 0.05, 0.001)
    worst = 0.0
    for i in range(len(x)):
        sim.x = x[i].copy()
        for _ in range(20):
            h = 0.001
            xs = sim.x
            for _ in range(50):
                k1 = sim._rhs(xs, u[i][1], u[i][0])
                k2 = sim._rhs(xs + 0.5 * h * k1, u[i][1], u[i][0])
                k3 = sim._rhs(xs + 0.5 * h * k2, u[i][1], u[i][0])
                k4 = sim._rhs(xs + h * k3, u[i][1], u[i][0])
                xs = xs + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            xs[2] = (xs[2] + math.pi) % (2 * math.pi) - math.pi
            sim.x = xs
        d = np.abs(xt[i].numpy() - sim.x)
        d[2] = abs((d[2] + math.pi) % (2 * math.pi) - math.pi)
        worst = max(worst, float(d.max()))
    print(f'  ② 1 초 적분 차 최대 {worst:.1e}')
    assert worst < 1e-8


def test_float32_substep():
    m = model()
    B = 200
    g = torch.Generator().manual_seed(1)
    X0 = torch.rand(B, generator=g, dtype=torch.float64) * 3
    A = torch.rand(B, generator=g, dtype=torch.float64) * 3.5
    W = torch.rand(B, generator=g, dtype=torch.float64) * 2
    S = (torch.rand(B, generator=g, dtype=torch.float64) * 2 - 1) * 0.3

    def run(sub, dtype):
        vv = VecVehicle.from_model(m, B, dtype=dtype)
        x = torch.zeros(B, 8, dtype=dtype)
        x[:, 3] = X0.to(dtype)
        x[:, 6] = x[:, 3]
        for k in range(60):
            u = torch.stack([A * (1 + 0.3 * torch.sin(W * k * 0.05)), S * torch.cos(W * k * 0.05)], 1).to(dtype)
            x = vv.rk4(x, u, vv.base, 0.05, sub)
        return x.double()
    ref, got = run(0.0005, torch.float64), run(0.005, torch.float32)
    err = float((got[:, :2] - ref[:, :2]).norm(dim=1).max())
    print(f'  ③ float32·5 ms vs float64·0.5 ms — 3 초 위치 차 최대 {err * 1000:.3f} mm')
    assert err < 1e-4


def test_frenet_matches_track():
    from mlcs_mpc.env.track import Track, find
    from mlcs_mpc.env.vec.track import VecTrack
    tr = Track(find('rl17_track'))
    vt = VecTrack(tr, dtype=torch.float64)
    rng = np.random.default_rng(5)
    s0 = rng.uniform(0, vt.length, 300)
    n0 = rng.uniform(-0.45, 0.45, 300)
    x, y, h = vt.pose_at(torch.tensor(s0), torch.tensor(n0))
    yaw = h + torch.tensor(rng.uniform(-0.5, 0.5, 300))
    s, n, mu, _ = vt.frenet(x, y, yaw, torch.full((300,), -1, dtype=torch.long))
    es, en, emu = [], [], []
    for i in range(300):
        tr.reset()
        s1, n1, mu1 = tr.frenet(float(x[i]), float(y[i]), float(yaw[i]))
        es.append(abs((s1 - float(s[i]) + vt.length / 2) % vt.length - vt.length / 2))
        en.append(abs(n1 - float(n[i])))
        emu.append(abs(mu1 - float(mu[i])))
    ds = np.abs((s.numpy() - s0 + vt.length / 2) % vt.length - vt.length / 2)
    print(f'  ④ Frenet: Track 과 차 s {max(es):.1e}  n {max(en):.1e}  mu {max(emu):.1e} / 넣은 (s, n) 과 s {ds.max() * 1000:.1f} mm '
          f'n {np.abs(n.numpy() - n0).max() * 1000:.1f} mm')
    assert max(es) < 1e-6 and max(en) < 1e-6 and max(emu) < 1e-6
    # (s, n) → (x, y) → Frenet 왕복: 곡선에서 heading 을 점 사이 선형 보간하므로 |n| 0.45 m 에서 cm 이하 (Track 도 같다)
    assert ds.max() < 0.02 and np.abs(n.numpy() - n0).max() < 0.01


def test_env_api():
    from mlcs_mpc.env.vec.race_env import RaceEnvConfig, VecRaceEnv
    env = VecRaceEnv(RaceEnvConfig(num_envs=16, dr='friction', dr_sigma=0.05), seed=0)
    obs = env.reset()
    assert obs.shape == (16, env.obs_dim) and torch.isfinite(obs).all()
    assert env.P.friction.std() > 0                 # 환경마다 다른 마찰
    # 오른쪽으로 꺾고 가속 → 언젠가 벽 → 보상 −1, 종료, 리셋 전까지 상태 유지
    hit = None
    for _ in range(200):
        obs, r, te, tr, info = env.step(torch.tensor([[-1.0, 1.0]]).repeat(16, 1))
        assert torch.isfinite(obs).all()
        if te.any():
            hit = (r, te, info)
            break
    assert hit is not None, '벽에 닿지 않았다'
    r, te, info = hit
    assert torch.all(r[te] == -1.0) and torch.all(info['out'][te])
    before = env.x[te].clone()
    obs2 = env.reset(te.nonzero().squeeze(1))
    assert not torch.equal(before, env.x[te]) and obs2.shape == obs.shape
    # actuator='none' — 한 주기 뒤 ws, δ 가 명령 그대로
    env2 = VecRaceEnv(RaceEnvConfig(num_envs=4, actuator='none', dr='none'), seed=1)
    env2.reset()
    env2.step(torch.tensor([[0.5, 1.0]]).repeat(4, 1))
    assert torch.allclose(env2.x[:, 7], torch.full((4,), 0.5 * env2.max_steer), atol=1e-6)
    assert torch.allclose(env2.x[:, 6], env2.ws_ref, atol=1e-6)
    print(f'  ⑤ 환경: 관측 {env.obs_dim}, 벽 → −1·종료 OK, 리셋 OK, 마찰 표준편차 {float(env.P.friction.std()):.4f}, '
          f'actuator none OK')


def test_vesc_matches_simulator():
    """actuator='vesc' (torch, 바퀴 1 ms + 차체 RK4 5 ms) == Simulator(wheel_model='vesc') (numpy, 1 ms) — 직선 가속·유지·정지."""
    from mlcs_mpc.env.simulator import Simulator
    from mlcs_mpc.env.vec.race_env import RaceEnvConfig, VecRaceEnv
    from mlcs_mpc.env.vesc import Vesc
    m = model()
    env = VecRaceEnv(RaceEnvConfig(num_envs=1, actuator='vesc', dr='none', reset_v=(0.0, 0.0), dtype='float64'))
    env.reset()
    env.x[:] = 0.0
    env.dtr.reset(torch.arange(1))
    env.u_applied[:] = 0.0
    sim = Simulator(m, None, dt=0.05, substep=0.001, vesc=Vesc.load(), wheel_model='vesc')
    sim.x = np.zeros(8)
    cmds = [min(0.15 * k, 3.0) for k in range(40)] + [3.0] * 20 + [0.0] * 40      # 3 m/s 까지 → 유지 → 정지
    worst_v = worst_w = 0.0
    for c in cmds:
        u = torch.tensor([[c, 0.0]], dtype=torch.float64)
        env.ws_ref[:] = c
        env.x = env._integrate(u)
        sim.step(0.0, c)
        worst_v = max(worst_v, abs(float(env.x[0, 3]) - sim.x[3]))
        worst_w = max(worst_w, abs(float(env.x[0, 6]) - sim.x[6]))
    print(f'  ⑥ VESC 저수준: torch vs Simulator 5 초 — vx 차 최대 {worst_v:.3f}  ω 차 최대 {worst_w:.3f} m/s '
          f'(끝 vx {float(env.x[0, 3]):.3f} / {sim.x[3]:.3f})')
    assert worst_v < 0.05 and worst_w < 0.08


def test_stuck_is_penalized():
    """멈춰 서면 1 초 뒤 종료 + −1 (페널티가 없으면 '멈춰서 종료' 가 탈출구가 된다)."""
    from mlcs_mpc.env.vec.race_env import RaceEnvConfig, VecRaceEnv
    env = VecRaceEnv(RaceEnvConfig(num_envs=2, dr='none', reset_v=(0.0, 0.0)), seed=0)
    env.reset()
    for k in range(40):
        obs, r, te, tr, info = env.step(torch.tensor([[0.0, -1.0], [0.0, -1.0]]))
        if te.all():
            break
    assert te.all() and info['stuck'].all() and torch.all(r == -1.0), (k, te, r)
    print(f'  ⑦ 멈춤: {(k + 1) * 0.05:.2f} s 에 종료, 보상 −1')


def test_command_drop():
    """명령 끊김: 끊긴 주기에는 차가 직전 명령을 유지 (lag·vesc 둘 다), 끊김은 연속으로 이어진다."""
    from mlcs_mpc.env.vec.race_env import RaceEnvConfig, VecRaceEnv
    for act in ('lag', 'vesc'):
        env = VecRaceEnv(RaceEnvConfig(num_envs=256, actuator=act, dr='none', reset_v=(1.0, 1.0),
                                       dr_terms=dict(drop=0.2, drop_len=3.0)), seed=3)
        env.reset()
        runs, cur, n_drop = [], 0, 0
        for k in range(60):
            a = torch.zeros(256, 2)
            a[:, 0] = 1.0 if (k // 5) % 2 == 0 else -1.0          # 조향을 계속 바꾼다
            u_before = env.u_applied.clone()
            env.step(a)
            d = env.dropped
            n_drop += int(d.sum())
            assert torch.equal(env.u_applied[d], u_before[d]), '끊긴 환경의 명령이 바뀌었다'
            if bool(d[0]):
                cur += 1
            elif cur:
                runs.append(cur)
                cur = 0
        frac = n_drop / (256 * 60)
        print(f'  ⑧ 명령 끊김 ({act}): 끊긴 주기 비율 {frac:.2f}, 연속 길이 평균 {np.mean(runs) if runs else 0:.1f} 주기')
        assert 0.2 < frac < 0.6 and (not runs or np.mean(runs) > 1.5)


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    n = 0
    for t in tests:
        print(t.__name__)
        t()
        n += 1
    print(f'{n}개 통과')
