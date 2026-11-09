#!/usr/bin/env python3
"""
test_vehicle_model.py — 공통 차 모델 (mlcs_mpc.env.vehicle_model) 시험. ROS·차·acados 불필요 (casadi 만).

  python3 test/test_vehicle_model.py

  ① 슬립 0 에서 방어 (slip_eps) 가 0/0 을 막는가 — 없으면 NaN 이고 solver.solve() 가 돌아오지 않는다
  ② 항력만 남기면 가속도가 −Cd/실제 질량 인가 (질량을 실제 값으로 쓴다)
  ③ frenet() 과 cartesian() 의 몸체 식이 같은가 (MPC 와 시뮬레이터가 같은 식)
  ④ HyperMPC 원저자 식과 같은가 — ~/hypermpc_code 가 있을 때만 (없으면 건너뛴다)
"""
import os
import sys

import casadi as cs
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.env.vehicle_model import VehicleModel, default_params  # noqa: E402

CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')
CODE = os.environ.get('HPM_CODE', os.path.expanduser('~/hypermpc_code'))
ZERO = (1.0, 0.0, 0.0, 0.41, 1.0, 0.0)          # vx vy r friction ws delta — 슬립 정확히 0


def test_zero_slip_guard():
    P = default_params()
    raw = VehicleModel(slip_eps=0.0).function('body')
    safe = VehicleModel().function('body')
    assert np.isnan(np.array(raw(*ZERO, P))).any()
    a = [cs.SX.sym(n) for n in 'abcdef']
    f = cs.vertcat(*VehicleModel().body(*a, P))
    j = cs.Function('j', a, [cs.jacobian(f, cs.vertcat(*a))])
    assert np.isfinite(np.array(safe(*ZERO, P))).all()
    assert np.isfinite(np.array(j(*ZERO))).all()
    x = (1.5, 0.1, 0.8, 0.41, 1.6, 0.2)
    assert np.abs(np.array(raw(*x, P)) - np.array(safe(*x, P))).max() < 1e-8
    print('  ✓ 슬립 0 방어')


def test_drag_uses_real_mass():
    m = VehicleModel(mass=3.284)
    f = m.function('body')
    drag = 0.01 + 0.01 * 1.5 + 0.01 * 1.5 ** 2       # 기본 Cd0, Cd1, Cd2
    vx_d = float(f(1.5, 0.0, 0.0, 0.0, 1.5, 0.0, m.p)[0])   # 마찰 0 → 타이어 힘 0, 항력만
    assert abs(vx_d + drag / 3.284) < 1e-12, vx_d
    print('  ✓ 항력 / 실제 질량')


def test_frenet_and_cartesian_agree():
    m = VehicleModel.from_yaml(CFG)
    ff, fc = m.function('frenet'), m.function('cartesian')
    rng = np.random.default_rng(0)
    for _ in range(50):
        vx, vy, r = rng.uniform(0.5, 4), rng.uniform(-.3, .3), rng.uniform(-3, 3)
        ws, d, u = vx * rng.uniform(.9, 1.1), rng.uniform(-.35, .35), rng.uniform(-1, 1, 2)
        a = np.array(ff([0, 0, 0, vx, vy, r, m.friction, ws, d], u, m.p, 0.0)).ravel()
        b = np.array(fc([0, 0, 0.3, vx, vy, r, ws, d], u, m.p)).ravel()
        assert np.allclose(a[3:6], b[3:6], rtol=1e-12, atol=1e-12) and np.allclose(a[7:9], b[6:8])
    print('  ✓ frenet / cartesian 같은 몸체 식')


def test_matches_author_model():
    path = os.path.join(CODE, 'robot_model/car/casadi_car_model_naive.py')
    if not os.path.isfile(path):
        print('  – hypermpc_code 없음 — 건너뜀')
        return
    import importlib.util
    spec = importlib.util.spec_from_file_location('naive', path)
    naive = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(naive)

    class SafeSqrt:         # 원저자 파일의 sqrt 만 0 에서 안전하게 (예전 hypermpc/offline_ocp.py 가 이렇게 썼다)
        def __getattr__(self, k):
            return getattr(cs, k)

        @staticmethod
        def sqrt(x):
            return cs.sqrt(x + 1e-12)
    naive.casadi = SafeSqrt()
    dyn, _ = naive.pacejca_single_track_casadi()
    m = VehicleModel.from_yaml(CFG)
    k = 5.1 / m.mass        # 원저자 식은 질량 5.1 이 박혀 있다 → I_z, Cd 를 환산해 넘긴다
    pa = m.p.copy()
    pa[[0, 2, 3, 4]] *= k
    fb = m.function('body')
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(300):
        vx = rng.uniform(0.2, 5.0)
        x = [vx, rng.uniform(-.4, .4), rng.uniform(-4, 4), rng.uniform(.3, 1.0)]
        u = [vx * rng.uniform(.7, 1.3), rng.uniform(-.4, .4)]
        a = np.array(dyn(x, u, pa)).ravel()[:3]
        b = np.array(fb(*x, *u, m.p)).ravel()
        worst = max(worst, float(np.max(np.abs(a - b) / (1 + np.abs(a)))))
    assert worst < 1e-9, worst
    print(f'  ✓ 원저자 식과 같다 (최대 상대오차 {worst:.1e})')


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        print(f'■ {f.__name__}')
        f()
    print(f'{len(tests)}개 통과')
