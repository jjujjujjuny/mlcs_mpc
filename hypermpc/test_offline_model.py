#!/usr/bin/env python3
"""
test_offline_model.py — offline_ocp 의 저자 모델 포장 회귀 시험 (차·빌드 불필요)

  ~/hmpc-venv/bin/python hypermpc/test_offline_model.py

슬립이 정확히 0 이면 방어 없이는 NaN 이고 solver.solve() 가 돌아오지 않는다.
"""
import os
import sys

import casadi as cs
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import offline_ocp as o  # noqa: E402

CODE = os.environ.get('HPM_CODE', os.path.expanduser('~/hypermpc_code'))
P = o.default_params()
ZERO = ([1.0, 0.0, 0.0, 0.41], [1.0, 0.0])


def test_zero_slip():
    raw, _ = o.load_naive_dynamics(CODE, guard=False)
    safe, _ = o.load_naive_dynamics(CODE, guard=True)
    assert np.isnan(np.array(raw(*ZERO, P))).any()      # 저자 식의 문제가 그대로인지
    x, u = cs.MX.sym('x', 4), cs.MX.sym('u', 2)
    f = safe(x, u, P)
    j = cs.Function('j', [x, u], [cs.jacobian(f, cs.vertcat(x, u))])
    assert np.isfinite(np.array(safe(*ZERO, P))).all()
    assert np.isfinite(np.array(j(*ZERO))).all()


def test_same_as_author_with_slip():
    raw, _ = o.load_naive_dynamics(CODE, guard=False)
    safe, _ = o.load_naive_dynamics(CODE, guard=True)
    x, u = [1.5, 0.1, 0.8, 0.41], [1.6, 0.2]
    assert np.abs(np.array(raw(x, u, P)) - np.array(safe(x, u, P))).max() < 1e-8


def test_mass_scaling():
    # 저자 식은 m=5.1. 맞게 넘기면 항력 가속도는 Cd/실제 질량, r_dot 은 실제 질량/I_z 에 비례.
    safe, _ = o.load_naive_dynamics(CODE, guard=True)
    scaled = np.array(cs.DM(o.scale_for_mass(cs.DM(P)))).ravel()
    x, u = [1.5, 0.0, 0.0, 0.0], [1.5, 0.0]     # friction 0 → 타이어 힘 0, 항력만
    drag = 0.01 + 0.01 * 1.5 + 0.01 * 1.5 ** 2
    assert abs(float(safe(x, u, scaled)[0]) + drag / o.MASS) < 1e-12
    x, u = [1.5, 0.1, 0.8, 0.41], [1.6, 0.2]
    base = np.array(safe(x, u, P)).ravel()
    fixed = np.array(safe(x, u, scaled)).ravel()
    assert abs(fixed[2] / base[2] - o.MASS / o.AUTHOR_MASS) < 1e-9


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        f()
        print(f'ok  {f.__name__}')
    print(f'{len(tests)}개 통과')
