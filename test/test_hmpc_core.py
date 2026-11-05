#!/usr/bin/env python3
"""test_hmpc_core.py — hmpc_core 시험 (ROS·차 불필요).  python3 test/test_hmpc_core.py"""
import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.hmpc_core import Track, make_state, solution_ok, to_command  # noqa: E402


def circle(R=2.0, n=100):
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'prep.csv')
    a = np.arange(n) * 2 * np.pi / n
    np.savetxt(p, np.c_[R * a, R * np.cos(a), R * np.sin(a), a + np.pi / 2, 0 * a + 1 / R, 0 * a + 0.5],
               delimiter=',', header='s,x,y,heading,curvature,track_width', comments='')
    return Track(p), R


def test_frenet_sign_and_lap():
    t, R = circle()
    # 반시계 원에서 안쪽(왼쪽)이 n > 0
    s, n, mu = t.frenet(R - 0.1, 0.0, np.pi / 2)
    assert abs(n - 0.1) < 1e-9 and abs(mu) < 1e-9 and abs(s) < 1e-9
    t.reset()
    total, last = 0.0, None
    for a in np.arange(0, 3 * 2 * np.pi, 0.05):     # 세 바퀴
        s, _, _ = t.frenet(R * math.cos(a), R * math.sin(a), a + np.pi / 2)
        if last is not None:
            assert 0 <= s - last < 0.2, (s, last)
        last = s
    assert abs(last - 3 * 2 * np.pi * R) < 0.2


def test_kappa_wraps():
    t, R = circle()
    assert abs(t.kappa_at(np.array([0.0, t.length * 5 + 1.0]))[1] - 1 / R) < 1e-9


def test_state_cog_and_floor():
    t, R = circle()
    lr = 0.15
    x = make_state(t, (R, -lr, np.pi / 2), (0.0, 0.0, 1.0), 0.0, 0.1, lr, 0.25, 0.41)
    assert abs(x[1]) < 1e-9             # 무게중심이 트랙 위
    assert x[3] == 0.25 and x[7] == 0.25    # 서 있어도 분모가 0 이 아니다
    assert abs(x[4] - lr) < 1e-12       # v_y 뒤축 → 무게중심
    assert x[6] == 0.41 and x[8] == 0.1


def test_solution_ok():
    assert solution_ok(0, np.ones(2), np.ones(9))
    assert not solution_ok(4, np.ones(2), np.ones(9))
    assert not solution_ok(0, np.array([np.nan, 0]), np.ones(9))
    assert not solution_ok(0, np.ones(2), np.array([np.inf] * 9))


def test_command_limits():
    assert to_command([9.0, 0.0], 0.7, 0.4, 0.0, 0.1)[0] == 0.7
    assert to_command([-1.0, 0.0], 0.7, 0.4, 0.0, 0.1)[0] == 0.0
    assert to_command([0.5, 0.5], 0.7, 0.4, 0.0, 0.1)[1] == 0.1     # 변화율
    assert to_command([0.5, 0.5], 0.7, 0.4, 0.38, 0.1)[1] == 0.4    # 한계


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        f()
        print(f'ok  {f.__name__}')
    print(f'{len(tests)}개 통과')
