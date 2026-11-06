#!/usr/bin/env python3
"""test_hmpc_core.py — hmpc_core 시험 (ROS·차 불필요).  python3 test/test_hmpc_core.py"""
import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.mocap_health import PoseHealth  # noqa: E402
from mlcs_mpc.hmpc_core import Track, make_state, solution_ok, steer_to_command, to_command  # noqa: E402


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


def test_mu_smooth_between_points():
    # 점 간격 12.6 cm, R 2 m. 트랙 위를 접선 방향으로 달리면 mu 는 어디서나 0 이어야 한다.
    t, R = circle()
    for a in np.arange(0, 2 * np.pi, 0.013):
        _, n, mu = t.frenet(R * math.cos(a), R * math.sin(a), a + np.pi / 2)
        assert abs(mu) < 1e-3 and abs(n) < 2e-3, (a, n, mu)


def test_closing_point_dropped():
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'prep.csv')
    a = np.linspace(0, 2 * np.pi, 21)
    np.savetxt(p, np.c_[2 * a, 2 * np.cos(a), 2 * np.sin(a), a + np.pi / 2, 0 * a + 0.5, 0 * a + 0.5],
               delimiter=',', header='s,x,y,heading,curvature,track_width', comments='')
    t = Track(p)
    assert t.n == 20 and abs(t.length - 4 * np.pi) < 1e-9
    assert np.all(np.diff(np.r_[t.s, t.length]) > 0)


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


MAP = ([0.0, 0.2, 0.3, 0.4], [0.0, 0.2021, 0.2972, 0.3776])


def test_command_limits():
    assert to_command([9.0, 0.0], 0.7, 0.4, 0.0, 0.1, *MAP)[0] == 0.7
    assert to_command([-1.0, 0.0], 0.7, 0.4, 0.0, 0.1, *MAP)[0] == 0.0
    assert to_command([0.5, 0.3], 0.7, 0.4, 0.0, 0.1, *MAP)[1] == 0.1     # 변화율
    assert to_command([0.5, 0.5], 0.7, 0.4, 0.38, 0.1, *MAP)[1] == 0.4    # 한계


def test_steer_map():
    assert abs(steer_to_command(0.3776, *MAP) - 0.4) < 1e-12
    assert abs(steer_to_command(-0.3776, *MAP) + 0.4) < 1e-12
    assert steer_to_command(0.0, *MAP) == 0.0
    assert abs(steer_to_command(0.2972, *MAP) - 0.3) < 1e-12
    assert steer_to_command(0.5, *MAP) == 0.4       # 맵 밖은 끝값


def _frames(n, t0=0.0, x0=0.0, v=0.0, dt=0.01):
    # 세워 둔 차도 잡음이 있다 (v=0 이어도 프레임마다 값이 다르다)
    rng = np.random.default_rng(int(t0 * 1000) + 1)
    for i in range(n):
        e = rng.normal(0, 2e-5, 2)
        yield (t0 + i * dt, x0 + v * i * dt + e[0], e[1], 0.1, 0.0, 0.0, 0.0, 1.0)


def test_health_clean():
    h = PoseHealth()
    assert all(h.update(*f)[0] for f in _frames(500))
    assert all(h.update(*f)[0] for f in _frames(500, 5.0, 0.0, 3.0))


def test_health_frozen_and_recover():
    h = PoseHealth(freeze_frames=5, recover_time=0.3)
    for f in _frames(100):
        h.update(*f)
    res = [h.update(1.0 + i * 0.01, 0.5, 0.5, 0.1, 0.0, 0.0, 0.0, 1.0) for i in range(20)]
    # 첫 프레임은 위치가 튀어 바로 불량이고, 그 뒤 멈춘 동안 내내 불량이다.
    assert not any(r[0] for r in res)
    assert sum(bool(r[1]) for r in res) == 1
    good = [h.update(*f)[0] for f in _frames(50, 1.2, 0.5)]
    assert good[0] is False and good[-1] is True
    assert 25 <= good.index(True) <= 36     # recover_time 만큼 기다린다


def test_health_two_repeats_are_fine():
    h = PoseHealth(freeze_frames=5)
    for f in _frames(50):
        h.update(*f)
    f = (0.5, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0)
    for i in range(4):      # 같은 값 4 프레임
        assert h.update(0.5 + i * 0.01, *f[1:])[0]


def test_health_yaw_jump():
    h = PoseHealth()
    assert h.update(0.0, 0, 0, 0, 0, 0, 0, 1.0)[0]
    a = 0.5     # 10 ms 에 0.5 rad
    ok, why = h.update(0.01, 1e-5, 0, 0, 0, 0, math.sin(a / 2), math.cos(a / 2))
    assert not ok and '자세' in why


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        f()
        print(f'ok  {f.__name__}')
    print(f'{len(tests)}개 통과')
