#!/usr/bin/env python3
"""
test_simulator.py — 시뮬레이터 (mlcs_mpc.env.simulator) 시험. ROS·차 불필요 (casadi 만).

  python3 test/test_simulator.py

  ① 서 있다가 출발·정지하는가, 값이 유한한가
  ② 정상 원선회 반경 — v_lo 아래는 운동학 L / tan δ, 그 위는 원래 식 (유한한지만 본다)
  ③ 서보 명령 → 실제 조향 (command_to_steer) 이 mpcc_core.steer_to_command 의 역인가
  ④ 트랙 위 출발점·Frenet·랩 계산
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402
from mlcs_mpc.env.mpc.core import command_to_steer, steer_to_command  # noqa: E402
from mlcs_mpc.env.track import Track  # noqa: E402
from mlcs_mpc.env.simulator import Simulator  # noqa: E402

CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')


def sim(track=None):
    return Simulator(VehicleModel.from_yaml(CFG), track)


def test_start_and_stop():
    s = sim()
    s.reset(0.0, 0.0, 0.0)
    for _ in range(300):               # 3 s, 0.7 m/s
        s.step(0.0, 0.7)
    assert np.all(np.isfinite(s.x))
    vx, vy, r = s.twist()
    assert abs(vx - 0.7) < 0.05 and abs(vy) < 1e-6 and abs(r) < 1e-6, (vx, vy, r)
    x, y, _ = s.pose()
    assert 1.4 < x < 2.1 and abs(y) < 1e-6, (x, y)
    for _ in range(300):
        s.step(0.0, 0.0)
    assert abs(s.twist()[0]) < 0.02, s.twist()
    print(f'  ✓ 출발 0.7 m/s ({x:.2f} m), 정지')


def test_circle_radius():
    s = sim()
    for cmd, v in ((0.2, 0.15), (0.4, 0.15), (0.3, 1.0), (0.4, 1.5)):
        s.reset(0.0, 0.0, 0.0)
        for _ in range(500):
            s.step(cmd, v)
        vx, _, r = s.twist()
        R = vx / r
        R_kin = s.model.L / math.tan(command_to_steer(cmd, s.map_cmd, s.map_act))
        print(f'    명령 {cmd:.1f} {v:.2f} m/s → 반경 {R:.3f} m (운동학 {R_kin:.3f})')
        assert np.isfinite(R) and R > 0
        if v <= 0.2:
            assert abs(R - R_kin) / R_kin < 0.03, (R, R_kin)
    print('  ✓ 원선회 반경')


def test_steer_map_inverse():
    s = sim()
    for a in np.linspace(-0.37, 0.37, 15):
        cmd = steer_to_command(a, s.map_cmd, s.map_act)
        assert abs(command_to_steer(cmd, s.map_cmd, s.map_act) - a) < 1e-9
    print('  ✓ 조향 맵 역함수')


def test_track_lap():
    # 반경 2 m 원 트랙, 운동학 반경에 맞춘 조향으로 한 바퀴
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'prep.csv')
    R, n = 2.0, 200
    a = np.arange(n) * 2 * np.pi / n
    np.savetxt(p, np.c_[R * a, R * np.cos(a), R * np.sin(a), a + np.pi / 2, 0 * a + 1 / R, 0 * a + 1.0],
               delimiter=',', header='s,x,y,heading,curvature,track_width', comments='')
    s = sim(Track(p))
    s.reset_to_s(0.0)
    info = s.step(0.0, 0.0)
    assert abs(info['n']) < 1e-6 and not info['on_wall']
    cmd = steer_to_command(math.atan(s.model.L / R), s.map_cmd, s.map_act)
    for _ in range(12000):
        info = s.step(cmd, 0.15)
        if s.laps:
            break
    assert s.laps, '한 바퀴를 못 돌았다'
    assert s.wall_hits == 0
    print(f'  ✓ 원 트랙 한 바퀴 {s.laps[0]:.1f} s (이론 {2 * math.pi * R / 0.15:.1f} s), 벽 0')


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        print(f'■ {f.__name__}')
        f()
    print(f'{len(tests)}개 통과')
