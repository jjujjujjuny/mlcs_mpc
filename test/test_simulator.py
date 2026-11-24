#!/usr/bin/env python3
"""
test_simulator.py — 시뮬레이터 (mlcs_mpc.env.simulator) 시험. ROS·차 불필요 (casadi 만).

  python3 test/test_simulator.py

  ① 서 있다가 출발·정지하는가, 값이 유한한가
  ② 정상 원선회 반경 — v_lo 아래는 운동학 L / tan δ, 그 위는 원래 식 (유한한지만 본다)
  ③ 서보 명령 → 실제 조향 (command_to_steer) 이 mpcc_core.steer_to_command 의 역인가
  ④ 트랙 위 출발점·Frenet·랩 계산
  ⑤ 저수준 제어기 (env/lowlevel.py) — 기본값은 예전 식 (모델에 직접 ws_ref·δ_ref) 과 같다, 조향 지연·뱅뱅·각속도 한계,
     gym 속도 P 제어·가속 한계, VESC 바퀴 모델에서 속도 명령 지연
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
from mlcs_mpc.env.lowlevel import LowLevelConfig  # noqa: E402

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


def test_lowlevel_default_is_old_model():
    """기본 저수준 = 공용 모델에 (ws_ref, δ_ref) 를 그대로 (예전 시뮬레이터) — 같은 RK4 를 손으로 돌려 비교."""
    s = sim()
    s.reset(0.0, 0.0, 0.0)
    x = s.x.copy()
    f = lambda xx, u: np.array(s.f(xx, u, s.p)).ravel()  # noqa: E731
    cmds = [(0.3 * math.sin(0.02 * k), 1.5 + 0.5 * math.cos(0.03 * k)) for k in range(300)]
    for sc, v in cmds:
        s.step(sc, v)
        u = [s.vesc.speed(v) if s.vesc else v, command_to_steer(sc, s.map_cmd, s.map_act)]
        h = s.dt / 10
        for _ in range(10):
            k1 = f(x, u); k2 = f(x + 0.5 * h * k1, u); k3 = f(x + 0.5 * h * k2, u); k4 = f(x + h * k3, u)  # noqa: E702
            x = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi
    err = float(np.max(np.abs(x - s.x)))
    assert err < 1e-9, err
    print(f'  ✓ 기본 저수준 == 예전 식 (3 s, 차 최대 {err:.1e})')


def test_lowlevel_steer_delay_and_rate():
    m = VehicleModel.from_yaml(CFG)
    # 지연 30 ms: 그 전에는 조향이 움직이지 않는다
    s = Simulator(m, lowlevel=LowLevelConfig(steer_delay=0.03))
    s.reset(0.0, 0.0, 0.0)
    for _ in range(100):
        s.step(0.0, 1.0)
    s.step(0.3, 1.0); s.step(0.3, 1.0)  # noqa: E702
    assert abs(s.steer) < 1e-12, s.steer
    s.step(0.3, 1.0); s.step(0.3, 1.0)  # noqa: E702
    assert s.steer > 0.01, s.steer
    # gym 뱅뱅 3.2 rad/s: 0.05 s 에 0.16 rad, 목표 근처에서 멈춘다
    s = Simulator(m, lowlevel=LowLevelConfig(steer_model='gym', steer_rate=3.2))
    s.reset(0.0, 0.0, 0.0)
    for _ in range(5):
        s.step(0.4, 0.0)
    assert abs(s.steer - 0.16) < 2e-3, s.steer
    for _ in range(50):
        s.step(0.4, 0.0)
    tgt = command_to_steer(0.4, s.map_cmd, s.map_act)
    assert abs(s.steer - tgt) < 3.2 * s.substep, (s.steer, tgt)
    # lag + 각속도 한계 1 rad/s: 큰 계단에서 기울기가 1 rad/s 로 묶인다
    s = Simulator(m, lowlevel=LowLevelConfig(steer_rate=1.0))
    s.reset(0.0, 0.0, 0.0)
    for _ in range(10):
        s.step(0.4, 0.0)
    assert abs(s.steer - 0.1) < 2e-3, s.steer
    print(f'  ✓ 조향 지연 30 ms, gym 뱅뱅 3.2 rad/s (0.05 s → 0.160), 각속도 한계 1 rad/s (0.1 s → {s.steer:.3f})')


def test_lowlevel_gym_speed():
    m = VehicleModel.from_yaml(CFG)
    s = Simulator(m, lowlevel=LowLevelConfig.gym())
    s.reset(0.0, 0.0, 0.0)
    s.step(0.0, 3.0)
    # ω̇ = min(4.755·(3 − ω), 9.51) → 처음엔 9.51 m/s² 로 묶이다가 오차 2 m/s 아래에서 P
    assert abs(s.wheel_speed - 9.51 * 0.01) < 2e-3, s.wheel_speed
    for _ in range(200):
        s.step(0.0, 3.0)
    assert abs(s.wheel_speed - 3.0) < 0.01, s.wheel_speed
    w0 = s.wheel_speed
    s.step(0.0, 2.9)                          # 감속 P: 오차 0.1 이 exp(−19.02 t) 로 → 10 ms 평균 1.73 m/s² (한계 안)
    dec = (w0 - s.wheel_speed) / 0.01
    assert abs(dec - 0.1 * (1 - math.exp(-19.02 * 0.01)) / 0.01) < 0.02, dec
    print(f'  ✓ gym 속도: 가속 한계 9.51 m/s², P 가속 4.755·e / 감속 19.02·e, 3 m/s 수렴')


def test_lowlevel_vesc_speed_delay():
    from mlcs_mpc.env.drivetrain import Drivetrain
    from mlcs_mpc.env.vesc import Vesc
    m = VehicleModel.from_yaml(CFG)
    vesc = Vesc()
    out = {}
    for dly in (0.0, 0.05):
        s = Simulator(m, vesc=vesc, wheel_model='vesc', drivetrain=Drivetrain(vesc),
                      lowlevel=LowLevelConfig(speed_delay=dly))
        s.reset(0.0, 0.0, 0.0)
        ws = []
        for _ in range(150):
            s.step(0.0, 2.0)
            ws.append(s.wheel_speed)
        assert np.all(np.isfinite(s.x))
        out[dly] = np.array(ws)
    i0 = int(np.argmax(out[0.0] > 0.05))
    i5 = int(np.argmax(out[0.05] > 0.05))
    assert 4 <= i5 - i0 <= 6, (i0, i5)
    assert abs(out[0.05][-1] - out[0.0][-1]) < 0.05
    print(f'  ✓ VESC 바퀴 + 속도 명령 지연 50 ms: 출발 {i0 * 10} → {i5 * 10} ms, 끝 ω {out[0.05][-1]:.2f} m/s')


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        print(f'■ {f.__name__}')
        f()
    print(f'{len(tests)}개 통과')
