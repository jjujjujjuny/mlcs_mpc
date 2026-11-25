#!/usr/bin/env python3
"""env/estimation.py (실차 상태 추정) + env/timing.py (비동기 사건 큐) 시험. acados 없이 (간단한 추종 컨트롤러).

  python3 test/test_realtime.py
"""
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))

from mlcs_mpc.env.estimation import EstimatorConfig, PoseKF, StateEstimator  # noqa: E402
from mlcs_mpc.env.lowlevel import LowLevelConfig  # noqa: E402
from mlcs_mpc.env.simulator import Simulator  # noqa: E402
from mlcs_mpc.env.timing import Timing, TimingConfig, run_realtime  # noqa: E402
from mlcs_mpc.env.track import Track, find  # noqa: E402
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402

CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')
TRACK = '1108_7_track_mocap'


def _drive(mode, vy='kf', T=12.0, timing=None, seed=0):
    tr = Track(find(TRACK))
    sim = Simulator(VehicleModel.from_yaml(CFG), tr, lowlevel=LowLevelConfig.from_yaml())
    sim.reset_to_s(0.0)
    est = StateEstimator(EstimatorConfig(mode=mode, vy_mode=vy), rng=np.random.default_rng(seed))
    est.reset(sim)
    E = []

    def control(pose, twist, ws, t):
        E.append((t, np.array(twist), np.array(sim.twist())))
        x, y, yaw = pose
        _, n, mu = tr.frenet(x, y, yaw)
        return (float(np.clip(-1.5 * n - 1.0 * mu + 0.3 * math.sin(2 * t), -0.4, 0.4)), 2.0 + 0.8 * math.sin(t)), 0.0, {}
    st = run_realtime(control, sim, est, timing or Timing(rng=np.random.default_rng(seed + 1)), 1 / 30, sim.t + T)
    return st, E


def test_posekf_same_as_bridge():
    """PoseKF 가 io/mocap_bridge.PoseKF 와 같은 식인지 (같은 입력 → 같은 출력)."""
    src = open(os.path.join(ROOT, 'src/mlcs_mpc/mlcs_mpc/io/mocap_bridge.py'), encoding='utf-8').read()
    body = src[src.index('class PoseKF'):src.index('class MocapBridge')]
    ns = {'np': np}
    exec(body, ns)
    a, b = ns['PoseKF'](0.01, 1.0, 1e-4), PoseKF(0.01, 1.0, 1e-4)
    rng = np.random.default_rng(0)
    for k in range(300):
        z = math.sin(0.05 * k) + 1e-3 * rng.standard_normal()
        assert np.allclose(a.update(z, 0.01), b.update(z, 0.01))
    print('  ① PoseKF = mocap_bridge.PoseKF (300 스텝 같음)')


def test_timing_matches_jetson():
    """사건 큐의 상태→명령 지연·건너뜀이 젯슨 실측 (34 ms, 1.8 %) 근처."""
    st, _ = _drive('true', T=20.0)
    age = np.median(st['age']) * 1e3
    skip = st['skipped'] / (st['ticks'] + st['skipped'])
    print(f'  ② 상태→명령 중앙 {age:.1f} ms (실차 33.6), 건너뜀 {skip * 100:.1f} % (실차 1.8 %), 풀이 중앙 '
          f'{np.median(st["solve"]) * 1e3:.1f} ms (실차 23.8)')
    assert 25 < age < 42 and skip < 0.08
    st0, _ = _drive('true', T=5.0, timing=Timing(TimingConfig(mocap='fixed', transport='zero', solve='zero')))
    assert max(st0['age']) < 0.0101 and st0['skipped'] == 0      # 지연 0 이면 mocap 주기만큼만


def test_estimator_lag_and_fake_vy():
    """kf: r 오차가 크고 코너에서 vy 가 안쪽으로 치우친다 (실차 11-10 과 같은 방향). gyro: r 오차가 줄어든다.
    vy rot: 가짜 vy 가 없어진다."""
    out = {}
    for mode, vy in (('true', 'kf'), ('kf', 'kf'), ('gyro', 'kf'), ('gyro', 'rot')):
        _, E = _drive(mode, vy)
        E = E[30:]
        d = np.array([e[1] - e[2] for e in E])
        r = np.array([e[2][2] for e in E])
        big = np.abs(r) > 0.8
        out[(mode, vy)] = (np.sqrt(np.mean(d ** 2, 0)), np.mean((d[:, 1] * np.sign(r))[big]))
        rm, b = out[(mode, vy)]
        print(f'  ③ {mode:4s} vy {vy:3s}: 오차 rms vx {rm[0]:.3f} vy {rm[1]:.3f} r {rm[2]:.3f}, 코너 vy 치우침 {b:+.3f}')
    assert out[('true', 'kf')][0][2] < 0.1
    assert out[('kf', 'kf')][1] < -0.1                                    # 가짜 vy (안쪽)
    assert out[('gyro', 'kf')][0][2] < 0.6 * out[('kf', 'kf')][0][2]      # 자이로가 r 을 덜 늦게
    assert abs(out[('gyro', 'rot')][1]) < 0.5 * abs(out[('kf', 'kf')][1])


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    for t in tests:
        print(t.__name__)
        t()
    print(f'{len(tests)}개 통과')
