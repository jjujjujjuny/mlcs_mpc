"""
timing — 실차의 비동기 신호 흐름을 사건 큐 하나로 (main.py --realtime). ROS 없음.

  실차 (젯슨 11-10 연구실 mocap 로그 실측, data/timing/jetson_mocap_20261110.json):
    mocap 프레임   간격 중앙 10.0 ms (p95 12.1, p99 15.2, 드물게 150 ms 끊김) → 전달 2.7 ms (p95 5.6) 뒤 도착
                   → 추정 (env/estimation.py) → /mpc/state 발행
    제어           30 Hz 타이머. 지난 풀이가 아직이면 그 주기를 건너뛴다 (pending)
                   그 순간 가장 최근 /mpc/state 로 풀이 → 풀이 시간 (중앙 23.8 ms, p99 38.5) 뒤 /drive 도착
    VESC           /sensors/core 50 Hz (간격 중앙 20 ms) — 바퀴 속도 (gyro 모드는 자이로도)
  → 상태를 잰 순간부터 명령까지 실차 약 34 ms (로그 추정). 지금 main.py 기본은 0.

  분포는 json 의 분위수 사이를 선형 보간한 역 CDF 로 뽑는다 (1 % 아래·99.9 % 위는 끝값).
"""
import heapq
import json
import math
import os
from dataclasses import dataclass

import numpy as np

DEFAULT_JSON = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..',
                            'data', 'timing', 'jetson_mocap_20261110.json')


class QuantileDist:
    """{'1': ms, '5': ms, ...} 분위수 → 표본 (s)."""

    def __init__(self, quantiles, scale=1e-3):
        q = sorted((float(k) / 100.0, float(v) * scale) for k, v in quantiles.items())
        self.p = np.array([p for p, _ in q])
        self.v = np.array([v for _, v in q])

    def sample(self, rng):
        return float(np.interp(rng.random(), self.p, self.v))

    @property
    def median(self):
        return float(np.interp(0.5, self.p, self.v))


@dataclass
class TimingConfig:
    source: str = ''                # 분포 json (비우면 DEFAULT_JSON)
    mocap: str = 'measured'         # measured = 실측 간격 분포 | fixed = 정확히 10 ms
    transport: str = 'measured'     # measured | zero
    solve: str = 'jetson'           # jetson = 젯슨 풀이 시간 분포 | measured = 이 PC 의 실제 풀이 시간 | zero
    vesc: str = 'measured'          # measured = /sensors/core 간격 분포 | fixed = 20 ms
    skip_pending: bool = True       # 풀이 중이면 그 주기를 건너뛴다 (실차 노드)
    max_gap: float = 0.05           # s  mocap 간격 표본 상한 (150 ms 끊김을 넣으려면 키운다)

    def check(self):
        for k, ok in (('mocap', ('measured', 'fixed')), ('transport', ('measured', 'zero')),
                      ('solve', ('jetson', 'measured', 'zero')), ('vesc', ('measured', 'fixed'))):
            if getattr(self, k) not in ok:
                raise ValueError(f'timing {k}: {" | ".join(ok)} ({getattr(self, k)!r})')


class Timing:
    def __init__(self, cfg=None, rng=None):
        self.cfg = cfg or TimingConfig()
        self.cfg.check()
        self.rng = rng or np.random.default_rng(0)
        d = json.load(open(self.cfg.source or DEFAULT_JSON, encoding='utf-8'))
        self.d_pose = QuantileDist(d['pose_interval_ms']['quantiles'])
        self.d_trans = QuantileDist(d['transport_ms']['quantiles'])
        self.d_solve = QuantileDist(d['solve_ms']['quantiles'])
        self.d_vesc = QuantileDist(d['vesc_state_interval_ms']['quantiles'])

    def mocap_interval(self):
        if self.cfg.mocap == 'fixed':
            return 0.01
        return min(max(self.d_pose.sample(self.rng), 0.002), self.cfg.max_gap)

    def transport(self):
        return 0.0 if self.cfg.transport == 'zero' else self.d_trans.sample(self.rng)

    def solve(self, ms_measured=0.0):
        c = self.cfg.solve
        if c == 'zero':
            return 0.0
        if c == 'measured':
            return max(float(ms_measured), 0.0) / 1000.0
        return self.d_solve.sample(self.rng)

    def vesc_interval(self):
        if self.cfg.vesc == 'fixed':
            return 0.02
        return min(max(self.d_vesc.sample(self.rng), 0.005), self.cfg.max_gap)


def run_realtime(control, sim, est, timing, period, t_end, stop=None, on_step=None):
    """사건 큐로 시뮬레이터를 진행한다.

    control(pose, twist, ws, t) → ((steer, speed), ms, tick)   — 한 제어 주기 (ms = 이 PC 의 풀이 시간)
    est      env/estimation.StateEstimator (reset 된 상태)
    stop()   True 면 끝 (랩 수 등)
    on_step(info, cmd)   적분 한 구간마다 (기록·출력)
    → 통계 dict (주기 수, 건너뜀, 상태 나이 = 상태를 잰 시각 → 명령 도착)
    """
    q = []
    seq = [0]

    def push(t, kind, data=None):
        seq[0] += 1
        heapq.heappush(q, (t, seq[0], kind, data))

    t0 = sim.t
    push(t0, 'capture')
    push(t0, 'vesc')
    push(t0, 'tick')
    published = (est.state()[0], est.state()[1], t0)        # (pose, twist, 잰 시각) — 노드가 가진 최신 /mpc/state
    pending = None
    cmd = None
    st = dict(ticks=0, skipped=0, age=[], solve=[])
    info = None
    while q:
        t, _, kind, data = heapq.heappop(q)
        if t > t_end:
            break
        if t > sim.t + 1e-12:
            info = sim.step(None, None, dt=t - sim.t)
            if on_step is not None:
                on_step(info, cmd)
            if stop is not None and stop():
                break
        if kind == 'capture':               # mocap 프레임 촬영 — 지금 상태를 재고, 전달 뒤 발행
            est.mocap(t, sim)
            pose, twist, _ = est.state()
            push(t + timing.transport(), 'publish', (pose, twist, t))
            push(t + timing.mocap_interval(), 'capture')
        elif kind == 'publish':
            if data[2] >= published[2]:
                published = data
        elif kind == 'vesc':
            est.vesc(t, sim)
            push(t + timing.vesc_interval(), 'vesc')
        elif kind == 'tick':
            push(t + period, 'tick')
            if pending is not None and timing.cfg.skip_pending:
                st['skipped'] += 1
                continue
            pose, twist, t_meas = published
            c, ms, tick = control(pose, twist, est.state()[2], t)
            st['ticks'] += 1
            d = timing.solve(ms)
            st['solve'].append(d)
            pending = (c, t_meas, tick)
            push(t + d, 'cmd')
        elif kind == 'cmd':
            c, t_meas, tick = pending
            pending = None
            cmd = c
            sim.command(*c)
            st['age'].append(t - t_meas)
            if isinstance(tick, dict):
                tick['t_cmd'] = t
                tick['age'] = t - t_meas
    return st


def summary(st):
    a = np.array(st['age']) * 1e3 if st['age'] else np.zeros(1)
    s = np.array(st['solve']) * 1e3 if st['solve'] else np.zeros(1)
    n = st['ticks'] + st['skipped']
    return (f'제어 {st["ticks"]}회 (건너뜀 {st["skipped"]}, {100.0 * st["skipped"] / max(n, 1):.1f} %), '
            f'풀이 중앙 {np.median(s):.1f} ms (p99 {np.percentile(s, 99):.1f}), '
            f'상태→명령 중앙 {np.median(a):.1f} ms (p95 {np.percentile(a, 95):.1f})')


def _demo():
    """간단한 추종 컨트롤러로 실시간 루프만 돌려 본다 (acados 없이)."""
    import argparse
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
    from mlcs_mpc.env.estimation import EstimatorConfig, StateEstimator
    from mlcs_mpc.env.lowlevel import LowLevelConfig
    from mlcs_mpc.env.simulator import Simulator
    from mlcs_mpc.env.track import Track, find
    from mlcs_mpc.env.vehicle_model import VehicleModel
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', default='1108_7_track_mocap')
    ap.add_argument('--estimator', default='kf')
    ap.add_argument('--time', type=float, default=10.0)
    a = ap.parse_args()
    cfgp = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'config', 'mpcc.yaml')
    tr = Track(find(a.track))
    sim = Simulator(VehicleModel.from_yaml(cfgp), tr, lowlevel=LowLevelConfig.from_yaml())
    sim.reset_to_s(0.0)
    est = StateEstimator(EstimatorConfig(mode=a.estimator))
    est.reset(sim)

    def control(pose, twist, ws, t):
        x, y, yaw = pose
        s, n, mu = tr.frenet(x, y, yaw)
        return (float(np.clip(-1.2 * n - 0.8 * mu, -0.4, 0.4)), 1.5), 0.0, {}
    st = run_realtime(control, sim, est, Timing(), 1 / 30, sim.t + a.time)
    print(summary(st), f'벽 {sim.wall_hits}')


if __name__ == '__main__':
    _demo()
