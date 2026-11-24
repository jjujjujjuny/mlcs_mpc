#!/usr/bin/env python3
"""
evaluate.py — 학습한 정책을 결정적으로 (tanh(평균)) 돌려 논문 지표로 잰다 (arXiv:2504.02420 IV, Table I).

  python3 learned/rl/evaluate.py logs/rl/<run>/best.pt                    학습 환경 (vec, 랜덤화 없음) + 시뮬레이터 둘 다
  python3 learned/rl/evaluate.py <ckpt> --wheel vesc                      시뮬레이터 차를 VESC 저수준 모델로 (env/drivetrain.py)
  python3 learned/rl/evaluate.py <ckpt> --vec-actuator vesc               학습 환경의 저수준 모델만 바꿔서 (교차 평가)
  python3 learned/rl/evaluate.py <ckpt> --car friction=0.38 --plot run.png

  ① vec       학습에 쓴 torch 환경 (같은 식). 벽을 넘어도 리셋하지 않고 계속 달린다 (논문처럼 E_off 로 잰다)
  ② sim       env/simulator.Simulator (casadi 식, 1 ms RK4) — 학습 코드와 따로 쓴 시뮬레이터로 옮겨 보는 시험 (sim-to-sim).
              명령은 실차와 같은 길: δ_ref → 서보 명령 (조향 맵 역) → VESC 서보 한계 → 조향 맵, ω_ref → 속도 명령 → VESC.
              --wheel vesc 면 바퀴는 1차 지연 대신 VESC 펌웨어 속도 PID + 정지 제동 패치 모델.
  지표        최고 랩 (벽 안에서 돈 랩만 — 논문 Table I '*'), 평균 랩 ± 표준편차, E_off = (1/N)∫ max(|n|−W, 0) dt  [m·s] (식 4, 5)
  저수준 지표  (저수준 제어기를 정책이 쓰는가)
              steer_lead  δ_ref 가 트랙 곡률 κ(s) 보다 얼마나 먼저 움직이나 [s] (상호상관 최대 지연, 논문 Fig. 5)
              ws_gap      |ω_ref − ω| 평균 [m/s] — 지연을 알고 명령을 앞질러 주면 커진다
              brake_lead  감속 명령 (ω̇_ref < 0) 시작부터 실제 바퀴 감속까지 [s] 평균
"""
import argparse
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
sys.path.insert(0, HERE)

from mlcs_mpc.env.vec.race_env import RaceEnvConfig, VecRaceEnv  # noqa: E402
from ppo import ActorCritic  # noqa: E402
from mlcs_mpc.env.lowlevel import LowLevelConfig  # noqa: E402

TRACE_KEYS = ('t', 'x', 'y', 'yaw', 'fx', 'fy', 'vx', 'vy', 'r', 'delta', 'delta_ref', 'ws', 'ws_ref', 'a1',
              's', 'n', 'kappa', 'off')


def load(path):
    ck = torch.load(path, map_location='cpu', weights_only=False)
    meta = ck['meta']
    ac = ActorCritic(meta['obs_dim'], meta['act_dim'])
    ac.load_state_dict(ck['model'])
    ac.eval()
    return ac, meta


def make_env(meta, car=None, num_envs=1, actuator=None, drop=None):
    known = set(RaceEnvConfig.__dataclass_fields__)
    e = {k: v for k, v in dict(meta['env']).items() if k in known}
    e.update(num_envs=num_envs, dr='none', dr_level=0.0, dr_nominal_frac=0.0, device='cpu', dr_terms={})
    if drop:                                    # 명령 끊김만 켜서 강건성 시험 (p, 평균 길이)
        e.update(dr_level=1.0, dr_terms=dict(drop=drop[0], drop_len=drop[1]))
    if actuator:
        e['actuator'] = actuator
    if car:
        e['car'] = dict(e.get('car') or {}, **car)
    e['reset_v'] = tuple(e['reset_v'])
    return VecRaceEnv(RaceEnvConfig(**e))


class LapMeter:
    """랩 시간·벽 밖 적분. 출발은 s = 0 에서 정지 상태."""

    def __init__(self, L, dt):
        self.L, self.dt = L, dt
        self.prog, self.t, self.lap_t0, self.lap_p0 = 0.0, 0.0, 0.0, 0.0
        self.laps, self.clean, self.off, self.lap_off = [], [], 0.0, 0.0
        self.crash_t = self.crash_prog = None       # 처음 실패 (벽 밖 / 멈춤) 한 시각·진행
        self.crash_why = None
        self.slow = 0.0

    def add(self, ds, off, vx=None):
        if off > 0 and self.crash_t is None:
            self.crash_t, self.crash_prog, self.crash_why = self.t + self.dt, self.prog + ds, '이탈'
        # 멈춤 — 학습 환경과 같은 기준 (1 s 넘게 0.3 m/s 아래). 출발 2 s 는 뺀다
        self.slow = self.slow + self.dt if (vx is not None and vx < 0.3) else 0.0
        if self.slow > 1.0 and self.t > 2.0 and self.crash_t is None:
            self.crash_t, self.crash_prog, self.crash_why = self.t + self.dt, self.prog + ds, '멈춤'
        self.prog += ds
        self.t += self.dt
        self.off += off * self.dt
        self.lap_off += off
        if self.prog - self.lap_p0 >= self.L:
            self.laps.append(self.t - self.lap_t0)
            self.clean.append(self.lap_off == 0.0)
            self.lap_t0, self.lap_p0, self.lap_off = self.t, self.lap_p0 + self.L, 0.0

    def summary(self, skip_first=True):
        laps = np.array(self.laps[1:] if skip_first and len(self.laps) > 1 else self.laps)
        clean = np.array(self.clean[1:] if skip_first and len(self.clean) > 1 else self.clean, dtype=bool)
        n = max(len(self.laps), 1)
        best = float(laps[clean].min()) if clean.any() else float('nan')
        return dict(crash_t=self.crash_t, crash_prog=self.crash_prog, crash_why=self.crash_why,
                    laps=len(laps), best=best, mean=float(laps.mean()) if len(laps) else float('nan'),
                    std=float(laps.std()) if len(laps) else float('nan'), e_off=self.off / n,
                    clean=int(clean.sum()))


def _record(tr, env, t, delta_ref, a1, off):
    x = env.x[0]
    fx, fy = env.front_axle()
    kap = float(env.track.interp(env.track.kappa, env.s)[0])
    for k, v in zip(TRACE_KEYS, (t, x[0], x[1], x[2], fx[0], fy[0], x[3], x[4], x[5], x[7], delta_ref,
                                 x[6], env.ws_ref[0], a1, env.s[0], env.n[0], kap, off)):
        tr[k].append(float(v))


def _start(env):
    """s = 0 센터라인 정지 출발."""
    env.reset()
    x, y, h = env.track.pose_at(torch.zeros(1, dtype=env.dtype))
    st = torch.zeros(1, 8, dtype=env.dtype)
    st[0, 0], st[0, 1], st[0, 2] = x[0], y[0], h[0]
    env.idx[:] = -1
    env.idx_m[:] = -1
    if env.dtr is not None:
        env.dtr.reset(torch.arange(1))
    env.u_applied[:] = 0.0
    env.set_state(st, ws_ref=torch.zeros(1), prev_a=torch.zeros(1, 2))


def run_vec(ac, meta, laps, car=None, actuator=None, max_time=200.0, drop=None, stop_on_out=False):
    env = make_env(meta, car, actuator=actuator, drop=drop)
    _start(env)
    m = LapMeter(env.track.length, env.cfg.dt)
    obs = env.observation()
    tr = {k: [] for k in TRACE_KEYS}
    _record(tr, env, 0.0, 0.0, 0.0, 0.0)
    while len(m.laps) < laps + 1 and m.t < max_time:
        a, _, _ = ac.act(obs, deterministic=True)
        obs, _, _, _, info = env.step(a)
        off = float(info['off'][0])
        m.add(float(info['progress'][0]), off, float(env.x[0, 3]))
        _record(tr, env, m.t, float(a[0, 0]) * env.max_steer, float(a[0, 1]), off)
        if stop_on_out and m.crash_t is not None:             # 실차처럼 트랙을 나가면 끝 (학습 환경의 종료와 같은 기준)
            break
    return m.summary(), {k: np.array(v) for k, v in tr.items()}


def run_sim(ac, meta, laps, car=None, wheel='lag', drive_mass=None, max_time=200.0, stop_on_out=False, lowlevel=None):
    """env/simulator.Simulator (casadi) 로. 관측은 같은 VecRaceEnv 코드로 계산한다 (상태만 옮겨 넣는다)."""
    import yaml
    from mlcs_mpc.env.mpc.core import steer_to_command
    from mlcs_mpc.env.simulator import Simulator
    from mlcs_mpc.env.track import Track, find
    from mlcs_mpc.env.vehicle_model import VehicleModel
    from mlcs_mpc.env.vesc import Vesc
    env = make_env(meta, car)
    yaml_path = meta['env'].get('mpcc_yaml') or os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')
    prm = yaml.safe_load(open(yaml_path, encoding='utf-8'))['/**']['ros__parameters']
    model = VehicleModel.from_yaml(yaml_path, **dict(meta['env'].get('car') or {}, **(car or {})))
    vesc = Vesc.load()
    drivetrain = None
    if wheel == 'vesc':
        from mlcs_mpc.env.drivetrain import Drivetrain
        drivetrain = Drivetrain(vesc, m_eq=drive_mass or Drivetrain.M_EQ_DEFAULT)
    track = Track(find(meta['env']['track']))
    sim = Simulator(model, track, dt=0.01, steer_map_cmd=prm['steer_map_cmd'], steer_map_act=prm['steer_map_act'],
                    max_steer=prm['max_steer'], vesc=vesc, wheel_model=wheel, drivetrain=drivetrain,
                    half_width=0.0, lowlevel=lowlevel)
    sim.reset_to_s(0.0)
    dt = env.cfg.dt
    k = int(round(dt / sim.dt))
    m = LapMeter(env.track.length, dt)
    ws_ref = torch.zeros(1)
    env.idx[:] = -1
    env.idx_m[:] = -1
    env.set_state(torch.as_tensor(sim.x[None, :]), ws_ref=ws_ref, prev_a=torch.zeros(1, 2))
    obs = env.observation()
    tr = {kk: [] for kk in TRACE_KEYS}
    _record(tr, env, 0.0, 0.0, 0.0, 0.0)
    while len(m.laps) < laps + 1 and m.t < max_time:
        a, _, _ = ac.act(obs, deterministic=True)
        a = a.clamp(-1, 1)
        ws_ref = torch.clamp(ws_ref + a[:, 1] * env.cfg.acc_max * dt, 0.0, env.v_max)
        delta_ref = float(a[0, 0]) * env.max_steer
        steer_cmd = steer_to_command(delta_ref, prm['steer_map_cmd'], prm['steer_map_act'])
        s_old = float(env.s[0])
        for _ in range(k):
            sim.step(steer_cmd, float(ws_ref[0]))
        env.set_state(torch.as_tensor(sim.x[None, :]), ws_ref=ws_ref, prev_a=a)
        obs = env.observation()
        L = env.track.length
        ds = (float(env.s[0]) - s_old + L / 2) % L - L / 2
        wl, wr = env.track.walls(env.s)
        off = max(float(env.n[0] - wl[0]), float(-env.n[0] - wr[0]), 0.0)
        m.add(ds, off, float(sim.x[3]))
        _record(tr, env, m.t, delta_ref, float(a[0, 1]), off)
        if stop_on_out and m.crash_t is not None:
            break
    return m.summary(), {kk: np.array(v) for kk, v in tr.items()}


def lowlevel_metrics(tr, dt=0.05, skip_t=None):
    """저수준 제어기를 정책이 쓰는가 — 정상 주행 구간 (첫 랩 뒤) 에서."""
    t = tr['t']
    sel = t > (skip_t if skip_t is not None else min(8.0, t[-1] / 3))
    if sel.sum() < 40:
        return dict(steer_lead=float('nan'), ws_gap=float('nan'), brake_lead=float('nan'))
    d = tr['delta_ref'][sel] - tr['delta_ref'][sel].mean()
    k = tr['kappa'][sel] - tr['kappa'][sel].mean()
    lags = np.arange(0, 21)                       # 0 ~ 1 s
    cc = [np.dot(d[:len(d) - L], k[L:]) for L in lags]   # δ_ref(t) 와 κ(t + L) — δ 가 먼저면 L > 0
    lead = float(lags[int(np.argmax(cc))] * dt)
    gap = float(np.mean(np.abs(tr['ws_ref'][sel] - tr['ws'][sel])))
    # 감속 명령 시작 → 바퀴가 실제로 줄기 시작할 때까지
    a1 = tr['a1'][sel]
    ws = tr['ws'][sel]
    starts = np.where((a1[1:] < -0.2) & (a1[:-1] >= -0.2))[0] + 1
    leads = []
    for i in starts:
        j = i
        while j + 1 < len(ws) and ws[j + 1] >= ws[j] - 1e-3:
            j += 1
        if j + 1 < len(ws) and j - i < 20:
            leads.append((j - i) * dt)
    return dict(steer_lead=lead, ws_gap=gap, brake_lead=float(np.mean(leads)) if leads else float('nan'))


def fmt(name, r, ll=None):
    crash = (f'첫 실패 ({r["crash_why"]}) {r["crash_t"]:.2f} s, 진행 {r["crash_prog"]:.1f} m' if r.get('crash_t') is not None
             else '실패 없음')
    s = (f'  {name:<10} {crash}  랩 {r["laps"]} 개 (벽 안 {r["clean"]})  최고 {r["best"]:.3f} s  평균 {r["mean"]:.3f} ± {r["std"]:.2f} s  '
         f'E_off {r["e_off"]:.5f} m·s')
    if ll:
        s += f'  | 조향 선행 {ll["steer_lead"]:.2f} s  |ω_ref−ω| {ll["ws_gap"]:.3f} m/s  제동 반응 {ll["brake_lead"]:.2f} s'
    return s


def main(argv=None):
    ap = argparse.ArgumentParser(description='RL 정책 평가 (arXiv:2504.02420 지표)')
    ap.add_argument('ckpt')
    ap.add_argument('--laps', type=int, default=20, help='첫 (정지 출발) 랩 뒤로 셀 랩 수 (논문 20)')
    ap.add_argument('--car', nargs='*', default=[], metavar='KEY=VALUE', help='평가 차만 다른 값 (friction=0.38 …)')
    ap.add_argument('--wheel', default='lag', choices=['lag', 'vesc'], help='시뮬레이터 바퀴 모델')
    ap.add_argument('--lowlevel', nargs='*', metavar='KEY=VALUE',
                    help='시뮬레이터 저수준 제어기 (env/lowlevel.py) — gym 프리셋 또는 steer_delay=0.02 …')
    ap.add_argument('--vec-actuator', default='', choices=['', 'lag', 'vesc', 'none'], help='vec 평가의 저수준 모델')
    ap.add_argument('--drive-mass', type=float, default=0.0)
    ap.add_argument('--drop', type=float, nargs=2, default=None, metavar=('P', 'LEN'),
                    help='vec 평가에 명령 끊김 (주기당 시작 확률, 평균 길이 주기) — 예 0.05 3')
    ap.add_argument('--only', choices=['vec', 'sim'], default=None)
    ap.add_argument('--json', default='', help='결과를 json 으로')
    a = ap.parse_args(argv)
    torch.set_grad_enabled(False)
    ac, meta = load(a.ckpt)
    car = {k: float(v) for k, v in (kv.split('=', 1) for kv in a.car)}
    e = meta['env']
    print(f'■ {a.ckpt}  트랙 {e["track"]}  학습: 액추에이터 {e["actuator"]}, 랜덤화 {e["dr"]}'
          + (f'   평가 차 {car}' if car else ''))
    res = {}
    if a.only in (None, 'vec'):
        r, tr = run_vec(ac, meta, a.laps, car, a.vec_actuator or None, drop=a.drop)
        res[f'vec/{a.vec_actuator or e["actuator"]}'] = (r, lowlevel_metrics(tr))
    if a.only in (None, 'sim'):
        r, tr = run_sim(ac, meta, a.laps, car, a.wheel, a.drive_mass or None,
                        lowlevel=LowLevelConfig.parse(a.lowlevel) if a.lowlevel else None)
        res[f'sim/{a.wheel}'] = (r, lowlevel_metrics(tr))
    for k, (r, ll) in res.items():
        print(fmt(k, r, ll))
    if a.json:
        import json
        json.dump({k: dict(r, **ll) for k, (r, ll) in res.items()}, open(a.json, 'w'), indent=2)
    return 0


if __name__ == '__main__':
    sys.exit(main())
