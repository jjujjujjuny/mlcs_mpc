#!/usr/bin/env python3
"""
sim_fidelity — 시뮬레이터 (env/simulator.Simulator) 가 실차와 얼마나 맞는지 개루프 재생으로 잰다. acados 필요 없음.

  실차 drive 로그의 /drive 명령을 **도착 시각 (cmd_recv) 그대로** 시뮬레이터에 넣고, 1 s 창마다 0.25·0.5·1 s 뒤
  위치·헤딩·vx 를 정답과 비교한다. 정답·시작 상태는 위치를 SG 비인과 미분한 값 (로그의 vx_kf 는 칼만 필터라 80~90 ms
  늦어서 쓰지 않는다). 조향은 물리 맵 (mpcc.yaml steer_map_act — race.yaml 의 MPC 쪽 배율과 무관).

  python3 tools/analysis/sim_fidelity.py <drive_*.csv ...> [--friction 0.41] [--sweep servo] [--jobs 8]
  python3 tools/analysis/sim_fidelity.py ~/mlcs_data/runs_20261110_mocap_HvsC/*/drive_*mocap*.csv --sweep servo
  python3 tools/analysis/sim_fidelity.py <대회장 로그> --lidar --friction 0.52 0.7 --sweep servo

  --sweep servo   서보 τ 0.13 / 0.06 / 0.03 / +15 ms / +50 ms / gym 3.2 rad/s + 20 ms
  --sweep rate    11-26 서보 계단의 '지연 + 각속도 상한' 격자 (지연 0~60 ms × 2.0~3.2 rad/s)
  --sweep none    config/lowlevel.yaml 그대로 하나
  --lidar         정답이 Cartographer 위치 (대회장): SG 창을 넓히고, 위치 미분 속도와 바퀴 속도가 1 m/s 넘게 어긋나는
                  창 (위치 튐) 은 뺀다. ★ 라이다 정답은 자체 잡음·지연이 커서 절대 오차보다 설정 간 비교로 볼 것

  결과 (2026-11-25, 1 s 위치 오차 중앙): 연구실 mocap τ 0.13 10.3 cm → 0.03 5.1 cm, 대회장 f0.7 42 → 33 cm.
  같은 식의 RL 환경 판은 env/vec (test/test_vec_vehicle.py), 실시간 비동기는 main.py --realtime.
"""
import argparse
import math
import multiprocessing as mp
import os
import sys

import numpy as np
import pandas as pd
import yaml
from scipy.signal import savgol_filter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'src', 'mlcs_mpc'))
from mlcs_mpc.env.lowlevel import LowLevelConfig  # noqa: E402
from mlcs_mpc.env.mpc.core import command_to_steer  # noqa: E402
from mlcs_mpc.env.simulator import Simulator  # noqa: E402
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402
from mlcs_mpc.env.vesc import Vesc  # noqa: E402

CFG = os.path.join(ROOT, 'src', 'mlcs_mpc', 'config', 'mpcc.yaml')
PRM = yaml.safe_load(open(CFG, encoding='utf-8'))['/**']['ros__parameters']
MAP_CMD, MAP_ACT = PRM['steer_map_cmd'], PRM['steer_map_act']
H = (0.25, 0.5, 1.0)
G = {}                                  # 작업 프로세스 전역 (로그·설정)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load(path, lidar=False, front=None):
    """drive 로그 → 정답 vx·vy·r (SG 미분), 위치 튐 표시.

    front: 로그 x·y 가 앞바퀴 축 (2026-11-26 부터 mocap) 이면 뒤축으로 옮긴다. None 이면 파일 날짜로 (≥ 20261126)."""
    d = pd.read_csv(path)
    d = d[np.isfinite(d.stamp)].reset_index(drop=True)
    if front is None:
        import re
        m = re.search(r'(\d{8})_\d{6}', os.path.basename(path))
        front = bool(m) and int(m.group(1)) >= 20261126 and not lidar
    if front:
        mdl = VehicleModel.from_yaml(CFG)
        wb = mdl.L
        d['x'] = d.x - wb * np.cos(d.yaw)
        d['y'] = d.y - wb * np.sin(d.yaw)
    t = d.stamp.values
    tg = np.arange(t[0], t[-1], 0.01)
    X, Y, P = np.interp(tg, t, d.x), np.interp(tg, t, d.y), np.interp(tg, t, np.unwrap(d.yaw))
    w = 31 if lidar else 15
    xd, yd, r = (savgol_filter(v, w, 2, deriv=1, delta=0.01) for v in (X, Y, P))
    d['vx_t'] = np.interp(t, tg, xd * np.cos(P) + yd * np.sin(P))
    d['vy_t'] = np.interp(t, tg, -xd * np.sin(P) + yd * np.cos(P))
    d['r_t'] = np.interp(t, tg, r)
    d['jump'] = np.abs(np.hypot(np.interp(t, tg, xd), np.interp(t, tg, yd)) - d.motor_rpm.values / 4781.5 * 1.044)
    return d


def starts(d, every=0.3, lidar=False):
    """움직이는 중 (명령 > 0.05, vx > 0.5) 이고 1 s 동안 끊김 없는 창의 시작 행."""
    st, v = d.stamp.values, d.vx_t.values
    mv = (d.cmd_speed.fillna(0).values > 0.05) & (v > 0.5)
    out, tl = [], -1e9
    for i in range(len(d)):
        if mv[i] and st[i] - tl >= every and st[i] + 1.1 < st[-1]:
            j = np.searchsorted(st, st[i] + 1.0)
            if mv[i:j].all() and np.all(np.diff(st[i:j]) < 0.05) and (not lidar or d.jump.values[i:j].max() < 1.0):
                out.append(i)
                tl = st[i]
    return out


def make(var):
    car = {k: var[k] for k in ('friction', 'wheel_speed_tau') if k in var}
    m = VehicleModel.from_yaml(CFG, **car)
    ll = LowLevelConfig.from_yaml().replace(**var.get('ll', {}))
    sim = Simulator(m, None, dt=0.01, steer_map_cmd=MAP_CMD, steer_map_act=MAP_ACT, ws_dec=PRM.get('ws_dec_max', 2.5),
                    vesc=Vesc(), wheel_model=var.get('wheel', 'lag'), lowlevel=ll)
    return m, sim


def run(d, var, idx, T=1.0):
    m, sim = make(var)
    ll = sim.lowlevel
    tau = ll.cfg.steer_tau or m.steering_tau
    dly = ll.cfg.steer_delay
    st = d.stamp.values
    ev_t, k = np.unique(d.cmd_recv.values, return_index=True)
    ok = np.isfinite(ev_t)
    ev_t, k = ev_t[ok], k[ok]
    ev_s, ev_v = d.cmd_steer.fillna(0).values[k], d.cmd_speed.fillna(0).values[k]
    phys = [command_to_steer(s, MAP_CMD, MAP_ACT, None) for s in ev_s]
    out = []
    for i0 in idx:
        t0 = st[i0]
        # 시작 조향 = 지난 1.5 s 명령을 같은 서보 (지연 + 1차) 로 거른 값
        de, tp, dref = 0.0, None, 0.0
        for tt, a in zip(ev_t[(ev_t > t0 - 1.5) & (ev_t <= t0 - dly)], np.array(phys)[(ev_t > t0 - 1.5) & (ev_t <= t0 - dly)]):
            if tp is not None:
                de += (1 - math.exp(-(tt - tp) / tau)) * (dref - de)
            dref, tp = a, tt
        if tp is not None:
            de += (1 - math.exp(-(t0 - dly - tp) / tau)) * (dref - de)
        sim.reset(d.x[i0], d.y[i0], d.yaw[i0])
        sim.x[3:8] = [d.vx_t[i0], d.vy_t[i0] + m.lr * d.r_t[i0], d.r_t[i0], d.motor_rpm[i0] / 4781.5 * ll.cfg.speed_scale, de]
        sim.t = t0
        ll.reset()
        jn = np.searchsorted(ev_t, t0, side='right')
        if jn > 0:
            ll.d_buf.cur = phys[jn - 1] if dly == 0 else de
            ll.w_buf.cur = ev_v[jn - 1] * ll.cfg.speed_scale
            for jj in np.where((ev_t > t0 - dly) & (ev_t <= t0))[0] if dly > 0 else []:
                ll.d_buf.buf.append((ev_t[jj], phys[jj]))
        res, tg = {}, [(h, np.searchsorted(st, t0 + h)) for h in H]
        while sim.t < t0 + T - 1e-9:
            nxt = min(sim.t + 0.01, ev_t[jn] if jn < len(ev_t) else 1e18)
            sim.step(None, None, dt=max(nxt - sim.t, 1e-4))
            if jn < len(ev_t) and sim.t >= ev_t[jn] - 1e-9:
                sim.command(ev_s[jn], ev_v[jn])
                jn += 1
            for h, j in tg:
                if h not in res and sim.t >= t0 + h - 1e-9 and j < len(d):
                    px, py, pyaw = sim.pose()
                    res[h] = (math.hypot(px - d.x[j], py - d.y[j]), wrap(pyaw - d.yaw[j]), sim.twist()[0] - d.vx_t[j])
        out.append((float(d.vx_t[i0]), res))
    return out


def sweep(name, frictions):
    S = {}
    for f in frictions:
        base = {'friction': f}
        if name == 'none':
            S[f'f{f} lowlevel.yaml'] = base
            continue
        S.update({f'f{f} τ0.13': dict(base, ll=dict(steer_tau=0.13)),
                  f'f{f} τ0.06': dict(base, ll=dict(steer_tau=0.06)),
                  f'f{f} τ0.03': dict(base, ll=dict(steer_tau=0.03)),
                  f'f{f} τ0.03 +15ms': dict(base, ll=dict(steer_tau=0.03, steer_delay=0.015)),
                  f'f{f} τ0.03 +50ms': dict(base, ll=dict(steer_tau=0.03, steer_delay=0.05)),
                  f'f{f} gym3.2 +20ms': dict(base, ll=dict(steer_model='gym', steer_rate=3.2, steer_delay=0.02))})
    return S


def sweep_rate(frictions):
    """11-26 거치대 서보 계단 (data/timing/servo_step_20261126.json): 지연 40~60 ms + 각속도 상한 2.0~2.5 rad/s."""
    S = {}
    for f in frictions:
        base = {'friction': f}
        S[f'f{f} τ0.03'] = dict(base, ll=dict(steer_tau=0.03))
        S[f'f{f} τ0.13'] = dict(base, ll=dict(steer_tau=0.13))
        for dl in (0.0, 0.02, 0.04, 0.06):
            for rt in (2.0, 2.2, 2.5, 3.2):
                S[f'f{f} {dl*1e3:.0f}ms {rt}r/s'] = dict(base, ll=dict(steer_tau=0.005, steer_rate=rt, steer_delay=dl))
        S[f'f{f} 60ms τ0.08'] = dict(base, ll=dict(steer_tau=0.08, steer_delay=0.06))
    return S


def _job(a):
    path, idx = a
    if path not in G['logs']:
        G['logs'][path] = load(path, G['lidar'])
    d = G['logs'][path]
    return {n: run(d, v, idx) for n, v in G['vars'].items()}


def _init(vars_, lidar):
    G.update(vars=vars_, lidar=lidar, logs={})


def main():
    ap = argparse.ArgumentParser(description='시뮬레이터 ↔ 실차 개루프 재생 (1 s 위치 오차)')
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--friction', nargs='+', type=float, default=[PRM['friction']])
    ap.add_argument('--sweep', default='servo', choices=['servo', 'rate', 'none'])
    ap.add_argument('--lidar', action='store_true', help='정답이 라이다 위치 (대회장)')
    ap.add_argument('--every', type=float, default=0.3, help='창 시작 간격 s')
    ap.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    a = ap.parse_args()
    V = sweep_rate(a.friction) if a.sweep == 'rate' else sweep(a.sweep, a.friction)
    jobs = []
    for p in a.logs:
        idx = starts(load(p, a.lidar), a.every, a.lidar)
        print(f'{os.path.basename(p)}: 창 {len(idx)}', flush=True)
        jobs += [(p, idx[k:k + 12]) for k in range(0, len(idx), 12)]
    if not jobs:
        sys.exit('쓸 수 있는 창이 없습니다.')
    with mp.Pool(a.jobs, initializer=_init, initargs=(V, a.lidar)) as pool:
        parts = pool.map(_job, jobs)
    bins = [(0, 2.5), (2.5, 3.5), (3.5, 99)]
    print(f'\n{"설정":18s} 1 s 위치 중앙 (p90) │ 헤딩 rms (평균) │ vx 평균 │ ' + ' │ '.join(f'v {lo}~{hi}' for lo, hi in bins))
    for n in V:
        L = [x for p in parts for x in p[n] if 1.0 in x[1]]
        e = np.array([r[1.0][0] for _, r in L])
        hd = np.array([r[1.0][1] for _, r in L])
        dv = np.array([r[1.0][2] for _, r in L])
        v = np.array([v0 for v0, _ in L])
        b = ' │ '.join(f'{np.median(e[(v >= lo) & (v < hi)]) * 100:5.1f} cm ({((v >= lo) & (v < hi)).sum()})'
                       if ((v >= lo) & (v < hi)).any() else '   -' for lo, hi in bins)
        print(f'{n:18s} {np.median(e) * 100:5.1f} cm ({np.percentile(e, 90) * 100:5.1f}) │ '
              f'{np.degrees(np.sqrt(np.mean(hd ** 2))):4.1f}° ({np.degrees(np.mean(hd)):+4.1f}) │ {np.mean(dv):+.2f} │ {b}')


if __name__ == '__main__':
    main()
