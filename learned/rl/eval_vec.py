#!/usr/bin/env python3
"""
eval_vec.py — 벡터화 평가. 조건 격자 (저수준 모델 × 마찰 × 명령 끊김 × 시작 위치 × 반복) 를 환경 하나씩 배치로 동시에 돌린다.

  python3 learned/rl/eval_vec.py logs/rl/exp/A_lag_cur/best.pt logs/rl/exp/B_vesc_cur/best.pt \
      --actuators lag vesc none --friction 0.9 1.0 1.1 --drop 0 0.05 --starts 8 --time 40 [-o out.md]

  저수준 모델마다 배치 하나 (VecRaceEnv 의 actuator 는 배치 전체에 하나). 그 안에서 환경 i 마다
    마찰 배율 (P.friction), 명령 끊김 확률 (lvl[i] × drop — 평균 길이 --drop-len), 시작 s (트랙을 --starts 등분, 정지 출발)
  지표 (모두 텐서로 한 번에): 실패율 (벽 밖·1 s 멈춤 — 학습 종료와 같은 기준), 첫 실패 시각, 랩 (정지 출발 첫 랩 제외)
  평균·최고, E_off = ∫ max(|n| − W, 0) dt / 랩 [m·s] (논문 식 5). 실패해도 끝까지 달려 E_off 를 잰다.
  랜덤화·센서 노이즈 없음 (명령 끊김만). 결정적 정책 tanh(평균).
"""
import argparse
import itertools
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from evaluate import ROOT, load  # noqa: E402
from mlcs_mpc.env.vec.race_env import DR_FULL, RaceEnvConfig, VecRaceEnv  # noqa: E402


@torch.no_grad()
def run_batch(ac, meta, actuator, grid, T, device='cpu', drop_len=3.0, seed=0):
    """grid = [(friction 배율, drop p, 시작 s[, DR 배율]), ...] → 지표 dict (각 (N,) numpy).

    DR 배율이 있으면 학습의 전체 랜덤화 (DR_FULL — 마찰·차·타이어·시정수·조향·PID·지연·센서 노이즈) 를 환경마다
    그 배율로 뽑아 건다 (수준 ℓ = 배율, 1 보다 커도 된다). 마찰은 뽑은 값에 곱하고, 끊김 확률은 환경마다 따로 (drop_p).
    """
    known = set(RaceEnvConfig.__dataclass_fields__)
    e = {k: v for k, v in dict(meta['env']).items() if k in known}
    N = len(grid)
    use_dr = len(grid[0]) > 3
    terms = dict(DR_FULL) if use_dr else {}
    terms.update(drop=1.0, drop_len=drop_len)
    e.update(num_envs=N, dr='none', dr_level=1.0, dr_nominal_frac=0.0, device=device, actuator=actuator,
             dr_terms=terms, start_spread=0.0, reset_v=(0.0, 0.0))
    env = VecRaceEnv(RaceEnvConfig(**e), seed=seed)
    ac = ac.to(device)
    fr = torch.tensor([g[0] for g in grid], device=device, dtype=env.dtype)
    dp = torch.tensor([g[1] for g in grid], device=device, dtype=env.dtype)
    s0 = torch.tensor([g[2] for g in grid], device=device, dtype=env.dtype)
    env.reset()
    # 시작: 지정한 s 의 센터라인, 정지
    x, y, h = env.track.pose_at(s0)
    st = torch.zeros(N, 8, device=device, dtype=env.dtype)
    st[:, 0], st[:, 1], st[:, 2] = x, y, h
    if use_dr:
        ds = torch.tensor([g[3] for g in grid], device=device, dtype=env.dtype)
        for v in torch.unique(ds).tolist():           # 배율마다 다시 뽑는다 (set_dr_level 은 1 로 자르므로 직접)
            env.dr_level = float(v)
            env._randomize((ds == v).nonzero().squeeze(1))
        env.P.friction = env.P.friction * fr
        env.drop_p = dp
    else:
        env.P.friction = env.veh.base.friction * fr
        env.lvl[:] = dp                               # drop 확률 = 1.0 × lvl
    env.idx[:] = -1
    env.idx_m[:] = -1
    env.u_applied[:] = 0.0
    if env.dtr is not None:
        env.dtr.reset(torch.arange(N, device=device))
    env.set_state(st, ws_ref=torch.zeros(N, device=device), prev_a=torch.zeros(N, 2, device=device))
    obs = env.observation()
    z = lambda: torch.zeros(N, device=device, dtype=env.dtype)  # noqa: E731
    fail_t = torch.full((N,), float('nan'), device=device, dtype=env.dtype)
    off_int, prog = z(), z()
    lap_n = torch.zeros(N, device=device, dtype=torch.long)
    lap_sum, lap_best = z(), torch.full((N,), float('inf'), device=device, dtype=env.dtype)
    slow = z()
    steps = int(round(T / env.cfg.dt))
    for k in range(steps):
        a, _, _ = ac.act(obs, deterministic=True)
        obs, _, _, _, info = env.step(a)
        t = (k + 1) * env.cfg.dt
        off = info['off']
        off_int += off * env.cfg.dt
        prog += info['progress']
        slow = torch.where(env.x[:, 3] < 0.3, slow + env.cfg.dt, torch.zeros_like(slow))
        failed = (off > 0) | ((slow > 1.0) & (t > 2.0))
        fail_t = torch.where(torch.isnan(fail_t) & failed, torch.full_like(fail_t, t), fail_t)
        lt = info['lap_time']
        done_lap = ~torch.isnan(lt)
        first = done_lap & (lap_n == 0)               # 정지 출발 첫 랩은 세지 않는다
        later = done_lap & (lap_n > 0)
        lap_sum += torch.where(later, lt, torch.zeros_like(lt))
        lap_best = torch.where(later, torch.minimum(lap_best, lt), lap_best)
        lap_n += done_lap.long()
        del first
    laps = (lap_n - 1).clamp(min=0)
    out = dict(failed=(~torch.isnan(fail_t)), fail_t=fail_t, laps=laps,
               lap_mean=torch.where(laps > 0, lap_sum / laps.clamp(min=1), torch.full_like(lap_sum, float('nan'))),
               lap_best=torch.where(laps > 0, lap_best, torch.full_like(lap_best, float('nan'))),
               e_off=off_int / lap_n.clamp(min=1), prog=prog)
    return {k: v.cpu().numpy() for k, v in out.items()}


def summarize(r, sl, lab, trained, act, f, d, ds):
    fail = r['failed'][sl]
    lm, lb = r['lap_mean'][sl], r['lap_best'][sl]
    ok = ~fail
    return dict(policy=lab, trained=trained, eval=act, dr_scale=float(ds or 0.0), friction=f, drop=d,
                n=int(fail.size), fail_rate=float(fail.mean()),
                fail_t=float(np.nanmean(r['fail_t'][sl])) if fail.any() else float('nan'),
                lap_mean=float(np.nanmean(lm)) if np.isfinite(lm).any() else float('nan'),
                lap_best_clean=float(np.nanmin(lb[ok])) if (ok & np.isfinite(lb)).any() else float('nan'),
                e_off=float(r['e_off'][sl].mean()), laps=int(r['laps'][sl].sum()))


def main(argv=None):
    ap = argparse.ArgumentParser(description='벡터화 평가 (조건 격자를 배치로)')
    ap.add_argument('ckpts', nargs='+')
    ap.add_argument('--labels', nargs='*', default=[])
    ap.add_argument('--actuators', nargs='+', default=['lag', 'vesc', 'none'])
    ap.add_argument('--friction', nargs='+', type=float, default=[1.0])
    ap.add_argument('--drop', nargs='+', type=float, default=[0.0])
    ap.add_argument('--drop-len', type=float, default=3.0)
    ap.add_argument('--starts', type=int, default=1, help='시작 s 개수 (트랙 등분, 1 = s 0)')
    ap.add_argument('--time', type=float, default=40.0)
    ap.add_argument('--dr-scales', nargs='*', type=float, default=[],
                    help='학습 전체 랜덤화 (DR_FULL) 의 배율들 — 주면 조건마다 이 배율로 차·센서·지연을 환경마다 뽑아 건다 '
                         '(0 = 없음, 0.5 = E 학습 수준, 1 = full, 2 = 학습보다 두 배)')
    ap.add_argument('--reps', type=int, default=1, help='시작 위치마다 반복 (랜덤화 표본 수 = starts × reps)')
    ap.add_argument('--device', default='cpu')
    ap.add_argument('-o', '--out', default='')
    a = ap.parse_args(argv)
    torch.set_grad_enabled(False)
    rows = []
    for i, ck in enumerate(a.ckpts):
        ac, meta = load(ck)
        lab = a.labels[i] if i < len(a.labels) else os.path.basename(os.path.dirname(ck))
        L = None
        for act in a.actuators:
            if L is None:
                from mlcs_mpc.env.track import Track, find
                L = Track(find(meta['env']['track'])).length
            starts = [L * j / a.starts for j in range(a.starts)] * a.reps
            # 배치 하나에 모든 조건 (커널 호출 오버헤드가 병목이라 환경 수를 늘려도 시간은 거의 같다)
            ds_list = a.dr_scales or [None]
            combos = list(itertools.product(ds_list, a.drop, a.friction))
            grid = [(f, d, s) + ((ds,) if ds is not None else ()) for (ds, d, f) in combos for s in starts]
            r = run_batch(ac, meta, act, grid, a.time, a.device, a.drop_len)
            for ci, (ds, d, f) in enumerate(combos):
                sl = slice(ci * len(starts), (ci + 1) * len(starts))
                rows.append(summarize(r, sl, lab, meta['env']['actuator'], act, f, d, ds))
                w = rows[-1]
                print(f"  {lab:>10} 학습 {w['trained']:<4} 평가 {act:<4} DR×{w['dr_scale']:.2f} "
                      f"마찰×{f:.2f} 끊김 {d:.2f} | 실패 {w['fail_rate']:.0%} "
                      f"(평균 {w['fail_t']:.1f}s) 랩 {w['laps']} 평균 {w['lap_mean']:.2f}s "
                      f"최고(실패없음) {w['lap_best_clean']:.2f}s E_off {w['e_off']:.4f}", flush=True)
    md = ['| 정책 | 학습 저수준 | 평가 저수준 | DR × | 마찰 × | 명령 끊김 p | 실패율 | 평균 실패 시각 s | 랩 수 | 평균 랩 s | 최고 랩 (실패 없는 시작) s | E_off m·s |',
          '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for w in rows:
        star = ' ★' if w['eval'] == w['trained'] and w['friction'] == 1.0 and w['drop'] == 0 and not w['dr_scale'] else ''
        md.append(f"| {w['policy']} | {w['trained']} | {w['eval']}{star} | {w['dr_scale']:.2f} | {w['friction']:.2f} | {w['drop']:.2f} | "
                  f"{w['fail_rate']:.0%} | {w['fail_t']:.1f} | {w['laps']} | {w['lap_mean']:.2f} | "
                  f"{w['lap_best_clean']:.2f} | {w['e_off']:.4f} |")
    text = '\n'.join(md) + f'\n\n시작 {a.starts} 곳 (트랙 등분, 정지 출발) × 반복 {a.reps} × 조건, {a.time:.0f} s. DR × = 학습 전체 랜덤화 (DR_FULL) 배율. ★ = 학습 조건. 실패 = 벽 밖 또는 1 s 멈춤.\n'
    out = a.out or os.path.join(ROOT, 'logs/rl/eval_vec.md')
    open(out, 'w').write(text)
    import json
    json.dump(rows, open(os.path.splitext(out)[0] + '.json', 'w'), indent=2)
    print(f'✓ {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
