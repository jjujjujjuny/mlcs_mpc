#!/usr/bin/env python3
"""
compare.py — 여러 정책을 여러 저수준 모델에서 교차 평가 → 표 (markdown) + json.
"저수준 제어기가 학습에 실제로 들어가는가" 를 본다: 학습 때 본 저수준 모델과 다른 모델에서 얼마나 무너지나,
조향을 얼마나 먼저 꺾나 (steer_lead), 명령을 얼마나 앞질러 주나 (|ω_ref − ω|).

  python3 learned/rl/compare.py logs/rl/exp/A_lag_cur logs/rl/exp/B_vesc_cur ... [--laps 10] [-o logs/rl/exp/compare.md]

  평가 조건 (행마다): vec/lag, vec/vesc, vec/none (학습 환경 식, 저수준만 바꿈), sim/lag, sim/vesc (casadi 시뮬레이터)
  모두 랜덤화 없음, s = 0 정지 출발, 첫 랩 빼고 --laps 랩.
"""
import argparse
import json
import os
import sys

import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from evaluate import load, lowlevel_metrics, run_sim, run_vec  # noqa: E402

CONDS = ('vec/lag', 'vec/vesc', 'vec/none', 'sim/lag', 'sim/vesc')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+', help='학습 폴더 (best.pt 를 쓴다) 또는 체크포인트')
    ap.add_argument('--laps', type=int, default=10)
    ap.add_argument('--ckpt', default='best.pt')
    ap.add_argument('--max-time', type=float, default=120.0)
    ap.add_argument('-o', '--out', default='')
    a = ap.parse_args(argv)
    torch.set_grad_enabled(False)
    rows, res = [], {}
    for run in a.runs:
        ck = run if run.endswith('.pt') else os.path.join(run, a.ckpt)
        ac, meta = load(ck)
        name = os.path.basename(os.path.dirname(ck)) if run.endswith('.pt') else os.path.basename(run.rstrip('/'))
        e = meta['env']
        res[name] = {}
        for cond in CONDS:
            kind, low = cond.split('/')
            if kind == 'vec':
                r, tr = run_vec(ac, meta, a.laps, actuator=low, max_time=a.max_time)
            else:
                r, tr = run_sim(ac, meta, a.laps, wheel=low, max_time=a.max_time)
            ll = lowlevel_metrics(tr)
            res[name][cond] = dict(r, **ll)
            trained = (kind == 'vec' and low == e['actuator'])
            rows.append(f'| {name} | {e["actuator"]} | {cond}{" ★" if trained else ""} | {r["laps"]} ({r["clean"]}) | '
                        f'{r["best"]:.2f} | {r["mean"]:.2f} ± {r["std"]:.2f} | {r["e_off"]:.4f} | '
                        f'{ll["steer_lead"]:.2f} | {ll["ws_gap"]:.3f} |')
            print(rows[-1], flush=True)
    head = ('| 정책 | 학습 저수준 | 평가 조건 | 랩 (벽 안) | 최고 랩 s | 평균 랩 s | E_off m·s | 조향 선행 s | \\|ω_ref−ω\\| m/s |\n'
            '|---|---|---|---|---|---|---|---|---|')
    md = head + '\n' + '\n'.join(rows) + '\n\n★ = 학습한 조건. 랩은 첫 (정지 출발) 랩을 뺀 수, 괄호는 벽 밖으로 안 나간 랩.\n'
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.runs[0].rstrip('/'))), 'compare.md')
    open(out, 'w').write(md)
    json.dump(res, open(os.path.splitext(out)[0] + '.json', 'w'), indent=2)
    print(f'✓ {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
