#!/usr/bin/env python3
"""
prep_hpm_dataset.py — 정리된 주행 로그를 HyperPM 학습 형식으로 바꾼다.

  ./drive.sh cleanlog          먼저 불량 구간을 자르고
  ./drive.sh hpmdata           ~/mlcs_data/hpm/{train,val,test}.csv

■ 무엇에 맞췄나

  저자 데이터 saved_artifacts/car/dataset/lab_rss1_*.csv 와 같은 열·단위다.
  저자 로더(F1tenthHyperDataset2)를 고치지 않고 그대로 읽는다.

    v_x, v_y, r    무게중심 기준 차체 속도 [m/s, rad/s]
    omega_wheels   바퀴 **선속도** [m/s] (이름과 달리 rad/s 가 아니다.
                   base_tire_model 이 v_x 와 바로 뺀다)
    delta          조향각 [rad]
    friction       상수. 타이어 힘 전체에 곱해진다
    run_id         끊긴 자리 표시. 윈도가 이것을 넘지 않는다

■ 변환

  ① 10 ms 격자로 다시 샘플 — Savitzky-Golay 는 등간격을 가정한다.
  ② x, y, yaw 를 SG(31, 2) 로 미분 → 차체 좌표로. 저자 데이터에서 창·차수를
     바꿔 가며 역산했을 때 가장 잘 맞은 값이다 (저장소에 설정이 없다).
     양 끝은 SG 가 부정확하니 창 절반씩 버린다.
  ③ /mpc/state 는 **뒤축** 기준이다. 모델은 무게중심 기준이므로
     v_y += lr · r. (v_x 는 같다)
  ④ delta 는 측정값이 없어서 명령으로 만든다. 실제 한계 ±0.40 으로 자르고
     서보 속도 3.2 rad/s 로 제한한다 (config/vehicle.yaml).
  ⑤ 전진만 남긴다. 저자 데이터도 v_x ≥ 0.25 이다 — 후진·정지에서는 슬립률의
     분모가 0 근처라 타이어 모델이 터진다.

■ 분할

  **run 통째로** train/val/test 에 나눈다. 시간으로 이어진 구간을 쪼개면
  거의 같은 데이터가 양쪽에 들어가 검증이 의미를 잃는다.
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

DT = 0.01


def rate_limit(u, max_rate, dt=DT):
    """서보처럼 일정 속도 이상으로는 못 따라가게 한다."""
    out = np.empty_like(u)
    out[0] = u[0]
    step = max_rate * dt
    for i in range(1, len(u)):
        out[i] = out[i - 1] + np.clip(u[i] - out[i - 1], -step, step)
    return out


def convert_segment(s, a):
    """clean 로그의 segment 하나 → 저자 형식 DataFrame."""
    t = np.arange(s['time'].iloc[0], s['time'].iloc[-1], DT)
    if len(t) <= a.sg_window:
        return None

    src = s['time'].to_numpy()
    grid = lambda v: np.interp(t, src, v)  # noqa: E731
    x, y = grid(s['x'].to_numpy()), grid(s['y'].to_numpy())
    yaw = grid(np.unwrap(s['yaw'].to_numpy()))

    sg = lambda v, d=0: savgol_filter(  # noqa: E731
        v, a.sg_window, a.sg_order, deriv=d, delta=DT)
    yaw_s, dx, dy = sg(yaw), sg(x, 1), sg(y, 1)
    r = sg(yaw, 1)
    v_x = dx * np.cos(yaw_s) + dy * np.sin(yaw_s)
    v_y = -dx * np.sin(yaw_s) + dy * np.cos(yaw_s) + a.lr * r

    delta = rate_limit(np.clip(grid(s['cmd_steer'].to_numpy()),
                               -a.max_steer, a.max_steer), a.max_steer_rate)

    out = pd.DataFrame({
        'time': t, 'x': x, 'y': y, 'yaw': yaw_s,
        'v_x': v_x, 'v_y': v_y, 'r': r,
        'omega_wheels': grid(s['motor_rpm'].to_numpy()) / a.erpm_gain,
        'friction': a.friction, 'delta': delta,
        'Iq': grid(s['motor_iq'].to_numpy()),
    })
    edge = a.sg_window // 2
    return out.iloc[edge:-edge]


def forward_runs(df, a):
    """전진 구간만 잘라서 run 목록으로. 윈도(관측+예측)보다 짧으면 버린다."""
    ok = ((df['v_x'] >= a.min_vx) & (df['omega_wheels'] >= 0.0)).to_numpy()
    d = np.diff(np.concatenate(([0], ok.astype(int), [0])))
    n_min = int(round(a.min_run / DT))
    return [df.iloc[i:j] for i, j in zip(np.where(d == 1)[0], np.where(d == -1)[0])
            if j - i >= n_min]


def split_runs(runs, ratios, seed):
    """run 을 섞어 시간 비율대로 나눈다."""
    order = np.random.default_rng(seed).permutation(len(runs))
    total = sum(len(r) for r in runs)
    parts = [[] for _ in ratios]
    filled = [0] * len(ratios)
    for i in order:
        # 목표 대비 가장 덜 찬 쪽에 넣는다.
        k = int(np.argmin([filled[j] / (ratios[j] * total) for j in range(len(ratios))]))
        parts[k].append(i)
        filled[k] += len(runs[i])
    return parts


def main():
    p = argparse.ArgumentParser(description='clean 로그 → HyperPM 학습 데이터')
    data_dir = os.environ.get('MLCS_DATA', os.path.expanduser('~/mlcs_data'))
    p.add_argument('--clean-dir', default=os.path.join(data_dir, 'clean'))
    p.add_argument('--out-dir', default=os.path.join(data_dir, 'hpm'))
    p.add_argument('--sg-window', type=int, default=31)
    p.add_argument('--sg-order', type=int, default=2)
    p.add_argument('--lr', type=float, default=0.1498, help='뒤축~무게중심 [m]')
    p.add_argument('--max-steer', type=float, default=0.40)
    p.add_argument('--max-steer-rate', type=float, default=3.2)
    p.add_argument('--erpm-gain', type=float, default=4781.5,
                   help='vesc.yaml speed_to_erpm_gain')
    p.add_argument('--friction', type=float, default=0.43,
                   help='실측 횡방향 운동 마찰 (2026-11-04)')
    p.add_argument('--min-vx', type=float, default=0.25)
    p.add_argument('--min-run', type=float, default=1.5,
                   help='이보다 짧은 run 은 버린다 [s] — 윈도 1.25 s')
    p.add_argument('--seed', type=int, default=42)
    a = p.parse_args()

    files = sorted(glob.glob(os.path.join(a.clean_dir, '*_clean.csv')))
    if not files:
        print(f'{a.clean_dir} 에 정리된 로그가 없습니다 — ./drive.sh cleanlog 먼저')
        sys.exit(1)

    runs = []
    for f in files:
        df = pd.read_csv(f)
        before = len(runs)
        for _, s in df.groupby('segment'):
            conv = convert_segment(s, a)
            if conv is not None:
                runs += forward_runs(conv, a)
        got = sum(len(r) for r in runs[before:]) * DT
        print(f'{os.path.basename(f)}: {len(df) * DT:.0f} 초 → 전진 {got:.0f} 초, '
              f'run {len(runs) - before}개')
    if len(runs) < 3:
        print(f'run 이 {len(runs)}개뿐입니다 — 나눌 수 없습니다.')
        sys.exit(1)

    os.makedirs(a.out_dir, exist_ok=True)
    names = ('train', 'val', 'test')
    for name, idx in zip(names, split_runs(runs, (0.7, 0.2, 0.1), a.seed)):
        out = pd.concat([runs[i].assign(run_id=i) for i in idx], ignore_index=True)
        dst = os.path.join(a.out_dir, name + '.csv')
        out.to_csv(dst, index=False)
        print(f'{name:<5} {len(out) * DT / 60:5.1f} 분, run {len(idx):3d}개 → {dst}')


if __name__ == '__main__':
    main()
