#!/usr/bin/env python3
"""
clean_drive_log.py — 주행 로그에서 mocap 이 못 믿을 구간을 잘라낸다.

  ./drive.sh cleanlog                  ~/mlcs_data/drive_*.csv 전부
  ./drive.sh cleanlog drive_XXX.csv    지정한 로그만

  결과: ~/mlcs_data/clean/<원본이름>_clean.csv
        원본 열 + segment 열. 원본은 건드리지 않는다.

■ 왜 segment 인가

  잘라낸 앞뒤는 시간이 이어지지 않는다. HyperPM 학습은 0.85 s 를 굴리는
  rollout 이고 속도는 pose 를 미분해서 얻으므로, 윈도와 미분이 끊긴 자리를
  넘어가면 그 자체가 새 오염이 된다. **segment 하나 안에서만 쓸 것.**

■ 무엇을 자르나 (2026-11-03 로그 실측)

  ① 추적 끊김 — pose 가 소수점까지 똑같이 멈춰 있다. 실제로 서 있으면
     mocap 잡음 때문에 값이 조금씩은 변한다. 모터가 12000 rpm 으로 도는데
     3.4 초간 pose 가 그대로인 구간이 있었다. 이때 KF 속도도 틀리다.
  ② 위치 튐 — 프레임 간 이동이 차 최고속(약 4.2 m/s)으로 설명이 안 된다.
  ③ 자세 튐 — yaw 변화가 15 rad/s 를 넘는다.
     (최대 조향·4 m/s 에서 정상 선회 약 5 rad/s, 슬립을 넉넉히 봐도 안 나온다)
  ④ 빈 칸 — 명령·VESC 값이 아직 안 왔다 (주로 시작 직후).
  ⑤ 시간 구멍 — 프레임이 0.03 s 이상 비었다. 값은 멀쩡하니 끊기만 한다.

■ 왜 뒤쪽을 1 초나 자르나

  pose 는 튐이 끝나면 바로 정상이지만 vx_kf·yaw_rate_kf 는 아니다. 칼만
  필터가 튄 값을 삼켰다가 회복하는 데 약 1 초 걸렸다 (rpm 환산 속도와의
  차이 중앙값: 직후 1.27 → 0.5 초 뒤 0.36 → 1 초 뒤 0.14 m/s).
  논문처럼 pose 를 Savitzky-Golay 로 다시 미분해서만 쓴다면
  --pad-after 0.2 로 줄여 데이터를 더 건질 수 있다.
"""
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

REQUIRED = ['cmd_speed', 'cmd_steer', 'motor_rpm', 'motor_iq', 'servo_position']


def frozen_rows(same, min_run):
    """앞 행과 pose 가 똑같은 행이 min_run 개 이상 이어지면 True."""
    d = np.diff(np.concatenate(([0], same.astype(int), [0])))
    out = np.zeros(len(same), bool)
    for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
        if b - a >= min_run:
            out[a:b] = True
    return out


def dilate(t, bad, before, after):
    """bad 인 행의 앞뒤 시간 범위까지 True 로 넓힌다."""
    idx = np.where(bad)[0]
    d = np.zeros(len(t) + 1, int)
    np.add.at(d, np.searchsorted(t, t[idx] - before), 1)
    np.add.at(d, np.searchsorted(t, t[idx] + after, side='right'), -1)
    return np.cumsum(d[:-1]) > 0


def find_bad(df, a):
    """이유별 불량 행 마스크. 키 순서가 보고 순서다."""
    t = df['time'].to_numpy()
    x, y, yaw = (df[c].to_numpy() for c in ('x', 'y', 'yaw'))
    dt = np.maximum(np.diff(t, prepend=t[0]), 1e-3)
    dxy = np.hypot(np.diff(x, prepend=x[0]), np.diff(y, prepend=y[0]))
    dyaw = np.abs(np.angle(np.exp(1j * np.diff(yaw, prepend=yaw[0]))))
    same = (dxy == 0) & (dyaw == 0)
    same[0] = False
    return {
        '추적 끊김': frozen_rows(same, a.freeze_frames),
        '위치 튐': dxy / dt > a.max_speed,
        '자세 튐': dyaw / dt > a.max_yaw_rate,
        '빈 칸': df[REQUIRED].isna().any(axis=1).to_numpy(),
    }


def clean(df, a):
    """(살린 행 + segment 열, 이유별 마스크) 를 돌려준다."""
    t = df['time'].to_numpy()
    reasons = find_bad(df, a)
    cut = dilate(t, reasons['추적 끊김'] | reasons['위치 튐'] | reasons['자세 튐'],
                 a.pad_before, a.pad_after)
    keep = ~(cut | reasons['빈 칸'])

    # 살아남은 행을 이어진 덩어리로 묶는다. 시간 구멍이어도 끊는다.
    gap = np.diff(t, prepend=t[0]) > a.max_gap
    d = np.diff(np.concatenate(([0], keep.astype(int))))
    seg = np.cumsum((d == 1) | (gap & keep))

    out = df[keep].copy()
    out['segment'] = seg[keep]
    dur = out.groupby('segment')['time'].agg(lambda s: s.iloc[-1] - s.iloc[0])
    out = out[out['segment'].isin(dur[dur >= a.min_segment].index)]
    out['segment'] = pd.factorize(out['segment'])[0]
    return out, reasons


def process(path, a):
    """로그 하나를 정리한다. 남은 분을 돌려준다 (쓸 행이 없으면 0)."""
    name = os.path.basename(path)
    df = pd.read_csv(path)
    print(f'■ {name}')
    if len(df) < 2:
        print('   행이 없습니다 — 건너뜀\n')
        return 0.0

    out, reasons = clean(df, a)
    t = df['time'].to_numpy()
    total = t[-1] - t[0]
    rate = 1.0 / max(np.median(np.diff(t)), 1e-9)
    print(f'   원본 {total:.1f} 초, {len(df)} 행')
    for why, mask in reasons.items():
        print(f'     {why:<6} {mask.sum():6d} 행')

    kept = len(out) / rate
    print(f'   → 남음 {kept:.1f} 초 ({100 * len(out) / len(df):.0f}%), '
          f'segment {out["segment"].nunique()}개')
    if out.empty:
        print('   ✗ 쓸 구간이 없습니다 — 파일을 만들지 않습니다.\n')
        return 0.0

    s = out['cmd_steer'].abs()
    mid = ((s > 0.01) & (s < 0.415)).mean()
    still = (out['motor_rpm'].abs() < 500).mean()
    print(f'   중간 조향 {100 * mid:.0f}%, 정지 {100 * still:.0f}%, '
          f'|r_kf| 최대 {out["yaw_rate_kf"].abs().max():.1f} rad/s, '
          f'|vx_kf| 최대 {out["vx_kf"].abs().max():.1f} m/s')

    outdir = os.path.join(os.path.dirname(os.path.abspath(path)), 'clean')
    os.makedirs(outdir, exist_ok=True)
    dst = os.path.join(outdir, os.path.splitext(name)[0] + '_clean.csv')
    out.to_csv(dst, index=False)
    print(f'   저장: {dst}\n')
    return kept / 60.0


def main():
    p = argparse.ArgumentParser(description='주행 로그에서 mocap 불량 구간 제거')
    p.add_argument('logs', nargs='*', help='없으면 데이터 폴더의 drive_*.csv 전부')
    p.add_argument('--data-dir', default=os.environ.get(
        'MLCS_DATA', os.path.expanduser('~/mlcs_data')))
    p.add_argument('--max-speed', type=float, default=6.0,
                   help='프레임 간 이동 속도 한계 [m/s]')
    p.add_argument('--max-yaw-rate', type=float, default=15.0,
                   help='프레임 간 yaw 변화 한계 [rad/s]')
    p.add_argument('--freeze-frames', type=int, default=3,
                   help='pose 가 이만큼 연속으로 같으면 추적 끊김')
    p.add_argument('--max-gap', type=float, default=0.03,
                   help='이보다 프레임이 비면 segment 를 끊는다 [s]')
    p.add_argument('--pad-before', type=float, default=0.2,
                   help='불량 앞쪽 추가 제거 [s]')
    p.add_argument('--pad-after', type=float, default=1.0,
                   help='불량 뒤쪽 추가 제거 [s] — KF 회복 시간')
    p.add_argument('--min-segment', type=float, default=1.5,
                   help='이보다 짧은 segment 는 버린다 [s] — rollout 0.85 s + 이력')
    a = p.parse_args()

    paths = []
    for f in a.logs or sorted(glob.glob(os.path.join(a.data_dir, 'drive_*.csv'))):
        paths.append(f if os.path.exists(f) else os.path.join(a.data_dir, f))
    missing = [f for f in paths if not os.path.exists(f)]
    if missing or not paths:
        print(f'로그를 찾을 수 없습니다: {missing or a.data_dir}')
        sys.exit(1)

    minutes = sum(process(f, a) for f in paths)
    print(f'■ 합계 {minutes:.1f} 분  (논문 36 분)')


if __name__ == '__main__':
    main()
