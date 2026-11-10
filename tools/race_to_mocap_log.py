#!/usr/bin/env python3
"""
race_to_mocap_log.py — 라이다로 달린 race 로그의 자세를 mocap 정답으로 바꿔 학습용 로그로 만든다.

  race 로그(data_logger)의 x, y, yaw 는 라이다 위치추정 값이라 mocap 보다 시끄럽다 (위치 약 10 cm).
  같이 기록한 mocap (map_mocap_align record) 의 뒤축 자세로 바꿔 넣고, mocap 주기(100 Hz) 격자로
  다시 뽑는다. 그러면 clean_drive_log → prep_hpm_dataset 가 mocap 주행과 똑같이 처리한다.

    x, y, yaw          mocap 뒤축 자세 (mocap 프레임, mocap.yaml 오프셋 적용)를 stamp 에 맞춰 보간
    cmd_*, servo       직전 값 유지 (명령은 계단이다). recv/cmd_recv 열이 있으면 도착 시각 기준 (아래 ★)
    나머지 (모터 등)    선형 보간
    mocap 이 30 ms 넘게 없거나 로거가 100 ms 넘게 비면 motor_rpm 을 비운다 → clean 이 '빈 칸' 으로 자른다

  ※ race 로그는 라이다 주기(약 49 Hz, 간격 최대 40 ms)로 찍혀서 그대로는 clean 의 max_gap(30 ms)에
    전부 잘린다 (2026-11-08).

  python3 tools/race_to_mocap_log.py <drive_..._race.csv> <align/..._race.csv> [출력]
    출력 기본: drive_<날짜>_<시각>_racemocap.csv (같은 폴더)
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_mocap_align as mma  # noqa: E402

HOLD = {'cmd_speed', 'cmd_steer', 'servo_position'}
DT = 0.01


def convert(drive_csv, align_csv):
    d = pd.read_csv(drive_csv)
    M, _ = mma.load_log(align_csv, mma.load_mocap_cfg())
    t = M['t']
    if len(t) < 100:
        raise ValueError(f'mocap 기록이 {len(t)}개뿐입니다 (natnet 이 Motive 에 못 붙은 주행)')
    st = d['stamp'].to_numpy()
    g = np.arange(max(t[0], st[0]), min(t[-1], st[-1]), DT)
    out = pd.DataFrame({'time': g - st[0] + d['time'].iloc[0], 'dt': DT, 'stamp': g})
    out['x'] = np.interp(g, t, M['x'])
    out['y'] = np.interp(g, t, M['y'])
    out['yaw'] = np.angle(np.exp(1j * np.interp(g, t, np.unwrap(M['yaw']))))
    # ★ 명령·VESC 값의 시간축 (2026-11-10). 로거는 상태가 '도착'할 때 행을 쓰는데 stamp 는 그 상태의
    #   시각이라, 라이다 주행에서는 위치추정 지연 (15~51 ms) 만큼 이르다. 그 행의 명령을 stamp 에 붙이면
    #   차가 명령에 그만큼 늦게 반응하는 것처럼 보인다. recv (행 쓴 시각) / cmd_recv (명령 도착 시각) 열이
    #   있으면 그걸 쓴다. 없으면 (예전 로그) stamp.
    has_recv = 'recv' in d.columns and d['recv'].notna().mean() > 0.9
    tv = d['recv'].to_numpy(dtype=float) if has_recv else st      # VESC·서보 값의 시각
    if has_recv and 'cmd_recv' in d.columns and d['cmd_recv'].notna().any():
        e = d[['cmd_recv', 'cmd_speed', 'cmd_steer']].dropna().drop_duplicates('cmd_recv')
        te = e['cmd_recv'].to_numpy()
        jc = np.searchsorted(te, g, side='right') - 1
        for c in ('cmd_speed', 'cmd_steer'):
            v = e[c].to_numpy(dtype=float)
            out[c] = np.where(jc >= 0, v[np.clip(jc, 0, len(v) - 1)], np.nan)
    j = np.clip(np.searchsorted(tv, g, side='right') - 1, 0, len(tv) - 1)
    for c in d.columns:
        if c in ('time', 'dt', 'stamp', 'x', 'y', 'yaw') or c in out.columns:
            continue
        v = d[c].to_numpy(dtype=float)
        out[c] = v[j] if c in HOLD else np.interp(g, tv, v)

    def nearest_gap(src):
        k = np.clip(np.searchsorted(src, g), 1, len(src) - 1)
        return np.minimum(np.abs(src[k] - g), np.abs(src[k - 1] - g))
    bad = (nearest_gap(t) > 0.03) | (nearest_gap(st) > 0.1)
    out.loc[bad, 'motor_rpm'] = np.nan
    return out[d.columns], bad


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('drive')
    ap.add_argument('align')
    ap.add_argument('out', nargs='?')
    a = ap.parse_args()
    out = a.out or a.drive.replace('_race.csv', '_racemocap.csv')
    if out == a.drive:
        out = os.path.splitext(a.drive)[0] + '_racemocap.csv'
    df, bad = convert(a.drive, a.align)
    df.to_csv(out, index=False)
    print(f'{os.path.basename(a.drive)} → {os.path.basename(out)}: {len(df) * DT:.0f} s @100 Hz, '
          f'빈 곳 {bad.mean() * 100:.1f}%')


if __name__ == '__main__':
    main()
