#!/usr/bin/env python3
"""
steer_from_log.py — 원선회할 공간이 없을 때, 자율주행 기록으로 조향 점검 (11-14 대회장).

  ~/hmpc-venv/bin/python tools/steer_from_log.py ~/mlcs_data/drive_<날짜>_*_race.csv [...]

  조향 명령이 0.35 s 동안 거의 그대로였던 순간만 모아 실제 곡률 = 자이로 요레이트 / 바퀴 속도 (odom_vx).
  필요: odom_* 열 (wheel_odom), yaw_rate_kf = 자이로 (11-11 22:55 이후 기록).

  기준 (출발 전): 중립 곡률 −0.020 (로비 기록) / −0.023 (연구실 원선회), 최대 조향 반경 0.83~0.86 m 좌우 대칭.
  11-14 대회장: 중립 −0.012, 반경 좌 0.82 / 우 0.82 m → 운반 중 변화 없음.
  중립이 기준에서 0.02 넘게 달라지거나 반경이 5 cm 넘게 달라지면 조향이 틀어진 것 (steer_check.py 의 중립 계산과 같은 식으로 고친다).
"""
import math
import sys

import numpy as np
import pandas as pd

L = 0.33
MC, MA = [0, 0.2, 0.3, 0.4], [0, 0.2021, 0.2972, 0.3776]
GAIN, OFFSET = -0.8845, 0.5077          # vesc.yaml (바뀌면 맞춰 줄 것)


def collect(files):
    K, C, V, AY = [], [], [], []
    for f in files:
        d = pd.read_csv(f)
        need = ['time', 'odom_vx', 'yaw_rate_kf', 'cmd_steer', 'cmd_speed']
        if any(c not in d for c in need):
            print(f'  건너뜀 (열 없음): {f}')
            continue
        d = d[np.all([np.isfinite(d[c]) for c in need], axis=0)]
        if len(d) < 500:
            continue
        t = d.time.values
        tg = np.arange(t[0] + 1, t[-1] - 1, 0.02)
        v = np.interp(tg, t, d.odom_vx.values)
        r = np.interp(tg, t, d.yaw_rate_kf.values)
        c = np.interp(tg, t, d.cmd_steer.values)
        cs = np.interp(tg, t, d.cmd_speed.values)
        w = int(0.35 / 0.02)
        steady = (pd.Series(c).rolling(w).max().values - pd.Series(c).rolling(w).min().values) < 0.02
        m = steady & (v > 0.5) & (cs > 0.2)
        K.append(r[m] / v[m]); C.append(c[m]); V.append(v[m]); AY.append(np.abs(r[m] * v[m]))
    if not K:
        raise SystemExit('쓸 수 있는 기록이 없습니다')
    return [np.concatenate(x) for x in (K, C, V, AY)]


def main():
    files = sys.argv[1:]
    if not files:
        raise SystemExit(__doc__)
    k, c, v, ay = collect(files)
    print(f'표본 {len(k)} ({len(files)} 파일)')
    neu = np.abs(c) < 0.015
    if neu.sum() > 50:
        k0 = float(np.median(k[neu]))
        d_err = math.atan(k0 * L)
        print(f'중립 곡률 {k0:+.4f} 1/m (n {neu.sum()}) → 실제 조향 {math.degrees(d_err):+.2f}°, '
              f'고친다면 servo offset {OFFSET:.4f} → {OFFSET - GAIN * d_err:.4f}')
    else:
        print('중립 표본 부족 (직선 구간이 더 필요)')
    for lo, hi, lab in [(0.37, 0.41, '최대 왼쪽'), (-0.41, -0.37, '최대 오른쪽')]:
        m = (c >= lo) & (c <= hi) & (v < 1.6) & (ay < 2.5)
        if m.sum() > 30:
            kk = float(np.median(np.abs(k[m])))
            print(f'{lab} 반경 {1 / kk:.2f} m (n {m.sum()}, 평균 속도 {v[m].mean():.2f}, 모델 0.83 m)')
        else:
            print(f'{lab}: 저속 표본 부족')


if __name__ == '__main__':
    main()
