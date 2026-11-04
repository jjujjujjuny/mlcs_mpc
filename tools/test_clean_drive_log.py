#!/usr/bin/env python3
"""
test_clean_drive_log.py — clean_drive_log.py 회귀 시험 (차 불필요)

  python3 tools/test_clean_drive_log.py

합성 로그에 불량을 하나씩 심고, 그 자리만 잘리는지 본다.
"""
import argparse
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from clean_drive_log import clean, REQUIRED  # noqa: E402

ARGS = argparse.Namespace(max_speed=6.0, max_yaw_rate=15.0, freeze_frames=3,
                          max_gap=0.03, pad_before=0.2, pad_after=1.0,
                          min_segment=1.5)


def make_log(secs=30.0, seed=0):
    """100 Hz, 2 m/s 로 반경 2 m 원을 도는 깨끗한 로그. 잡음 0.2 mm."""
    rng = np.random.default_rng(seed)
    t = np.arange(0, secs, 0.01)
    th = t * 1.0
    df = pd.DataFrame({
        'time': t,
        'x': 2 * np.cos(th) + rng.normal(0, 2e-4, len(t)),
        'y': 2 * np.sin(th) + rng.normal(0, 2e-4, len(t)),
        'yaw': np.angle(np.exp(1j * (th + np.pi / 2))),
        'vx_kf': 2.0, 'yaw_rate_kf': 1.0,
    })
    for c in REQUIRED:
        df[c] = 1.0
    return df


def test_clean_log_untouched():
    out, _ = clean(make_log(), ARGS)
    assert len(out) == 3000 and out['segment'].nunique() == 1


def test_freeze_cut_with_pads():
    df = make_log()
    df.loc[1000:1200, ['x', 'y', 'yaw']] = df.loc[1000, ['x', 'y', 'yaw']].to_numpy()
    out, r = clean(df, ARGS)
    assert r['추적 끊김'].sum() == 200
    assert out['segment'].nunique() == 2
    gap = out[out['segment'] == 1]['time'].iloc[0] - out[out['segment'] == 0]['time'].iloc[-1]
    assert gap > 2.0 + 0.2 + 1.0 - 0.05, gap


def test_short_repeat_kept():
    # 한두 프레임 겹치는 건 끊김이 아니다.
    df = make_log()
    df.loc[1001, ['x', 'y', 'yaw']] = df.loc[1000, ['x', 'y', 'yaw']].to_numpy()
    _, r = clean(df, ARGS)
    assert r['추적 끊김'].sum() == 0


def test_position_jump():
    df = make_log()
    df.loc[1000:, 'x'] += 0.5
    out, r = clean(df, ARGS)
    assert r['위치 튐'].sum() == 1 and out['segment'].nunique() == 2


def test_yaw_jump_and_wrap():
    df = make_log()
    df.loc[1000, 'yaw'] += 1.0
    _, r = clean(df, ARGS)
    assert r['자세 튐'].sum() == 2
    # ±π 넘어가는 건 튐이 아니다 (원 주행이라 make_log 에 이미 있다).
    _, r = clean(make_log(), ARGS)
    assert r['자세 튐'].sum() == 0


def test_leading_blanks_no_pad():
    df = make_log()
    df.loc[:99, 'cmd_speed'] = np.nan
    out, _ = clean(df, ARGS)
    assert len(out) == 2900 and out['time'].iloc[0] == df['time'].iloc[100]


def test_time_gap_splits_only():
    df = make_log().drop(index=range(1000, 1010)).reset_index(drop=True)
    df.loc[1000:, ['x', 'y']] = df.loc[1000:, ['x', 'y']].to_numpy()
    out, _ = clean(df, ARGS)
    # 프레임 사이 0.11 s 지만 그 사이 이동은 정상 속도라 튐이 아니다.
    assert len(out) == len(df) and out['segment'].nunique() == 2


def test_short_segment_dropped():
    df = make_log(10.0)
    for i in (300, 500):   # 사이가 2 s - 패딩 1.2 s = 0.8 s 라 너무 짧다
        df.loc[i:, 'x'] += 0.5
    out, _ = clean(df, ARGS)
    assert out['segment'].nunique() == 2
    assert out['time'].between(3.0, 5.0, inclusive='neither').sum() == 0


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        f()
        print(f'ok  {f.__name__}')
    print(f'{len(tests)}개 통과')
