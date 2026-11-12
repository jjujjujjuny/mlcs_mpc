#!/usr/bin/env python3
"""
venue_intake.py — 대회장에서 푸시된 주행 묶음(data/runs/*.tar.gz)을 받자마자 1차 분석한다 (메인 PC).

  ~/hpm-venv/bin/python tools/venue_intake.py                     git pull 뒤, 아직 안 본 묶음 전부
  ~/hpm-venv/bin/python tools/venue_intake.py data/runs/runs_X.tar.gz [--track 1114_1] [--quick] [--no-model]

  결과: ~/mlcs_data/venue/<묶음>/report.md  (풀린 로그도 같은 폴더). 본 묶음은 ~/mlcs_data/venue/done.txt 에 적는다.

■ 주행 로그마다 (drive_*_race.csv)
  · 길이, 최고 속도, 해석기 계산 시간 (p50 / p99 / 최대), 후진 횟수, 배터리 (시작 / 최저), duty 최대
  · 트랙을 알면 (--track, 또는 MANIFEST 에서 그 주행 줄에 적힌 트랙 이름):
      랩 수·랩타임, 벽까지 여유 최소값과 차체가 벽에 닿은 비율 (여유 < 차 반폭 0.15 m), 가장 가까웠던 자리 (s)
■ 묶음 전체 — tools/venue_check.py (바닥 마찰, 조향 지연, 속도 구간별 모델 오차). odom_* 열이 있는 로그만.
■ 맵 파일 (*.pgm / *.yaml / *.pbstream) 이 들어 있으면 목록과 트랙 만들기 명령을 적는다 (실행은 사람이 판단).
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import tarfile

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.expanduser('~/mlcs_data/venue')
DONE = os.path.join(OUT, 'done.txt')
TRACKS = os.path.join(ROOT, 'lab_map', 'data', 'tracks')
HALF_W = 0.15          # 차 반폭 [m]
WHEEL = 4781.5         # motor_rpm / WHEEL = 바퀴 선속도 [m/s]


def load_track(name):
    for d in (name, name + '_track'):
        f = os.path.join(TRACKS, d, d + '.csv')
        if os.path.exists(f):
            t = pd.read_csv(f)
            return d, t[['x_m', 'y_m']].to_numpy(), t.w_tr_right_m.to_numpy(), t.w_tr_left_m.to_numpy()
    return None


def track_stats(df, trk):
    _, C, wr, wl = trk
    P = df[['x', 'y']].to_numpy()
    ok = np.isfinite(P).all(1)
    P, tm = P[ok], df.time.to_numpy()[ok]
    if len(P) < 50:
        return {}
    i = np.empty(len(P), int)
    for a in range(0, len(P), 4000):                       # 가장 가까운 중심선 점 (메모리 때문에 나눠서)
        i[a:a + 4000] = np.argmin(((P[a:a + 4000, None, :] - C[None]) ** 2).sum(-1), 1)
    T = np.roll(C, -1, 0) - np.roll(C, 1, 0)
    T /= np.linalg.norm(T, axis=1, keepdims=True)
    dv = P - C[i]
    n = T[i, 0] * dv[:, 1] - T[i, 1] * dv[:, 0]            # 왼쪽 +
    margin = np.where(n > 0, wl[i] - n, wr[i] + n)
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(C, axis=0, append=C[:1]), axis=1))]
    s = seg[i]
    N = len(C)
    cross = np.where((i[:-1] > 0.85 * N) & (i[1:] < 0.15 * N))[0] + 1
    laps = []                                              # 한 바퀴 = 출발선 통과 사이에 트랙 1/3·2/3 지점을 차례로 지남
    for a0, b0 in zip(cross[:-1], cross[1:]):
        k = i[a0:b0]
        h1, h2 = np.where(np.abs(k - N // 3) < N // 20)[0], np.where(np.abs(k - 2 * N // 3) < N // 20)[0]
        if len(h1) and len(h2) and h1[0] < h2[-1] and tm[b0] - tm[a0] > seg[-1] / 6.0:
            laps.append(tm[b0] - tm[a0])
    laps = np.array(laps)
    w = int(np.argmin(margin))
    return dict(랩=len(laps), 랩타임=' / '.join(f'{x:.2f}' for x in laps[:12]) + (' …' if len(laps) > 12 else ''),
                최고랩=f'{laps.min():.2f}' if len(laps) else '-',
                랩중앙=f'{np.median(laps[laps < 1.5 * np.median(laps)]):.2f}' if len(laps) else '-',   # 교착 랩 (중앙값 ×1.5 초과) 은 뺀다
                벽여유최소=f'{margin.min():+.2f} m (s {s[w]:.1f})',
                벽닿음=f'{(margin < HALF_W).mean():.1%}',
                벽닿은자리=', '.join(f'{x:.1f}' for x in sorted(set(np.round(s[margin < HALF_W], 0)))[:10]))


def log_stats(f):
    df = pd.read_csv(f)
    t = df.time.to_numpy()
    sp = df.odom_vx if 'odom_vx' in df and df.odom_vx.notna().mean() > 0.5 else df.motor_rpm / WHEEL
    sm = df.solve_ms.dropna() if 'solve_ms' in df else pd.Series(dtype=float)
    rev = int(((df.cmd_speed < -0.05) & ~(df.cmd_speed.shift() < -0.05)).sum()) if 'cmd_speed' in df else -1
    r = dict(로그=os.path.basename(f).replace('drive_', '').replace('_race.csv', ''),
             길이=f'{(t[-1] - t[0]) / 60:.1f}분', 최고속=f'{sp.max():.2f}',
             해석ms=f'{sm.median():.1f}/{sm.quantile(.99):.1f}/{sm.max():.0f}' if len(sm) else '-',
             후진=rev,
             배터리=f'{df.voltage.dropna().iloc[:50].median():.2f}/{df.voltage.min():.2f}' if 'voltage' in df else '-',
             duty=f'{df.duty_cycle.abs().max():.2f}' if 'duty_cycle' in df else '-',
             odom='O' if 'odom_yaw' in df and df.odom_yaw.notna().mean() > 0.5 else 'X')
    return df, r


def manifest_tracks(text):
    """MANIFEST 줄 '20261111_235653  1111_5 ...' → {stamp: 트랙}. 트랙 칸이 비면 바로 위 줄을 따른다."""
    out, last = {}, None
    for ln in text.splitlines():
        m = re.match(r'\s*(\d{8}_\d{6})\s+(\S+)?', ln)
        if not m:
            continue
        tok = m.group(2) or ''
        if load_track(tok):
            last = tok
        out[m.group(1)] = last
    return out


def md_table(rows):
    if not rows:
        return '(없음)\n'
    k = list(rows[0])
    for r in rows[1:]:
        k += [c for c in r if c not in k]
    s = '| ' + ' | '.join(k) + ' |\n|' + '---|' * len(k) + '\n'
    for r in rows:
        s += '| ' + ' | '.join(str(r.get(c, '')) for c in k) + ' |\n'
    return s


def intake(tar, a):
    name = os.path.basename(tar).replace('.tar.gz', '').replace('.tgz', '')
    dst = os.path.join(OUT, name)
    os.makedirs(dst, exist_ok=True)
    with tarfile.open(tar) as tf:
        for m in tf.getmembers():                          # 묶음 밖으로 나가는 경로는 거른다
            if m.name.startswith('/') or '..' in m.name.split('/') or not (m.isfile() or m.isdir()):
                continue
            tf.extract(m, dst)
    files = glob.glob(os.path.join(dst, '**', '*'), recursive=True)
    man = next((f for f in files if os.path.basename(f) == 'MANIFEST.txt'), None)
    mtext = open(man, encoding='utf-8', errors='replace').read() if man else ''
    mt = manifest_tracks(mtext)
    logs = sorted(f for f in files if re.search(r'drive_\d{8}_\d{6}_race\.csv$', f))
    print(f'\n■ {name}: 주행 로그 {len(logs)}개 → {dst}')

    rows = []
    for f in logs:
        df, r = log_stats(f)
        st = '_'.join(os.path.basename(f).split('_')[1:3])
        tn = a.track or mt.get(st)
        trk = load_track(tn) if tn else None
        r['트랙'] = trk[0] if trk else (f'{tn}?' if tn else '-')
        if trk:
            r.update(track_stats(df, trk))
        rows.append(r)
        print('  ', r)

    rep = [f'# {name} 1차 분석\n', f'생성: tools/venue_intake.py · 로그 {len(logs)}개 · 폴더 `{dst}`\n']
    if mtext:
        rep += ['## MANIFEST\n', '```\n' + mtext.strip() + '\n```\n']
    rep += ['## 주행별\n',
            '해석ms = p50/p99/최대 · 배터리 = 시작/최저 V · 벽여유 = 차 중심에서 벽까지 (차 반폭 0.15 m 보다 작으면 닿음)\n\n',
            md_table(rows)]

    maps = sorted({os.path.splitext(os.path.basename(f))[0] for f in files if f.endswith(('.pgm', '.pbstream'))})
    if maps:
        rep += ['\n## 맵\n', '들어 있는 맵: ' + ', '.join(maps) + '\n\n',
                '트랙 만들기 (젯슨 또는 저장소에서): `./drive.sh maptrack <맵> <트랙>` → `./drive.sh preview <트랙>`\n']

    good = [f for f, r in zip(logs, rows) if r['odom'] == 'O']
    if good and not a.no_model:
        cmd = [sys.executable, os.path.join(ROOT, 'tools', 'venue_check.py'), *good] + (['--quick'] if a.quick else [])
        print('\n■ venue_check …')
        p = subprocess.run(cmd, capture_output=True, text=True)
        txt = '\n'.join(ln for ln in (p.stdout + p.stderr).splitlines()
                        if not re.search(r'Warning|warn\(|PacejkaParameters|Using RK4', ln))
        print(txt)
        rep += ['\n## 바닥·지연·모델 (venue_check)\n', '```\n' + txt.strip() + '\n```\n']
    elif not good:
        rep += ['\n## 바닥·지연·모델\n', 'odom_* 열이 있는 로그가 없어 건너뜀 (wheel_odom 을 띄우고 기록해야 함)\n']

    out = os.path.join(dst, 'report.md')
    open(out, 'w', encoding='utf-8').write(''.join(rep))
    with open(DONE, 'a') as fh:
        fh.write(os.path.basename(tar) + '\n')
    print(f'\n→ {out}')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('tars', nargs='*')
    ap.add_argument('--track', help='모든 로그에 이 트랙을 쓴다 (기본: MANIFEST 에서 찾음)')
    ap.add_argument('--quick', action='store_true', help='venue_check 격자를 줄인다')
    ap.add_argument('--no-model', action='store_true', help='venue_check 를 건너뛴다')
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    tars = a.tars
    if not tars:
        subprocess.run(['git', '-C', ROOT, 'pull', '--rebase', '--autostash', '-q'])
        done = set(open(DONE).read().split()) if os.path.exists(DONE) else set()
        tars = [f for f in sorted(glob.glob(os.path.join(ROOT, 'data', 'runs', '*.tar.gz')), key=os.path.getmtime)
                if os.path.basename(f) not in done]
        if not tars:
            print('새 묶음 없음.')
            return
    for t in tars:
        intake(t, a)


if __name__ == '__main__':
    main()
