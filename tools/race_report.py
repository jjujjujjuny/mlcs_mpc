#!/usr/bin/env python3
"""
race_report.py — 라이다로 달린 주행을 mocap 정답으로 채점한다. 연구실용.

  ./drive.sh racereport              가장 최근 라이다 주행 (./race.sh 가 끝나면 자동)
  python3 tools/race_report.py --drive <drive.csv> --align <기록.csv> --map <맵> --track <prep.csv>

■ 두 가지를 본다

  ① 위치추정: 라이다가 낸 자세가 mocap 과 얼마나 다른가 (map_mocap_align eval 과 같다)
  ② 주행: 차가 **실제로** 트랙 선에서 얼마나 벗어났나. mocap 을 map 프레임으로 옮겨 잰다.
     MPC 가 본 값(라이다)과 나란히 놓으면 위치추정 오차가 주행을 얼마나 흐렸는지 보인다.

  기록 두 개를 쓴다: data_logger (라이다 상태, 명령) 와 map_mocap_align record (mocap, 라이다 원본).
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_mocap_align as mma                       # noqa: E402
from track_from_lap import dist_to_polyline         # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LR = 0.1537     # 뒤축 → 무게중심. 트랙 선은 무게중심 기준 (hmpc_core.make_state)


def signed_offset(P, C):
    """점 → 닫힌 선까지 거리. 진행 방향 왼쪽이 +."""
    d = dist_to_polyline(P, C)
    i = np.argmin(((P[:, None, :] - C[None, :, :]) ** 2).sum(-1), axis=1)
    t = np.roll(C, -1, axis=0) - np.roll(C, 1, axis=0)
    cross = t[i, 0] * (P[:, 1] - C[i, 1]) - t[i, 1] * (P[:, 0] - C[i, 0])
    return np.where(cross >= 0, d, -d)


def stats(a):
    a = np.abs(a)
    return f'평균 {a.mean()*100:5.1f}  p95 {np.percentile(a, 95)*100:5.1f}  최대 {a.max()*100:5.1f} cm'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('--drive', required=True, help='data_logger CSV')
    ap.add_argument('--align', required=True, help='map_mocap_align record CSV')
    ap.add_argument('--map', required=True, help='맵 이름 (maps/<맵>.mocap.yaml 이 있어야 한다)')
    ap.add_argument('--track', required=True, help='map 프레임 prep CSV')
    a = ap.parse_args()

    T_path = os.path.join(REPO, 'maps', f'{a.map}.mocap.yaml')
    if not os.path.isfile(T_path):
        print(f'✗ {T_path} 가 없습니다 — ./drive.sh align {a.map} 으로 먼저 맞추세요')
        sys.exit(1)

    # ★ 위치추정은 첫 출발 뒤부터 채점한다. 그 전에는 RViz 로 시작 위치를 찍기 전이라 Cartographer 가
    #   엉뚱한 곳에 있을 수 있다 (11-08 1108_8: 전체 72 cm / 출발 뒤 47 cm, 정지 구간 yaw 51°).
    D0 = np.genfromtxt(os.path.expanduser(a.drive), delimiter=',', names=True)
    moving = np.nan_to_num(D0['cmd_speed']) > 0.05
    align_log = a.align
    if moving.any():
        t_go = float(D0['stamp'][moving][0])
        with open(a.align, encoding='utf-8') as f:
            head = f.readline()
            rows = [ln for ln in f if float(ln.split(',')[1]) >= t_go]
        align_log = os.path.splitext(a.align)[0] + '_go.csv'   # 그림도 이 옆에 남는다
        with open(align_log, 'w', encoding='utf-8') as f:
            f.write(head)
            f.writelines(rows)
        print(f'(첫 출발 {t_go - float(D0["stamp"][0]):.1f} s 뒤부터 채점 — 그 전은 위치를 찍기 전일 수 있다)')

    print('════════ ① 위치추정 (라이다 vs mocap) ════════')
    with open(align_log, encoding='utf-8') as f:
        n_mocap = sum(1 for ln in f if ln.startswith('m,'))
    if n_mocap < 50:
        # natnet 이 Motive 에 못 붙은 주행 (11-08 두 번). 정답이 없으니 라이다만으로 본다
        print(f'   ⚠ mocap 기록 {n_mocap}개 — 정답이 없어 위치추정 채점을 건너뜁니다 (natnet / Motive 확인)')
    else:
        mma.cmd_eval(argparse.Namespace(log=align_log, align=T_path, mocap_yaml=mma.MOCAP_YAML,
                                        no_offset=False))

    print('\n════════ ② 주행 — 트랙 선에서 벗어난 거리 ════════')
    T = mma.load_T(T_path)
    M, _ = mma.load_log(a.align, mma.load_mocap_cfg())
    D = np.genfromtxt(os.path.expanduser(a.drive), delimiter=',', names=True)
    tr = np.genfromtxt(a.track, delimiter=',', names=True)
    C = np.c_[tr['x'], tr['y']]

    # 자율 구간: 명령 속도가 있는 행. 수동 조종도 명령이 있지만 ./race.sh 주행은 자율이다
    auto = np.nan_to_num(D['cmd_speed']) > 0.01
    if auto.sum() < 50:
        print('✗ 달린 구간이 거의 없습니다')
        sys.exit(1)
    ts = D['stamp'][auto]
    # mocap (뒤축, mocap 프레임) → map 프레임 → 무게중심
    R = mma.rot(-T['theta'])
    Pm = (np.c_[M['x'], M['y']] - T['t']) @ R.T
    psi = M['yaw'] - T['theta']
    Pm = Pm + LR * np.c_[np.cos(psi), np.sin(psi)]
    inside = (M['t'] >= ts[0]) & (M['t'] <= ts[-1])
    near = np.abs(np.interp(M['t'], ts, ts) - M['t']) < 0.05      # 자율 구간 안의 mocap 만
    k = inside & near
    has_true = k.sum() >= 50
    if not has_true:
        print('   ⚠ 자율 구간에 mocap 이 없어 MPC 가 본 값(라이다)으로만 평가합니다')
    n_true = signed_offset(Pm[k], C) if has_true else None
    yawl = D['yaw'][auto]
    Pl = np.c_[D['x'][auto] + LR * np.cos(yawl), D['y'][auto] + LR * np.sin(yawl)]
    n_lidar = signed_offset(Pl, C)
    v = D['vx_kf'][auto]
    print(f'   자율 {auto.sum()*0.01:.1f} s, 평균 속도 {np.mean(v):.2f} m/s (최대 {np.max(v):.2f})')
    if has_true:
        print(f'   실제 (mocap)   {stats(n_true)}   치우침 {np.mean(n_true)*100:+.1f} cm')
    print(f'   MPC 가 본 값   {stats(n_lidar)}   치우침 {np.mean(n_lidar)*100:+.1f} cm')
    # 바퀴 시간 (실제 궤적의 진행. mocap 이 없으면 라이다 궤적)
    P_lap, t_lap = (Pm[k], M['t'][k]) if has_true else (Pl, ts)
    i = np.argmin(((P_lap[:, None, :] - C[None, :, :]) ** 2).sum(-1), axis=1)
    laps = np.where(np.diff(i) < -len(C) // 2)[0]
    if len(laps) >= 2:
        lt = np.diff(t_lap[laps])
        print(f'   바퀴 {len(lt)}개{"" if has_true else " (라이다 기준)"}: '
              + ', '.join(f'{x:.2f}' for x in lt) + ' s')
    if 'solve_ms' in D.dtype.names:
        sv = D['solve_ms'][auto]
        sv = sv[np.isfinite(sv)]
        if len(sv):
            print(f'   풀이 평균 {sv.mean():.1f}  p95 {np.percentile(sv, 95):.1f}  최대 {sv.max():.1f} ms')
    st = D['cmd_steer'][auto]
    print(f'   조향 0.39 이상 {np.mean(np.abs(st) > 0.39)*100:.0f}%')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import draw_track as dt
        dt.setup_font()
        fig, ax = plt.subplots(1, 2, figsize=(13, 6))
        ax[0].plot(*np.vstack([C, C[:1]]).T, 'k-', lw=1, label='트랙 선')
        if has_true:
            ax[0].plot(*Pm[k].T, '-', color='tab:blue', lw=1, label='실제 (mocap)')
        ax[0].plot(*Pl.T, '-', color='tab:orange', lw=0.8, alpha=0.7, label='MPC 가 본 값 (라이다)')
        ax[0].set_aspect('equal')
        ax[0].legend(fontsize=8)
        ax[0].set_title('map 프레임 궤적')
        if has_true:
            ax[1].plot(M['t'][k] - ts[0], n_true * 100, lw=0.8, label='실제')
        ax[1].plot(ts - ts[0], n_lidar * 100, lw=0.8, alpha=0.7, label='MPC 가 본 값')
        ax[1].set_xlabel('s')
        ax[1].set_ylabel('선에서 벗어난 거리 cm (왼쪽 +)')
        ax[1].legend(fontsize=8)
        ax[1].grid(alpha=0.3)
        png = os.path.splitext(os.path.expanduser(a.drive))[0] + '_report.png'
        fig.savefig(png, dpi=100, bbox_inches='tight')
        print(f'\n   그림 {png}')
    except Exception as e:      # 그림은 덤이다
        print(f'   (그림 생략: {e})')


if __name__ == '__main__':
    main()
