"""
sim_report — 튜닝 주행의 출력: 결과 정리 (터미널) 와 그림 (png). main.py · main_sim.py 가 쓴다.

  report(sim, ticks, rows, params, title, terms)   한 회 정리 — 랩·속도·트랙·풀이·입력·비용 구성·모델 일치
  summary(...)                                      report + 걸린 시간
  plot(track, rows, path, ..., ticks, params)       트랙 위 궤적 + 명령이 차까지 가는 길 (MPC → /drive → VESC → 차)
  plot_path(tag)                                    <저장소>/logs/tune/<시각>[_tag].png

  rows  = 적분 주기 (100 Hz) 기록, 열은 COLS
  ticks = 제어 주기 기록 (main.control_tick)
"""
import os
import time

import numpy as np

# 기록 열 (rows)
COLS = 't,x,y,yaw,vx,vy,r,wheel_speed,delta,cmd_steer,cmd_speed,s,n,on_wall'.split(',')
C = {k: i for i, k in enumerate(COLS)}


def cost_names(ctrl):
    t = getattr(getattr(ctrl, 'mpc', None), 'terms', None)
    return (t[0], t[1]) if t else None


def report(sim, ticks, rows, params, title='결과', terms=None, mpc=None):
    """한 바퀴 (또는 한 회) 를 정리해 출력한다 — 랩·속도·경계·풀이 시간·입력·모델 일치."""
    rows = np.array(rows)
    cfg_v, max_cmd = float(params['v_max']), float(params['max_steer'])
    budget = 1000.0 / float(params['control_rate'])
    solved = [k for k in ticks if k['status'] >= 0]
    n_start = len(ticks) - len(solved)
    ms = np.array([k['ms'] for k in solved[1:]] or [k['ms'] for k in solved] or [0.0])
    st = np.array([k['status'] for k in solved] or [0])
    codes, counts = np.unique(st, return_counts=True)
    print(f'\n■ {title}  (시뮬 {sim.t:.1f} s, 제어 {len(ticks)}회)')
    if sim.laps:
        print(f'  랩        {"  ".join(f"{t:.2f}" for t in sim.laps)} s   최고 {min(sim.laps):.2f} s')
    else:
        print('  랩        미완')
    if len(rows):
        n = rows[:, C['n']]
        print(f'  속도      평균 {rows[:, C["vx"]].mean():.2f}  최대 {rows[:, C["vx"]].max():.2f} m/s  (v_max {cfg_v})')
        print(f'  트랙      벽 {sim.wall_hits}회   |n| 최대 {np.abs(n).max():.3f} m')
    if len(ms):
        print(f'  풀이      평균 {ms.mean():.1f}  p95 {np.percentile(ms, 95):.1f}  최대 {ms.max():.1f} ms   '
              f'(예산 {budget:.0f} ms, 초과 {(ms > budget).mean() * 100:.1f}%)   status {dict(zip(codes.tolist(), counts.tolist()))}'
              + (f'   출발 직진 {n_start}주기 ({n_start / float(params["control_rate"]):.2f} s, 풀지 않음)' if n_start else ''))
    ok = [k for k in ticks if k['status'] == 0]
    if ok:
        u = np.array([k['u'] for k in ok])
        cmd = np.array([k['cmd'] for k in ok])
        ap = np.array([k['applied'] for k in ok])
        print(f'  입력      바퀴 ref {u[:, 0].min():.2f}~{u[:, 0].max():.2f} m/s, 실제 조향 ref {u[:, 1].min():+.3f}~{u[:, 1].max():+.3f} rad')
        print(f'            /drive 조향 명령 {cmd[:, 1].min():+.3f}~{cmd[:, 1].max():+.3f} (한계 ±{max_cmd}, 한계에 걸림 '
              f'{(np.abs(cmd[:, 1]) >= max_cmd - 1e-6).mean() * 100:.0f}%), 속도 v_max 에 걸림 {(cmd[:, 0] >= cfg_v - 1e-6).mean() * 100:.0f}%')
        d_in = np.abs(ap[:, 1] - u[:, 1])
        print(f'            MPC 조향 ref → 차가 받은 실제 조향 ref 차이: 평균 {d_in.mean():.4f}  최대 {d_in.max():.4f} rad '
              f'(변화율 제한·VESC 서보 한계·조향 맵)')
    # 비용 구성 — 예측 구간 Σ 항별 비용의 평균. 어떤 항이 비용을 지배하는지 본다
    tv = [k['terms'] for k in ticks if k.get('terms') is not None]
    if tv and terms is not None:
        names, groups = terms
        m = np.mean(tv, axis=0)
        tot = np.abs(m).sum()
        for g, label in (('perf', '성능'), ('reg', '정규화')):
            items = [f'{n} {v:+.2f} ({abs(v) / tot * 100:.0f}%)' for n, gg, v in zip(names, groups, m) if gg == g]
            print(f'  비용 {label:<4} ' + '   '.join(items))
    if mpc is not None and getattr(mpc, 'last_P', None) is not None:
        pin = mpc.stage_params_in_solver()          # 솔버 안의 값 (N+1, 30)
        base = np.asarray(mpc.params, float)
        names = ['I_z', 'lr', 'Cd0', 'Cd2', 'Cd1'] + [f'f{i}' for i in range(10)] + [f'r{i}' for i in range(10)]
        diff_stage = np.abs(pin - pin[0]).max(axis=0)
        varying = [f'{names[i] if i < 5 else ("앞" if i < 20 else "뒤") + str((i - 10) % 10)}' for i in np.flatnonzero(diff_stage > 1e-12) if i < 30]
        print(f'  동역학 P  솔버 안 {pin.shape[0]} 스테이지 × {pin.shape[1]}  ({mpc.pm.name})   넣은 값과 차이 {np.abs(pin - mpc.last_P).max():.1e}   '
              f'mpcc.yaml 과 차이 {np.abs(pin - base).max():.1e}   '
              + ('스테이지마다 같음 (상수)' if not varying else f'스테이지마다 다른 항목 {len(varying)}개'))
    # 1주기 예측 − 실제 (같은 모델이라 작아야 한다. 크면 입력 경로나 모델이 어긋난 것)
    pairs = [(a, b) for a, b in zip(ticks, ticks[1:]) if a['status'] == 0 and b['status'] == 0 and a['x'][3] > 0.3]
    if pairs:
        e = np.array([b['x'] - a['x1'] for a, b in pairs])
        e[:, 2] = (e[:, 2] + np.pi) % (2 * np.pi) - np.pi
        rms = np.sqrt(np.mean(e ** 2, axis=0))
        print('  모델 일치 1주기 예측 − 실제 (RMS)  '
              + '  '.join(f'{nm} {rms[i]:.4f}' for i, nm in ((0, 's'), (1, 'n'), (2, 'mu'), (3, 'vx'), (4, 'vy'), (5, 'r'), (7, 'ws'), (8, 'δ'))))
    return rows


def summary(sim, ticks, rows, t_wall, params, ctrl=None):
    rows = report(sim, ticks, rows, params, terms=cost_names(ctrl), mpc=getattr(ctrl, 'mpc', None))
    print(f'  (실제 걸린 시간 {time.perf_counter() - t_wall:.1f} s)')
    return rows


# 저장소 아래 logs/ (.gitignore). utils/ → mlcs_mpc/ → src/mlcs_mpc/ → src/ → 저장소
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..'))
LOG_DIR = os.path.join(REPO, 'logs')
TUNE_DIR = os.path.join(LOG_DIR, 'tune')


def plot_path(tag=''):
    os.makedirs(TUNE_DIR, exist_ok=True)
    return os.path.join(TUNE_DIR, time.strftime('%Y%m%d_%H%M%S') + (f'_{tag}' if tag else '') + '.png')


def plot(track, rows, path, title='', margin=None, ticks=None, params=None, show=False):
    """한 회 주행 정리 그림.

    왼쪽  트랙 (면·벽·중심선·MPC 경계) 위 궤적 (속도 색)
    오른쪽 명령이 차까지 가는 길 — MPC 해 → /drive 명령 → VESC 를 지난 값 → 실제 차
           ① 속도  ② 조향  ③ 주기마다 입력 변화 (Δ)  ④ 횡편차 n·풀이 시간
    """
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ss = np.linspace(0.0, track.length, max(400, int(track.length / 0.02)))
    wl = np.interp(ss, np.r_[track.s, track.length], np.r_[track.w_left, track.w_left[0]])
    wr = np.interp(ss, np.r_[track.s, track.length], np.r_[track.w_right, track.w_right[0]])
    lx, ly = track.xy(ss, wl)
    rx, ry = track.xy(ss, -wr)
    cx, cy = track.xy(ss, 0 * ss)
    fig = plt.figure(figsize=(17, 9))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.0, 1.25])
    a = fig.add_subplot(gs[:, 0])
    a.fill(np.r_[lx, rx[::-1]], np.r_[ly, ry[::-1]], color='0.88', zorder=0)
    a.plot(lx, ly, 'k-', lw=1.5)
    a.plot(rx, ry, 'k-', lw=1.5)
    a.plot(cx, cy, '--', color='0.45', lw=0.9, label='centerline (reference)')
    if margin is not None:
        for n in (wl - margin, -(wr - margin)):
            mx, my = track.xy(ss, n)
            a.plot(mx, my, ':', color='tab:red', lw=0.8)
        a.plot([], [], ':', color='tab:red', label=f'MPC bound (wall - {margin})')
    a.plot(cx[0], cy[0], 'k>', ms=9, label='start (s=0)')
    sc = a.scatter(rows[:, C['x']], rows[:, C['y']], c=rows[:, C['vx']], s=3, cmap='turbo', zorder=3)
    hit = rows[:, C['on_wall']] > 0
    if hit.any():
        a.plot(rows[hit, C['x']], rows[hit, C['y']], 'rx', ms=4, zorder=4, label='wall hit')
    a.set_aspect('equal')
    a.legend(loc='upper right', fontsize=8)
    fig.colorbar(sc, ax=a, label='vx [m/s]', shrink=0.7)
    a.set_title(title or 'trajectory')
    t = rows[:, C['t']]
    ax = [fig.add_subplot(gs[i, 1]) for i in range(4)]
    for x_ in ax[1:]:
        x_.sharex(ax[0])
    tk = None
    if ticks:
        tk = dict(t=np.array([k['t'] for k in ticks]), u=np.array([k['u'] for k in ticks]),
                  cmd=np.array([k['cmd'] for k in ticks]), ap=np.array([k['applied'] for k in ticks]),
                  ok=np.array([k['status'] == 0 for k in ticks]), ms=np.array([k['ms'] for k in ticks]))
    # ① 속도
    if tk is not None:
        ax[0].step(tk['t'], tk['u'][:, 0], where='post', lw=1, label='MPC ws_ref')
        ax[0].step(tk['t'], tk['cmd'][:, 0], where='post', lw=1, ls='--', label='/drive speed')
        ax[0].step(tk['t'], tk['ap'][:, 0], where='post', lw=1, ls=':', label='after VESC')
    ax[0].plot(t, rows[:, C['wheel_speed']], lw=1.4, label='wheel speed (actual)')
    ax[0].plot(t, rows[:, C['vx']], lw=1.0, alpha=0.8, label='vx (actual)')
    ax[0].set_ylabel('speed [m/s]')
    # ② 조향
    if tk is not None:
        ax[1].step(tk['t'], tk['u'][:, 1], where='post', lw=1, label='MPC delta_ref (actual angle)')
        ax[1].step(tk['t'], tk['cmd'][:, 1], where='post', lw=1, ls='--', label='/drive steer (servo cmd)')
        ax[1].step(tk['t'], tk['ap'][:, 1], where='post', lw=1, ls=':', label='after VESC + map (actual ref)')
    ax[1].plot(t, rows[:, C['delta']], lw=1.4, label='steer (actual)')
    if params is not None:
        for v in (float(params['max_steer']), -float(params['max_steer'])):
            ax[1].axhline(v, color='0.6', lw=0.6)
    ax[1].set_ylabel('steer [rad]')
    # ③ 입력 변화 (주기마다)
    if tk is not None and len(tk['t']) > 1:
        dt_ = np.diff(tk['t'])
        ax[2].plot(tk['t'][1:], np.diff(tk['cmd'][:, 1]) / dt_, lw=0.9, label='d(steer cmd)/dt [rad/s]')
        ax[2].plot(tk['t'][1:], np.diff(tk['cmd'][:, 0]) / dt_, lw=0.9, label='d(speed cmd)/dt [m/s2]')
        if params is not None:
            r = float(params['max_steer_rate'])
            ax[2].axhline(r, color='0.6', lw=0.6)
            ax[2].axhline(-r, color='0.6', lw=0.6)
    ax[2].set_ylabel('input rate')
    # ④ 횡편차·풀이 시간
    ax[3].plot(t, rows[:, C['n']], lw=1.2, label='n (lateral to reference)')
    ax[3].axhline(0, color='0.6', lw=0.6)
    ax[3].set_ylabel('n [m]')
    if tk is not None:
        b2 = ax[3].twinx()
        b2.plot(tk['t'], tk['ms'], color='tab:gray', lw=0.6, alpha=0.7)
        bad = np.array([k['status'] > 0 for k in ticks])
        if bad.any():
            b2.plot(tk['t'][bad], tk['ms'][bad], 'rx', ms=4)
        b2.set_ylabel('solve [ms] (x = status!=0)', color='tab:gray')
    ax[3].set_xlabel('t [s]')
    for x_ in ax:
        x_.grid(alpha=0.3)
        x_.legend(fontsize=7, loc='upper right', ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    if show:
        plt.show(block=False)
        plt.pause(0.1)
    else:
        plt.close(fig)
