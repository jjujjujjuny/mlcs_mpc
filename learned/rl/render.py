#!/usr/bin/env python3
"""
render.py — 학습한 정책의 주행을 영상 (mp4) 으로. 여러 정책을 한 트랙에 겹쳐 (고스트) 비교할 수 있다.

  python3 learned/rl/render.py logs/rl/A/best.pt logs/rl/B/best.pt --labels lag vesc --sim vesc --laps 2 -o out.mp4

  --sim lag|vesc   env/simulator.Simulator (casadi) 로 달린다 (기본 vesc — 실차 저수준 경로에 가장 가까운 모델)
  --sim vec        학습 환경 그대로 (저수준 모델은 체크포인트 설정, --vec-actuator 로 바꿀 수 있다)

  그림 — 차: 차체 사각형 (무게중심 기준) + ● 모캡 강체 기준점 = 앞축 중심 (config/mocap.yaml, 실차 로그의 위치가 이 점이다),
         궤적은 앞축 점 (모캡이 보는 선) 을 속도로 색칠. 오른쪽: 속도 vx·바퀴 ω·명령 ω_ref, 조향 δ·명령 δ_ref (시간축).
  ffmpeg 이 필요하다 (conda 환경 mlcs_ros 에 있음). 없으면 .gif 로 저장한다.
"""
import argparse
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from evaluate import ROOT, load, make_env, run_sim, run_vec  # noqa: E402

COLORS = ('tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple')
# 실제 차 치수 (config/mocap.yaml 주석: 휠베이스 0.33, 앞오버행 0.10, 뒤오버행 0.075 / 폭 0.27 = simulator half_width 0.135 /
#   바퀴 지름 0.083 = docs/vesc/vesc6_f1tenth_1124.xml si_wheel_diameter)
OVH_F, OVH_R, CAR_W = 0.10, 0.075, 0.27
WHEEL_D, WHEEL_W = 0.083, 0.035


def main(argv=None):
    ap = argparse.ArgumentParser(description='RL 주행 영상')
    ap.add_argument('ckpts', nargs='+')
    ap.add_argument('--labels', nargs='*', default=[])
    ap.add_argument('--sim', default='vesc', choices=['lag', 'vesc', 'vec'])
    ap.add_argument('--vec-actuator', default='')
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--max-time', type=float, default=40.0, help='영상 최대 길이 s (랩을 못 채워도)')
    ap.add_argument('--keep-going', action='store_true',
                    help='나가거나 멈춰도 계속 달린다 (기본: 그 차는 거기서 끝 — ✗ 표시, 마지막 차가 끝나면 영상 끝)')
    ap.add_argument('--fps', type=int, default=20)
    ap.add_argument('--speed', type=float, default=1.0, help='재생 배속')
    ap.add_argument('-o', '--out', default='')
    a = ap.parse_args(argv)
    torch.set_grad_enabled(False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import animation, font_manager
    for f in ('AppleGothic', 'Apple SD Gothic Neo', 'NanumGothic', 'Noto Sans CJK KR', 'Noto Sans CJK JP', 'Malgun Gothic'):
        if any(f == x.name for x in font_manager.fontManager.ttflist):
            plt.rcParams['font.family'] = f
            break
    plt.rcParams['axes.unicode_minus'] = False
    from matplotlib.patches import Polygon

    runs = []
    for i, ck in enumerate(a.ckpts):
        ac, meta = load(ck)
        if a.sim == 'vec':
            summ, tr = run_vec(ac, meta, a.laps, actuator=a.vec_actuator or None, max_time=a.max_time,
                               stop_on_out=not a.keep_going)
        else:
            summ, tr = run_sim(ac, meta, a.laps, wheel=a.sim, max_time=a.max_time, stop_on_out=not a.keep_going)
        lab = a.labels[i] if i < len(a.labels) else os.path.basename(os.path.dirname(ck))
        runs.append((lab, summ, tr))
        crash = (f'✗ {summ["crash_why"]} {summ["crash_t"]:.2f} s (진행 {summ["crash_prog"]:.1f} m)'
                 if summ['crash_t'] is not None else '실패 없음')
        print(f'  {lab}: {crash}  랩 {summ["laps"]} 최고 {summ["best"]:.2f} s 평균 {summ["mean"]:.2f} s E_off {summ["e_off"]:.4f}')
    env = make_env(load(a.ckpts[0])[1])
    T = max(len(r[2]['t']) for r in runs)
    dt = env.cfg.dt
    # 트랙
    trk = env.track.track
    h = trk.heading
    xl, yl = trk.x - np.sin(h) * trk.w_left, trk.y + np.cos(h) * trk.w_left
    xr, yr = trk.x + np.sin(h) * trk.w_right, trk.y - np.cos(h) * trk.w_right

    fig = plt.figure(figsize=(13, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1])
    axm = fig.add_subplot(gs[:, 0])
    axv = fig.add_subplot(gs[0, 1])
    axd = fig.add_subplot(gs[1, 1], sharex=axv)
    axm.fill(np.r_[xl, xl[0], xr[::-1], xr[-1]], np.r_[yl, yl[0], yr[::-1], yr[-1]], color='0.88', zorder=0)
    axm.plot(np.r_[xl, xl[0]], np.r_[yl, yl[0]], 'k-', lw=1)
    axm.plot(np.r_[xr, xr[0]], np.r_[yr, yr[0]], 'k-', lw=1)
    axm.plot(np.r_[trk.x, trk.x[0]], np.r_[trk.y, trk.y[0]], '--', color='0.6', lw=0.8)
    axm.set_aspect('equal')
    axm.set_title(f'{env.cfg.track}  ({"학습 환경" if a.sim == "vec" else "시뮬레이터 바퀴 " + a.sim})  ● = 모캡 기준점 (앞축)')
    tmax = T * dt
    vmax = max(float(r[2]['vx'].max()) for r in runs) + 0.3
    axv.set_xlim(0, tmax)
    axv.set_ylim(0, max(vmax, env.v_max * 0.6))
    axv.set_ylabel('m/s')
    axv.grid(True)
    axd.set_ylim(-env.max_steer * 1.15, env.max_steer * 1.15)
    axd.set_ylabel('δ [rad]')
    axd.set_xlabel('t [s]')
    axd.grid(True)
    artists = []
    for i, (lab, summ, tr) in enumerate(runs):
        c = COLORS[i % len(COLORS)]
        trail = axm.scatter([], [], s=4, c=[], cmap='viridis', vmin=0, vmax=vmax, zorder=2)
        body = Polygon(np.zeros((5, 2)), closed=True, fc=c, ec='k', lw=0.8, alpha=0.85, zorder=3)
        axm.add_patch(body)
        wheels = [Polygon(np.zeros((4, 2)), closed=True, fc='k', ec='k', zorder=4) for _ in range(4)]
        for w in wheels:
            axm.add_patch(w)
        mark, = axm.plot([], [], 'o', color='red', ms=4, zorder=4)
        xmark, = axm.plot([], [], 'x', color='red', ms=14, mew=3, zorder=5)
        lv, = axv.plot([], [], '-', color=c, lw=1.5, label=f'{lab} vx')
        lw, = axv.plot([], [], ':', color=c, lw=1.2, label=f'{lab} ω')
        lwr, = axv.plot([], [], '--', color=c, lw=0.8, alpha=0.6, label=f'{lab} ω_ref')
        ld, = axd.plot([], [], '-', color=c, lw=1.5, label=f'{lab} δ')
        ldr, = axd.plot([], [], '--', color=c, lw=0.8, alpha=0.6, label=f'{lab} δ_ref')
        artists.append((tr, trail, body, mark, lv, lw, lwr, ld, ldr, xmark, wheels))
    axv.legend(fontsize=7, ncol=len(runs), loc='upper left')
    axd.legend(fontsize=7, ncol=len(runs), loc='upper left')
    txt = axm.text(0.01, 0.99, '', transform=axm.transAxes, va='top', fontsize=8,
                   bbox=dict(fc='white', alpha=0.8, ec='0.7'))
    fig.colorbar(artists[0][1], ax=axm, shrink=0.6, label='vx [m/s]')
    lr = float(env.P.p[0, 1])
    lf = float(env.P.wheelbase[0]) - lr
    xr, xf = -lr - OVH_R, lf + OVH_F                    # 무게중심 기준 차체 앞·뒤 끝
    # 차체: 뒤는 네모, 앞은 뾰족 (세모) — 앞뒤·헤딩이 바로 보이게
    corners = np.array([[xr, -CAR_W / 2], [lf - 0.02, -CAR_W / 2], [xf, 0.0], [lf - 0.02, CAR_W / 2], [xr, CAR_W / 2]])
    wheel = np.array([[-WHEEL_D / 2, -WHEEL_W / 2], [WHEEL_D / 2, -WHEEL_W / 2],
                      [WHEEL_D / 2, WHEEL_W / 2], [-WHEEL_D / 2, WHEEL_W / 2]])
    wy = CAR_W / 2 - WHEEL_W / 2
    wheel_pos = [(lf, wy, True), (lf, -wy, True), (-lr, wy, False), (-lr, -wy, False)]

    def rot(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s], [s, c]])

    def draw(k):
        lines = [f't = {k * dt:5.2f} s']
        out = [txt]
        for i, (tr, trail, body, mark, lv, lw, lwr, ld, ldr, xmark, wheels) in enumerate(artists):
            j = min(k, len(tr['t']) - 1)
            ended = k >= len(tr['t']) - 1 and runs[i][1]['crash_t'] is not None and not a.keep_going
            xmark.set_data([tr['fx'][j]] if ended else [], [tr['fy'][j]] if ended else [])
            body.set_alpha(0.35 if ended else 0.85)
            R = rot(tr['yaw'][j])
            P0 = np.array([tr['x'][j], tr['y'][j]])
            body.set_xy(corners @ R.T + P0)
            for w, (wx, wyy, front) in zip(wheels, wheel_pos):
                shape = wheel @ rot(tr['delta'][j]).T if front else wheel      # 앞바퀴는 실제 조향각만큼
                w.set_xy((shape + [wx, wyy]) @ R.T + P0)
            mark.set_data([tr['fx'][j]], [tr['fy'][j]])
            trail.set_offsets(np.c_[tr['fx'][:j + 1], tr['fy'][:j + 1]])
            trail.set_array(tr['vx'][:j + 1])
            tt = tr['t'][:j + 1]
            lv.set_data(tt, tr['vx'][:j + 1])
            lw.set_data(tt, tr['ws'][:j + 1])
            lwr.set_data(tt, tr['ws_ref'][:j + 1])
            ld.set_data(tt, tr['delta'][:j + 1])
            ldr.set_data(tt, tr['delta_ref'][:j + 1])
            lab, summ = runs[i][0], runs[i][1]
            status = f'[실패] {summ["crash_why"]} {summ["crash_t"]:.2f}s' if ended else f'최고랩 {summ["best"]:.2f}'
            lines.append(f'{lab:>8}: vx {tr["vx"][j]:4.2f}  ω {tr["ws"][j]:4.2f}/{tr["ws_ref"][j]:4.2f}  '
                         f'δ {tr["delta"][j]:+.2f}  n {tr["n"][j]:+.2f}  {status}')
            out += [trail, body, mark, lv, lw, lwr, ld, ldr, xmark] + wheels
        txt.set_text('\n'.join(lines))
        return out

    step = max(1, int(round(a.speed * (1 / dt) / a.fps)))
    frames = range(0, T, step)
    anim = animation.FuncAnimation(fig, draw, frames=frames, blit=False)
    out = a.out or os.path.join(ROOT, 'logs/rl', 'run.mp4')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    try:
        import shutil
        ff = shutil.which('ffmpeg') or os.path.join(os.path.dirname(sys.executable), 'ffmpeg')
        plt.rcParams['animation.ffmpeg_path'] = ff
        anim.save(out, writer=animation.FFMpegWriter(fps=a.fps, bitrate=2400))
    except Exception as e:
        out = os.path.splitext(out)[0] + '.gif'
        print(f'  ffmpeg 실패 ({e}) → gif')
        anim.save(out, writer=animation.PillowWriter(fps=a.fps))
    print(f'✓ {out}  ({len(frames)} 프레임, {T * dt:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
