#!/usr/bin/env python3
"""
main.py — MPC 튜닝용 메인. ROS 없이 한 프로세스에서 설정 → 트랙 → 차 모델 → MPC → 시뮬레이터 루프를 돈다.

  python3 src/mlcs_mpc/main.py                          race.yaml 의 track·mpcc 설정으로 3바퀴
  python3 src/mlcs_mpc/main.py --track s_track --set v_max=1.2 q_n=0.05 --laps 5 --plot /tmp/run.png
  python3 src/mlcs_mpc/main.py --car wheel_speed_tau=0.66 friction=0.35     차만 다르게 (모델 불일치 시험)
  python3 src/mlcs_mpc/main.py -i                       ★ 대화형 튜닝 — 한 바퀴 → 결과 → 값 입력 → 다시

  설정 우선순위: config/mpcc.yaml → race.yaml 의 mpcc: → --config 파일들 → --set
  acados 가 있는 파이썬에서 돌린다 (젯슨은 ~/hmpc-venv, 이 PC 는 시스템 python3 + ~/acados).

■ 이 파일은 명령줄·루프·대화형 튜닝만. 공통 환경 (설정·트랙·모델·MPC·시뮬 구성, 제어 한 주기) 은
  mlcs_mpc/env/racing.py, 출력 (결과 정리·그림) 은 utils/sim_report.py.

■ 실차·ROS 경로와 같은 계산이다
  컨트롤러  env/mpc/mpcc.MpccController — mpcc_node 의 한 주기와 같은 식 (상태 변환·벽·명령 변환)
  솔버      env/mpc/mpcc.MPCC            — mpcc_node 의 워커가 쓰는 것과 같은 클래스
  차        env/simulator.Simulator      — env/vehicle_model (MPC 와 같은 식) + env/vesc (차의 VESC 값)
  주기      제어 control_rate (30 Hz), 적분 100 Hz. 풀이 시간만큼의 지연은 --delay 로 넣는다.

  최종 확인은 ./sim.sh (ROS — 실제 토픽·타이밍·감시까지), 실차는 ./race.sh.
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))       # src/mlcs_mpc
sys.path.insert(0, HERE)


from mlcs_mpc.utils.runenv import ensure_acados  # noqa: E402

ensure_acados()

from mlcs_mpc.env.racing import CAR_KEYS, RACE, control_tick, load_params, parse_kv, setup  # noqa: E402,F401
from mlcs_mpc.utils.sim_report import C, COLS, cost_names, plot, plot_path, report, summary  # noqa: E402,F401

def add_args(ap):
    ap.add_argument('--track', help='트랙 이름 (env/track/data/) 또는 prep csv. 없으면 race.yaml 의 track')
    ap.add_argument('--race', default=RACE, help='race.yaml (mpcc: 덮어쓰기와 track). 없애려면 --race ""')
    ap.add_argument('--config', nargs='*', help='mpcc.yaml 형식 덮어쓸 파일')
    ap.add_argument('--set', nargs='*', metavar='KEY=VALUE', help='mpcc.yaml 값 덮어쓰기 (MPC 와 차 모두)')
    ap.add_argument('--car', nargs='*', metavar='KEY=VALUE',
                    help=f'차만 다른 값 ({", ".join(CAR_KEYS)}, ws_acc_max, ws_dec_max) — MPC 는 그대로')
    ap.add_argument('--start-s', type=float, default=0.0)
    ap.add_argument('--vesc', default='', help='vesc.yaml (기본 젯슨 경로, 없으면 패치 값)')
    ap.add_argument('--build-dir', default=os.path.expanduser('~/mpcc_build'))


def run(params, ctrl, sim, laps, time_max=120.0, delay=False, verbose=True):
    """제어 control_rate 로 풀고 100 Hz 로 적분. laps 바퀴 돌거나 time_max 가 되면 끝. → (ticks, rows)."""
    period = 1.0 / float(params['control_rate'])
    t_next, cmd, pending = 0.0, (0.0, 0.0), None
    ticks, rows, laps_seen, wall0 = [], [], 0, 0
    while sim.t < time_max and len(sim.laps) < laps:
        if sim.t + 1e-9 >= t_next:
            c, tick = control_tick(ctrl, sim)
            ticks.append(tick)
            if tick['tripped'] and verbose:
                print(f'  ✗ {sim.t:.2f} s: 비정상 해 {ctrl.max_bad}번 연속 — 실차라면 여기서 래치해 멈춘다')
            if delay:
                pending = (sim.t + tick['ms'] / 1000.0, c)
            else:
                cmd = c
            t_next += period
        if pending is not None and sim.t + 1e-9 >= pending[0]:
            cmd, pending = pending[1], None
        info = sim.step(*cmd)
        if info['new_hit'] and verbose:
            print(f'  ⚠ {sim.t:6.2f} s 벽 — s {info["s"]:.2f} n {info["n"]:+.3f}')
        if len(sim.laps) > laps_seen:
            laps_seen = len(sim.laps)
            if verbose:
                print(f'  랩 {laps_seen}: {sim.laps[-1]:.2f} s  (벽 {sim.wall_hits - wall0}회)')
            wall0 = sim.wall_hits
        rows.append((sim.t, *sim.pose(), info['vx'], info['vy'], info['r'], info['ws'], info['delta'],
                     cmd[0], cmd[1], info['s'], info['n'], int(info['on_wall'])))
    return ticks, rows


def interactive(args):
    """한 바퀴씩 돌리며 값을 바꾼다. 차·MPC 를 매번 새로 만든다 (생성 코드는 바뀐 값이 ★ 일 때만 다시 빌드)."""
    extra, car_extra, hist = {}, {}, []
    laps = args.laps or 1
    params, track, ctrl, sim = setup(args)
    print('''
■ 대화형 튜닝 — 매 회 한 바퀴 (laps=N 으로 바꿈). 입력:
    q_n=0.05 v_max=1.2        mpcc.yaml 값 (MPC·차 모두)        car.wheel_speed_tau=0.66   차만
    엔터 = 같은 설정 다시      show = 지금 바꾼 값      reset = 처음 값으로      plot = 그림 경로      q = 끝\n    (그림은 매 회 <저장소>/logs/tune/ 에 저장, --show 면 창으로)
  ★ 가중치(q_*)·N·dt·rk_steps·시정수 등은 생성 코드에 들어가 처음 한 번 다시 빌드한다 (이 PC 약 4초)''')
    rows = []
    while True:
        k = len(hist) + 1
        t0 = time.perf_counter()
        ticks, rows = run(params, ctrl, sim, laps, args.time, args.delay, verbose=False)
        rows = report(sim, ticks, rows, params, terms=cost_names(ctrl), mpc=ctrl.mpc, title=f'#{k}  ' + (' '.join(
            f'{a}={b}' for a, b in {**extra, **{'car.' + a: b for a, b in car_extra.items()}}.items()) or '(기본 설정)'))
        sol = [t for t in ticks if t['status'] >= 0]
        ms_ = np.array([t['ms'] for t in sol[1:]] or [t['ms'] for t in sol] or [0.0])
        bad = int(sum(t['status'] > 0 for t in ticks))
        res = dict(k=k, change=' '.join(f'{a}={b}' for a, b in {**extra, **{'car.' + a: b for a, b in car_extra.items()}}.items()) or '(기본)',
                   laps=list(sim.laps), walls=sim.wall_hits, n=float(np.abs(rows[:, C['n']]).max()),
                   v=float(rows[:, C['vx']].mean()), p95=float(np.percentile(ms_, 95)), bad=bad, t=time.perf_counter() - t0)
        hist.append(res)
        res['plot'] = plot_path(f'tune{k}')
        plot(track, rows, res['plot'], f"#{k} {res['change']}", ctrl.cfg.wall_margin, ticks, params, show=args.show)
        print(f"  ({res['t']:.1f} s 걸림)   그림 {res['plot']}")
        if len(hist) > 1:
            print('  ─ 기록 ─────────────────────────────────────────────')
            for h in hist[-8:]:
                best = f"{min(h['laps']):6.2f}" if h['laps'] else '   미완'
                print(f"   #{h['k']:<3} {best} s  벽 {h['walls']:<2} |n| {h['n']:.3f}  v {h['v']:.2f}  "
                      f"풀이 p95 {h['p95']:5.1f} ms  비정상 {h['bad']:<3} {h['change']}")
        while True:
            try:
                line = input('\n  변경 > ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if line in ('q', 'quit', 'exit'):
                return 0
            if line == 'show':
                print(f'  MPC·차 {extra}   차만 {car_extra}   laps {laps}')
                continue
            if line == 'plot':
                print(f"  그림 {res['plot']}")
                continue
            if line == 'reset':
                extra, car_extra = {}, {}
                break
            try:
                new, new_car, new_laps = dict(extra), dict(car_extra), laps
                for kk, v in parse_kv(line.split()).items():
                    if kk == 'laps':
                        new_laps = int(v)
                    elif kk.startswith('car.'):
                        if kk[4:] not in CAR_KEYS + ('ws_acc_max', 'ws_dec_max'):
                            raise KeyError(f'차만 바꿀 수 있는 값: {", ".join(CAR_KEYS)}, ws_acc_max, ws_dec_max')
                        new_car[kk[4:]] = v
                    else:
                        new[kk] = v
                load_params(args, new)          # 이름 검사
            except (KeyError, ValueError) as e:
                print(f'  ✗ {e}')
                continue
            extra, car_extra, laps = new, new_car, new_laps
            break
        params, track, ctrl, sim = setup(args, extra, car_extra, quiet=True)


def main():
    ap = argparse.ArgumentParser(description='MPC 튜닝 — 설정·트랙·차 모델·MPC·시뮬레이터를 한 프로세스에서')
    add_args(ap)
    ap.add_argument('-i', '--interactive', action='store_true', help='대화형 — 한 바퀴씩 돌리며 값을 바꾼다')
    ap.add_argument('--laps', type=int, default=None, help='바퀴 수 (기본 3, 대화형은 1)')
    ap.add_argument('--time', type=float, default=120.0, help='최대 시뮬 시간 s (한 회)')
    ap.add_argument('--delay', action='store_true', help='명령을 풀이 시간만큼 늦게 적용 (실차처럼)')
    ap.add_argument('--log', help='주기별 기록 csv')
    ap.add_argument('--plot', help='그림 경로 (기본 <저장소>/logs/tune/<시각>.png — 매번 저장)')
    ap.add_argument('--show', action='store_true', help='그림을 창으로 띄운다')
    args = ap.parse_args()
    if args.interactive:
        return interactive(args)
    params, track, ctrl, sim = setup(args)
    t_wall = time.perf_counter()
    ticks, rows = run(params, ctrl, sim, args.laps or 3, args.time, args.delay)
    rows = summary(sim, ticks, rows, t_wall, params, ctrl)
    if args.log:
        np.savetxt(args.log, rows, delimiter=',', fmt='%.5f', comments='', header=','.join(COLS))
        print(f'  기록 {args.log}')
    path = args.plot or plot_path()
    plot(track, rows, path, os.path.basename(track.path), ctrl.cfg.wall_margin, ticks, params, show=args.show)
    print(f'  그림 {path}')
    if args.show:
        input('  (그림 창 — 엔터로 끝)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
