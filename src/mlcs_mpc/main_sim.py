#!/usr/bin/env python3
"""
main_sim.py — MPCC 튜닝을 눈으로 본다. 한 프로세스에서 실시간으로 제어하며 RViz 에 그린다 (race_mpc main.py 역할).

  python3 src/mlcs_mpc/main_sim.py                     race.yaml 의 track·mpcc 설정으로 (ROS 2·acados 환경은 알아서 잡는다)
  python3 src/mlcs_mpc/main_sim.py --track s_track --set v_max=1.2 q_n=0.05
  python3 src/mlcs_mpc/main_sim.py --car wheel_speed_tau=0.66      차만 다르게 (모델 불일치)
  python3 src/mlcs_mpc/main_sim.py --speed 2.0 --laps 5            2배속, 5바퀴 뒤 끝

  초기화  env/racing.setup()  config/mpcc.yaml (+ race.yaml mpcc:, --config, --set) → 트랙 → 차 모델 → MPC → 시뮬레이터
  루프    (이 파일)           control_rate (30 Hz) 마다 MPC 풀이, 그 사이 100 Hz 로 ODE 적분, 벽시계에 맞춘다
  출력    utils/sim_rviz.py      RViz (트랙·차·예측·자취·글), '2D Pose Estimate' 로 다시 세우기
  끝      utils/sim_report.py  Ctrl+C 또는 --laps → 결과 정리 + 그림 (<저장소>/logs/tune/)

  빠른 반복·대화형 튜닝은 main.py (실시간 아님), 실제 토픽·타이밍·감시까지 보려면 ./sim.sh (ROS 노드).
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mlcs_mpc.utils.runenv import ensure_acados, ensure_ros2  # noqa: E402

ensure_ros2()
ensure_acados()

from main import add_args  # noqa: E402
from mlcs_mpc.env.racing import control_tick, setup  # noqa: E402
from mlcs_mpc.utils.sim_report import COLS, plot, plot_path, summary  # noqa: E402
from mlcs_mpc.utils.sim_rviz import SimRviz  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description='MPCC 실시간 시뮬 + RViz')
    add_args(ap)
    ap.add_argument('--laps', type=int, default=0, help='이만큼 돌면 끝 (0 = Ctrl+C 까지)')
    ap.add_argument('--speed', type=float, default=1.0, help='실시간 배속 (1 = 실시간)')
    ap.add_argument('--delay', action='store_true', help='명령을 풀이 시간만큼 늦게 적용 (실차처럼)')
    ap.add_argument('--trail', type=float, default=0.0, help='자취 길이 s (0 = 처음부터 전부)')
    ap.add_argument('--no-rviz', action='store_true', help='RViz 를 띄우지 않는다 (이미 떠 있을 때)')
    ap.add_argument('--log', help='주기별 기록 csv')
    ap.add_argument('--plot', help='끝낼 때 그림 경로 (기본 <저장소>/logs/tune/<시각>_sim.png)')
    args = ap.parse_args()

    params, track, ctrl, sim = setup(args)
    period = 1.0 / float(params['control_rate'])
    viz = SimRviz(track, ctrl.cfg.wall_margin, sim.model.lr, 2 * sim.half_width,
                  trail=args.trail, period=period, rviz=not args.no_rviz)
    print('  RViz: 트랙 면·벽(흰)·기준 라인(초록)·MPC 경계(빨강)·출발선(노랑), 차, 예측(주황), 자취(노랑). '
          '2D Pose Estimate = 다시 세우기, Ctrl+C = 끝')

    t_next, cmd, pending = 0.0, (0.0, 0.0), None
    ticks, rows, laps_seen = [], [], 0
    t_wall, wall0 = time.perf_counter(), time.monotonic()
    try:
        while viz.ok() and (args.laps <= 0 or len(sim.laps) < args.laps):
            for x, y, yaw in viz.spin():                     # RViz 로 다시 세우기
                sim.reset(x, y, yaw)
                ctrl.reset()
                viz.clear_trail()
                cmd, pending = (0.0, 0.0), None
                print(f'  다시 세움 ({x:+.2f}, {y:+.2f})')
            control = sim.t + 1e-9 >= t_next
            if control:                                      # MPC 한 주기
                c, tick = control_tick(ctrl, sim)
                ticks.append(tick)
                if args.delay:
                    pending = (sim.t + tick['ms'] / 1000.0, c)
                else:
                    cmd = c
                t_next += period
            if pending is not None and sim.t + 1e-9 >= pending[0]:
                cmd, pending = pending[1], None
            info = sim.step(*cmd)                            # 차 (ODE) 100 Hz
            x, y, yaw = sim.pose()
            rows.append((sim.t, x, y, yaw, info['vx'], info['vy'], info['r'], info['ws'], info['delta'],
                         cmd[0], cmd[1], info['s'], info['n'], int(info['on_wall'])))
            if info['new_hit']:
                print(f'  ⚠ {sim.t:6.2f} s 벽 — s {info["s"]:.2f} n {info["n"]:+.3f}')
            if len(sim.laps) > laps_seen:
                laps_seen = len(sim.laps)
                print(f'  랩 {laps_seen}: {sim.laps[-1]:.2f} s  (벽 누적 {sim.wall_hits}회)')
            if control:                                      # 그리기 (제어 주기마다)
                xs = ctrl.mpc.last_xs if tick['status'] == 0 else None
                viz.draw(sim, info, tick, track.xy(xs[:, 0], xs[:, 1]) if xs is not None else None)
            ahead = sim.t / args.speed - (time.monotonic() - wall0)   # 벽시계에 맞춘다
            if ahead > 0:
                time.sleep(ahead)
    except KeyboardInterrupt:
        pass
    finally:
        rows = summary(sim, ticks, rows, t_wall, params, ctrl)
        if args.log:
            np.savetxt(args.log, rows, delimiter=',', fmt='%.5f', comments='', header=','.join(COLS))
            print(f'  기록 {args.log}')
        if len(rows):
            path = args.plot or plot_path('sim')
            plot(track, rows, path, os.path.basename(track.path), ctrl.cfg.wall_margin, ticks, params)
            print(f'  그림 {path}')
        viz.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
