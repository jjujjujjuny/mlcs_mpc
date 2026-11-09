#!/usr/bin/env python3
"""
offline — 차 없이 MPCC 를 풀어본다. 설정은 노드와 같은 config/mpcc.yaml (+ 덮어쓸 파일, 옵션).

  python3 src/mlcs_mpc/mlcs_mpc/env/mpc/offline.py                      직선·곡선·트랙 (status, 풀이 시간)
  python3 .../offline.py --scenario track --plant --track 1108_8_track
                                        같은 모델로 굴리는 진짜 폐루프 (노드 워커와 같은 흐름)

  ~/hmpc-venv (acados) 에서 돌린다. 옵션을 안 주면 전부 mpcc.yaml 값이다 (N, dt, 경계, 가중치, 차).

■ 시나리오 (--scenario) — 곡률은 트랙으로 생성 코드에 들어가므로 시나리오마다 따로 빌드된다 (한 번만)

    straight   곡률 0
    curve      곡률 0.575 /m 고정 (track_5 평균, R 1.74 m)
    track      --track 의 κ(s) 와 지점별 벽 (track_bounds walls 면)
    zero       슬립이 정확히 0 인 직진 초기값 (--no-guard 로 0/0 방어 없이 확인)

■ ★ 저속에서 RK4 한 번으로는 적분이 발산한다 (rk_steps)

  슬립률 분모가 v_x 라 타이어 강성이 v_x 에 반비례해 커진다. 기본 계수·마찰 0.41 이면 v_x 방향 고유값이
  약 157/v_x 1/s 로, dt 0.033 에서 RK4 안정 한계(2.78)를 v_x 약 1.9 m/s 아래에서 넘는다. 구간당 3회면
  0.25~1 m/s 모두 정상이었지만, 동정값은 뒤 타이어 Bx 가 커서 0.25 m/s 출발에서 4회가 필요하다 (2026-11-05).
  ★ 적분과 별개로, 타이어 Cx > 1 이면 종방향 힘이 정점 뒤 떨어져 v_x 고유값이 양수가 된다 (모델 자체가
  불안정). 그래서 mpcc.yaml 과 HyperPM 은 Cx ≤ 1 로 묶는다 (learned/hyperpm clamp_cx, 2026-11-09).

■ --plant 없으면 직전 해의 x[1] 을 다음 상태로 쓴다 (모델 오차 없음). SQP_RTI 는 한 번만 반복해서 그 값이
  동역학을 정확히 만족하지 않으므로, 진짜 폐루프는 --plant (같은 모델을 RK4 로 적분하는 '차').
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..', '..', '..'))       # src/mlcs_mpc
REPO = os.path.abspath(os.path.join(PKG, '..', '..'))
sys.path[:0] = [p for p in (PKG, REPO) if p not in sys.path]

from mlcs_mpc.env.config import MpccConfig  # noqa: E402
from mlcs_mpc.env.mpc.acados_ocp import kappa_function  # noqa: E402
from mlcs_mpc.env.mpc.acados_solver import make_solver  # noqa: E402
from mlcs_mpc.env.mpc.bounds import restart, set_hard_bounds, set_input_bounds, set_wall_bounds  # noqa: E402
from mlcs_mpc.env.track import Track, find as find_track  # noqa: E402
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402
from mlcs_mpc.env.mpc import param_model  # noqa: E402

DEFAULT_CFG = os.path.join(PKG, 'config', 'mpcc.yaml')


def constant_grid(kappa, length=1000.0):
    """곡률이 일정한 '트랙' (straight / curve 시나리오)."""
    S = np.linspace(0.0, length, 11)
    return S, np.full_like(S, kappa)


def make_plant(model, kappa_fn, dt, substeps=16, ws_tau=None, ws_acc=None, ws_dec=None):
    """OCP 와 같은 연속 모델을 RK4 로 dt 만큼 적분하는 '차'. → step(x, u, p) → x_next.

    곡률은 OCP 와 같은 kappa_fn(s). ws_tau / ws_acc / ws_dec 를 주면 '차' 의 바퀴만 실제처럼 바꾼다
    (MPC 는 그대로) — 실제 차의 느린 바퀴 응답에서 MPC 가 어떻게 되는지 (tools/analysis/actuator_id.py 값).
    """
    plant = model.copy(wheel_speed_tau=ws_tau) if ws_tau is not None else model
    f = plant.function('frenet')
    h = dt / substeps
    tau = plant.wheel_speed_tau

    def actuate(x, u):
        if ws_acc is None and ws_dec is None:
            return u
        rate = np.clip((u[0] - x[7]) / tau, -(ws_dec or np.inf), ws_acc or np.inf)
        return np.array([x[7] + rate * tau, u[1]])

    def rhs(x, u, p):
        return np.array(f(x, u, p, float(kappa_fn(x[0])))).ravel()

    def step(x, u0, p):
        x = np.asarray(x, float)
        for _ in range(substeps):
            u = actuate(x, u0)
            k1 = rhs(x, u, p)
            k2 = rhs(x + h / 2 * k1, u, p)
            k3 = rhs(x + h / 2 * k2, u, p)
            k4 = rhs(x + h * k3, u, p)
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x
    return step


def initial_state(friction, v=1.0, slip=0.02):
    """직진 v m/s. wheel_speed 는 선속도 [m/s] 다."""
    x0 = np.zeros(9)
    x0[3], x0[6], x0[7] = v, friction, v + slip
    return x0


def make_opponent(track, v, d, gap):
    """--opponent: 트랙 선에서 d 만큼 옆을 v 로 도는 가상 상대 + mpcc_node 와 같은 opponent 로직."""
    from mlcs_mpc.env.opponent import Opponent
    return dict(track=track, logic=Opponent(track), v=v, d=d, s=gap, modes={}, min_dist=np.inf,
                hits=0, passes=0, last_ds=None)


def _opp_cycle(o, i, dt, qs, x, lo, hi, v_max):
    """가상 상대를 dt 만큼 움직이고 통로·속도 상한을 낸다. 거리 기록도 여기서."""
    tr = o['track']
    t = i * dt
    for j in range(3):          # mocap 100 Hz 쯤
        so = o['s'] + o['v'] * dt * (j + 1) / 3
        ox, oy = tr.xy(so, o['d'])
        o['logic'].measure(t - dt + dt * (j + 1) / 3, float(ox), float(oy), tr.heading_at(so))
    o['s'] += o['v'] * dt
    s_now = np.r_[qs[1:], 2 * qs[-1] - qs[-2]] if i > 0 else qs     # node 와 같은 한 칸 당김
    lo, hi, cap, mode = o['logic'].step(t, s_now, x[1], lo, hi, v_max, len(qs) - 1, dt)
    o['modes'][mode] = o['modes'].get(mode, 0) + 1
    ex, ey = tr.xy(x[0], x[1])
    ox, oy = tr.xy(o['s'], o['d'])
    o['min_dist'] = min(o['min_dist'], float(np.hypot(ex - ox, ey - oy)))
    ds = (x[0] - o['s'] + tr.length / 2) % tr.length - tr.length / 2
    if abs(ds) < 0.55 and abs(x[1] - o['d']) < 0.30:   # 차 길이·폭으로 본 겹침
        o['hits'] += 1
    if o['last_ds'] is not None and o['last_ds'] < 0 <= ds and abs(ds) < 1.0:
        o['passes'] += 1
    o['last_ds'] = ds
    return lo, hi, cap


def run_scenario(solver, args, cfg, name, x0, laps, params, walls=None, plant=None,
                 plant_friction=None, opp=None, hyper=None, plant_hyper=None):
    """폐루프로 laps 번 푼다. walls(q) → (n 하한, n 상한) 이면 지점별 벽 경계 (MPCC). opp 면 가상 상대.

    hyper: MPC 가 스테이지별 파라미터를 학습 모델에서 받는다 (env/mpc/param_model, 노드 워커와 같다).
    plant_hyper: 차(plant) 가 학습 모델의 지금 파라미터로 움직인다. 없으면 차는 params (상수).
    """
    N, dt = cfg.N, cfg.dt
    P_used = []
    wall_hist, s_hist, trace = [], [], []
    restart(solver, N, x0, cfg)
    x = x0.copy()
    times, cycles, status, n_hist = [], [], [], []
    first_bad = None
    bad_run, trips, first_trip, need_restart = 0, 0, None, False
    for i in range(laps):
        if need_restart:        # 노드: 비정상 해 뒤 다음 요청은 cmd 1 (초기 추측을 지금 상태로)
            restart(solver, N, x, cfg)
            need_restart = False
        t_cycle = time.perf_counter()
        qs = np.array([x[0] + x[3] * k * dt if i == 0 else solver.get(k, 'x')[0] for k in range(N + 1)])
        t_now = i * dt
        P_k = np.tile(params, (N + 1, 1))
        if hyper is not None:
            hyper.observe(t_now, x)
            P_k, _ = hyper.stage_params(t_now, x)
            P_used.append(P_k[0] / params)
        for k in range(N + 1):
            solver.set(k, 'p', P_k[k])
        solver.set(0, 'lbx', x)
        solver.set(0, 'ubx', x)
        if cfg.track_hard and walls is not None:
            lo, hi = walls(qs)
            if opp is not None:
                lo, hi, cap = _opp_cycle(opp, i, dt, qs, x, lo, hi, cfg.v_max)
                set_input_bounds(solver, N, cfg, cap)
            set_wall_bounds(solver, N, lo, hi, x[1])
            lo0, hi0 = walls(x[0])
            wall_hist.append((x[1] - lo0 - 1e-9 < 0) or (x[1] - hi0 > 1e-9))
            s_hist.append(x[0])
        elif cfg.track_hard:
            set_hard_bounds(solver, N, cfg.track_width, x[1])
        t0 = time.perf_counter()
        st = solver.solve()
        t1 = time.perf_counter()
        u0 = solver.get(0, 'u')
        bad = st != 0 or not np.all(np.isfinite(u0))
        for hs in (hyper, plant_hyper):
            if hs is None:
                continue
            xs = np.array([solver.get(k, 'x') for k in range(N + 1)])
            if bad or not np.all(np.isfinite(xs)):
                hs.clear_plan()
            else:
                hs.set_plan(t_now, xs)
        if bad and not args.apply_bad:
            # mpcc_node 와 같게: 비정상 해는 버리고 정지 명령 (바퀴 0, 조향 0), 다음엔 다시 깐다.
            # 노드는 max_bad_solves(3) 번 이어지면 래치해 멈춘다 — 여기서는 세기만 하고 계속 돈다.
            u0 = np.array([0.0, 0.0])
            need_restart = True
            bad_run += 1
            if bad_run == 3:
                trips += 1
                first_trip = i if first_trip is None else first_trip
        else:
            bad_run = 0
        if plant is not None:
            xp = np.array(x, float)
            if plant_friction is not None:      # 차만 다른 마찰 (MPC 는 x0 의 값을 믿는다)
                xp[6] = plant_friction
            p_plant = params
            if plant_hyper is not None:
                plant_hyper.observe(t_now, x)
                p_plant = plant_hyper.stage_params(t_now, x)[0][0]
            x_next = plant(xp, u0, p_plant)
            x_next[6] = x[6]
        elif bad and not args.apply_bad:
            print(f'  ✗ {name}: {i} 번째 비정상 해 (status {st}) — --plant 없이는 버린 해의 다음 상태가 없어 멈춘다')
            status.append(st)
            break
        else:
            x_next = solver.get(1, 'x')
        cycles.append((time.perf_counter() - t_cycle) * 1000)
        times.append((t1 - t0) * 1000)
        status.append(st)
        finite = np.all(np.isfinite(x_next)) and np.all(np.isfinite(u0))
        if st != 0 and first_bad is None:
            first_bad = i
        if not finite:
            print(f'  ✗ {name}: {i} 번째 풀이에서 NaN/inf (status {st})')
            break
        x = x_next
        n_hist.append(abs(x[1]))
        trace.append(np.r_[x, u0])

    ts = np.array(times[1:]) if len(times) > 1 else np.array(times)
    codes, counts = np.unique(status, return_counts=True)
    budget = dt * 1000
    print(f'\n■ {name}  ({len(times)}회, 첫 풀이 {times[0]:.1f} ms 는 통계에서 뺌)')
    print(f'  status      {dict(zip(codes.tolist(), counts.tolist()))}'
          + (f'  첫 비정상 {first_bad} 번째' if first_bad is not None else ''))
    if first_bad is not None and not args.apply_bad:
        print(f'  비정상 해는 버리고 정지 명령 (노드와 같게). 3번 연속 {trips}회'
              + (f' — ★ 실차라면 {first_trip} 번째에 래치해 멈춘다' if trips else ''))
    print(f'  풀이 시간   평균 {ts.mean():7.2f}  p95 {np.percentile(ts, 95):7.2f}  '
          f'최대 {ts.max():7.2f} ms   (예산 {budget:.1f} ms, 초과 {(ts > budget).mean() * 100:.1f}%)')
    print(f'  주기 전체   평균 {np.mean(cycles[1:] or cycles):7.2f} ms')
    print(f'  |n| 최대 {max(n_hist or [0]):.3f} m' + (f'  (경계 {cfg.track_width:.2f})' if cfg.track_hard and walls is None else ''))
    if wall_hist:
        sh = np.array(s_hist)
        print(f'  벽 경계 밖 {np.mean(wall_hist) * 100:.1f}%  진행 {sh[-1] - sh[0]:.2f} m '
              f'({(sh[-1] - sh[0]) / (len(sh) * dt):.2f} m/s 평균)')
        if getattr(args, 'trace', None):
            np.save(args.trace, np.array(trace))
    print(f'  마지막 상태 vx {x[3]:.3f}  vy {x[4]:.3f}  r {x[5]:.3f}  n {x[1]:.3f}  '
          f'ws {x[7]:.3f}  delta {x[8]:.3f}')
    if P_used:
        R_ = np.array(P_used)
        print(f'  학습 모델 파라미터/기준 (스테이지 0) 최소 ×{R_.min():.2f} 최대 ×{R_.max():.2f}, '
              f'주기간 변화 p99 {np.percentile(np.abs(np.diff(R_, axis=0)), 99) * 100:.2f}%')
    if opp is not None:
        print(f'  상대 {opp["v"]:.2f} m/s, d {opp["d"]:+.2f}: 추월 {opp["passes"]}번, '
              f'최소 중심 거리 {opp["min_dist"]:.2f} m, 겹침(충돌) {opp["hits"]} 주기, 모드 {opp["modes"]}')


def main():
    ap = argparse.ArgumentParser(description='설정은 --config (기본 mpcc.yaml). 아래 옵션은 그 값을 덮어쓴다.')
    ap.add_argument('--config', nargs='+', default=[DEFAULT_CFG],
                    help='mpcc.yaml 형식 파일들 (뒤 파일이 앞을 덮는다). 차 값도 여기서')
    # 설정 덮어쓰기 (없으면 config 값)
    ap.add_argument('--N', type=int)
    ap.add_argument('--dt', type=float)
    ap.add_argument('--rk-steps', type=int)
    ap.add_argument('--cost', choices=('mpcc', 'tracking'))
    ap.add_argument('--v-max', type=float)
    ap.add_argument('--track-hard', choices=('true', 'false'))
    ap.add_argument('--track-width', type=float)
    ap.add_argument('--track-slack', type=float)
    ap.add_argument('--wall-margin', type=float)
    ap.add_argument('--ws-acc-max', type=float)
    ap.add_argument('--ws-dec-max', type=float)
    ap.add_argument('--friction', type=float, help='MPC 가 믿는 마찰 (상태 x[6])')
    ap.add_argument('--steering-tau', type=float)
    ap.add_argument('--wheel-speed-tau', type=float)
    # 시험
    ap.add_argument('--laps', type=int, default=1000, help='시나리오별 풀이 횟수')
    ap.add_argument('--scenario', nargs='+', choices=('straight', 'curve', 'track', 'zero'),
                    default=('straight', 'curve', 'track'))
    ap.add_argument('--track', default='s_track', help='트랙 이름 (env/track/data/) 또는 prep csv')
    ap.add_argument('--start-v', type=float, default=1.0, help='초기 v_x [m/s]')
    ap.add_argument('--no-guard', action='store_true', help='0/0 방어를 끈다')
    ap.add_argument('--plant', action='store_true', help='다음 상태를 같은 모델의 RK4 적분으로 (진짜 폐루프)')
    ap.add_argument('--plant-friction', type=float, help='--plant: 차의 실제 마찰. 없으면 MPC 와 같음')
    ap.add_argument('--plant-ws-tau', type=float, help='--plant: 차의 바퀴 시정수 (실제처럼)')
    ap.add_argument('--plant-ws-acc', type=float, help='--plant: 차의 바퀴 가속도 한계')
    ap.add_argument('--plant-ws-dec', type=float, help='--plant: 차의 바퀴 감속도 한계')
    ap.add_argument('--param-model', help="MPC 가 학습 모델 파라미터를 쓴다 (예 'hyperpm:config/hyper/J_ep10.npz')")
    ap.add_argument('--plant-param-model', help='--plant: 차가 학습 모델 파라미터로 움직인다')
    ap.add_argument('--opponent', type=float, metavar='V', help='track: 속도 V 로 도는 가상 상대')
    ap.add_argument('--opp-d', type=float, default=0.0, help='가상 상대의 선에서 횡위치 m (왼쪽 +)')
    ap.add_argument('--opp-gap', type=float, default=2.0, help='가상 상대가 앞서 출발하는 거리 m')
    ap.add_argument('--apply-bad', action='store_true', help='status≠0 해도 그대로 적용 (예전 동작)')
    ap.add_argument('--trace', help='track: 상태·입력 기록 .npy')
    ap.add_argument('--build-only', action='store_true', help='빌드만 하고 끝낸다')
    ap.add_argument('--build-dir', default=os.path.expanduser('~/mpcc_build'))
    args = ap.parse_args()

    over = {k: v for k, v in dict(N=args.N, dt=args.dt, rk_steps=args.rk_steps, cost=args.cost,
                                   v_max=args.v_max, track_width=args.track_width,
                                   track_slack=args.track_slack, wall_margin=args.wall_margin,
                                   ws_acc_max=args.ws_acc_max, ws_dec_max=args.ws_dec_max).items()
            if v is not None}
    if args.track_hard is not None:
        over['track_hard'] = args.track_hard == 'true'
    cfg = MpccConfig.from_yaml(*args.config, **over)
    mo = {k: v for k, v in dict(friction=args.friction, steering_tau=args.steering_tau,
                                 wheel_speed_tau=args.wheel_speed_tau).items() if v is not None}
    if args.no_guard:
        mo['slip_eps'] = 0.0
    model = VehicleModel.from_yaml(*args.config, **mo)
    params = model.p.copy()

    print(f'\n■ MPCC  N={cfg.N}  dt={cfg.dt}  예측구간 {cfg.N * cfg.dt:.2f}초  비용 {cfg.cost}  '
          f'RK4 {cfg.rk_steps}회  v_max {cfg.v_max}  경계 {"하드" if cfg.track_hard else "배리어"} '
          f'({cfg.track_bounds})')
    print(f'  차  질량 {model.mass}  마찰 {model.friction}  시정수 조향 {model.steering_tau:.3f} '
          f'바퀴 {model.wheel_speed_tau:.3f}  ({", ".join(os.path.basename(c) for c in args.config)})')

    track = None
    for name in args.scenario:
        if name == 'track':
            track = track or Track(find_track(args.track))
            grid = track.kappa_grid()
        else:
            grid = constant_grid(0.575 if name == 'curve' else 0.0)
        solver = make_solver(model, grid, cfg, args.build_dir)
        if args.build_only:
            continue
        set_input_bounds(solver, cfg.N, cfg)
        x0 = initial_state(model.friction, args.start_v, slip=0.0 if name == 'zero' else 0.02)
        walls = None
        if name == 'track' and cfg.track_bounds == 'walls':
            walls = lambda q: track.bounds_at(q, cfg.wall_margin)  # noqa: E731
        kfn = kappa_function(*grid)
        plant = (make_plant(model, kfn, cfg.dt, ws_tau=args.plant_ws_tau,
                            ws_acc=args.plant_ws_acc, ws_dec=args.plant_ws_dec) if args.plant else None)
        opp = (make_opponent(track, args.opponent, args.opp_d, args.opp_gap)
               if (walls is not None and args.opponent is not None) else None)
        hyper = (param_model.load(args.param_model, cfg.N, cfg.dt, params, root=REPO)
                 if args.param_model else None)
        plant_hyper = (param_model.load(args.plant_param_model, cfg.N, cfg.dt, params, root=REPO)
                       if args.plant_param_model else None)
        run_scenario(solver, args, cfg, name, x0, args.laps, params, walls, plant,
                     plant_friction=args.plant_friction, opp=opp, hyper=hyper, plant_hyper=plant_hyper)
    return 0


if __name__ == '__main__':
    sys.exit(main())
