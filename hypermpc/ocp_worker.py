#!/usr/bin/env python3
"""
ocp_worker.py — HyperMPC OCP 를 풀기만 하는 프로세스. hmpc_node 가 띄운다.

■ 왜 프로세스를 나누나

  슬립이 0 이거나 NaN 이 들어가면 acados solve() 는 돌아오지 않는다 (2026-11-05 실측).
  C 안에서 멈춘 호출은 파이썬 스레드로 끊을 수 없다. 그런데 VESC 는 새 명령이 없으면
  마지막 속도를 유지하므로, 멈춘 쪽과 정지 명령을 보내는 쪽이 같은 프로세스면 위험하다.
  그래서 노드는 여기서 답이 안 오면 이 프로세스를 SIGKILL 하고 차를 세운다.

  casadi·acados 는 ~/hmpc-venv 에만 있어서 환경을 나누는 이점도 있다.

■ 통신 (전용 파이프, double 고정 길이)

  acados 가 C 에서 stdout 에 직접 쓰므로 stdout 은 통신에 쓸 수 없다.

    시작  → b'R' 한 바이트
    요청  [id, cmd, t, x(9), kappa(N+1), n_lo(N+1), n_hi(N+1), v_cap(N)]   cmd 1 이면 초기 추측을 x 로 다시 깐다
          t: 노드 시각 [s]. HyperPM 관측 기록과 직전 해의 시각 맞춤에만 쓴다
          n_lo/n_hi: 스테이지별 벽 경계 (MPCC, 상대 차 통로 포함). track_hard 일 때만 쓴다
          v_cap: 스테이지 0..N-1 의 바퀴 속도 명령 상한 (상대 뒤따르기). v_max 보다 크면 v_max
    응답  [id, status, ms, u(2), x1(9), s(N+1)]

  파이프가 닫히면 (노드가 죽으면) 끝난다.

■ 재시작 다시 풀기

  cmd 1 은 매번 x 를 전 구간에 깐 같은 추측에서 RTI 한 번으로 푼다. 그래서 한 번 깨지면 노드가 보내는
  다음 재시작도 똑같이 깨지고, max_bad_solves 번이면 솔버 고장으로 래치된다 (2026-11-09 젯슨 노드 시험:
  새 H 에서 정지 뒤 재출발이 줄줄이 실패). 재시작이 깨지면 RESTART_BUDGET 안에서 조금 빠른 속도로 깐
  추측으로 다시 푼다 (응답 ms 는 전부 합친 값, 노드 restart_timeout 0.5 s 안).

■ HyperPM (설정 hyper = .npz 경로, 비우면 끔)

  매 요청마다 지금 x 를 관측 기록에 넣고, 직전 해의 바퀴 속도·조향 궤적과 함께 네트워크로
  스테이지별 동역학 파라미터 (N+1, 30) 를 만든다 (hyper_params.py). 추론은 풀이 시간(ms)에
  포함하지 않지만 노드의 제한 시간 안에는 들어간다. 출력이 유한하지 않으면 상수 params 를 쓴다.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import offline_ocp as oo  # noqa: E402
from hyper_params import HyperStage  # noqa: E402

RESTART_WS = (0.3, 0.6)     # 재시작이 깨지면 vx·ws 를 현재 + 이만큼으로 깔아 다시 푼다
RESTART_BUDGET = 0.25       # [s] 다시 풀기는 재시작 요청 시작부터 이 시간 안에서만


def read_exact(fd, n):
    buf = b''
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


# 통계 CSV 에 남길 예측 스테이지 (dt 0.033 이면 0.33 / 0.66 / 1.0 s 뒤). 젯슨 11-09: 벽에 닿기 전
# 예측 n 과 실제 n 을 비교하려고. s 는 풀이의 s 그대로 (노드의 앵커 보정 전)
STATS_STAGES = (0, 10, 20, 30)
# 지연 보상 (2026-11-10): 요청 끝에 [켬, 상태 측정 시각, 목표 시각, (시각, ws_ref, δ) × DELAY_HIST] 가 붙는다.
#   켜져 있으면 풀기 직전에 그 명령들로 모델을 적분해 x0 를 측정 시각 → 목표 시각 (지금 + 예상 풀이) 로 당긴다.
DELAY_HIST = 8
DELAY_MAX = 0.20           # 이보다 긴 지연은 이만큼만 (상태가 아주 오래됐으면 노드가 이미 멈춘다)


def solve_stat(solver, key):
    """acados 통계 하나. qp_iter 는 SQP 반복별 배열이라 합한다. 못 읽으면 -1."""
    try:
        v = np.atleast_1d(solver.get_stats(key))
        return int(np.sum(v))
    except Exception:
        return -1


def slack_n_max(solver, N, cfg):
    """n 경계 슬랙의 최대값 m (경계를 얼마나 넘는 해인가). 슬랙이 없으면 0, 못 읽으면 -1."""
    if not (cfg.get('track_hard') and cfg.get('track_slack', 0) > 0):
        return 0.0
    try:
        if cfg.get('bounds_param', False):
            # 슬랙 순서 [g (ws-rate, 있으면), h]. h 의 첫 두 줄이 n − lo ≥ 0, hi − n ≥ 0 (하한 슬랙)
            g = 1 if (cfg.get('ws_acc_max', 0) > 0 or cfg.get('ws_dec_max', 0) > 0) else 0
            vals = []
            for k in range(1, N + 1):
                sl = solver.get(k, 'sl')
                i0 = 0 if k == N else g             # 터미널은 h 만
                vals.append(max(sl[i0], sl[i0 + 1]))
            return float(max(vals))
        # 상자 경계: 첫 슬랙 칸이 n 경계다 (idxsbx = [0], 그 뒤에 ws-rate 슬랙)
        return float(max(max(solver.get(k, 'sl')[0], solver.get(k, 'su')[0]) for k in range(1, N + 1)))
    except Exception:
        return -1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fd-in', type=int, required=True)
    ap.add_argument('--fd-out', type=int, required=True)
    ap.add_argument('--config', required=True, help='JSON: N dt rk_steps steering_tau wheel_speed_tau mass v_max max_steer params core')
    ap.add_argument('--hypermpc', default=os.path.expanduser('~/hypermpc_code'))
    ap.add_argument('--build-dir', default=os.path.expanduser('~/hmpc_build'))
    ap.add_argument('--no-guard', action='store_true', help='시험 전용. 슬립 0 에서 solve 가 멈춘다')
    ap.add_argument('--stats-csv', default='/tmp/mlcs_hmpc_solve_stats.csv',
                    help='주기별 풀이 통계 (t, status, ms, sqp_iter, qp_iter, 재시작 풀이 수, n 슬랙 최대, '
                         '스테이지 0/10/20/30 의 예측 s·n). 빈 값이면 끔')
    a = ap.parse_args()
    c = json.loads(a.config)

    if abs(c['mass'] - oo.MASS) > 1e-6:
        # 질량은 생성 코드에 들어 있어 파라미터로 못 바꾼다.
        print(f'✗ 질량 불일치: 설정 {c["mass"]} / offline_ocp.MASS {oo.MASS}', flush=True)
        return 2
    if c.get('core', -1) >= 0:
        os.sched_setaffinity(0, {c['core']})

    N = c['N']
    cfg = oo.make_cfg('author', c['rk_steps'], c['steering_tau'], c['wheel_speed_tau'],
                      track_hard=c.get('track_hard', False),
                      track_width=c.get('track_width', 0.25),
                      track_slack=c.get('track_slack', 0.0),
                      ws_acc_max=c.get('ws_acc_max', 0.0),
                      ws_dec_max=c.get('ws_dec_max', 0.0),
                      wall_heading=c.get('wall_heading', 0.0),
                      wall_heading_r=c.get('wall_heading_r', 0.25),
                      bounds_param=c.get('bounds_param', False))
    cfg['q_delta'] = float(c.get('q_delta', cfg['q_delta']))     # 조향 명령−상태 비용 (생성 코드에 들어감)
    # ★ 마찰은 상태 x[6] 로 들어간다 (생성 코드 밖). 예전에는 노드 값을 안 받고 make_cfg 기본값으로
    #   덮어써서 hmpc.yaml 의 friction 이 적용되지 않았다 (2026-11-08 발견).
    fric = float(c.get('friction', cfg['friction']))
    print(f'friction {fric}', flush=True)
    dyn_fun, _ = oo.load_naive_dynamics(a.hypermpc, guard=not a.no_guard)
    solver = oo.make_solver(dyn_fun, N, c['dt'], cfg, a.build_dir, guard=not a.no_guard)
    params = np.array(c['params'], dtype=float)
    assert params.size == 30
    hyper = None
    if c.get('hyper'):
        hyper = HyperStage(c['hyper'], N, c['dt'], params, v_min=float(c.get('hyper_v_min', 0.0)),
                           blend=float(c.get('hyper_blend', 0.0)))
        print(f'HyperPM {c["hyper"]} (v_min {hyper.v_min}, blend {hyper.blend})', flush=True)
    n_hyper, a_sum, ms_hyper = 0, 0.0, 0.0
    lbu = np.array([cfg['min_ws_ref'], -c['max_steer']])
    ubu = np.array([c['v_max'], c['max_steer']])
    for k in range(N):
        solver.constraints_set(k, 'lbu', lbu)
        solver.constraints_set(k, 'ubu', ubu)

    def restart(x0, dv=0.0):
        solver.reset()
        xg = x0.copy()
        xg[3] += dv
        xg[7] += dv
        for k in range(N + 1):
            solver.set(k, 'x', xg)
        for k in range(N):
            solver.set(k, 'u', np.array([np.clip(xg[7], lbu[0], ubu[0]), 0.0]))
        for k in range(N):      # reset 이 경계까지 지우지는 않지만 다시 넣어 둔다
            solver.constraints_set(k, 'lbu', lbu)
            solver.constraints_set(k, 'ubu', ubu)

    # 첫 풀이는 메모리를 처음 건드려 느리다. 감시 제한에 걸리지 않게 준비 전에 한 번 푼다.
    # 슬립을 0 이 아니게 둔다.
    x_dry = np.zeros(9)
    x_dry[3], x_dry[6], x_dry[7] = 1.0, fric, 1.02
    for _ in range(3):
        restart(x_dry)
        w = np.full(N + 1, cfg['track_width'])
        B0 = oo.bound_params(N, -w, w, 0.0, 0.0, solver.heading) if solver.bounds_param else np.zeros((N + 1, 0))
        for k in range(N + 1):
            solver.set(k, 'p', np.concatenate([params, [0.0], B0[k]]))
        solver.set(0, 'lbx', x_dry)
        solver.set(0, 'ubx', x_dry)
        if cfg['track_hard'] and not solver.bounds_param:
            oo.set_hard_bounds(solver, N, cfg['track_width'], 0.0)
        solver.solve()

    req_len = 8 * (3 + 9 + 3 * (N + 1) + N + 3 + 3 * DELAY_HIST)
    # 예측 간격 33 ms × RK4 2번 — 80 ms 면 모델 평가 약 30번 (11 ms × 4 번이면 200 ms 에 주기를 넘겼다)
    pred_dt = c['dt']
    predictor = oo.make_plant(dyn_fun, cfg, pred_dt, substeps=2)     # MPC 가 믿는 모델 그대로

    def predict(x, t0, t1, hist, kap):
        """x (t0 의 상태) 를 hist 의 명령 (시각부터 다음 시각까지 유지) 으로 t1 까지 적분."""
        h = hist[np.isfinite(hist[:, 0])]
        h = h[np.argsort(h[:, 0])]
        t, xx = t0, np.array(x, float)
        p_pred = np.concatenate([params, [kap]])
        while t < t1 - 1e-6:
            j = np.searchsorted(h[:, 0], t, side='right') - 1 if len(h) else -1
            u = h[max(j, 0), 1:3] if len(h) else xx[7:9]
            step = min(pred_dt, t1 - t)
            xn = predictor(xx, u, p_pred)
            xx = xn if step >= pred_dt - 1e-9 else xx + step / pred_dt * (xn - xx)
            t += step
        return xx
    ubu_k = np.full(N, np.inf)      # 지금 스테이지별로 걸려 있는 속도 상한 (바뀔 때만 다시 넣는다)
    os.write(a.fd_out, b'R')
    stats = None
    if a.stats_csv:
        header = ('t,status,ms,sqp_iter,qp_iter,resolves,slack_n_max,'
                  + ','.join(f'{c}{k}' for k in STATS_STAGES for c in ('s', 'n')) + ',delay_ms\n')
        if os.path.exists(a.stats_csv):
            with open(a.stats_csv) as f:
                old = f.readline()
            if old != header:       # 열이 바뀐 예전 파일에 이어 쓰면 열 수가 어긋난다 — 옆으로 치운다
                os.replace(a.stats_csv, a.stats_csv + '.old')
        new = not os.path.exists(a.stats_csv)
        stats = open(a.stats_csv, 'a', buffering=1)     # 워커가 다시 떠도 앞 기록을 지우지 않는다
        if new:
            stats.write(header)
    while True:
        raw = read_exact(a.fd_in, req_len)
        if raw is None:
            return 0
        req = np.frombuffer(raw, dtype=np.float64)
        rid, cmd, t_req, x = req[0], req[1], req[2], req[3:12]
        kappa, n_lo, n_hi = req[12:12 + 3 * (N + 1)].reshape(3, N + 1)
        base = 12 + 3 * (N + 1)
        v_cap = np.clip(req[base:base + N], lbu[0], ubu[0])
        dc_on, t_meas, t_tgt = req[base + N:base + N + 3]
        hist = req[base + N + 3:].reshape(DELAY_HIST, 3)
        x = x.copy()
        x[6] = fric
        d_pred = 0.0
        if dc_on > 0 and np.isfinite(t_meas) and t_tgt > t_meas:
            d_pred = min(t_tgt - t_meas, DELAY_MAX)
            v_floor = min(x[3], x[7])       # 노드가 v_floor 로 올려 둔 값 — 적분 뒤에도 지킨다
            x = predict(x, t_tgt - d_pred, t_tgt, hist, kappa[0])
            x[6] = fric
            x[3], x[7] = max(x[3], v_floor), max(x[7], v_floor)
        if cmd == 1.0:
            restart(x)
            ubu_k[:] = ubu[0]
            if hyper is not None:
                hyper.plan = None       # 다시 깐 추측은 계획이 아니다. 관측 기록은 둔다
        for k in np.flatnonzero(v_cap != ubu_k):
            solver.constraints_set(int(k), 'ubu', np.array([v_cap[k], ubu[1]]))
            ubu_k[k] = v_cap[k]
        if hyper is not None:
            th = time.perf_counter()
            hyper.observe(t_req, x)
            P_k, a = hyper.stage_params(t_req, x)
            ms_hyper += (time.perf_counter() - th) * 1000
            n_hyper += 1
            a_sum += a
            if n_hyper % 300 == 0:
                print(f'HyperPM 300회: 평균 {ms_hyper / 300:.2f} ms, 쓴 비율 {a_sum / 300:.2f}', flush=True)
                ms_hyper, a_sum = 0.0, 0.0
        # 경계: bounds_param 이면 p 에 같이 (constraints_set 120~240 번 생략, 주기당 ~5 ms), 아니면 따로
        B = (oo.bound_params(N, n_lo, n_hi, x[1], x[2], solver.heading) if solver.bounds_param
             else np.zeros((N + 1, 0)))
        if hyper is None:
            P_k = np.broadcast_to(params, (N + 1, len(params)))
        for k in range(N + 1):
            solver.set(k, 'p', np.concatenate([P_k[k], [kappa[k]], B[k]]))
        solver.set(0, 'lbx', x)
        solver.set(0, 'ubx', x)
        if cfg['track_hard'] and not solver.bounds_param:
            oo.set_wall_bounds(solver, N, n_lo, n_hi, x[1], x[2])
        t0 = time.perf_counter()
        st = solver.solve()
        xs = np.array([solver.get(k, 'x') for k in range(N + 1)])
        n_resolve = 0
        if cmd == 1.0:
            for dv in RESTART_WS:
                if (st == 0 and np.all(np.isfinite(xs))) or time.perf_counter() - t0 > RESTART_BUDGET:
                    break
                st0 = st
                restart(x, dv)
                solver.set(0, 'lbx', x)     # 경계·파라미터는 reset 이 지우지 않지만 첫 스테이지는 다시 둔다
                solver.set(0, 'ubx', x)
                st = solver.solve()
                n_resolve += 1
                xs = np.array([solver.get(k, 'x') for k in range(N + 1)])
                print(f'재시작 다시 풀기: status {int(st0)} → 추측 +{dv} m/s 로 status {int(st)}', flush=True)
        ms = (time.perf_counter() - t0) * 1000
        s_pred = xs[:, 0]
        if hyper is not None:
            if st == 0 and np.all(np.isfinite(xs)):
                hyper.set_plan(t_req, xs)
            else:
                hyper.plan = None
        res = np.concatenate([[rid, st, ms], solver.get(0, 'u'), solver.get(1, 'x'), s_pred])
        os.write(a.fd_out, res.tobytes())
        if stats is not None:
            # 응답을 보낸 뒤에 쓴다 (노드를 기다리게 하지 않는다). 젯슨 11-09: 느린 풀이를 반복 수로 가르려고
            stats.write(f'{t_req:.4f},{int(st)},{ms:.2f},{solve_stat(solver, "sqp_iter")},'
                        f'{solve_stat(solver, "qp_iter")},{n_resolve},{slack_n_max(solver, N, cfg):.4f},'
                        + ','.join(f'{xs[min(k, N), 0]:.4f},{xs[min(k, N), 1]:.4f}' for k in STATS_STAGES)
                        + f',{d_pred * 1000:.1f}\n')


if __name__ == '__main__':
    sys.exit(main())
