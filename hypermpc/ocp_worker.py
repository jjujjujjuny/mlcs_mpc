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
    요청  [id, cmd, x(9), kappa(N+1), n_lo(N+1), n_hi(N+1)]   cmd 1 이면 초기 추측을 x 로 다시 깐다
          n_lo/n_hi: 스테이지별 벽 경계 (MPCC). track_hard 일 때만 쓴다
    응답  [id, status, ms, u(2), x1(9), s(N+1)]

  파이프가 닫히면 (노드가 죽으면) 끝난다.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import offline_ocp as oo  # noqa: E402


def read_exact(fd, n):
    buf = b''
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fd-in', type=int, required=True)
    ap.add_argument('--fd-out', type=int, required=True)
    ap.add_argument('--config', required=True, help='JSON: N dt rk_steps steering_tau wheel_speed_tau mass v_max max_steer params core')
    ap.add_argument('--hypermpc', default=os.path.expanduser('~/hypermpc_code'))
    ap.add_argument('--build-dir', default=os.path.expanduser('~/hmpc_build'))
    ap.add_argument('--no-guard', action='store_true', help='시험 전용. 슬립 0 에서 solve 가 멈춘다')
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
                      track_slack=c.get('track_slack', 0.0))
    # ★ 마찰은 상태 x[6] 로 들어간다 (생성 코드 밖). 예전에는 노드 값을 안 받고 make_cfg 기본값으로
    #   덮어써서 hmpc.yaml 의 friction 이 적용되지 않았다 (2026-11-08 발견).
    fric = float(c.get('friction', cfg['friction']))
    print(f'friction {fric}', flush=True)
    dyn_fun, _ = oo.load_naive_dynamics(a.hypermpc, guard=not a.no_guard)
    solver = oo.make_solver(dyn_fun, N, c['dt'], cfg, a.build_dir, guard=not a.no_guard)
    params = np.array(c['params'], dtype=float)
    assert params.size == 30
    lbu = np.array([cfg['min_ws_ref'], -c['max_steer']])
    ubu = np.array([c['v_max'], c['max_steer']])
    for k in range(N):
        solver.constraints_set(k, 'lbu', lbu)
        solver.constraints_set(k, 'ubu', ubu)

    def restart(x0):
        solver.reset()
        for k in range(N + 1):
            solver.set(k, 'x', x0)
        for k in range(N):
            solver.set(k, 'u', np.array([np.clip(x0[7], lbu[0], ubu[0]), 0.0]))
        for k in range(N):      # reset 이 경계까지 지우지는 않지만 다시 넣어 둔다
            solver.constraints_set(k, 'lbu', lbu)
            solver.constraints_set(k, 'ubu', ubu)

    # 첫 풀이는 메모리를 처음 건드려 느리다. 감시 제한에 걸리지 않게 준비 전에 한 번 푼다.
    # 슬립을 0 이 아니게 둔다.
    x_dry = np.zeros(9)
    x_dry[3], x_dry[6], x_dry[7] = 1.0, fric, 1.02
    for _ in range(3):
        restart(x_dry)
        for k in range(N + 1):
            solver.set(k, 'p', np.concatenate([params, [0.0]]))
        solver.set(0, 'lbx', x_dry)
        solver.set(0, 'ubx', x_dry)
        if cfg['track_hard']:
            oo.set_hard_bounds(solver, N, cfg['track_width'], 0.0)
        solver.solve()

    req_len = 8 * (2 + 9 + 3 * (N + 1))
    os.write(a.fd_out, b'R')
    while True:
        raw = read_exact(a.fd_in, req_len)
        if raw is None:
            return 0
        req = np.frombuffer(raw, dtype=np.float64)
        rid, cmd, x = req[0], req[1], req[2:11]
        kappa, n_lo, n_hi = req[11:].reshape(3, N + 1)
        x = x.copy()
        x[6] = fric
        if cmd == 1.0:
            restart(x)
        for k in range(N + 1):
            solver.set(k, 'p', np.concatenate([params, [kappa[k]]]))
        solver.set(0, 'lbx', x)
        solver.set(0, 'ubx', x)
        if cfg['track_hard']:
            oo.set_wall_bounds(solver, N, n_lo, n_hi, x[1])
        t0 = time.perf_counter()
        st = solver.solve()
        ms = (time.perf_counter() - t0) * 1000
        s_pred = [solver.get(k, 'x')[0] for k in range(N + 1)]
        res = np.concatenate([[rid, st, ms], solver.get(0, 'u'), solver.get(1, 'x'), s_pred])
        os.write(a.fd_out, res.tobytes())


if __name__ == '__main__':
    sys.exit(main())
