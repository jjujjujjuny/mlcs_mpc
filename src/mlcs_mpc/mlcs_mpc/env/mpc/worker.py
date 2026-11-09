#!/usr/bin/env python3
"""
worker — MPCC OCP 를 풀기만 하는 프로세스. control/mpcc_node.py 가 띄운다 (~/hmpc-venv 의 파이썬으로).

■ 왜 프로세스를 나누나

  슬립이 0 이거나 NaN 이 들어가면 acados solve() 는 돌아오지 않는다 (2026-11-05 실측).
  C 안에서 멈춘 호출은 파이썬 스레드로 끊을 수 없다. 그런데 VESC 는 새 명령이 없으면
  마지막 속도를 유지하므로, 멈춘 쪽과 정지 명령을 보내는 쪽이 같은 프로세스면 위험하다.
  그래서 노드는 여기서 답이 안 오면 이 프로세스를 SIGKILL 하고 차를 세운다.

  casadi·acados 는 ~/hmpc-venv 에만 있어서 환경을 나누는 이점도 있다.

■ 설정 (--config JSON) — mpcc_node 의 ROS 파라미터 (= config/mpcc.yaml + 덮어쓰기) 그대로

  MpccConfig 의 이름들 (N, dt, 경계, 가중치 …), 차 값 (mass, friction, vehicle_params, tire_*, 시정수),
  track_csv (곡률이 생성 코드에 들어간다), param_model (learned/), solver_core, repo_root.

■ 통신 (전용 파이프, double 고정 길이)

  acados 가 C 에서 stdout 에 직접 쓰므로 stdout 은 통신에 쓸 수 없다.

    시작  → b'R' 한 바이트
    요청  [id, cmd, t, x(9), n_lo(N+1), n_hi(N+1), v_cap(N)]   cmd 1 이면 초기 추측을 x 로 다시 깐다
          t: 노드 시각 [s]. 학습 모델 관측 기록과 직전 해의 시각 맞춤에만 쓴다
          n_lo/n_hi: 스테이지별 벽 경계 (MPCC, 상대 차 통로 포함). track_hard 일 때만 쓴다
          v_cap: 스테이지 0..N-1 의 바퀴 속도 명령 상한 (상대 뒤따르기). v_max 보다 크면 v_max
    응답  [id, status, ms, u(2), x1(9), s(N+1)]

  파이프가 닫히면 (노드가 죽으면) 끝난다.

■ 스테이지별 동역학 파라미터 (env/mpc/param_model.py)

  param_model 이 비면 상수 params 를 모든 스테이지에. 학습 모델 (예 'hyperpm:<.npz>') 을 주면 매 요청마다
  지금 x 를 관측으로 넣고 (N+1, 30) 파라미터를 받는다. 추론은 풀이 시간(ms)에 넣지 않지만 노드의 제한
  시간 안에는 들어간다. 출력이 유한하지 않으면 상수를 쓴다.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..', '..', '..'))       # src/mlcs_mpc
REPO = os.path.abspath(os.path.join(PKG, '..', '..'))             # 저장소 (learned/)
sys.path[:0] = [p for p in (PKG, REPO) if p not in sys.path]

from mlcs_mpc.env.config import MpccConfig  # noqa: E402
from mlcs_mpc.env.mpc import param_model  # noqa: E402
from mlcs_mpc.env.mpc.mpcc import MPCC  # noqa: E402
from mlcs_mpc.env.track import Track  # noqa: E402
from mlcs_mpc.env.vehicle_model import SLIP_EPS, VehicleModel  # noqa: E402


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
    ap.add_argument('--config', required=True, help='JSON — mpcc_node 의 파라미터')
    ap.add_argument('--build-dir', default=os.path.expanduser('~/mpcc_build'))
    ap.add_argument('--no-guard', action='store_true', help='시험 전용. 슬립 0 에서 solve 가 멈춘다')
    a = ap.parse_args()
    c = json.loads(a.config)
    if c.get('solver_core', -1) >= 0:
        os.sched_setaffinity(0, {c['solver_core']})

    cfg = MpccConfig.from_params(c)
    N = cfg.N
    # ★ 마찰은 상태 x[6] 로 들어간다 (생성 코드 밖). 예전에는 노드 값을 안 받고 기본값으로
    #   덮어써서 설정의 friction 이 적용되지 않았다 (2026-11-08 발견).
    model = VehicleModel(mass=c['mass'], friction=c['friction'], vehicle_params=c['vehicle_params'],
                         tire_front=c['tire_front'], tire_rear=c['tire_rear'],
                         steering_tau=c['steering_tau'], wheel_speed_tau=c['wheel_speed_tau'],
                         slip_eps=0.0 if a.no_guard else SLIP_EPS)
    print(f'friction {model.friction}  N {N}  dt {cfg.dt}  트랙 {c["track_csv"]}', flush=True)
    pm = param_model.load(c.get('param_model', ''), N, cfg.dt, model.p,
                          v_min=float(c.get('param_model_v_min', 0.0)),
                          blend=float(c.get('param_model_blend', 0.0)), root=c.get('repo_root', REPO))
    learned = pm.name != 'constant'
    if learned:
        print(f'param_model {pm.name}: {c["param_model"]}', flush=True)
    mpc = MPCC(model, Track(c['track_csv']), cfg, a.build_dir, param_model=pm,
               log=lambda m: print(m, flush=True))
    mpc.warmup()
    n_pm, a_sum, ms_pm = 0, 0.0, 0.0

    req_len = 8 * (3 + 9 + 2 * (N + 1) + N)
    os.write(a.fd_out, b'R')
    while True:
        raw = read_exact(a.fd_in, req_len)
        if raw is None:
            return 0
        req = np.frombuffer(raw, dtype=np.float64)
        rid, cmd, t_req, x = req[0], req[1], req[2], req[3:12]
        n_lo, n_hi = req[12:12 + 2 * (N + 1)].reshape(2, N + 1)
        r = mpc.solve(x, t_req, n_lo, n_hi, v_cap=req[12 + 2 * (N + 1):], reset=cmd == 1.0)
        if learned:
            ms_pm += mpc.pm_ms
            n_pm += 1
            a_sum += mpc.pm_w
            if n_pm % 300 == 0:
                print(f'{pm.name} 300회: 평균 {ms_pm / 300:.2f} ms, 쓴 비율 {a_sum / 300:.2f}', flush=True)
                ms_pm, a_sum = 0.0, 0.0
        res = np.concatenate([[rid, r['status'], r['ms']], r['u'], r['x1'], r['xs'][:, 0]])
        os.write(a.fd_out, res.tobytes())


if __name__ == '__main__':
    sys.exit(main())
