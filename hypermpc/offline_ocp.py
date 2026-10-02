#!/usr/bin/env python3
"""
offline_ocp.py — HyperMPC 정식화를 차 없이 한 번 풀어본다. (3단계)

  source ~/hmpc-venv/bin/activate
  python3 hypermpc/offline_ocp.py

■ 이 스크립트가 답하는 질문은 하나다

    "이 정식화가 우리 차 값으로 세워지고, 젯슨에서 실시간으로 풀리는가?"

  경로 추종 성능은 보지 않는다. status 와 solve time 두 숫자만 본다.
  논문은 N=80 · dt=0.033 에서 10.61 ms 였다 (x86 데스크톱). 30Hz 제어면
  예산이 33 ms 다. 젯슨에서 얼마가 나오는지가 실시간 가능 여부를 정한다.

■ 원본과의 관계

  동역학은 hypermpc_code 의 robot_model/car/casadi_car_model_naive.py 를
  그대로 쓴다 — casadi + numpy 만 쓰는 유일한 경로다 (다른 모델 파일은
  torch 와 l4casadi 를 import 한다).

  OCP 조립은 mpc/mpc_formulation_car.py 의 구조를 따르되, 여기서는
  sensitivity / p_global 경로를 뺀 최소판을 직접 세운다. 원본 래퍼는
  model 객체에 15가지 속성을 요구하는데 그 model 파일이 저장소에 없다.

■ 상태·입력 (원본 casadi_car_model.py 기준)

    x = [ s, n, mu,            Frenet 포즈 (진행, 횡편차, 헤딩편차)
          vx, vy, r, friction, 동역학
          wheel_speed, delta ] 액추에이터      → 9개
    u = [ wheel_speed_ref, delta_ref ]        → 2개

  액추에이터는 1차 지연으로 붙인다 (원본 config 의 시정수).
"""

import argparse
import os
import sys
import time

import numpy as np
import casadi as cs
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel


# ── 원본 naive 동역학을 가져온다 ────────────────────────────────────────────
def load_naive_dynamics(hypermpc_dir):
    """robot_model/car/casadi_car_model_naive.py 의 pacejca_single_track_casadi.

    원본 파일을 그대로 쓴다. 단 m(질량)은 하드코딩 5.1 이므로 아래에서
    우리 값으로 바꿔 쓰려면 파일을 복사해 고치거나, 여기처럼 import 한 뒤
    질량이 들어간 항을 따로 다루어야 한다.
    ★ 지금은 원본 그대로 쓴다 — 우리 질량을 아직 안 쟀다.
    """
    sys.path.insert(0, hypermpc_dir)
    from robot_model.car.casadi_car_model_naive import pacejca_single_track_casadi
    return pacejca_single_track_casadi()


def default_params():
    """p[30] — 원본의 기본값.

      0~9   차량   (single_track_params.py: default_params_tensor)
      10~19 앞타이어 (pacejka_params.py)
      20~29 뒤타이어 (같은 값)
    """
    vehicle = [
        # ★ I_z, lr 은 **우리 차 값**으로 바꿨다. 나머지는 아직 논문 값이다.
        #   논문 0.46 은 5.1 kg 에서 회전반경 0.300 m 인데, 길이 0.505 m
        #   물체의 이론 최대가 0.253 m 다 — 측정치가 아니라 학습 파라미터의
        #   초기값이므로 그대로 쓰면 안 된다 (vehicle.yaml 주석 참고).
        0.084,       # I_z   요 관성모멘트  (우리 차 추정, 논문 0.46)
        0.1498,      # lr    무게중심~뒤축  (우리 차 실측, 논문 0.115)
        0.01,        # Cd0   구름저항
        0.01,        # Cd2   공기저항 v^2
        0.01,        # Cd1   점성저항 v
        0.2,         # I_e   구동계 관성
        0.90064745,  # K_fi  모터 상수
        0.304115174, # b1
        0.50421894,  # b0
        0.05,        # R     바퀴 반경
    ]
    tire = [
        0.05,  # Sx_p
        2.0,   # Alpha_p
        0.35,  # By
        1.4,   # Cy
        1.0,   # Dy
        1.2,   # Ey
        30.0,  # Bx
        1.3,   # Cx
        1.0,   # Dx
        0.5,   # Ex
    ]
    return np.array(vehicle + tire + tire, dtype=float)


def build_ocp(dyn_fun, N, dt, cfg):
    """Frenet 컨투어링 OCP 를 세운다."""
    # ── 상태 ────────────────────────────────────────────────────────────
    s     = cs.MX.sym('s')          # 경로 진행거리
    n     = cs.MX.sym('n')          # 횡편차
    mu    = cs.MX.sym('mu')         # 헤딩편차
    vx    = cs.MX.sym('vx')
    vy    = cs.MX.sym('vy')
    r     = cs.MX.sym('r')
    fric  = cs.MX.sym('friction')
    ws    = cs.MX.sym('wheel_speed')
    delta = cs.MX.sym('delta')
    x = cs.vertcat(s, n, mu, vx, vy, r, fric, ws, delta)

    # ── 입력 ────────────────────────────────────────────────────────────
    ws_ref    = cs.MX.sym('wheel_speed_ref')
    delta_ref = cs.MX.sym('delta_ref')
    u = cs.vertcat(ws_ref, delta_ref)

    # ── 파라미터 ────────────────────────────────────────────────────────
    #   p[0:30] 차량+타이어,  p[30] 이 지점의 경로 곡률 kappa
    p = cs.MX.sym('p', 31)
    p_dyn = p[0:30]
    kappa = p[30]

    # ── 동역학 ──────────────────────────────────────────────────────────
    #   원본 naive 모델은 (vx, vy, r, friction) 만 다룬다.
    x_dyn = cs.vertcat(vx, vy, r, fric)
    u_dyn = cs.vertcat(ws, delta)          # ★ 액추에이터 '상태' 가 들어간다
    x_dyn_dot = dyn_fun(x_dyn, u_dyn, p_dyn)

    # Frenet 운동학 — 곡률 kappa 인 경로 위에서
    denom = 1.0 - n * kappa
    s_dot  = (vx * cs.cos(mu) - vy * cs.sin(mu)) / denom
    n_dot  = vx * cs.sin(mu) + vy * cs.cos(mu)
    mu_dot = r - kappa * s_dot

    # 액추에이터 1차 지연 (원본 config_car.yaml 의 시정수)
    ws_dot    = (ws_ref - ws) / cfg['wheel_speed_tau']
    delta_dot = (delta_ref - delta) / cfg['steering_tau']

    f_expl = cs.vertcat(s_dot, n_dot, mu_dot,
                        x_dyn_dot[0], x_dyn_dot[1], x_dyn_dot[2], x_dyn_dot[3],
                        ws_dot, delta_dot)

    xdot = cs.MX.sym('xdot', 9)

    model = AcadosModel()
    model.name = 'hmpc_car'
    model.x, model.u, model.xdot, model.p = x, u, xdot, p
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    # ── OCP ─────────────────────────────────────────────────────────────
    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt

    # 비용 — 원본 가중치 이름을 그대로 쓴다 (config_car.yaml)
    q_n, q_mu = cfg['q_n'], cfg['q_mu']
    q_delta, q_speed = cfg['q_delta'], cfg['q_speed_diff']
    v_target = cfg['v_target']

    stage = (q_n * n**2 + q_mu * mu**2
             + q_speed * (vx - v_target)**2
             + q_delta * delta_ref**2
             - cfg['q_progress'] * s_dot)       # 진행은 보상
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = stage
    ocp.model.cost_expr_ext_cost_e = cfg['q_e_scaler'] * (
        q_n * n**2 + cfg['q_mu_e'] * mu**2)

    ocp.constraints.x0 = np.zeros(9)
    ocp.constraints.lbu = np.array([cfg['min_ws_ref'], -cfg['max_steer_ref']])
    ocp.constraints.ubu = np.array([cfg['max_ws_ref'], +cfg['max_steer_ref']])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.parameter_values = np.concatenate([default_params(), [0.0]])

    # 솔버 — 원본과 같은 조합
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hpipm_mode = 'BALANCE'
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.regularize_method = 'PROJECT'
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    return ocp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hypermpc', default=os.path.expanduser('~/hypermpc_code'),
                    help='hypermpc_code 저장소 경로')
    ap.add_argument('--N', type=int, default=80)
    ap.add_argument('--dt', type=float, default=0.033)
    ap.add_argument('--laps', type=int, default=200, help='풀이 반복 횟수')
    args = ap.parse_args()

    if not os.path.isdir(args.hypermpc):
        print(f'✗ hypermpc_code 를 찾을 수 없습니다: {args.hypermpc}')
        print('  git clone https://github.com/hyper-mpc/hypermpc_code.git ~/hypermpc_code')
        return 1

    cfg = dict(
        # 원본 config_car.yaml
        wheel_speed_tau=0.046122448979591835,
        steering_tau=0.0436734693877551,
        q_n=0.02, q_mu=0.002, q_delta=2.0, q_speed_diff=2.0,
        q_e_scaler=1.0, q_mu_e=5.0,
        q_progress=1.0,
        min_ws_ref=0.01, max_ws_ref=7.0, max_steer_ref=0.5,
        # 우리 트랙 기준 목표 속도 (track.yaml 의 target_speed 와 맞춘다)
        v_target=1.2,
    )

    print(f'\n■ 정식화  N={args.N}  dt={args.dt}  '
          f'예측구간 {args.N*args.dt:.2f}초')
    dyn_fun, _ = load_naive_dynamics(args.hypermpc)
    print(f'  동역학 로드: {dyn_fun}')

    ocp = build_ocp(dyn_fun, args.N, args.dt, cfg)
    print('\n■ 코드 생성 + 빌드 중... (첫 실행은 수 분 걸립니다)', flush=True)
    t0 = time.perf_counter()
    solver = AcadosOcpSolver(ocp, verbose=False)
    print(f'  ✓ 빌드 완료 ({time.perf_counter()-t0:.1f}초)')

    # ── 한 바퀴 도는 상황을 흉내 ────────────────────────────────────────
    #   track_5 의 평균 곡률 0.575 /m 를 모든 스텝에 넣는다.
    par = np.concatenate([default_params(), [0.575]])
    for k in range(args.N + 1):
        solver.set(k, 'p', par)

    x0 = np.zeros(9)
    x0[3] = 1.0      # vx  — 정지에서 풀면 타이어 모델이 특이해진다
    x0[7] = 1.0/0.05 # wheel_speed ≈ vx / R
    solver.set(0, 'lbx', x0)
    solver.set(0, 'ubx', x0)

    ts, bad = [], 0
    for _ in range(args.laps):
        t0 = time.perf_counter()
        st = solver.solve()
        ts.append((time.perf_counter() - t0) * 1000)
        if st != 0:
            bad += 1

    ts = np.array(ts)
    print(f'\n■ 결과  ({args.laps}회 반복)\n')
    print(f'  status != 0 : {bad} 회  {"← 모두 정상" if bad == 0 else "← 확인 필요"}')
    print(f'  풀이 시간   중앙 {np.median(ts):7.2f} ms')
    print(f'              95%  {np.percentile(ts, 95):7.2f} ms')
    print(f'              최대 {ts.max():7.2f} ms')
    print(f'\n  u0 = {solver.get(0, "u")}   (wheel_speed_ref, delta_ref)')

    budget = args.dt * 1000
    print(f'\n■ 실시간 판정  (한 주기 예산 {budget:.1f} ms)\n')
    p95 = np.percentile(ts, 95)
    if p95 < budget * 0.5:
        print(f'  ✓ 여유 있음 — 95% 가 예산의 {p95/budget*100:.0f}%')
    elif p95 < budget:
        print(f'  △ 빠듯함 — 95% 가 예산의 {p95/budget*100:.0f}%. N 을 줄이는 것 검토')
    else:
        print(f'  ✗ 초과 — 95% 가 예산의 {p95/budget*100:.0f}%.')
        print(f'    N 을 줄이거나 제어 주기를 낮춰야 한다.')
    print(f'\n  참고: 논문 10.61 ms (N=80, x86 데스크톱)\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
