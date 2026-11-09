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

■ 측정 (--scenario)

    straight   곡률 0
    curve      곡률 0.575 /m 고정 (track_5 평균, R 1.74 m)
    track      s_track 의 곡률 κ(s) 를 예측 구간에 깔고 따라간다
    zero       슬립이 정확히 0 인 직진 초기값 (--no-guard 로 원본 그대로 확인)

  직전 해의 x[1] 을 다음 초기값으로 넘기는 폐루프다 (모델 오차 없음). 같은
  초기값을 반복해 풀면 2회째부터 이미 수렴한 해라 시간이 짧게 나온다.

■ ★ 저자 타이어 식의 0/0 (--no-guard 로 끌 수 있다)

  casadi_car_model_naive.py 의 합성 슬립은 sqrt(Sx² + Alpha²) 이고 이것으로
  나눈다. 슬립이 정확히 0 이면 0/0 이고 미분은 무한대다. 저자 파일을 고치지
  않고, 그 모듈이 보는 casadi 의 sqrt 만 sqrt(x + 1e-12) 로 바꿔 끼운다.
  (그 파일에서 sqrt 를 쓰는 곳은 여기 하나다. 값은 hpm/mlcs_car_model.py 와 같다)

■ ★ 저자 식의 질량 5.1 kg

  m 이 함수 안에 숫자로 박혀 있어 밖에서 바꿀 수 없다. 대신 m 이 들어가는 곳이
  항력/m 과 (타이어 힘 ∝ m)/I_z 뿐이므로, k = 5.1/실제 질량 으로 Cd 와 I_z 에
  k 를 곱해 넘기면 식이 실제 질량의 것과 정확히 같아진다 (scale_for_mass).
  타이어 힘/m 항에서는 m 이 약분된다.

■ ★ 저속에서 RK4 한 번으로는 적분이 발산한다 (--rk-steps)

  슬립률 분모가 v_x 라 타이어 강성이 v_x 에 반비례해 커진다. 기본 계수·마찰 0.41 이면
  v_x 방향 고유값이 약 157/v_x 1/s 로, dt 0.033 에서 RK4 안정 한계(2.78)를 v_x 약 1.9 m/s
  아래에서 넘는다. 실제로 RK4 1회면 0.7 m/s 이하에서 QP 실패가 대부분이고 0.25 m/s 에서는
  solve 가 돌아오지 않았다. 구간당 3회면 0.25~1 m/s 모두 정상이었지만, 동정값은 뒤 타이어
  Bx 가 커서 3회로는 0.25 m/s 출발에서 QP 실패가 대부분이고 4회면 정상이다 (2026-11-05).
  계수가 바뀌면 다시 볼 것.

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
import glob
import hashlib
import os
import sys
import time

import numpy as np
import casadi as cs
import yaml
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hyper_params import HyperStage  # noqa: E402


# ── 원본 naive 동역학을 가져온다 ────────────────────────────────────────────
SLIP_EPS = 1e-12    # hpm/mlcs_car_model.py 의 SafePacejkaTireModel 과 같은 값
MASS = 3.284        # 저울 실측 2026-11-05 (라이다 포함)
AUTHOR_MASS = 5.1   # casadi_car_model_naive.py 하드코딩
RK_STEPS = 4        # 구간당 RK4 횟수. 머리말 참고


class SafeSqrtCasadi:
    """casadi 그대로인데 sqrt 만 0 에서 안전하다."""

    def __getattr__(self, name):
        return getattr(cs, name)

    @staticmethod
    def sqrt(x):
        return cs.sqrt(x + SLIP_EPS)


def load_naive_dynamics(hypermpc_dir, guard=True):
    """robot_model/car/casadi_car_model_naive.py 의 pacejca_single_track_casadi.

    원본 파일을 그대로 쓴다. 질량은 scale_for_mass 로 맞춘다.
    """
    sys.path.insert(0, hypermpc_dir)
    import robot_model.car.casadi_car_model_naive as naive
    naive.casadi = SafeSqrtCasadi() if guard else cs
    return naive.pacejca_single_track_casadi()


def scale_for_mass(p, mass=MASS):
    """저자 식(m=5.1)에 넘길 p. 머리말 참고. p 는 숫자 배열도 casadi 도 된다."""
    k = AUTHOR_MASS / mass
    return cs.vertcat(p[0] * k, p[1], p[2] * k, p[3] * k, p[4] * k, p[5:])


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
        0.089,       # I_z   요 관성모멘트  (우리 차 추정, 논문 0.46)
        0.1537,      # lr    무게중심~뒤축  (우리 차 실측, 논문 0.115)
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
    x_dyn_dot = dyn_fun(x_dyn, u_dyn, scale_for_mass(p_dyn))

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

    build_ocp.last_f = cs.Function('f', [x, u, p], [f_expl])   # make_plant 가 쓴다

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

    if cfg['cost'] == 'author':
        # casadi_car_model.py 그대로: -s, 슬립각, 트랙 배리어, 슬립률
        def barrier(h):
            return cs.log(1 + cs.exp(-cfg['barrier_lambda'] * h))
        beta_kin = cs.atan(delta * 0.15 / 0.33)
        beta_dyn = cs.atan(vy / vx)
        common = (-s + cfg['q_beta'] * (beta_kin - beta_dyn)**2
                  + q_n * n**2 + q_mu * mu**2
                  + cfg['q_slip'] * (ws - vx)**2)
        if not cfg['track_hard']:
            common = common + barrier(cfg['track_width'] - n) + barrier(cfg['track_width'] + n)
        stage = (common + q_speed * (ws_ref - ws)**2
                 + q_delta * (delta_ref - delta)**2)
        terminal = cfg['q_e_scaler'] * (common + cfg['q_mu_e'] * mu**2)
    else:
        stage = (q_n * n**2 + q_mu * mu**2
                 + q_speed * (vx - v_target)**2
                 + q_delta * delta_ref**2
                 - cfg['q_progress'] * s_dot)       # 진행은 보상
        terminal = cfg['q_e_scaler'] * (q_n * n**2 + cfg['q_mu_e'] * mu**2)
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = stage
    ocp.model.cost_expr_ext_cost_e = terminal

    ocp.constraints.x0 = np.zeros(9)
    ocp.constraints.lbu = np.array([cfg['min_ws_ref'], -cfg['max_steer_ref']])
    ocp.constraints.ubu = np.array([cfg['max_ws_ref'], +cfg['max_steer_ref']])
    ocp.constraints.idxbu = np.array([0, 1])
    if cfg['track_hard']:
        # ★ 트랙 경계를 비용이 아니라 상태 경계로 (|n| ≤ track_width). 저자는 비용(배리어)이다.
        #   폭은 실행 중에 스테이지별로 바꾼다 (ocp_worker 의 경계 일정 참고).
        w = cfg['track_width']
        ocp.constraints.idxbx = np.array([1])
        ocp.constraints.lbx = np.array([-w])
        ocp.constraints.ubx = np.array([+w])
        ocp.constraints.idxbx_e = np.array([1])
        ocp.constraints.lbx_e = np.array([-w])
        ocp.constraints.ubx_e = np.array([+w])
        if cfg['track_slack'] > 0:
            # ★ 순수 하드 경계는 차가 폭 안에 못 머무는 순간(한계 코너, 외란) QP 해가 없어져
            #   status 4 가 된다 (1108_3, ±0.12 에서 94%). 슬랙에 큰 L1 무게를 걸면 지킬 수
            #   있을 때는 하드와 똑같이 지키고, 못 지킬 때만 최소로 넘는다 (정확 페널티).
            z = cfg['track_slack']
            ocp.constraints.idxsbx = np.array([0])
            ocp.constraints.idxsbx_e = np.array([0])
            ocp.cost.zl = ocp.cost.zu = np.array([z])
            ocp.cost.Zl = ocp.cost.Zu = np.array([z * 0.1])
            ocp.cost.zl_e = ocp.cost.zu_e = np.array([z])
            ocp.cost.Zl_e = ocp.cost.Zu_e = np.array([z * 0.1])

    if cfg.get('ws_acc_max', 0.0) > 0 or cfg.get('ws_dec_max', 0.0) > 0:
        # ★ 바퀴 가속도 한계 (2026-11-08). tau 0.046 이면 모델은 바퀴가 바로 명령을 따라가서
        #   언제든 설 수 있다고 믿는다. 실제는 감속 ~1.5 m/s² (제동이 꺼져 있던 race 로그,
        #   tools/actuator_id.py). tau 를 0.66 으로 올리면 QP 가 실패하므로, tau 는 두고
        #   ws' = (ws_ref − ws)/tau 를 [−ws_dec_max, ws_acc_max] 로 묶는다:
        #       −a_dec·tau ≤ ws_ref − ws ≤ a_acc·tau        (스테이지 0..N-1, 선형)
        #   현재 ws 가 v_max 보다 한참 높으면 ubu 와 겹쳐 해가 없어질 수 있어 슬랙을 둔다.
        tau = cfg['wheel_speed_tau']
        big = 1e3
        a_acc = cfg['ws_acc_max'] if cfg.get('ws_acc_max', 0.0) > 0 else big
        a_dec = cfg['ws_dec_max'] if cfg.get('ws_dec_max', 0.0) > 0 else big
        C = np.zeros((1, 9))
        C[0, 7] = -1.0
        ocp.constraints.C = C
        ocp.constraints.D = np.array([[1.0, 0.0]])
        ocp.constraints.lg = np.array([-a_dec * tau])
        ocp.constraints.ug = np.array([+a_acc * tau])
        zg = cfg.get('ws_rate_slack', 1000.0)
        ocp.constraints.idxsg = np.array([0])
        # 슬랙 순서는 [bx, g] — 트랙 경계 슬랙이 있으면 그 뒤에 붙인다
        zl = list(ocp.cost.zl) if len(ocp.constraints.idxsbx) else []
        Zl = list(ocp.cost.Zl) if len(ocp.constraints.idxsbx) else []
        ocp.cost.zl = ocp.cost.zu = np.array(zl + [zg])
        ocp.cost.Zl = ocp.cost.Zu = np.array(Zl + [zg * 0.1])
        # ★ 스테이지 0 은 x0 고정이라 트랙 경계(bx) 슬랙이 없고 이 g 슬랙만 있다. 개수가 달라서
        #   acados 가 zl_0 를 못 채운다 (젯슨 빌드에서 ValueError, 2026-11-08) — 직접 준다.
        ocp.cost.zl_0 = ocp.cost.zu_0 = np.array([zg])
        ocp.cost.Zl_0 = ocp.cost.Zu_0 = np.array([zg * 0.1])

    ocp.parameter_values = np.concatenate([default_params(), [0.0]])

    # 솔버 — 원본과 같은 조합
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hpipm_mode = 'BALANCE'
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.regularize_method = 'PROJECT'
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = cfg['rk_steps']
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    return ocp


def make_cfg(cost='author', rk_steps=RK_STEPS, steering_tau=0.0436734693877551,
             wheel_speed_tau=0.046122448979591835, track_hard=False, track_width=0.25,
             track_slack=0.0, ws_acc_max=0.0, ws_dec_max=0.0):
    """OCP 설정. 기본값은 저자 config_car.yaml 이다. 우리 차 시정수는 config/hmpc.yaml."""
    return dict(
        wheel_speed_tau=wheel_speed_tau,
        steering_tau=steering_tau,
        q_n=0.02, q_mu=0.002, q_delta=2.0, q_speed_diff=2.0,
        q_e_scaler=1.0, q_mu_e=5.0,
        q_beta=1.0, q_slip=0.1, barrier_lambda=200.0, track_width=track_width,
        # True 면 배리어 비용 대신 |n| ≤ track_width 상태 경계 (저자는 False)
        track_hard=track_hard,
        # > 0 이면 경계 넘는 양에 이 L1 무게 (정확 페널티). 0 이면 순수 하드
        track_slack=track_slack,
        # > 0 이면 바퀴 가속도 한계 m/s² (0 = 끔, 저자). tools/actuator_id.py 로 잰다
        ws_acc_max=ws_acc_max, ws_dec_max=ws_dec_max, ws_rate_slack=1000.0,
        q_progress=1.0,
        min_ws_ref=0.01, max_ws_ref=7.0, max_steer_ref=0.5,
        # 우리 트랙 기준 목표 속도 (track.yaml 의 target_speed 와 맞춘다)
        v_target=1.2,
        # 실측 횡방향 운동 마찰. 상태라서 0 으로 두면 타이어 힘이 통째로 사라진다
        friction=0.41,
        cost=cost,
        rk_steps=rk_steps,
    )


def make_solver(dyn_fun, N, dt, cfg, build_dir, guard=True):
    """생성 코드가 있으면 다시 쓰고, 없으면 빌드한다."""
    tag = (f'N{N}_{cfg["cost"]}_{"guard" if guard else "noguard"}_rk{cfg["rk_steps"]}'
           + ('_hard' if cfg.get('track_hard') else '')
           + ('_slack' if cfg.get('track_hard') and cfg.get('track_slack', 0) > 0 else '')
           + ('_wsrate' if cfg.get('ws_acc_max', 0) > 0 or cfg.get('ws_dec_max', 0) > 0 else ''))
    build = os.path.join(build_dir, tag)
    os.makedirs(build, exist_ok=True)
    ocp = build_ocp(dyn_fun, N, dt, cfg)
    ocp.code_export_directory = os.path.join(build, 'c_generated_code')
    json_file = os.path.join(build, 'ocp.json')
    # 라이브러리 이름에 해시가 붙으므로 이 파일과 dt, 설정이 같을 때만 다시 쓴다.
    with open(__file__, 'rb') as f:
        stamp = hashlib.md5(f.read() + repr((dt, sorted(cfg.items()))).encode()).hexdigest()
    stamp_file = os.path.join(build, 'source.md5')
    reuse = (os.path.exists(stamp_file) and open(stamp_file).read() == stamp
             and glob.glob(os.path.join(ocp.code_export_directory, 'libacados_ocp_solver_*.so')))
    print(f'■ {"이전 빌드 사용" if reuse else "코드 생성 + 빌드 중"}: {build}', flush=True)
    t0 = time.perf_counter()
    solver = AcadosOcpSolver(ocp, json_file=json_file, build=not reuse,
                             generate=not reuse, verbose=False)
    with open(stamp_file, 'w') as f:
        f.write(stamp)
    print(f'  ✓ {time.perf_counter()-t0:.1f}초', flush=True)
    return solver


def make_plant(dyn_fun, cfg, dt, substeps=16, ws_tau=None, ws_acc=None, ws_dec=None):
    """OCP 와 같은 연속 모델을 RK4 로 dt 만큼 적분하는 '차'. → step(x, u, p) → x_next.

    ★ 폐루프 시험에서 solver.get(1, 'x') 를 다음 상태로 쓰면 안 된다. SQP_RTI 는 한 번만 반복해서
      그 값이 동역학을 정확히 만족하지 않는다 (1108_6 시험에서 vx 0.7 인데 s 가 단계당 0.003 m).

    ws_tau / ws_acc / ws_dec 를 주면 '차' 의 바퀴만 실제처럼 바꾼다 (MPC 는 그대로). 실제 차의
    느린 바퀴 응답에서 MPC 가 어떻게 되는지 보려고 쓴다 (tools/actuator_id.py 값).
    """
    pc = dict(cfg)
    if ws_tau is not None:
        pc['wheel_speed_tau'] = ws_tau
    pc['ws_acc_max'] = pc['ws_dec_max'] = 0.0   # 식만 쓰므로 제약은 필요 없다
    build_ocp(dyn_fun, 2, dt, pc)           # 식만 만든다 (빌드 안 함)
    f = build_ocp.last_f
    h = dt / substeps
    tau = pc['wheel_speed_tau']

    def actuate(x, u):
        """바퀴 가속도 한계를 명령에 넣는다: ws' = clip((u−ws)/tau) 가 되도록 u 를 바꾼다."""
        if ws_acc is None and ws_dec is None:
            return u
        rate = (u[0] - x[7]) / tau
        rate = np.clip(rate, -(ws_dec or np.inf), ws_acc or np.inf)
        return np.array([x[7] + rate * tau, u[1]])

    def step(x, u0, p):
        x = np.asarray(x, float)
        for _ in range(substeps):
            u = actuate(x, u0)
            k1 = np.array(f(x, u, p)).ravel()
            k2 = np.array(f(x + h / 2 * k1, u, p)).ravel()
            k3 = np.array(f(x + h / 2 * k2, u, p)).ravel()
            k4 = np.array(f(x + h * k3, u, p)).ravel()
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x
    return step


def hard_bounds(N, w, n0, relax=10):
    """스테이지 1..N 의 |n| 상한.

    현재 |n| 이 이미 w 를 넘었으면 첫 스테이지부터 w 를 요구하면 해가 없다 (한 스텝 0.033 s
    에 못 돌아온다). 그래서 relax 스테이지 동안 현재 값에서 w 로 줄여 간다.
    """
    k = np.arange(1, N + 1)
    return np.maximum(w, abs(n0) * np.clip(1.0 - k / relax, 0.0, 1.0)) + 1e-3


def set_hard_bounds(solver, N, w, n0):
    for k, b in zip(range(1, N + 1), hard_bounds(N, w, n0)):
        solver.constraints_set(k, 'lbx', np.array([-b]))
        solver.constraints_set(k, 'ubx', np.array([+b]))


def wall_bounds(N, lo, hi, n0, relax=10):
    """스테이지 1..N 의 n 범위 [lo_k, hi_k] (MPCC: 지점별 벽 폭 − 여유).

    lo, hi 는 스테이지 0..N 의 값 (길이 N+1). 현재 n0 가 범위 밖이면 hard_bounds 처럼 relax
    스테이지 동안 n0 에서 범위로 줄여 간다. 좁아서 lo > hi 면 가운데 한 점으로 둔다.
    """
    lo, hi = np.asarray(lo, float)[1:N + 1].copy(), np.asarray(hi, float)[1:N + 1].copy()
    bad = lo > hi
    lo[bad] = hi[bad] = 0.5 * (lo[bad] + hi[bad])
    f = np.clip(1.0 - np.arange(1, N + 1) / relax, 0.0, 1.0)
    return np.minimum(lo, n0 * f) - 1e-3, np.maximum(hi, n0 * f) + 1e-3


def set_wall_bounds(solver, N, lo, hi, n0):
    for k, a, b in zip(range(1, N + 1), *wall_bounds(N, lo, hi, n0)):
        solver.constraints_set(k, 'lbx', np.array([a]))
        solver.constraints_set(k, 'ubx', np.array([b]))


def track_walls(path, margin):
    """prep_*.csv → q ↦ (κ, n 하한, n 상한). w_left/w_right 열이 없으면 track_width/2."""
    d = np.genfromtxt(path, delimiter=',', names=True)
    length = d['s'][-1] + np.hypot(d['x'][0] - d['x'][-1], d['y'][0] - d['y'][-1])
    if 'w_left' in d.dtype.names:
        wl, wr = d['w_left'], d['w_right']
    else:
        wl = wr = d['track_width'] / 2
    S = np.r_[d['s'], length]
    ext = lambda v: np.r_[v, v[0]]  # noqa: E731

    def at(q):
        q = np.mod(q, length)
        return (np.interp(q, S, ext(d['curvature'])),
                -(np.interp(q, S, ext(wr)) - margin),
                np.interp(q, S, ext(wl)) - margin)
    return at


def track_kappa(path):
    """prep_*.csv → κ(s) 함수 (닫힌 트랙)."""
    d = np.genfromtxt(path, delimiter=',', names=True)
    length = d['s'][-1]
    return lambda q: np.interp(np.mod(q, length), d['s'], d['curvature'])


def initial_state(cfg, slip=0.02):
    """직진 1 m/s. wheel_speed 는 선속도 [m/s] 다 (base_tire_model 이 v_x 와 뺀다)."""
    x0 = np.zeros(9)
    x0[3] = 1.0
    x0[6] = cfg['friction']
    x0[7] = 1.0 + slip
    return x0


def restart(solver, N, x0):
    """초기 추측을 x0 로 깐다. 0 으로 두면 v_x = 0 이라 슬립 분모가 eps 가 된다."""
    solver.reset()
    for k in range(N + 1):
        solver.set(k, 'x', x0)
    for k in range(N):
        solver.set(k, 'u', np.array([x0[7], 0.0]))


def load_params(path):
    """config/hmpc.yaml 의 차량·타이어 값 → p[30]."""
    with open(path, encoding='utf8') as f:
        d = yaml.safe_load(f)['/**']['ros__parameters']
    return np.array(d['vehicle_params'] + d['tire_front'] + d['tire_rear'], dtype=float)


def make_opponent(track_csv, v, d, gap):
    """--opponent: 트랙 선에서 d 만큼 옆을 v 로 도는 가상 상대 + hmpc_node 와 같은 opponent 로직."""
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../src/mlcs_mpc'))
    from mlcs_mpc.hmpc_core import Track
    from mlcs_mpc.opponent import Opponent
    tr = Track(track_csv)
    return dict(track=tr, logic=Opponent(tr), v=v, d=d, s=gap, modes={}, min_dist=np.inf,
                hits=0, passes=0, last_ds=None)


def _opp_cycle(o, i, dt, qs, x, lo, hi, v_max):
    """가상 상대를 dt 만큼 움직이고 통로·속도 상한을 낸다. 거리 기록도 여기서."""
    tr = o['track']
    t = i * dt
    for j in range(3):          # mocap 100 Hz 쯤
        so = o['s'] + o['v'] * dt * (j + 1) / 3
        h = float(np.interp(np.mod(so, tr.length), np.r_[tr.s, tr.length],
                            np.unwrap(np.r_[tr.heading, tr.heading[0]])))
        ox, oy = tr.xy(so, o['d'])
        o['logic'].measure(t - dt + dt * (j + 1) / 3, float(ox), float(oy), h)
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


def run_scenario(solver, args, cfg, name, kappa_of_s, x0, laps, params, walls=None, plant=None,
                 plant_friction=None, opp=None, hyper=None, plant_hyper=None):
    """폐루프로 laps 번 푼다. walls 가 있으면 지점별 벽 경계 (MPCC). opp 면 가상 상대 (make_opponent).

    hyper: MPC 가 HyperPM 스테이지별 파라미터를 쓴다 (hyper_params.HyperStage, 노드 워커와 같다).
    plant_hyper: 차(plant) 가 HyperPM 의 지금 파라미터로 움직인다. 없으면 차는 params (상수).
    """
    N = args.N
    P_used = []
    wall_hist, s_hist, trace = [], [], []
    v_top = args.v_max if args.v_max is not None else cfg['max_ws_ref']
    restart(solver, N, x0)
    x = x0.copy()
    times, cycles, status, n_hist = [], [], [], []
    first_bad = None
    bad_run, trips, first_trip, need_restart = 0, 0, None, False
    for i in range(laps):
        if need_restart:        # 노드: 비정상 해 뒤 다음 요청은 cmd 1 (초기 추측을 지금 상태로)
            restart(solver, N, x)
            need_restart = False
        t_cycle = time.perf_counter()
        qs = np.array([x[0] + x[3] * k * args.dt if i == 0 else solver.get(k, 'x')[0]
                       for k in range(N + 1)])
        t_now = i * args.dt
        P_k = np.tile(params, (N + 1, 1))
        if hyper is not None:
            hyper.observe(t_now, x)
            P_k, _ = hyper.stage_params(t_now, x)
            P_used.append(P_k[0] / hyper.net.p_const)
        for k in range(N + 1):
            solver.set(k, 'p', np.concatenate([P_k[k], [kappa_of_s(qs[k])]]))
        solver.set(0, 'lbx', x)
        solver.set(0, 'ubx', x)
        if cfg['track_hard'] and walls is not None:
            _, lo, hi = walls(qs)
            if opp is not None:
                lo, hi, cap = _opp_cycle(opp, i, args.dt, qs, x, lo, hi, v_top)
                cap = np.full(N, v_top) if cap is None else cap
                for k in range(N):
                    solver.constraints_set(k, 'ubu', np.array([max(cap[k], cfg['min_ws_ref']), args.max_steer]))
            set_wall_bounds(solver, N, lo, hi, x[1])
            lo0, hi0 = walls(x[0])[1:]
            wall_hist.append((x[1] - lo0 - 1e-9 < 0) or (x[1] - hi0 > 1e-9))
            s_hist.append(x[0])
        elif cfg['track_hard']:
            set_hard_bounds(solver, N, cfg['track_width'], x[1])
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
                hs.plan = None
            else:
                hs.set_plan(t_now, xs)
        if bad and not args.apply_bad:
            # hmpc_node 와 같게: 비정상 해는 버리고 정지 명령 (바퀴 0, 조향 0), 다음엔 다시 깐다.
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
            x_next = plant(xp, u0, np.concatenate([p_plant, [kappa_of_s(x[0])]]))
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
    budget = args.dt * 1000
    print(f'\n■ {name}  ({len(times)}회, 첫 풀이 {times[0]:.1f} ms 는 통계에서 뺌)')
    print(f'  status      {dict(zip(codes.tolist(), counts.tolist()))}'
          + (f'  첫 비정상 {first_bad} 번째' if first_bad is not None else ''))
    if first_bad is not None and not args.apply_bad:
        print(f'  비정상 해는 버리고 정지 명령 (노드와 같게). 3번 연속 {trips}회'
              + (f' — ★ 실차라면 {first_trip} 번째에 래치해 멈춘다' if trips else ''))
    print(f'  풀이 시간   평균 {ts.mean():7.2f}  p95 {np.percentile(ts, 95):7.2f}  '
          f'최대 {ts.max():7.2f} ms   (예산 {budget:.1f} ms, 초과 {(ts > budget).mean() * 100:.1f}%)')
    print(f'  주기 전체   평균 {np.mean(cycles[1:] or cycles):7.2f} ms  (곡률 갱신 포함)')
    print(f'  |n| 최대 {max(n_hist or [0]):.3f} m' + (f'  (경계 {cfg["track_width"]:.2f})' if cfg['track_hard'] else ''))
    if wall_hist:
        sh = np.array(s_hist)
        print(f'  벽 경계 밖 {np.mean(wall_hist) * 100:.1f}%  진행 {sh[-1] - sh[0]:.2f} m '
              f'({(sh[-1] - sh[0]) / (len(sh) * args.dt):.2f} m/s 평균)')
        if getattr(args, 'trace', None):
            np.save(args.trace, np.array(trace))
    print(f'  마지막 상태 vx {x[3]:.3f}  vy {x[4]:.3f}  r {x[5]:.3f}  n {x[1]:.3f}  '
          f'ws {x[7]:.3f}  delta {x[8]:.3f}')
    if P_used:
        R_ = np.array(P_used)
        print(f'  HyperPM 파라미터/기준 (스테이지 0) 최소 ×{R_.min():.2f} 최대 ×{R_.max():.2f}, '
              f'주기간 변화 p99 {np.percentile(np.abs(np.diff(R_, axis=0)), 99) * 100:.2f}%')
    if opp is not None:
        print(f'  상대 {opp["v"]:.2f} m/s, d {opp["d"]:+.2f}: 추월 {opp["passes"]}번, '
              f'최소 중심 거리 {opp["min_dist"]:.2f} m, 겹침(충돌) {opp["hits"]} 주기, 모드 {opp["modes"]}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hypermpc', default=os.path.expanduser('~/hypermpc_code'),
                    help='hypermpc_code 저장소 경로')
    ap.add_argument('--N', type=int, default=80)
    ap.add_argument('--dt', type=float, default=0.033)
    ap.add_argument('--laps', type=int, default=1000, help='시나리오별 풀이 횟수')
    ap.add_argument('--cost', choices=('offline', 'author'), default='author',
                    help='author = 저자 casadi_car_model.py 의 비용. offline = 예전 간이 비용')
    ap.add_argument('--no-guard', action='store_true', help='0/0 방어를 끈다')
    ap.add_argument('--rk-steps', type=int, default=RK_STEPS, help='구간당 RK4 횟수')
    ap.add_argument('--v-max', type=float, default=None, help='입력 속도 상한. 없으면 7 m/s')
    ap.add_argument('--params-yaml', default=None,
                    help='차량·타이어 값 파일 (config/hmpc.yaml). 없으면 저자 기본값')
    ap.add_argument('--start-v', type=float, default=1.0, help='초기 v_x [m/s]')
    ap.add_argument('--max-steer', type=float, default=0.378, help='--v-max 와 함께 쓰는 실제 조향 상한')
    ap.add_argument('--track-hard', type=float, default=None,
                    help='트랙 경계를 |n| ≤ 이 값 의 상태 경계로 (배리어 비용 대신)')
    ap.add_argument('--track-slack', type=float, default=0.0,
                    help='--track-hard 경계에 슬랙 L1 무게 (0 이면 순수 하드)')
    ap.add_argument('--walls', type=float, default=None, metavar='MARGIN',
                    help='track 시나리오: prep 의 w_left/w_right − MARGIN 을 지점별 경계로 (MPCC). '
                         '--track-hard 와 함께')
    ap.add_argument('--friction', type=float, default=None,
                    help='MPC 가 믿는 마찰 (상태 x[6]). 없으면 make_cfg 기본 0.41. 생성 코드와 무관')
    ap.add_argument('--plant-friction', type=float, default=None,
                    help='--plant: 차의 실제 마찰. 없으면 MPC 와 같음')
    ap.add_argument('--plant', action='store_true',
                    help='다음 상태를 솔버 예측 대신 같은 모델의 RK4 적분으로 (진짜 폐루프)')
    ap.add_argument('--trace', default=None, help='--walls: 상태·입력 기록 .npy')
    ap.add_argument('--opponent', type=float, default=None, metavar='V',
                    help='--walls 와 함께: 속도 V 로 도는 가상 상대 (추월·뒤따르기 시험). --v-max 도 줄 것')
    ap.add_argument('--opp-d', type=float, default=0.0, help='가상 상대의 선에서 횡위치 m (왼쪽 +)')
    ap.add_argument('--opp-gap', type=float, default=2.0, help='가상 상대가 앞서 출발하는 거리 m')
    ap.add_argument('--hyper', default=None, metavar='NPZ',
                    help='MPC 가 HyperPM 시변 파라미터를 쓴다 (config/hyper/*.npz). 없으면 --params-yaml 상수')
    ap.add_argument('--plant-hyper', default=None, metavar='NPZ',
                    help='--plant: 차가 HyperPM 파라미터로 움직인다 (상수 MPC 를 시변 차에 시험)')
    ap.add_argument('--apply-bad', action='store_true',
                    help='status≠0 해도 그대로 적용 (예전 동작). 기본은 노드처럼 버리고 정지 명령')
    ap.add_argument('--ws-acc-max', type=float, default=0.0,
                    help='MPC 바퀴 가속도 한계 m/s² (0 = 끔). tools/actuator_id.py')
    ap.add_argument('--ws-dec-max', type=float, default=0.0, help='MPC 바퀴 감속도 한계 m/s² (0 = 끔)')
    ap.add_argument('--plant-ws-tau', type=float, default=None,
                    help='--plant: 차의 바퀴 시정수 (실제처럼). 없으면 MPC 와 같음')
    ap.add_argument('--plant-ws-acc', type=float, default=None, help='--plant: 차의 바퀴 가속도 한계')
    ap.add_argument('--plant-ws-dec', type=float, default=None, help='--plant: 차의 바퀴 감속도 한계')
    ap.add_argument('--steering-tau', type=float, default=0.0436734693877551)
    ap.add_argument('--wheel-speed-tau', type=float, default=0.046122448979591835)
    ap.add_argument('--scenario', nargs='+',
                    choices=('straight', 'curve', 'track', 'zero', 'zero-guess'),
                    default=('straight', 'curve', 'track'))
    ap.add_argument('--track', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '../lab_map/data/tracks/s_track/prep_s_track.csv'))
    ap.add_argument('--build-only', action='store_true', help='빌드만 하고 끝낸다')
    ap.add_argument('--build-dir', default=os.path.expanduser('~/hmpc_build'),
                    help='생성 코드 위치. 있으면 다시 빌드하지 않는다')
    args = ap.parse_args()

    if not os.path.isdir(args.hypermpc):
        print(f'✗ hypermpc_code 를 찾을 수 없습니다: {args.hypermpc}')
        print('  git clone https://github.com/hyper-mpc/hypermpc_code.git ~/hypermpc_code')
        return 1

    cfg = make_cfg(args.cost, args.rk_steps, args.steering_tau, args.wheel_speed_tau,
                   track_hard=args.track_hard is not None,
                   track_width=args.track_hard if args.track_hard is not None else 0.25,
                   track_slack=args.track_slack,
                   ws_acc_max=args.ws_acc_max, ws_dec_max=args.ws_dec_max)

    print(f'\n■ 정식화  N={args.N}  dt={args.dt}  예측구간 {args.N*args.dt:.2f}초  '
          f'비용 {args.cost}  0/0 방어 {"끔" if args.no_guard else "켬"}  RK4 {args.rk_steps}회')
    print(f'  ★ 타이어 계수는 아직 저자 기본값이다 (질량 {MASS}, I_z, lr 만 우리 차). '
          '동정값이 나오면 다시 잴 것.')
    dyn_fun, _ = load_naive_dynamics(args.hypermpc, guard=not args.no_guard)

    solver = make_solver(dyn_fun, args.N, args.dt, cfg, args.build_dir, not args.no_guard)

    if args.build_only:
        return 0
    if args.v_max is not None:
        for k in range(args.N):
            solver.constraints_set(k, 'lbu', np.array([cfg['min_ws_ref'], -args.max_steer]))
            solver.constraints_set(k, 'ubu', np.array([args.v_max, args.max_steer]))

    params = load_params(args.params_yaml) if args.params_yaml else default_params()
    kappa_track = None
    for name in args.scenario:
        x0 = initial_state(cfg)
        x0[3], x0[7] = args.start_v, args.start_v + 0.02
        kappa = lambda q: 0.0  # noqa: E731
        if name == 'curve':
            kappa = lambda q: 0.575  # noqa: E731
        elif name == 'track':
            kappa_track = kappa_track or track_kappa(args.track)
            kappa = kappa_track
        elif name == 'zero':
            x0 = initial_state(cfg, slip=0.0)
            x0[3], x0[7] = args.start_v, args.start_v
        elif name == 'zero-guess':
            # 초기 추측을 안 깐 예전 방식. 내부 스테이지가 전부 0 이라 역시 0/0 이다.
            x0 = initial_state(cfg, slip=0.0)
            solver.reset()
            solver.set(0, 'lbx', x0)
            solver.set(0, 'ubx', x0)
            for k in range(args.N + 1):
                solver.set(k, 'p', np.concatenate([params, [0.0]]))
            st = solver.solve()
            print(f'\n■ zero-guess  status {st}  u0 {solver.get(0, "u")}')
            continue
        walls = track_walls(args.track, args.walls) if (name == 'track' and args.walls is not None) else None
        plant = (make_plant(dyn_fun, cfg, args.dt, ws_tau=args.plant_ws_tau,
                            ws_acc=args.plant_ws_acc, ws_dec=args.plant_ws_dec)
                 if args.plant else None)
        if args.friction is not None:
            x0[6] = args.friction
        opp = (make_opponent(args.track, args.opponent, args.opp_d, args.opp_gap)
               if (walls is not None and args.opponent is not None) else None)
        hyper = HyperStage(args.hyper, args.N, args.dt, params) if args.hyper else None
        plant_hyper = HyperStage(args.plant_hyper, args.N, args.dt, params) if args.plant_hyper else None
        run_scenario(solver, args, cfg, name, kappa, x0, args.laps, params, walls, plant,
                     plant_friction=args.plant_friction, opp=opp, hyper=hyper, plant_hyper=plant_hyper)
    return 0


if __name__ == '__main__':
    sys.exit(main())
