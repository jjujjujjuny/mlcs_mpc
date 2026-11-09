"""
acados_ocp — MPCC OCP (Frenet 컨투어링) 를 세운다. 빌드·옵션은 acados_solver.py.

    x = [ s, n, mu,            Frenet 포즈 (진행, 횡편차, 헤딩편차)
          vx, vy, r, friction, 동역학 (env/vehicle_model.py — 시뮬레이터와 같은 식)
          wheel_speed, delta ] 액추에이터 (1차 지연)     → 9
    u = [ wheel_speed_ref, delta_ref ]                   → 2
    p = 동역학 파라미터 30 (차량 10 + 앞·뒤 타이어 10) → 매 풀이마다 호라이즌의 스테이지 k = 0..N 마다
        밖에서 넣는다 (worker: solver.set(k, 'p', P[k])). 상수면 모든 스테이지가 같은 값이고, 학습 모델
        (env/mpc/param_model) 이면 스테이지별로 다르다.

■ 곡률은 파라미터가 아니다 — 트랙의 κ(s) 를 B-스플라인으로 생성 코드에 넣고, OCP 가 각 스테이지의
  상태 s 로 스스로 찾는다 (s 는 랩을 넘어 늘어나므로 트랙 길이로 감는다). 그래서 트랙마다 따로 빌드한다
  (acados_solver 가 트랙별로 저장해 두고 다시 쓴다).

■ 비용 (config.cost) — cost_terms() 가 성능 항과 정규화 항으로 나눠 만든다
    mpcc      성능: 진행 −s, 기준 라인 횡방향 n², 헤딩 mu²   정규화: 입력 변화, 슬립각, 슬립률, (배리어)
              항은 HyperMPC 논문 비용과 같다 (가중치는 config/mpcc.yaml)
    tracking  성능: 진행 ṡ, 횡방향, 목표 속도   정규화: 조향 입력 (예전 간이 비용)
  경계: track_hard 면 |n| 상태 경계 (스테이지별 값은 실행 중에 — bounds.py), 아니면 배리어 비용.
"""
import casadi as cs
import numpy as np
from acados_template import AcadosModel, AcadosOcp

NX, NU, NP = 9, 2, 30


def kappa_function(s_grid, kappa_grid):
    """닫힌 트랙 κ(s) — s 를 트랙 길이로 감아 B-스플라인으로. OCP 와 오프라인 '차' 가 같은 것을 쓴다."""
    S = np.asarray(s_grid, dtype=float)
    K = np.asarray(kappa_grid, dtype=float)
    spline = cs.interpolant('kappa_ref', 'bspline', [S], K)
    L = float(S[-1])
    s = cs.MX.sym('s')
    return cs.Function('kappa', [s], [spline(s - L * cs.floor(s / L))])


def cost_terms(x, u, kappa, cfg):
    """스테이지 비용의 항들 → [(이름, 묶음, 식, 끝 비용에도 넣나)]. OCP 와 진단 (term_function) 이 같은 것을 쓴다.

    묶음
      perf  성능 — 무엇을 잘하게 할지 (진행, 기준 라인 횡방향·헤딩 오차, 목표 속도)
      reg   정규화 — 상태·입력이 무리 없이 (입력 변화, 슬립각, 슬립률, 트랙 배리어)
    기준 라인 = 트랙 파일의 선 (n, mu 를 재는 선; env/track/ref_*.csv)
    """
    s, n, mu, vx, vy, r, fric, ws, delta = (x[i] for i in range(NX))
    ws_ref, delta_ref = u[0], u[1]
    if cfg.cost == 'mpcc':
        t = [('progress', 'perf', -cfg.q_progress * s, True),               # −s: 멀리 갈수록 좋다
             ('lateral', 'perf', cfg.q_n * n ** 2, True),                    # 기준 라인 횡방향 오차
             ('heading', 'perf', cfg.q_mu * mu ** 2, True),                  # 기준 라인 헤딩 오차
             ('steer_input', 'reg', cfg.q_delta * (delta_ref - delta) ** 2, False),
             ('speed_input', 'reg', cfg.q_speed_diff * (ws_ref - ws) ** 2, False),
             ('slip_angle', 'reg', cfg.q_beta * (cs.atan(delta * 0.15 / 0.33) - cs.atan(vy / vx)) ** 2, True),
             ('slip_ratio', 'reg', cfg.q_slip * (ws - vx) ** 2, True)]
        if not cfg.track_hard:
            def barrier(h):
                return cs.log(1 + cs.exp(-cfg.barrier_lambda * h))
            t.append(('barrier', 'reg', cfg.q_barrier * (barrier(cfg.track_width - n) + barrier(cfg.track_width + n)), True))
    elif cfg.cost == 'tracking':
        s_dot = (vx * cs.cos(mu) - vy * cs.sin(mu)) / (1.0 - n * kappa)
        t = [('progress', 'perf', -cfg.q_progress * s_dot, False),
             ('lateral', 'perf', cfg.q_n * n ** 2, True),
             ('heading', 'perf', cfg.q_mu * mu ** 2, False),
             ('speed', 'perf', cfg.q_speed_diff * (vx - cfg.v_target) ** 2, False),
             ('steer_input', 'reg', cfg.q_delta * delta_ref ** 2, False)]
    else:
        raise ValueError(f'모르는 cost: {cfg.cost}')
    return t


def term_function(track_grid, cfg):
    """진단용 — (x, u) → 항별 스테이지 비용. 이름·묶음 목록과 함께."""
    x, u = cs.MX.sym('x', NX), cs.MX.sym('u', NU)
    terms = cost_terms(x, u, kappa_function(*track_grid)(x[0]), cfg)
    return [n for n, _, _, _ in terms], [g for _, g, _, _ in terms], \
        cs.Function('cost_terms', [x, u], [cs.vertcat(*[e for _, _, e, _ in terms])])


def build_ocp(model, track_grid, cfg, name='mpcc_car'):
    """model = VehicleModel (질량·축거·시정수가 생성 코드에 들어간다), track_grid = Track.kappa_grid(),
    cfg = MpccConfig (config/mpcc.yaml)."""
    N, dt = cfg.N, cfg.dt
    # ── 상태·입력·파라미터 ─────────────────────────────────────────────
    s, n, mu = cs.MX.sym('s'), cs.MX.sym('n'), cs.MX.sym('mu')
    vx, vy, r = cs.MX.sym('vx'), cs.MX.sym('vy'), cs.MX.sym('r')
    fric, ws, delta = cs.MX.sym('friction'), cs.MX.sym('wheel_speed'), cs.MX.sym('delta')
    x = cs.vertcat(s, n, mu, vx, vy, r, fric, ws, delta)
    ws_ref, delta_ref = cs.MX.sym('wheel_speed_ref'), cs.MX.sym('delta_ref')
    u = cs.vertcat(ws_ref, delta_ref)
    p = cs.MX.sym('p', NP)

    # ── 동역학 — 곡률은 트랙에서 ──────────────────────────────────────
    kappa = kappa_function(*track_grid)(s)
    f_expl = model.frenet(x, u, p, kappa)

    xdot = cs.MX.sym('xdot', NX)
    am = AcadosModel()
    am.name = name      # 생성 라이브러리 이름 (libacados_ocp_solver_<name>.so)
    am.x, am.u, am.xdot, am.p = x, u, xdot, p
    am.f_expl_expr = f_expl
    am.f_impl_expr = xdot - f_expl

    ocp = AcadosOcp()
    ocp.model = am
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * dt

    # ── 비용 — cost_terms() 의 항을 모두 더한다 (성능 + 정규화) ─────────
    terms = cost_terms(x, u, kappa, cfg)
    stage = sum(e for _, _, e, _ in terms)
    terminal = cfg.q_e_scaler * (sum(e for _, _, e, term in terms if term) + cfg.q_mu_e * mu ** 2)
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    am.cost_expr_ext_cost = stage
    am.cost_expr_ext_cost_e = terminal

    # ── 경계 ───────────────────────────────────────────────────────────
    ocp.constraints.x0 = np.zeros(NX)
    ocp.constraints.lbu = np.array([cfg.v_min, -cfg.build_steer])
    ocp.constraints.ubu = np.array([cfg.build_v_max, +cfg.build_steer])
    ocp.constraints.idxbu = np.array([0, 1])
    if cfg.track_hard:
        # ★ 트랙 경계를 비용이 아니라 상태 경계로. 폭은 실행 중에 스테이지별로 바꾼다 (bounds.py).
        w = cfg.track_width
        ocp.constraints.idxbx = np.array([1])
        ocp.constraints.lbx = np.array([-w])
        ocp.constraints.ubx = np.array([+w])
        ocp.constraints.idxbx_e = np.array([1])
        ocp.constraints.lbx_e = np.array([-w])
        ocp.constraints.ubx_e = np.array([+w])
        if cfg.track_slack > 0:
            # ★ 순수 하드 경계는 차가 폭 안에 못 머무는 순간(한계 코너, 외란) QP 해가 없어져
            #   status 4 가 된다 (1108_3, ±0.12 에서 94%). 슬랙에 큰 L1 무게를 걸면 지킬 수
            #   있을 때는 하드와 똑같이 지키고, 못 지킬 때만 최소로 넘는다 (정확 페널티).
            z = cfg.track_slack
            ocp.constraints.idxsbx = np.array([0])
            ocp.constraints.idxsbx_e = np.array([0])
            ocp.cost.zl = ocp.cost.zu = np.array([z])
            ocp.cost.Zl = ocp.cost.Zu = np.array([z * 0.1])
            ocp.cost.zl_e = ocp.cost.zu_e = np.array([z])
            ocp.cost.Zl_e = ocp.cost.Zu_e = np.array([z * 0.1])

    if cfg.ws_acc_max > 0 or cfg.ws_dec_max > 0:
        # ★ 바퀴 가속도 한계 (2026-11-08). tau 가 작으면 모델은 바퀴가 바로 명령을 따라가서
        #   언제든 설 수 있다고 믿는다. 실제는 감속 ~1.5 m/s² (tools/analysis/actuator_id.py).
        #   tau 는 두고 ws' = (ws_ref − ws)/tau 를 [−ws_dec_max, ws_acc_max] 로 묶는다:
        #       −a_dec·tau ≤ ws_ref − ws ≤ a_acc·tau        (스테이지 0..N-1, 선형)
        #   현재 ws 가 v_max 보다 한참 높으면 ubu 와 겹쳐 해가 없어질 수 있어 슬랙을 둔다.
        tau = model.wheel_speed_tau
        big = 1e3
        a_acc = cfg.ws_acc_max if cfg.ws_acc_max > 0 else big
        a_dec = cfg.ws_dec_max if cfg.ws_dec_max > 0 else big
        C = np.zeros((1, NX))
        C[0, 7] = -1.0
        ocp.constraints.C = C
        ocp.constraints.D = np.array([[1.0, 0.0]])
        ocp.constraints.lg = np.array([-a_dec * tau])
        ocp.constraints.ug = np.array([+a_acc * tau])
        zg = cfg.ws_rate_slack
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

    ocp.parameter_values = np.asarray(model.p, dtype=float)
    return ocp
