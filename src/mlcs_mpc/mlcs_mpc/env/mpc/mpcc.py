"""
mpcc — MPCC 솔버를 한 주기씩 푸는 쪽. worker.py (mpcc_node 용 프로세스) 와 main.py (튜닝용) 가 같이 쓴다.

  MPCC             OCP 빌드 (트랙 곡률 내장) + solve(x, 벽, 속도 상한) — 낮은 수준
  MpccController   mpcc_node 와 같은 한 주기 계산 (뒤축 상태 → OCP x, 예측 s 로 벽, 해 → /drive 명령)
                   ROS 없이 시뮬레이터와 바로 붙인다 (main.py)
"""
import time

import numpy as np

from mlcs_mpc.env.config import MpccConfig
from mlcs_mpc.env.mpc import param_model as pmod
from mlcs_mpc.env.mpc.acados_ocp import NP, term_function
from mlcs_mpc.env.mpc.acados_solver import make_solver
from mlcs_mpc.env.mpc.bounds import restart, set_hard_bounds, set_input_bounds, set_wall_bounds
from mlcs_mpc.env.mpc.core import make_state, solution_ok, to_command


class MPCC:

    def __init__(self, model, track, cfg, build_dir, param_model=None, log=print, diagnose=False):
        self.model, self.track, self.cfg = model, track, cfg
        self.N = cfg.N
        self.solver = make_solver(model, track.kappa_grid(), cfg, build_dir, log=log)
        self.params = np.asarray(model.p, dtype=float).copy()
        self.pm = param_model or pmod.ConstantParams(self.params, self.N)
        set_input_bounds(self.solver, self.N, cfg)
        self.ubu_k = np.full(self.N, np.inf)    # 지금 스테이지별로 걸려 있는 속도 상한 (바뀔 때만 다시 넣는다)
        self.pm_ms, self.pm_w = 0.0, 0.0
        # 진단: 예측 구간 항별 비용 (튜닝용 main 만 — 워커는 끈다)
        self.terms = None
        if diagnose:
            names, groups, fn = term_function(track.kappa_grid(), cfg)
            self.terms = (names, groups, fn.map(self.N))

    def stage_params_in_solver(self):
        """솔버에 실제로 들어 있는 스테이지별 p 를 다시 읽는다 → (N+1, 30). 넣은 값이 그대로인지 확인용."""
        return np.array([self.solver.get(k, 'p') for k in range(self.N + 1)])

    def warmup(self, n=3):
        """첫 풀이는 메모리를 처음 건드려 느리다. 슬립이 0 이 아닌 직진으로 몇 번 푼다."""
        x = np.zeros(9)
        x[3], x[6], x[7] = 1.0, self.model.friction, 1.02
        for _ in range(n):
            restart(self.solver, self.N, x, self.cfg)
            for k in range(self.N + 1):
                self.solver.set(k, 'p', self.params)
            self.solver.set(0, 'lbx', x)
            self.solver.set(0, 'ubx', x)
            if self.cfg.track_hard:
                set_hard_bounds(self.solver, self.N, self.cfg.track_width, 0.0)
            self.solver.solve()
        self.ubu_k[:] = np.inf

    def solve(self, x, t=0.0, n_lo=None, n_hi=None, v_cap=None, reset=False):
        """x = OCP 상태 (9). n_lo/n_hi = 스테이지 0..N 벽, v_cap = 스테이지 0..N-1 바퀴 속도 상한.

        → dict(status, ms, u, x1, xs). reset 이면 초기 추측을 x 로 다시 깐다 (출발·비정상 해 뒤).
        """
        cfg, N, sv = self.cfg, self.N, self.solver
        x = np.array(x, dtype=float)
        x[6] = self.model.friction
        if reset:
            restart(sv, N, x, cfg)
            self.ubu_k[:] = cfg.v_max
            self.pm.clear_plan()            # 다시 깐 추측은 계획이 아니다. 관측 기록은 둔다
        cap = np.full(N, cfg.v_max) if v_cap is None else np.clip(v_cap, cfg.v_min, cfg.v_max)
        for k in np.flatnonzero(cap != self.ubu_k):
            sv.constraints_set(int(k), 'ubu', np.array([cap[k], cfg.max_steer_actual]))
            self.ubu_k[k] = cap[k]
        th = time.perf_counter()
        self.pm.observe(t, x)
        P, self.pm_w = self.pm.stage_params(t, x)
        self.pm_ms = (time.perf_counter() - th) * 1000
        P = np.asarray(P, dtype=float)
        if P.shape != (N + 1, NP):
            # 스테이지 0..N 마다 동역학 파라미터 30 개 — 한 줄이나 크기가 틀리면 받지 않는다
            raise ValueError(f'param_model {self.pm.name}: P 는 (N+1, {NP}) = ({N + 1}, {NP}) 이어야 한다 — {P.shape}')
        for k in range(N + 1):
            sv.set(k, 'p', P[k])            # 호라이즌 스테이지 k 의 동역학 파라미터
        self.last_P = P
        sv.set(0, 'lbx', x)
        sv.set(0, 'ubx', x)
        if cfg.track_hard:
            if n_lo is None:
                set_hard_bounds(sv, N, cfg.track_width, x[1])
            else:
                set_wall_bounds(sv, N, n_lo, n_hi, x[1])
        t0 = time.perf_counter()
        st = sv.solve()
        ms = (time.perf_counter() - t0) * 1000
        xs = np.array([sv.get(k, 'x') for k in range(N + 1)])
        if st == 0 and np.all(np.isfinite(xs)):
            self.pm.set_plan(t, xs)
        else:
            self.pm.clear_plan()
        self.last_xs = xs
        out = dict(status=int(st), ms=ms, u=sv.get(0, 'u'), x1=xs[1], xs=xs)
        if self.terms is not None and st == 0:
            us = np.array([sv.get(k, 'u') for k in range(N)])
            xr = xs[:N].copy()
            xr[:, 0] -= xs[0, 0]        # 진행은 지금 위치 기준 (s 는 랩을 넘어 늘어나 절댓값은 뜻이 없다)
            out['terms'] = np.array(self.terms[2](xr.T, us.T)).sum(axis=1)   # 항별 Σ_k (스테이지 0..N-1)
        return out

class MpccController:
    """mpcc_node._control_loop / _reply_cb 와 같은 계산 (안전 감시·ROS 는 빼고).

    params = config/mpcc.yaml (+ 덮어쓰기) 의 파라미터 사전 — 노드와 같은 이름.
    """

    def __init__(self, model, track, params, build_dir, log=print):
        self.cfg = MpccConfig.from_params(params)
        g = lambda k, d: params.get(k, d)  # noqa: E731
        self.track, self.model = track, model
        self.N, self.dt = self.cfg.N, self.cfg.dt
        self.v_floor = float(g('v_floor', 0.25))
        self.max_steer = float(g('max_steer', 0.40))
        self.max_dsteer = float(g('max_steer_rate', 3.2)) / float(g('control_rate', 30.0))
        self.map_cmd = list(g('steer_map_cmd', [0.0, 0.2, 0.3, 0.4]))
        self.map_act = list(g('steer_map_act', [0.0, 0.2021, 0.2972, 0.3776]))
        self.max_bad = int(g('max_bad_solves', 3))
        # 출발 — 서 있을 때는 풀지 않고 직진 (슬립 분모가 0 근처라 첫 풀이가 실패하기 쉽다). vx 가 start_vx 를
        #   넘으면 풀기 시작한다. start_speed 0 이면 끈다 (처음부터 푼다).
        self.start_speed = float(g('start_speed', 0.0))
        self.start_vx = float(g('start_vx', 0.3))
        pm = pmod.load(g('param_model', ''), self.N, self.dt, model.p,
                       v_min=float(g('param_model_v_min', 0.0)), blend=float(g('param_model_blend', 0.0)),
                       root=g('repo_root', ''))
        self.mpc = MPCC(model, track, self.cfg, build_dir, param_model=pm, log=log, diagnose=True)
        self.mpc.warmup()
        self.reset()

    def reset(self):
        self.track.reset()
        self.delta = self.cmd_steer = 0.0
        self.s_pred, self.running, self.bad = None, False, 0
        self.started = self.start_speed <= 0.0

    def step(self, pose, twist, wheel_speed, t=0.0):
        """뒤축 pose (x, y, yaw), 뒤축 twist (vx, vy, r), 바퀴 속도 → (속도 명령, 조향 명령, 정보)."""
        cfg = self.cfg
        x = make_state(self.track, pose, twist, wheel_speed, self.delta, self.model.lr,
                       self.v_floor, self.model.friction)
        if not self.started:
            if twist[0] < self.start_vx:
                # 출발 단계: 상태만 읽고 직진 (status −1 = 풀지 않음)
                return min(self.start_speed, cfg.v_max), 0.0, dict(
                    status=-1, ms=0.0, u=np.array([self.start_speed, 0.0]), x1=x.copy(), xs=None, x=x, tripped=False)
            self.started = True     # 여기서부터 푼다 (초기 추측을 지금 상태로 깐다)
        s = self.s_pred if (self.running and self.s_pred is not None) \
            else x[0] + x[3] * self.dt * np.arange(self.N + 1)
        if cfg.track_bounds == 'walls':
            n_lo, n_hi = self.track.bounds_at(s, cfg.wall_margin)
        else:
            n_lo = np.full(self.N + 1, -cfg.track_width)
            n_hi = -n_lo
        r = self.mpc.solve(x, t, n_lo, n_hi, reset=not self.running)
        r['x'] = x
        if not solution_ok(r['status'], r['u'], r['x1']):
            # 노드와 같게: 버리고 정지 명령, 다음엔 초기 추측을 다시 깐다. 노드는 max_bad 번 이어지면 래치한다.
            self.bad += 1
            self.running = False
            r['tripped'] = self.bad >= self.max_bad
            return 0.0, 0.0, r
        self.bad, self.running = 0, True
        self.s_pred = r['xs'][:, 0]
        self.delta = float(r['x1'][8])
        speed, steer = to_command(r['u'], cfg.v_max, self.max_steer, self.cmd_steer,
                                  self.max_dsteer, self.map_cmd, self.map_act)
        self.cmd_steer = steer
        r['tripped'] = False
        return speed, steer, r
