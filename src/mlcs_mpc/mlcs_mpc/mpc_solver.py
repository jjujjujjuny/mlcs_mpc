#!/usr/bin/env python3
"""
mpc_solver — 경로 추종 MPC 최적화기.

■ 두 개의 백엔드, 하나의 인터페이스

  solve(x0, ref) -> (u0, predicted_traj)

  backend='ilqr'   : 순수 numpy iLQR/DDP. 의존성 0.
  backend='acados' : acados SQP-RTI. 설치되어 있으면 훨씬 빠르고 제약이 엄밀.

  ★ 왜 iLQR 을 기본으로 두는가
    acados 는 Jetson 에 소스 빌드(+ C 코드 생성 + CMake)가 필요해서 설치가
    가장 잘 깨지는 부품이다. 그런데 MPC 노드의 나머지 전부 — mocap 연동,
    좌표계, 웨이포인트, 안전장치, VESC 명령 변환 — 는 솔버와 무관하게
    검증할 수 있어야 한다. 솔버 설치가 안 끝났다고 나머지를 못 돌리면
    디버깅이 한 덩어리로 뭉친다.

    iLQR 은 20Hz·N=20 에서 노트북 기준 수 ms 로 충분히 실시간이고, 제어
    결과도 (제약이 활성화되지 않는 한) acados 와 거의 같다. 먼저 이걸로
    전체 파이프라인을 맞춘 뒤 솔버만 갈아끼우는 순서를 권한다.

  ★ iLQR 이 제약을 다루는 방식 — 정직하게 적어둔다
    iLQR 은 원래 제약 없는 알고리즘이다. 여기서는
      - 입력 제약: 매 반복 clip (control-limited DDP 의 단순화)
      - 상태 제약(속도/조향): 비용의 2차 페널티
    로 근사한다. 즉 **하드 제약이 아니다.** 조향각은 clip 되므로 물리적으로
    넘을 수 없지만, 횡가속도 한계 같은 상태 제약은 페널티라 살짝 넘을 수
    있다. 한계 주행까지 갈 거면 acados 로 가야 한다.
"""

import time

import numpy as np

from mlcs_mpc.vehicle_model import (
    step_rk4, normalize_angle, NX, NU, X, Y, PSI, V, DELTA, A, DELTA_DOT)


class MPCConfig:
    """MPC 튜닝 파라미터."""

    def __init__(self,
                 N=20,              # 예측 구간 스텝 수
                 dt=0.05,           # 스텝 간격 (20Hz) → 예측 1.0초
                 # 비용 가중치
                 w_pos=10.0,        # 경로 횡오차
                 w_psi=2.0,         # 헤딩 오차
                 w_v=2.0,           # 속도 추종
                 w_a=0.05,          # 가속 입력 크기
                 w_ddelta=0.5,      # 조향 변화율 크기
                 w_terminal=20.0,   # 종단 가중 (예측 끝에서 경로 이탈 방지)
                 # 제약 페널티
                 w_vbound=100.0,
                 # iLQR 반복
                 # ★ 3 인 이유 — 측정해서 정한 값이다.
                 #   warm start 덕분에 매 스텝의 초기해가 이미 거의 수렴해
                 #   있어서, 반복을 늘려도 결과가 바뀌지 않는다. Levine
                 #   트랙 한 바퀴로 잰 값:
                 #       max_iter=8 → 24.3ms, 횡오차 평균 4.0cm
                 #       max_iter=3 →  9.9ms, 횡오차 평균 4.0cm  ← 같다
                 #   2.5배 빠른데 정확도가 같으므로 3 을 쓴다. Jetson ARM 은
                 #   이 x86 측정보다 느리므로 이 여유가 그대로 필요하다.
                 #   ⚠ 비용 가중치를 크게 바꾸거나 동역학 모델로 가면
                 #     수렴이 느려질 수 있다 — 그때 /mpc/solve_time 을 보고
                 #     다시 정하세요.
                 max_iter=3,
                 tol=1e-3):
        self.N = N
        self.dt = dt
        self.w_pos = w_pos
        self.w_psi = w_psi
        self.w_v = w_v
        self.w_a = w_a
        self.w_ddelta = w_ddelta
        self.w_terminal = w_terminal
        self.w_vbound = w_vbound
        self.max_iter = max_iter
        self.tol = tol

    @classmethod
    def from_dict(cls, d):
        known = cls().__dict__.keys()
        return cls(**{k: v for k, v in d.items() if k in known})


class MPCSolver:
    """경로 추종 MPC.

    ref 는 (N+1, 4) 배열: 각 예측 스텝의 [x, y, psi, v_ref].
    경로 생성(웨이포인트 → ref) 은 controller 쪽 책임이고, 여기서는
    주어진 ref 를 추종하는 최적화만 한다.
    """

    def __init__(self, params, cfg=None, backend='ilqr'):
        self.p = params
        self.cfg = cfg or MPCConfig()
        self.backend = backend
        self._pvec = params.as_vector()

        # warm start — 이전 해를 다음 스텝의 초기값으로 쓴다.
        # MPC 는 매 스텝 거의 같은 문제를 푼다. 이전 해를 한 칸 당겨서
        # 시작하면 반복 수가 크게 준다 (보통 8회 → 2~3회 수렴).
        self.u_prev = np.zeros((self.cfg.N, NU))

        self.last_solve_ms = 0.0
        self.last_cost = 0.0

        if backend == 'acados':
            self._init_acados()

    # ────────────────────────────────────────────────────────────────
    # acados 백엔드
    # ────────────────────────────────────────────────────────────────
    def _init_acados(self):
        try:
            from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
            import casadi as ca
        except ImportError as e:
            raise ImportError(
                f"backend='acados' 인데 acados_template/casadi 를 못 찾았습니다 ({e}).\n"
                "  설치: docs/SETUP_JETSON.md 의 acados 절 참고\n"
                "  또는 backend='ilqr' 로 두면 의존성 없이 동작합니다.") from e
        # 실제 acados OCP 구성은 설치 환경에서 검증이 필요하다.
        # 지금은 미구현임을 명시적으로 알린다 — 조용히 iLQR 로 떨어지면
        # "acados 로 돌고 있다" 고 착각한 채 성능을 비교하게 된다.
        raise NotImplementedError(
            "acados 백엔드는 아직 구성되지 않았습니다.\n"
            "  운동학 모델로 실차 검증 → 파라미터 동정 후 붙이는 것이 순서입니다.\n"
            "  docs/ROADMAP.md 2단계 참고. 지금은 backend='ilqr' 을 쓰세요.")

    # ────────────────────────────────────────────────────────────────
    # 비용
    # ────────────────────────────────────────────────────────────────
    def _stage_cost(self, x, u, r, terminal=False):
        c = self.cfg
        w = c.w_terminal if terminal else c.w_pos

        dx = x[X] - r[0]
        dy = x[Y] - r[1]
        dpsi = normalize_angle(x[PSI] - r[2])
        dv = x[V] - r[3]

        cost = w * (dx * dx + dy * dy) + c.w_psi * dpsi * dpsi + c.w_v * dv * dv

        # 속도 상·하한 페널티 (소프트 제약)
        over = max(0.0, x[V] - self.p.max_speed)
        under = max(0.0, self.p.min_speed - x[V])
        cost += c.w_vbound * (over * over + under * under)

        if not terminal:
            cost += c.w_a * u[A] ** 2 + c.w_ddelta * u[DELTA_DOT] ** 2
        return cost

    def _rollout(self, x0, useq):
        """입력 시퀀스로 궤적을 굴리고 총 비용을 낸다."""
        N = self.cfg.N
        xs = np.zeros((N + 1, NX))
        xs[0] = x0
        for k in range(N):
            xs[k + 1] = step_rk4(xs[k], useq[k], self._pvec, self.cfg.dt)
            # 조향각은 물리적으로 넘을 수 없다 — 상태에도 반영
            xs[k + 1][DELTA] = np.clip(
                xs[k + 1][DELTA], -self.p.max_steer, self.p.max_steer)
        return xs

    def _total_cost(self, xs, useq, ref):
        N = self.cfg.N
        c = sum(self._stage_cost(xs[k], useq[k], ref[k]) for k in range(N))
        c += self._stage_cost(xs[N], np.zeros(NU), ref[N], terminal=True)
        return c

    # ────────────────────────────────────────────────────────────────
    # iLQR
    # ────────────────────────────────────────────────────────────────
    def _solve_ilqr(self, x0, ref):
        cfg = self.cfg
        N = cfg.N

        # warm start: 이전 해를 한 칸 당기고 마지막은 복제
        useq = np.vstack([self.u_prev[1:], self.u_prev[-1:]])
        useq = self._clip_u(useq)

        xs = self._rollout(x0, useq)
        cost = self._total_cost(xs, useq, ref)

        for _ in range(cfg.max_iter):
            # 수치 미분으로 선형화. 5x2 짜리 작은 문제라 해석적 야코비안을
            # 손으로 쓰는 것보다 유지보수가 싸고, 모델을 바꿔도 그대로 돈다.
            improved, xs_new, useq_new, cost_new = self._ilqr_iter(
                x0, xs, useq, ref, cost)
            if not improved:
                break
            xs, useq, cost = xs_new, useq_new, cost_new
            if abs(cost) < cfg.tol:
                break

        self.u_prev = useq
        self.last_cost = float(cost)
        return useq, xs

    def _ilqr_iter(self, x0, xs, useq, ref, cost0):
        """한 번의 backward/forward pass. 개선되면 (True, ...) 를 준다."""
        cfg = self.cfg
        N = cfg.N

        # ── backward pass: 2차 근사로 피드백 게인 K, k 를 구한다
        Vx = self._grad_x(xs[N], np.zeros(NU), ref[N], terminal=True)
        Vxx = np.eye(NX) * cfg.w_terminal

        Ks = [None] * N
        ks = [None] * N

        for k in range(N - 1, -1, -1):
            xk, uk, rk = xs[k], useq[k], ref[k]

            A_j, B_j = self._linearize(xk, uk)
            lx = self._grad_x(xk, uk, rk)
            lu = self._grad_u(xk, uk, rk)
            lxx = np.eye(NX) * cfg.w_pos
            luu = np.diag([2.0 * cfg.w_a, 2.0 * cfg.w_ddelta])

            Qx = lx + A_j.T @ Vx
            Qu = lu + B_j.T @ Vx
            Qxx = lxx + A_j.T @ Vxx @ A_j
            Quu = luu + B_j.T @ Vxx @ B_j
            Qux = B_j.T @ Vxx @ A_j

            # 정칙화 — Quu 가 특이하면 게인이 발산한다
            Quu_reg = Quu + np.eye(NU) * 1e-6
            try:
                Quu_inv = np.linalg.inv(Quu_reg)
            except np.linalg.LinAlgError:
                return False, xs, useq, cost0

            K = -Quu_inv @ Qux
            kff = -Quu_inv @ Qu

            Ks[k], ks[k] = K, kff
            Vx = Qx + K.T @ Quu @ kff + K.T @ Qu + Qux.T @ kff
            Vxx = Qxx + K.T @ Quu @ K + K.T @ Qux + Qux.T @ K
            Vxx = 0.5 * (Vxx + Vxx.T)   # 대칭 유지 (수치오차 누적 방지)

        # ── forward pass: line search 로 실제 개선되는 步幅 을 찾는다
        for alpha in (1.0, 0.5, 0.25, 0.1, 0.01):
            xs_new = np.zeros_like(xs)
            us_new = np.zeros_like(useq)
            xs_new[0] = x0
            for k in range(N):
                dx = xs_new[k] - xs[k]
                us_new[k] = useq[k] + alpha * ks[k] + Ks[k] @ dx
                us_new[k] = self._clip_u1(us_new[k])
                xs_new[k + 1] = step_rk4(
                    xs_new[k], us_new[k], self._pvec, cfg.dt)
                xs_new[k + 1][DELTA] = np.clip(
                    xs_new[k + 1][DELTA], -self.p.max_steer, self.p.max_steer)

            cost_new = self._total_cost(xs_new, us_new, ref)
            if cost_new < cost0:
                return True, xs_new, us_new, cost_new

        return False, xs, useq, cost0

    def _linearize(self, x, u):
        """해석적 야코비안 A = df/dx, B = df/du (오일러 이산화 기준).

        ★ 왜 해석적으로 쓰는가 — 수치미분은 20Hz 를 못 맞춘다
          수치 야코비안은 스테이지마다 step_rk4 를 7번 부른다 (NX+NU+1).
          N=20, iLQR 8회 반복이면 한 solve 에 RK4 호출이 1100번 넘고,
          프로파일 결과 solve 당 9~65ms 가 나왔다 — x86 노트북에서 그렇고
          Jetson ARM 은 더 느리다. 20Hz 주기는 50ms 다.

          그런데 운동학 자전거 모델의 야코비안은 손으로 쓸 수 있을 만큼
          간단하다. 아래가 전부다. 이걸로 RK4 호출이 0 이 된다.

        ★ 왜 RK4 가 아니라 오일러 야코비안인가
          rollout(순방향 예측)은 정확도 때문에 RK4 를 쓰지만, 야코비안은
          오일러 근사로 충분하다. iLQR 에서 야코비안은 **탐색 방향**을
          정할 뿐이고, 비용은 매번 RK4 로 굴린 실제 궤적에서 평가한다
          (line search 가 진짜 비용이 줄었을 때만 받아들인다). 방향이 조금
          부정확하면 반복이 한 번 더 들 뿐, 답이 틀어지지는 않는다.
        """
        dt = self.cfg.dt
        wb = self._pvec[0]
        psi, v, delta = x[PSI], x[V], x[DELTA]
        sp, cp = np.sin(psi), np.cos(psi)
        td = np.tan(delta)
        sec2 = 1.0 / np.cos(delta) ** 2   # d(tan)/d(delta)

        A_j = np.eye(NX)
        # d(xdot)/d.  —  X' = v cos psi
        A_j[X, PSI] = -v * sp * dt
        A_j[X, V] = cp * dt
        # Y' = v sin psi
        A_j[Y, PSI] = v * cp * dt
        A_j[Y, V] = sp * dt
        # psi' = v tan(delta)/L
        A_j[PSI, V] = td / wb * dt
        A_j[PSI, DELTA] = v * sec2 / wb * dt

        B_j = np.zeros((NX, NU))
        B_j[V, A] = dt              # v' = a
        B_j[DELTA, DELTA_DOT] = dt  # delta' = delta_dot
        return A_j, B_j

    def _grad_x(self, x, u, r, terminal=False):
        c = self.cfg
        w = c.w_terminal if terminal else c.w_pos
        g = np.zeros(NX)
        g[X] = 2.0 * w * (x[X] - r[0])
        g[Y] = 2.0 * w * (x[Y] - r[1])
        g[PSI] = 2.0 * c.w_psi * normalize_angle(x[PSI] - r[2])
        g[V] = 2.0 * c.w_v * (x[V] - r[3])
        over = max(0.0, x[V] - self.p.max_speed)
        under = max(0.0, self.p.min_speed - x[V])
        g[V] += 2.0 * c.w_vbound * (over - under)
        return g

    def _grad_u(self, x, u, r):
        c = self.cfg
        return np.array([2.0 * c.w_a * u[A], 2.0 * c.w_ddelta * u[DELTA_DOT]])

    def _clip_u1(self, u):
        p, c = self.p, self.cfg
        return np.array([
            np.clip(u[A], -p.max_decel, p.max_accel),
            np.clip(u[DELTA_DOT], -p.max_steer_rate, p.max_steer_rate),
        ])

    def _clip_u(self, useq):
        return np.array([self._clip_u1(u) for u in useq])

    # ────────────────────────────────────────────────────────────────
    # 공개 인터페이스
    # ────────────────────────────────────────────────────────────────
    def solve(self, x0, ref):
        """현재 상태 x0 와 참조 궤적 ref 로 최적 입력을 푼다.

        반환: (u0, xs)
          u0 : 지금 당장 적용할 입력 [a, delta_dot]
          xs : 예측 궤적 (N+1, NX) — RViz 시각화·디버깅용
        """
        t0 = time.perf_counter()
        x0 = np.asarray(x0, dtype=float)
        ref = np.asarray(ref, dtype=float)

        if ref.shape[0] < self.cfg.N + 1:
            raise ValueError(
                f"ref 길이가 {ref.shape[0]} 인데 N+1={self.cfg.N + 1} 이 필요합니다")

        useq, xs = self._solve_ilqr(x0, ref)
        self.last_solve_ms = (time.perf_counter() - t0) * 1000.0
        return useq[0].copy(), xs

    def reset(self):
        """정지/재출발 시 warm start 를 비운다.

        멈춰 있는 동안 쌓인 이전 해를 그대로 쓰면 재출발 첫 스텝에 엉뚱한
        입력이 나간다.
        """
        self.u_prev = np.zeros((self.cfg.N, NU))
