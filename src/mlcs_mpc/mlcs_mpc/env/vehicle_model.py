"""
vehicle_model — 차 동역학. MPC 와 시뮬레이터가 같은 식을 쓴다.

  MPC     mpc/ocp.py 가 frenet() 을 casadi 기호로 받아 OCP 를 세운다 (acados 생성 코드).
  시뮬    env/simulator.py 가 cartesian() 을 casadi Function 으로 만들어 숫자로 적분한다.
  학습    파라미터 p[30] 을 스테이지마다 바꿔 넣으면 시변 모델이 된다 (learned/ 의 param model).

■ 식 — Pacejka 단일 궤적 + 액추에이터 1차 지연

  상태 (몸체)  vx, vy, r          무게중심 차체 좌표
  액추에이터   ws (바퀴 속도 m/s), delta (실제 조향 rad)   ← 입력 ws_ref, delta_ref 를 1차 지연으로 따라간다
  파라미터 p[30] = 차량 10 + 앞타이어 10 + 뒤타이어 10 (config/mpcc.yaml 의 vehicle_params, tire_front, tire_rear)
      차량  I_z, lr, Cd0, Cd2, Cd1, I_e, K_fi, b1, b0, R     (I_e 뒤 다섯은 이 식에서 쓰지 않는다)
      타이어 Sx_p, Alpha_p, By, Cy, Dy, Ey, Bx, Cx, Dx, Ex
  마찰 friction 은 타이어 힘에 곱한다. OCP 에서는 상태 (변하지 않음) 다.

  식은 HyperMPC 원저자의 casadi_car_model_naive.py 와 같다 (test/test_vehicle_model.py 가 대조한다).
  원저자 식은 질량 5.1 이 박혀 있어 I_z·Cd 를 환산해 넘겼는데, 여기서는 실제 질량으로 쓴다 — 같은 식이다
  (타이어 힘/질량 에서 질량이 약분되고, 남는 곳은 항력/m 과 힘·팔/I_z 뿐이다).
  슬립각 뒤 바퀴는 원저자 식대로 (vy − lf·r) 이다. 동정 값이 이 식에 맞춰져 있다.

■ 숫자 방어 — 합성 슬립 sqrt(Sx² + α²) 로 나누므로 슬립 0 에서 0/0 이다. slip_eps 를 더한다 (0 이면 끔).
"""
import casadi as cs
import numpy as np

G = 9.81
SLIP_EPS = 1e-12
DIV_EPS = 1e-6      # 슬립률·슬립각 분모 (원저자 식과 같다)
NP = 30
PARAM_KEYS = ('mass', 'wheelbase', 'friction', 'vehicle_params', 'tire_front', 'tire_rear',
              'steering_tau', 'wheel_speed_tau')


def default_params():
    """p[30] 의 기본값 — 원저자 single_track_params.py / pacejka_params.py, I_z·lr 만 우리 차."""
    vehicle = [0.089, 0.1537, 0.01, 0.01, 0.01, 0.2, 0.90064745, 0.304115174, 0.50421894, 0.05]
    tire = [0.05, 2.0, 0.35, 1.4, 1.0, 1.2, 30.0, 1.3, 1.0, 0.5]
    return np.array(vehicle + tire + tire, dtype=float)


class VehicleModel:

    def __init__(self, mass=3.284, wheelbase=0.33, friction=0.41, vehicle_params=None,
                 tire_front=None, tire_rear=None, steering_tau=0.0436734693877551,
                 wheel_speed_tau=0.046122448979591835, slip_eps=SLIP_EPS):
        self.mass, self.L, self.friction = float(mass), float(wheelbase), float(friction)
        p = default_params()
        if vehicle_params is not None:
            p[0:10] = vehicle_params
        if tire_front is not None:
            p[10:20] = tire_front
        if tire_rear is not None:
            p[20:30] = tire_rear
        self.p = p
        self.steering_tau, self.wheel_speed_tau = float(steering_tau), float(wheel_speed_tau)
        self.slip_eps = float(slip_eps)
        self._fun = {}

    @classmethod
    def from_yaml(cls, *paths, **over):
        """config/mpcc.yaml (/**: ros__parameters) 에서. 뒤 파일이 앞을 덮는다 (race.yaml 의 덮어쓰기처럼)."""
        import yaml
        d = {}
        for path in paths:
            if path:
                y = yaml.safe_load(open(path, encoding='utf-8')) or {}
                d.update((y.get('/**') or next(iter(y.values()), {})).get('ros__parameters', {}))
        kw = {k: d[k] for k in PARAM_KEYS if k in d}
        kw.update(over)
        return cls(**kw)

    @property
    def lr(self):
        return float(self.p[1])

    def copy(self, **over):
        kw = dict(mass=self.mass, wheelbase=self.L, friction=self.friction, vehicle_params=self.p[0:10],
                  tire_front=self.p[10:20], tire_rear=self.p[20:30], steering_tau=self.steering_tau,
                  wheel_speed_tau=self.wheel_speed_tau, slip_eps=self.slip_eps)
        kw.update(over)
        return VehicleModel(**kw)

    # ── 식 (casadi 기호·숫자 모두) ─────────────────────────────────────
    def tire(self, alpha, sigma, t):
        """(Fx, Fy) 정규화 힘. t = 타이어 10개."""
        sx_p, a_p, by, cy, dy, ey, bx, cx, dx, ex = (t[i] for i in range(10))
        sx_n = sigma / sx_p
        a_n = alpha * 180.0 / cs.pi / a_p
        s = cs.sqrt(sx_n ** 2 + a_n ** 2 + self.slip_eps) if self.slip_eps > 0 else cs.sqrt(sx_n ** 2 + a_n ** 2)
        sx_m, a_m = s * sx_p, s * a_p
        fy = (a_n / s) * dy * cs.sin(cy * cs.atan(by * a_m + ey * (by * a_m - cs.atan(by * a_m))))
        fx = (sx_n / s) * dx * cs.sin(cx * cs.atan(bx * sx_m + ex * (bx * sx_m - cs.atan(bx * sx_m))))
        return fx, -fy

    def body(self, vx, vy, r, friction, ws, delta, p):
        """(vx, vy, r)' — 동적 모델."""
        m, L = self.mass, self.L
        I_z, lr, cd0, cd2, cd1 = p[0], p[1], p[2], p[3], p[4]
        lf = L - lr
        f_drag = cd0 * cs.sign(vx) + cd1 * vx + cd2 * vx * vx
        a_f = cs.atan((vy + lf * r) / (vx + DIV_EPS)) - delta
        a_r = cs.atan((vy - lf * r) / (vx + DIV_EPS))
        v_f = vx * cs.cos(delta) + (vy + r * lr) * cs.sin(delta)
        s_f = (ws - v_f) / (v_f + DIV_EPS)
        s_r = (ws - vx) / (vx + DIV_EPS)
        fx_f, fy_f = self.tire(a_f, s_f, [p[10 + i] for i in range(10)])
        fx_r, fy_r = self.tire(a_r, s_r, [p[20 + i] for i in range(10)])
        fz_f, fz_r = m * G * lr / L, m * G * lf / L
        fxf, fyf = fz_f * fx_f * friction, fz_f * fy_f * friction
        fxr, fyr = fz_r * fx_r * friction, fz_r * fy_r * friction
        sd, cd = cs.sin(delta), cs.cos(delta)
        vx_d = (fxr + fxf * cd - fyf * sd - f_drag + m * vy * r) / m
        vy_d = (fxf * sd + fyr + fyf * cd - m * vx * r) / m
        r_d = ((fxf * sd + fyf * cd) * lf - fyr * lr) / I_z
        return vx_d, vy_d, r_d

    def body_kinematic(self, vx, vy, r, ws, delta, p, tau=0.05):
        """저속용 (시뮬레이터). 미끄럼 없는 자전거 — vx 는 바퀴를, vy·r 은 조향이 정하는 값을 tau 로 따라간다."""
        lr, tan_d = p[1], cs.tan(delta)
        return (ws - vx) / tau, (vx * tan_d * lr / self.L - vy) / tau, (vx * tan_d / self.L - r) / tau

    def actuators(self, ws, delta, ws_ref, delta_ref):
        return (ws_ref - ws) / self.wheel_speed_tau, (delta_ref - delta) / self.steering_tau

    def frenet(self, x, u, p, kappa):
        """OCP 모델. x = [s, n, mu, vx, vy, r, friction, ws, delta], u = [ws_ref, delta_ref]."""
        s, n, mu, vx, vy, r, fric, ws, delta = (x[i] for i in range(9))
        vx_d, vy_d, r_d = self.body(vx, vy, r, fric, ws, delta, p)
        s_dot = (vx * cs.cos(mu) - vy * cs.sin(mu)) / (1.0 - n * kappa)
        ws_d, d_d = self.actuators(ws, delta, u[0], u[1])
        return cs.vertcat(s_dot, vx * cs.sin(mu) + vy * cs.cos(mu), r - kappa * s_dot,
                          vx_d, vy_d, r_d, 0.0, ws_d, d_d)

    def cartesian(self, x, u, p, v_lo=0.0, v_hi=0.0):
        """시뮬레이터 모델. x = [X, Y, yaw, vx, vy, r, ws, delta] (X, Y 무게중심), u = [ws_ref, delta_ref].

        v_lo < v_hi 면 |vx| 가 v_lo~v_hi 사이에서 운동학 모델과 섞는다 (원래 식은 vx → 0 에서 아주 딱딱하다).
        """
        X, Y, yaw, vx, vy, r, ws, delta = (x[i] for i in range(8))
        dyn = self.body(vx, vy, r, self.friction, ws, delta, p)
        if v_hi > v_lo:
            kin = self.body_kinematic(vx, vy, r, ws, delta, p)
            w = cs.fmin(cs.fmax((cs.fabs(vx) - v_lo) / (v_hi - v_lo), 0.0), 1.0)
            dyn = tuple(w * a + (1 - w) * b for a, b in zip(dyn, kin))
        ws_d, d_d = self.actuators(ws, delta, u[0], u[1])
        c, s = cs.cos(yaw), cs.sin(yaw)
        return cs.vertcat(vx * c - vy * s, vx * s + vy * c, r, dyn[0], dyn[1], dyn[2], ws_d, d_d)

    # ── casadi Function (숫자로 부를 때) ────────────────────────────────
    def function(self, kind, **kw):
        """kind = 'frenet' (x9, u2, p30, kappa) | 'cartesian' (x8, u2, p30) | 'body' (vx, vy, r, friction, ws, delta, p30)."""
        key = (kind, tuple(sorted(kw.items())))
        if key not in self._fun:
            p = cs.SX.sym('p', NP)
            if kind == 'frenet':
                x, u, k = cs.SX.sym('x', 9), cs.SX.sym('u', 2), cs.SX.sym('kappa')
                f = cs.Function('f_frenet', [x, u, p, k], [self.frenet(x, u, p, k)])
            elif kind == 'cartesian':
                x, u = cs.SX.sym('x', 8), cs.SX.sym('u', 2)
                f = cs.Function('f_cartesian', [x, u, p], [self.cartesian(x, u, p, **kw)])
            elif kind == 'body':
                a = [cs.SX.sym(n) for n in ('vx', 'vy', 'r', 'friction', 'ws', 'delta')]
                f = cs.Function('f_body', a + [p], [cs.vertcat(*self.body(*a, p))])
            else:
                raise ValueError(kind)
            self._fun[key] = f
        return self._fun[key]
