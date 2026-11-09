"""
simulator — 차 없이 컨트롤러를 시험하는 ODE 시뮬레이터 (ROS 없음). io/sim_node.py 가 ROS 로 감싼다.

  sim = Simulator(VehicleModel.from_yaml('config/mpcc.yaml'), Track('prep.csv'))
  sim.reset_to_s(0.0)
  info = sim.step(steer_cmd, speed_cmd)      # /drive 와 같은 명령 (서보 명령, 바퀴 속도 m/s)

■ 차 모델은 MPC 와 같은 VehicleModel.cartesian() 이다 (env/vehicle_model.py). step() 은 그 ODE 를
  dt 만큼 푼다 — 기본은 고정 간격 RK4 (substep), integrator 로 scipy solve_ivp 방법 (RK45, LSODA ...) 도 된다.

  명령 → 액추에이터 입력은 차와 같다:
    VESC  vesc (env/vesc.py — 차의 vesc.yaml 튜닝값) 가 있으면 속도는 ERPM 상한, 조향은 서보 한계로 자른다
    조향  서보 명령 → 실제 조향 (steer_map, mpc/core.command_to_steer) → 1차 지연 (steering_tau)
    바퀴  속도 명령 → 1차 지연 (wheel_speed_tau, 실측), ws_acc / ws_dec 가 있으면 가속도를 묶는다

  model 과 다른 차를 쓰고 싶으면 (모델 불일치 시험) model.copy(wheel_speed_tau=0.66) 처럼 넘긴다.

■ 저속 — 원래 식은 vx → 0 에서 슬립 분모가 0 이 되어 아주 딱딱하다 (0.2 m/s 에서 고유값 약 −1000 /s).
  MPC 는 v_floor 로 피하지만 시뮬은 서 있다가 출발해야 하므로 v_lo~v_hi 에서 운동학 자전거와 섞는다.

■ 상태 x = [X, Y, yaw, vx, vy, r, ws, delta] — X, Y 무게중심. pose()/twist() 는 /mpc/state 와 같은 뒤축 기준.
"""
import math

import numpy as np

from mlcs_mpc.env.mpc.core import command_to_steer


class Simulator:

    def __init__(self, model, track=None, dt=0.01, substep=0.001, integrator='rk4',
                 steer_map_cmd=(0.0, 0.2, 0.3, 0.4), steer_map_act=(0.0, 0.2021, 0.2972, 0.3776),
                 max_steer=None, ws_acc=0.0, ws_dec=0.0, v_lo=0.2, v_hi=0.4, half_width=0.135, vesc=None):
        self.model, self.track = model, track
        self.dt, self.substep, self.integrator = dt, substep, integrator
        self.map_cmd, self.map_act, self.max_steer = list(steer_map_cmd), list(steer_map_act), max_steer
        self.ws_acc = ws_acc if ws_acc > 0 else math.inf
        self.ws_dec = ws_dec if ws_dec > 0 else math.inf
        self.half_width = half_width
        self.vesc = vesc            # env/vesc.Vesc — None 이면 자르지 않는다
        self.f = model.function('cartesian', v_lo=v_lo, v_hi=v_hi)
        self.p = np.asarray(model.p, dtype=float)
        self.x = np.zeros(8)
        self.t = 0.0
        self.on_wall, self.wall_hits = False, 0
        self.lap_t0, self.lap_s0, self.laps = None, None, []

    # ── 상태 ───────────────────────────────────────────────────────────
    def reset(self, x, y, yaw):
        """뒤축 pose 로 세운다 (속도 0)."""
        lr = self.model.lr
        self.x = np.array([x + lr * math.cos(yaw), y + lr * math.sin(yaw), yaw, 0, 0, 0, 0, 0], dtype=float)
        if self.track is not None:
            self.track.reset()
        self.on_wall, self.lap_t0, self.lap_s0 = False, None, None

    def track_pose(self, s):
        """트랙 s 지점 중심선 위에 무게중심을 두는 뒤축 pose."""
        x, y = self.track.xy(s, 0.0)
        h = self.track.heading_at(s)
        lr = self.model.lr
        return float(x) - lr * math.cos(h), float(y) - lr * math.sin(h), h

    def reset_to_s(self, s=0.0):
        self.reset(*self.track_pose(s))

    def pose(self):
        X, Y, yaw = self.x[0:3]
        lr = self.model.lr
        return X - lr * math.cos(yaw), Y - lr * math.sin(yaw), yaw

    def twist(self):
        """뒤축 차체 속도. mpc/core.make_state 의 vy_cog = vy + lr·r 의 반대."""
        vx, vy, r = self.x[3:6]
        return vx, vy - self.model.lr * r, r

    @property
    def wheel_speed(self):
        return float(self.x[6])

    @property
    def steer(self):
        return float(self.x[7])

    # ── 적분 ───────────────────────────────────────────────────────────
    def _rhs(self, x, ws_ref, delta_ref):
        # 바퀴 가속도 한계: ws' = clip((ref − ws)/tau) 가 되도록 입력을 바꾼다 (mpc/offline.make_plant 와 같다)
        if self.ws_acc < math.inf or self.ws_dec < math.inf:
            tau = self.model.wheel_speed_tau
            rate = min(max((ws_ref - x[6]) / tau, -self.ws_dec), self.ws_acc)
            ws_ref = x[6] + rate * tau
        return np.array(self.f(x, [ws_ref, delta_ref], self.p)).ravel()

    def step(self, steer_cmd, speed_cmd, dt=None):
        """/drive 명령으로 dt 만큼. 반환: 상태 요약 dict (트랙이 있으면 s, n, 벽, 랩)."""
        dt = self.dt if dt is None else dt
        if self.vesc is not None:   # 차의 vesc_driver 가 하는 것처럼
            steer_cmd, speed_cmd = self.vesc.steer(steer_cmd), self.vesc.speed(speed_cmd)
        d_ref = command_to_steer(steer_cmd, self.map_cmd, self.map_act, self.max_steer)
        if self.integrator == 'rk4':
            n = max(1, int(math.ceil(dt / self.substep - 1e-9)))
            h = dt / n
            x = self.x
            for _ in range(n):
                k1 = self._rhs(x, speed_cmd, d_ref)
                k2 = self._rhs(x + 0.5 * h * k1, speed_cmd, d_ref)
                k3 = self._rhs(x + 0.5 * h * k2, speed_cmd, d_ref)
                k4 = self._rhs(x + h * k3, speed_cmd, d_ref)
                x = x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        else:
            from scipy.integrate import solve_ivp
            x = solve_ivp(lambda _t, xx: self._rhs(xx, speed_cmd, d_ref), (0.0, dt), self.x,
                          method=self.integrator, max_step=self.substep * 10).y[:, -1]
        x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi
        self.x = x
        self.t += dt
        return self._info()

    # ── 트랙 ───────────────────────────────────────────────────────────
    def _info(self):
        vx, vy, r = self.twist()
        info = dict(t=self.t, vx=vx, vy=vy, r=r, ws=self.wheel_speed, delta=self.steer)
        if self.track is None:
            return info
        s, n, _ = self.track.frenet(self.x[0], self.x[1], self.x[2])
        lo, hi = self.track.bounds_at(s, self.half_width)
        hit = not (lo <= n <= hi)
        info.update(s=s, n=n, lo=lo, hi=hi, on_wall=hit, new_hit=hit and not self.on_wall, lap=None)
        if info['new_hit']:
            self.wall_hits += 1
        self.on_wall = hit
        # 랩: 움직이기 시작한 때부터, s 가 트랙 길이만큼 늘 때마다
        if self.lap_t0 is None:
            if vx > 0.05:
                self.lap_t0, self.lap_s0 = self.t, s
        elif s - self.lap_s0 >= self.track.length:
            info['lap'] = self.t - self.lap_t0
            self.laps.append(info['lap'])
            self.lap_t0, self.lap_s0 = self.t, self.lap_s0 + self.track.length
        info.update(wall_hits=self.wall_hits, laps=len(self.laps))
        return info
