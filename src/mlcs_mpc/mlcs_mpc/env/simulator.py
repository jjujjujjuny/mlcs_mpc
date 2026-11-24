"""
simulator — 차 없이 컨트롤러를 시험하는 ODE 시뮬레이터 (ROS 없음). io/sim_node.py 가 ROS 로 감싼다.

  sim = Simulator(VehicleModel.from_yaml('config/mpcc.yaml'), Track('prep.csv'))
  sim.reset_to_s(0.0)
  info = sim.step(steer_cmd, speed_cmd)      # /drive 와 같은 명령 (서보 명령, 바퀴 속도 m/s)

■ 차 모델은 MPC 와 같은 VehicleModel.cartesian() 이다 (env/vehicle_model.py). step() 은 그 ODE 를
  dt 만큼 푼다 — 기본은 고정 간격 RK4 (substep), integrator 로 scipy solve_ivp 방법 (RK45, LSODA ...) 도 된다.

  명령은 항상 저수준 제어기를 거쳐 적분된다 (f1tenth_gym 과 같은 구조 — 상위 → 저수준 → 동역학):
    /drive (서보 명령, 속도 m/s)
    → VESC  vesc (env/vesc.py — 차의 vesc.yaml 튜닝값) 가 있으면 속도는 ERPM 상한, 조향은 서보 한계로 자른다
    → 서보 맵  서보 명령 → 실제 조향 목표 δ_ref (steer_map, mpc/core.command_to_steer)
    → 저수준  env/lowlevel.py (lowlevel=LowLevelConfig) — 명령 지연 → 조향 각속도 δ̇ (lag: 1차 지연 | gym: 뱅뱅,
              각속도 한계), 바퀴 가속도 ω̇ (lag: 1차 지연 | gym: P 제어, 가속·감속 한계). substep·RK4 단계마다
    → 동역학  공용 VehicleModel.cartesian() 을 RK4 1 ms 로 적분
    바퀴  wheel_model='lag' (기본): 위 저수준의 속도 법칙 (기본 = 1차 지연 wheel_speed_tau, ws_acc / ws_dec 는 가속 한계)
          wheel_model='vesc': 실차 경로 그대로 — ackermann_to_vesc (정지 제동 패치) → VESC 펌웨어 속도 PID
          → 모터 전류 → 바퀴 힘 → m_eq·ws' = F − 타이어 반력 (env/drivetrain.py, docs/VEHICLE_DYNAMICS.md).
          조향·속도 명령 지연은 이때도 저수준 설정을 따른다
    기본 LowLevelConfig() 는 예전 시뮬레이터와 같은 식이다 (서보·바퀴 1차 지연, 지연 없음).

  model 과 다른 차를 쓰고 싶으면 (모델 불일치 시험) model.copy(wheel_speed_tau=0.66) 처럼 넘긴다.

■ 저속 — 원래 식은 vx → 0 에서 슬립 분모가 0 이 되어 아주 딱딱하다 (0.2 m/s 에서 고유값 약 −1000 /s).
  MPC 는 v_floor 로 피하지만 시뮬은 서 있다가 출발해야 하므로 v_lo~v_hi 에서 운동학 자전거와 섞는다.

■ 상태 x = [X, Y, yaw, vx, vy, r, ws, delta] — X, Y 무게중심. pose()/twist() 는 /mpc/state 와 같은 뒤축 기준.
"""
import math

import numpy as np

from mlcs_mpc.env.lowlevel import LowLevel, LowLevelConfig
from mlcs_mpc.env.mpc.core import command_to_steer


class Simulator:

    def __init__(self, model, track=None, dt=0.01, substep=0.001, integrator='rk4',
                 steer_map_cmd=(0.0, 0.2, 0.3, 0.4), steer_map_act=(0.0, 0.2021, 0.2972, 0.3776),
                 max_steer=None, ws_acc=0.0, ws_dec=0.0, v_lo=0.2, v_hi=0.4, half_width=0.135, vesc=None,
                 wheel_model='lag', drivetrain=None, lowlevel=None):
        self.model, self.track = model, track
        self.dt, self.substep, self.integrator = dt, substep, integrator
        self.map_cmd, self.map_act, self.max_steer = list(steer_map_cmd), list(steer_map_act), max_steer
        # 저수준. ws_acc / ws_dec (mpcc.yaml ws_acc_max·ws_dec_max) 는 설정에 한계가 없을 때 바퀴 가속 한계로
        ll = lowlevel if lowlevel is not None else LowLevelConfig()
        over = {}
        if ws_acc > 0 and ll.acc_max == 0:
            over['acc_max'] = ws_acc
        if ws_dec > 0 and ll.dec_max == 0:
            over['dec_max'] = ws_dec
        self.lowlevel = LowLevel(ll.replace(**over), model.steering_tau, model.wheel_speed_tau)
        self.half_width = half_width
        self.vesc = vesc            # env/vesc.Vesc — None 이면 자르지 않는다
        self.f = model.function('cartesian', v_lo=v_lo, v_hi=v_hi)
        self.p = np.asarray(model.p, dtype=float)
        self.wheel_model = wheel_model
        self.drivetrain = None
        if wheel_model == 'vesc':
            from mlcs_mpc.env.drivetrain import Drivetrain, reaction_function
            if vesc is None:
                from mlcs_mpc.env.vesc import Vesc
                self.vesc = vesc = Vesc()
            if integrator != 'rk4':
                raise ValueError("wheel_model='vesc' 는 integrator='rk4' 만 된다")
            self.drivetrain = drivetrain or Drivetrain(vesc)
            self.f_reaction = reaction_function(model, v_lo, v_hi)
        elif wheel_model != 'lag':
            raise ValueError(f'wheel_model: lag | vesc ({wheel_model!r})')
        self.x = np.zeros(8)
        self.t = 0.0
        self._vesc_n = 0
        self.on_wall, self.wall_hits = False, 0
        self.lap_t0, self.lap_s0, self.laps = None, None, []

    # ── 상태 ───────────────────────────────────────────────────────────
    def reset(self, x, y, yaw):
        """뒤축 pose 로 세운다 (속도 0)."""
        lr = self.model.lr
        self.x = np.array([x + lr * math.cos(yaw), y + lr * math.sin(yaw), yaw, 0, 0, 0, 0, 0], dtype=float)
        if self.track is not None:
            self.track.reset()
        if self.drivetrain is not None:
            self.drivetrain.reset()
        self.lowlevel.reset()
        self._vesc_n = 0
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
    def wheel_speed_meas(self):
        """/sensors/core 로 재는 바퀴 속도 (erpm 을 명목 gain 으로 되돌린 값 = 실제 / speed_scale)."""
        return float(self.x[6]) / self.lowlevel.cfg.speed_scale

    @property
    def steer(self):
        return float(self.x[7])

    # ── 적분 ───────────────────────────────────────────────────────────
    def _rhs(self, x, d_cmd, w_cmd, drive=True):
        """저수준 (상태 → δ̇·ω̇ → 모델 입력) + 공용 모델."""
        return np.array(self.f(x, self.lowlevel.model_input(x, d_cmd, w_cmd, drive), self.p)).ravel()

    def _rk4(self, x, h, d_cmd, w_cmd, drive=True):
        k1 = self._rhs(x, d_cmd, w_cmd, drive)
        k2 = self._rhs(x + 0.5 * h * k1, d_cmd, w_cmd, drive)
        k3 = self._rhs(x + 0.5 * h * k2, d_cmd, w_cmd, drive)
        k4 = self._rhs(x + h * k3, d_cmd, w_cmd, drive)
        return x + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)

    def step(self, steer_cmd, speed_cmd, dt=None):
        """/drive 명령으로 dt 만큼. 반환: 상태 요약 dict (트랙이 있으면 s, n, 벽, 랩)."""
        dt = self.dt if dt is None else dt
        if self.drivetrain is not None:
            return self._step_vesc(steer_cmd, speed_cmd, dt)
        if self.vesc is not None:   # 차의 vesc_driver 가 하는 것처럼
            steer_cmd, speed_cmd = self.vesc.steer(steer_cmd), self.vesc.speed(speed_cmd)
        d_ref = command_to_steer(steer_cmd, self.map_cmd, self.map_act, self.max_steer)
        ll = self.lowlevel
        ll.command(self.t, d_ref, speed_cmd)
        if self.integrator == 'rk4':
            n = max(1, int(math.ceil(dt / self.substep - 1e-9)))
            h = dt / n
            x = self.x
            for i in range(n):
                d_cmd, w_cmd = ll.targets(self.t + i * h)
                x = self._rk4(x, h, d_cmd, w_cmd)
        else:                       # scipy — 지연은 step 해상도 (step 시작 때의 목표를 dt 동안)
            from scipy.integrate import solve_ivp
            d_cmd, w_cmd = ll.targets(self.t)
            x = solve_ivp(lambda _t, xx: self._rhs(xx, d_cmd, w_cmd), (0.0, dt), self.x,
                          method=self.integrator, max_step=self.substep * 10).y[:, -1]
        x[2] = (x[2] + math.pi) % (2 * math.pi) - math.pi
        self.x = x
        self.t += dt
        return self._info()

    def _step_vesc(self, steer_cmd, speed_cmd, dt):
        """wheel_model='vesc'. substep 마다: PID·전류 → 바퀴 힘 → ws (선형화 후진 오일러, 타이어 반력이 딱딱해서)
        → ws 를 고정하고 나머지 상태 RK4. 속도 명령은 ackermann_to_vesc 가 erpm 으로 자른다 (vesc.speed 안 씀)."""
        steer_cmd = self.vesc.steer(steer_cmd)
        d_ref = command_to_steer(steer_cmd, self.map_cmd, self.map_act, self.max_steer)
        dtr, ll = self.drivetrain, self.lowlevel
        ll.command(self.t, d_ref, speed_cmd)
        n = max(1, int(math.ceil(dt / self.substep - 1e-9)))
        h = dt / n
        x = self.x.copy()
        for i in range(n):
            d_cmd, w_cmd = ll.targets(self.t + i * h)
            if ll.w_buf.n != self._vesc_n:          # 명령 하나 = /drive 메시지 하나 (지연을 거쳐 들어온 때 한 번)
                dtr.command(w_cmd, x[6])
                self._vesc_n = ll.w_buf.n
            F = dtr.force(h, x[6])
            R, dR = (float(v) for v in self.f_reaction(x, self.p))
            x[6] += h * (F - R) / (dtr.m_eq + h * max(dR, 0.0))
            x = self._rk4(x, h, d_cmd, w_cmd, drive=False)   # ω_ref = ω → ω̇ = 0 (RK4 동안 ω 고정)
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
