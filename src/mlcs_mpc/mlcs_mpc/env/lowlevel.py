"""
lowlevel — 시뮬레이터의 저수준 제어기 (/drive 목표값 → 조향 각속도·바퀴 가속도). ROS 없음.

  f1tenth_gym 과 같은 3 계층 (docs/VEHICLE_DYNAMICS.md §3):
    상위 제어기 (MPC·RL)    → 목표 조향 δ_ref, 목표 바퀴 속도 ω_ref        (/drive)
    저수준 (이 파일)         → 명령 지연 버퍼 → 조향 각속도 δ̇, 바퀴 가속도 ω̇ (제한 포함)
    차 동역학 (vehicle_model) → 적분 (simulator.py, RK4 1 ms)

  공용 차 모델 (vehicle_model.py, MPC·RL 과 같은 식) 의 입력은 (ω_ref, δ_ref) 이고 안에 1차 지연
  ω̇ = (ω_ref − ω)/τ_ω, δ̇ = (δ_ref − δ)/τ_δ 가 있다. 그래서 저수준이 정한 ẋ 를 u = x + τ·ẋ 로 바꿔 넣으면
  모델 안에서 정확히 그 ẋ 가 된다 (모델 식을 고치지 않는다 — vec/vehicle.py 와 같은 식을 유지).
  RK4 의 단계마다 다시 계산한다 (저수준 = 상태의 함수, 연속 시간).

■ 조향 (steer_model)
    lag   δ̇ = (δ_cmd − δ)/τ_δ                         실차 서보 근사. τ_δ = steer_tau (0 이면 MPC 모델 steering_tau)
    gym   δ̇ = sign(δ_cmd − δ)·steer_rate  (|e| > 1e-4) f1tenth_gym pid() 의 뱅뱅
    + steer_rate > 0 이면 |δ̇| ≤ steer_rate (lag 에도 — 서보 최대 각속도)
■ 속도 (speed_model) — wheel_model 'vesc' 이면 쓰지 않는다 (drivetrain.py 가 펌웨어 PID 로)
    lag   ω̇ = (ω_cmd − ω)/τ_ω                         (기본, 지금까지와 같다)
    gym   ω̇ = kp·(ω_cmd − ω), kp = kp_acc (가속) | kp_dec (감속)   f1tenth_gym pid() 의 전진 P 제어
    + 가속 한계 (gym 의 accl_constraints 와 같은 꼴): ω̇ ≤ acc_max·min(1, v_switch/ω), ω̇ ≥ −dec_max  (0 = 없음)
      gym 은 차 속도 v 를, 우리는 바퀴 속도 ω 를 제어한다 (VESC 도 모터 회전수를 제어한다).
■ 속도 환산 — speed_scale: 명령 ω_ref 가 실제로는 speed_scale 배 바퀴 속도가 된다 (erpm gain 오차, 실측 1.044).
  측정 (/sensors/core erpm → 바퀴 속도) 은 같은 gain 으로 되돌리므로 컨트롤러는 ω/speed_scale 을 본다
  (Simulator.wheel_speed_meas). vesc 바퀴 모델에서는 명령을 speed_scale 배 해서 넘긴다 (펌웨어 erpm 단위가 그만큼 근사).
■ 지연 — steer_delay / speed_delay [s]: 명령이 이만큼 늦게 저수준에 들어간다 (gym 은 조향 2 스텝 = 20 ms).
  substep 해상도로 처리. 지연 전에는 직전 (처음엔 0) 명령.

  차의 값은 config/lowlevel.yaml (LowLevelConfig.from_yaml — main.py·sim_node 기본). 코드 기본 LowLevelConfig() 는 예전 식.
  gym 기본값 그대로 쓰려면 LowLevelConfig.gym() — 조향 뱅뱅 3.2 rad/s, 지연 20 ms, kp 4.755 / 19.02, a_max 9.51.
"""
import math
import os
from collections import deque
from dataclasses import asdict, dataclass, fields

DEFAULT_YAML = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'config', 'lowlevel.yaml')


@dataclass
class LowLevelConfig:
    steer_model: str = 'lag'        # lag | gym
    steer_rate: float = 0.0         # rad/s 조향 각속도 한계 (gym 이면 뱅뱅 크기, 0 이면 3.2)
    steer_delay: float = 0.0        # s
    steer_tau: float = 0.0          # s 시뮬 차 서보 1차 시정수 (lag). 0 = MPC 모델 steering_tau 와 같게 (예전)
    speed_model: str = 'lag'        # lag | gym
    kp_acc: float = 4.755           # 1/s (gym: 10·a_max/v_max)
    kp_dec: float = 19.02           # 1/s (gym: 10·a_max/|v_min|)
    acc_max: float = 0.0            # m/s² 바퀴 가속 한계 (0 = 없음, gym 9.51)
    dec_max: float = 0.0            # m/s² 바퀴 감속 한계 (0 = 없음, gym 9.51)
    v_switch: float = 0.0           # m/s 이보다 빠르면 가속 한계가 acc_max·v_switch/ω (0 = 없음, gym 7.319)
    speed_delay: float = 0.0        # s
    speed_scale: float = 1.0        # 실제 바퀴 속도 / 명령 (erpm 환산) — 실차 실측 1.044 (환산보다 4.4 % 빠르다)

    GYM_STEER_RATE = 3.2

    @classmethod
    def gym(cls, **over):
        """f1tenth_gym main 기본값 (f110_env.py, dynamic_models.py, base_classes.py)."""
        c = cls(steer_model='gym', steer_rate=cls.GYM_STEER_RATE, steer_delay=0.02, speed_model='gym',
                kp_acc=4.755, kp_dec=19.02, acc_max=9.51, dec_max=9.51, v_switch=7.319)
        return c.replace(**over)

    @classmethod
    def from_yaml(cls, path=None, **over):
        """config/lowlevel.yaml (시뮬레이터 차의 저수준 — 실차 값). 없으면 기본 (예전 식)."""
        import os
        import yaml
        path = path or DEFAULT_YAML
        d = {}
        if os.path.isfile(path):
            d = (yaml.safe_load(open(path)) or {}).get('lowlevel', {}) or {}
        d.update(over)
        return cls().replace(**d)

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]

    def replace(self, **over):
        bad = [k for k in over if k not in self.names()]
        if bad:
            raise KeyError(f'저수준 설정에 없는 이름: {bad} (있는 것: {self.names()})')
        d = asdict(self)
        d.update(over)
        c = LowLevelConfig(**d)
        c.check()
        return c

    @classmethod
    def parse(cls, items, base=None):
        """['steer_model=gym', 'steer_delay=0.02', ...] 또는 'gym' (프리셋) → base (기본 LowLevelConfig()) 위에 덮어쓴 설정.
        'none' 이면 base 를 버리고 기본 (예전 식)."""
        base, over = (base if base is not None else cls()), {}
        for kv in items or []:
            if kv == 'gym':
                base = cls.gym()
                continue
            if kv == 'none':
                base = cls()
                continue
            k, v = kv.split('=', 1)
            over[k] = v if k in ('steer_model', 'speed_model') else float(v)
        return base.replace(**over)

    def check(self):
        if self.steer_model not in ('lag', 'gym'):
            raise ValueError(f'steer_model: lag | gym ({self.steer_model!r})')
        if self.speed_model not in ('lag', 'gym'):
            raise ValueError(f'speed_model: lag | gym ({self.speed_model!r})')
        for k in ('steer_rate', 'steer_delay', 'steer_tau', 'kp_acc', 'kp_dec', 'acc_max', 'dec_max', 'v_switch', 'speed_delay'):
            if getattr(self, k) < 0:
                raise ValueError(f'{k} < 0')
        if self.speed_scale <= 0:
            raise ValueError('speed_scale ≤ 0')

    @property
    def is_default(self):
        return self == LowLevelConfig()


class _Delay:
    """명령 지연 버퍼 — push(t, 값), at(t) = t − delay 시각에 유효했던 값."""

    def __init__(self, delay, init=0.0):
        self.delay = delay
        self.buf = deque()
        self.cur = init
        self.n = 0                      # 저수준에 들어간 명령 수 (새 명령이 들어왔는지 — VESC 는 메시지마다 한 번)

    def reset(self, init=0.0):
        self.buf.clear()
        self.cur = init
        self.n = 0

    def push(self, t, v):
        if self.delay <= 0:
            self.cur = v
            self.n += 1
        else:
            self.buf.append((t, v))

    def at(self, t):
        while self.buf and self.buf[0][0] + self.delay <= t + 1e-9:
            self.cur = self.buf.popleft()[1]
            self.n += 1
        return self.cur


class LowLevel:
    """저수준 제어기 한 대분 (지연 버퍼 상태를 가진다)."""

    def __init__(self, cfg=None, steering_tau=0.13, wheel_speed_tau=0.4):
        self.cfg = cfg or LowLevelConfig()
        self.cfg.check()
        self.tau_d, self.tau_w = float(steering_tau), float(wheel_speed_tau)
        self.d_buf = _Delay(self.cfg.steer_delay)
        self.w_buf = _Delay(self.cfg.speed_delay)

    def reset(self):
        self.d_buf.reset()
        self.w_buf.reset()

    # ── 명령 ───────────────────────────────────────────────────────────
    def command(self, t, delta_ref, ws_ref):
        """시각 t 에 들어온 /drive 목표 (서보 맵·VESC 한계를 거친 실제 조향 δ_ref, 바퀴 속도 ω_ref)."""
        self.d_buf.push(t, float(delta_ref))
        self.w_buf.push(t, float(ws_ref) * self.cfg.speed_scale)

    def targets(self, t):
        """지연을 거쳐 지금 저수준에 들어가 있는 (δ_cmd, ω_cmd)."""
        return self.d_buf.at(t), self.w_buf.at(t)

    # ── 제어 법칙 (상태 → 각속도·가속도) ───────────────────────────────
    def steer_rate(self, delta, d_cmd):
        c = self.cfg
        e = d_cmd - delta
        if c.steer_model == 'gym':
            rate = c.steer_rate or LowLevelConfig.GYM_STEER_RATE
            return math.copysign(rate, e) if abs(e) > 1e-4 else 0.0
        dd = e / (c.steer_tau or self.tau_d)
        if c.steer_rate > 0:
            dd = min(max(dd, -c.steer_rate), c.steer_rate)
        return dd

    def wheel_acc(self, ws, w_cmd):
        c = self.cfg
        e = w_cmd - ws
        if c.speed_model == 'gym':
            a = (c.kp_acc if e > 0 else c.kp_dec) * e
        else:
            a = e / self.tau_w
        if c.acc_max > 0:
            hi = c.acc_max * (c.v_switch / ws if c.v_switch > 0 and ws > c.v_switch else 1.0)
            a = min(a, hi)
        if c.dec_max > 0:
            a = max(a, -c.dec_max)
        return a

    def model_input(self, x, d_cmd, w_cmd, drive=True):
        """상태 x (simulator 순서 [X, Y, yaw, vx, vy, r, ws, δ]) → 공용 모델 입력 [ω_ref, δ_ref].
        drive=False 면 바퀴는 바깥 (VESC drivetrain) 이 정하므로 ω_ref = ω (ω̇ = 0)."""
        ws, delta = x[6], x[7]
        u_w = ws + self.tau_w * self.wheel_acc(ws, w_cmd) if drive else ws
        u_d = delta + self.tau_d * self.steer_rate(delta, d_cmd)
        return [u_w, u_d]
