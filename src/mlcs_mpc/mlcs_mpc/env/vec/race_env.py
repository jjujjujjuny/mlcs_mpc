"""
vec.race_env — 강화학습 레이싱 환경 (배치). arXiv:2504.02420 "On learning racing policies with reinforcement learning"
(Czechmanowski et al., IROS 2025) 의 설정을 우리 차 모델 (env/vehicle_model.py → vec/vehicle.py) 로.

  env = VecRaceEnv(RaceEnvConfig(track='rl17_track', num_envs=400))
  obs = env.reset()
  obs, reward, terminated, truncated, info = env.step(action)      action (B, 2) ∈ [−1, 1]
  done = terminated | truncated
  if done.any(): obs = env.reset(done.nonzero().squeeze(1))       ★ 스스로 리셋하지 않는다 (final obs 보존)
  env.set_dr_level(0.5)                                            커리큘럼 — 랜덤화 크기 배율 (다음 리셋부터)

■ 논문과 같은 것 (III-E, IV)
  주기         dt 0.05 s (제어 20 Hz)
  행동         a ∈ [−1, 1]² → δ_ref = a₀·δ_max, ω̇_ref = a₁·5 m/s² 를 적분해 ω_ref (바퀴 속도 명령) — 급변을 막는 정식화
  보상         r = s_t − s_{t−1} (센터라인 진행), 트랙 경계를 넘으면 −1 + 종료 → 센터라인 위 임의 위치로 리셋
  관측 (식 2)  [vx, vy, μ, n, r, δ, δ_ref(직전), ω̇_ref(직전), ω_ref, ω, c(N), w(N)] / 각 최댓값
  랜덤화       에피소드 시작마다 값 × N(1, σ) — 'friction' (논문 기본 정책 σ 0.02) | 'all' (단일 궤적 값 전부)

■ ★ 저수준 제어기 (actuator) — 논문의 첫째 축
  'lag'   논문 식 (1): 조향·바퀴 속도 1차 지연 (steering_tau 0.13, wheel_speed_tau 0.4 — 실측). MPC 와 같은 모델.
          + motor_limit: 바퀴 가속·감속을 모터가 낼 수 있는 만큼으로 자른다 (60 A, 배터리 60 A / 회생 −5 A → 빠를수록
          제동 약함) — vec/drivetrain.accel_limits. 최고 속도는 ERPM 상한 (4.86 m/s) 과 항력·모터 한계가 정한다.
  'vesc'  실차의 저수준 경로를 그대로: 정지 제동 패치 → VESC 펌웨어 속도 PID (1 kHz, 램프 5.23 m/s²) → 전류 제한 →
          바퀴 힘 → 바퀴 관성 (vec/drivetrain.py = env/drivetrain.py 의 torch 판). 실측 τ 0.4 보다 빠르게 따라간다.
  'none'  지연 없음 (논문 Table I 'RL w/o am') — 명령이 곧 실제 조향·바퀴 속도.

■ ★ 랜덤화 (dr) — 논문의 둘째 축. 항목마다 수준 1 의 크기를 두고 dr_level (0~1, 커리큘럼) 을 곱한다
  friction   마찰 × N(1, σ)                    vehicle   p[0:10] (I_z, lr, 항력…) 과 질량 × N(1, σ)
  tire       p[10:30] (앞뒤 타이어) × N(1, σ)    tau       steering_tau, wheel_speed_tau × N(1, σ)
  steer      조향 이득 × N(1, σ_gain) + 오프셋 N(0, σ_off) rad (서보 캘리브레이션 오차)
  pid        VESC PID 이득 × N(1, σ), m_eq × N(1, σ)   (actuator='vesc')
  latency    명령 지연 U(0, ℓ·최대) s — substep 단위로 직전 명령을 그만큼 더 쓴다
  sensor     모캡 측정 노이즈 (매 스텝): ★ 모캡 강체 기준점은 **앞축 중심** 이다 (config/mocap.yaml — Y자 중심 마커,
             mocap_bridge 가 offset_x −0.33 으로 뒤축으로 옮긴다). 그래서 앞축 점 위치 N(0, σ_p), yaw N(0, σ_ψ) 를 넣고
             같은 변환으로 무게중심을 추정해 Frenet 관측 (n, μ, 앞쪽 트랙) 을 만든다 — yaw 노이즈가 휠베이스만큼 위치로 번진다.
             속도·요레이트·바퀴 속도에도 N(0, σ). 보상·경계 판정은 참값.
  drop       ★ 명령 끊김 ('제어가 씹힘') — 매 주기 확률 p (× 수준) 로 끊김이 시작돼 평균 drop_len 주기 (기본 3 = 150 ms)
             이어진다 (연속 끊김 — 무선·USB 끊김은 몰려서 난다). 끊긴 동안 차 (서보·VESC) 는 마지막으로 받은 명령을
             그대로 쓴다 (실차 VESC 도 명령이 멈추면 마지막 속도를 유지한다). 정책은 모른다 — 관측의 ω_ref·δ_ref 는
             정책이 보낸 값이고, 실제 ω·δ 가 안 따라오는 것으로만 알 수 있다. 프리셋에는 없다 (dr_terms 로 켠다).
             ★ VESC 의 명령 timeout (app 설정 timeout_msec, 넘으면 timeout_brake_current) 은 모델에 없다 — 끊김이
             timeout 보다 짧을 때만 맞다.
  프리셋: 'none' | 'friction' | 'all' (논문 두 가지, 크기 = dr_sigma) | 'full' (위 전부, DR_FULL 의 크기)

■ 우리 차에 맞춘 것 (논문과 다름)
  δ_max = 우리 조향 한계 (0.378 rad, 논문 0.5), ω_ref ≤ VESC 모터 상한 4.86 m/s, 폭은 왼쪽·오른쪽 따로 (2N),
  N = 20 (논문에 없음), 시간 제한 max_steps (부트스트랩).
"""
import math
from dataclasses import asdict, dataclass, field

import torch

from mlcs_mpc.env.vec.track import VecTrack, wrap
from mlcs_mpc.env.vec.vehicle import VecVehicle

NX = 8

# 'full' 프리셋 — 수준 1 의 크기 (docs/RL_RACING.md '랜덤화 크기의 근거')
#   마찰 0.05 (논문 σ 0.02~0.1 중간), 질량·관성·타이어는 작게 (Xie 2020: 이득 작고 느려짐), 시정수 ±10 %
#   (Tan 2018 모터 ±20 % 의 절반), 조향 오프셋 0.02 rad (Werner 2023 ±0.03), 지연 0~40 ms (Tan 0~40 ms),
#   모캡 노이즈 위치 2 mm·yaw 0.005 rad·속도 0.03 m/s (실측 1 mm 노이즈 + 칼만 속도)
DR_FULL = dict(friction=0.05, vehicle=0.02, tire=0.02, tau=0.1, steer_gain=0.03, steer_off=0.02,
               pid=0.1, latency=0.04, pos=0.002, yaw=0.005, vel=0.03, yawrate=0.05, ws=0.03)


def dr_preset(name, sigma):
    if name in ('none', '', None):
        return {}
    if name == 'friction':
        return dict(friction=sigma)
    if name == 'all':           # 논문 'all' — 단일 궤적 값 전부 (차량·타이어·질량·마찰) + 액추에이터 시정수
        return dict(friction=sigma, vehicle=sigma, tire=sigma, tau=sigma)
    if name == 'full':
        return dict(DR_FULL)
    raise ValueError(f'dr: {name}')


@dataclass
class RaceEnvConfig:
    track: str = 'rl17_track'
    num_envs: int = 400
    dt: float = 0.05
    substep: float = 0.005              # RK4 간격. 0.001 과 3 s 위치 차 < 0.05 mm (float32)
    device: str = 'cpu'
    dtype: str = 'float32'
    mpcc_yaml: str = ''                 # 차 값 (비우면 src/mlcs_mpc/config/mpcc.yaml)
    car: dict = field(default_factory=dict)     # 차 값 덮어쓰기 (VehicleModel 인자: friction, mass, ...)
    max_steer: float = 0.0              # 0 → mpcc.yaml max_steer_actual (0 이면 조향 맵 끝값)
    acc_max: float = 5.0                # ω̇_ref 한계 m/s² (논문 5)
    v_max: float = 0.0                  # ω_ref 상한. 0 → VESC 모터 상한
    lookahead_n: int = 20
    lookahead_ds: float = 0.3
    reset_v: tuple = (0.0, 1.5)         # 리셋 속도 범위 m/s (논문에 없음)
    # ★ 시작 조건 커리큘럼 (start_spread > 0 이면 켬, 크기 × start_level — 학습이 되면 train.py 가 dr_level 과 같이 올린다)
    #   센터라인 위 임의 s (논문) 에 더해: 횡 위치 U(±start_n·벽 폭), 헤딩 오차 U(±start_mu), 속도 상한 + start_dv,
    #   start_still 비율은 정지 출발 (v = 0 — 실차 출발과 같은 조건)
    start_spread: float = 0.0
    start_n: float = 0.6
    start_mu: float = 0.35
    start_dv: float = 2.0
    start_still: float = 0.1
    max_steps: int = 2000
    stuck_v: float = 0.3                # 이 속도 아래로 stuck_time 넘게 있으면 종료 + 보상 −1 (벽과 같게) — 멈춰 서서 보상 0 으로
    stuck_time: float = 1.0             #   버티는 국소해를 막는다 (참고 레포의 low_speed 종료와 같은 취지)
    actuator: str = 'lag'               # ★ 저수준 제어기 — 'lag' | 'vesc' | 'none'
    motor_limit: bool = True            #   lag 에 모터 가속·감속 한계
    drive_mass: float = 0.3             #   m_eq (바퀴 둘레로 옮긴 회전 관성, ★ 추정 — env/drivetrain.py)
    vesc_dt: float = 0.001              #   vesc 의 PID·바퀴·차체 공통 적분 간격 (펌웨어 PID 1 kHz = 0.001). 키우면 빠르다 —
                                        #   셋을 같은 간격으로 묶으면 분할 오차가 없다 (정확도는 docs/RL_RACING.md 표)
    vesc_body: str = 'rk2'              #   'rk2' | 'rk4' | 'euler'
    vesc_dR: str = 'fd'                 #   바퀴 안정화 dR/dws — 'fd' (반력 두 번) | 'approx' (타이어 선형 강성, 반력 한 번)              #   vesc 의 1 ms 차체 적분 — 'rk2' (빠름) | 'rk4'. 1 ms 에서는 둘 차이가 mm 이하 (test ⑥)
    dr: str = 'friction'                # ★ 랜덤화 프리셋 — 'none' | 'friction' | 'all' | 'full'
    dr_sigma: float = 0.02              #   'friction'·'all' 의 σ (논문 0, 0.02, 0.05, 0.1)
    dr_terms: dict = field(default_factory=dict)   # 항목별 크기 덮어쓰기 (프리셋 위에)
    dr_level: float = 1.0               #   크기 배율 (커리큘럼이 0 → 1 로 올린다)
    dr_nominal_frac: float = 0.0        #   리셋 때 이 비율은 랜덤화 없이 (망각·속도 손실 방지 — Rudin 2021 / ADR 식)
    v_lo: float = 0.2                   # 저속 운동학 섞기 (Simulator 와 같다)
    v_hi: float = 0.4
    # ★ 관측 속도 (vx, vy, r) 를 어떻게 얻나 — 실차는 참값을 모른다 (2026-11-25, 메인 PC)
    #   'true'  참값 + sensor 노이즈 (지금까지)
    #   'delay' obs_delay 만큼 늦은 참값 + 노이즈 (vx·vy·r 따로)
    #   'kf'    실차 mocap_bridge 그대로: mocap 100 Hz (앞축 점 + pos·yaw 노이즈 → 뒤축) 에 x·y·yaw 정속도 칼만 필터
    #           (PoseKF, q_pos 0.01·q_vel 1.0·r 1e-4), 전역 속도를 KF yaw 로 돌린다 → vx·r 80~90 ms 늦고 코너에서 가짜 vy.
    #           위치·헤딩 관측 (n, μ, 앞쪽 트랙) 도 KF 출력
    #   'gyro'  제안된 실차 개선안: r = 자이로 (obs_delay gyro 만큼 늦음), vx = 바퀴 속도 (같은 지연), vy·위치는 'kf' 와 같다
    obs_vel: str = 'true'
    obs_vy: str = 'kf'                  #   kf·gyro 의 vy — 'kf' 지금 실차 | 'rot' KF 속도를 yaw − r·vy_lag 로 돌림 (가짜 vy 없앰)
    vy_lag: float = 0.10                #   11-10 실제 로그: vy 오차 rms 0.18~0.26 → 0.03~0.04 (0.09~0.12)
    obs_delay: dict = field(default_factory=lambda: dict(vx=0.085, vy=0.085, r=0.085, gyro=0.02))
    mocap_dt: float = 0.01              #   센서 틱 (mocap 100 Hz). dt·substep 이 이 값의 배수·약수여야 정확하다
    kf: dict = field(default_factory=lambda: dict(q_pos=0.01, q_vel=1.0, r_meas=1e-4))
    # 관측 정규화 최댓값 (배포 때 같은 값을 써야 한다 — 체크포인트에 같이 저장)
    obs_scale: dict = field(default_factory=lambda: dict(
        vx=5.0, vy=2.0, mu=1.0, n=0.5, r=6.0, ws=5.0, kappa=2.0, w=1.0))

    def to_dict(self):
        return asdict(self)


class VecRaceEnv:

    def __init__(self, cfg: RaceEnvConfig, seed=0):
        import os
        from mlcs_mpc.env.track import Track, find
        from mlcs_mpc.env.vehicle_model import VehicleModel
        from mlcs_mpc.env.vesc import Vesc
        self.cfg = cfg
        self.B = int(cfg.num_envs)
        self.device = torch.device(cfg.device)
        self.dtype = getattr(torch, cfg.dtype)
        self.gen = torch.Generator(device='cpu').manual_seed(int(seed))
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        yaml_path = cfg.mpcc_yaml or os.path.join(root, 'config', 'mpcc.yaml')
        model = VehicleModel.from_yaml(yaml_path, **cfg.car)
        import yaml
        prm = yaml.safe_load(open(yaml_path, encoding='utf-8'))['/**']['ros__parameters']
        self.max_steer = float(cfg.max_steer or prm.get('max_steer_actual') or prm['steer_map_act'][-1])
        vesc = Vesc.load()
        self.v_max = float(cfg.v_max or vesc.v_motor)
        self.model = model
        self.veh = VecVehicle.from_model(model, self.B, self.device, self.dtype, cfg.v_lo, cfg.v_hi)
        self.P = self.veh.base.clone()
        self.track = VecTrack(Track(find(cfg.track)), self.device, self.dtype)
        self.offsets = torch.arange(1, cfg.lookahead_n + 1, device=self.device, dtype=self.dtype) * cfg.lookahead_ds
        self.dtr = None
        if cfg.actuator == 'vesc' or (cfg.actuator == 'lag' and cfg.motor_limit):
            from mlcs_mpc.env.vec.drivetrain import VecDrivetrain
            self.dtr = VecDrivetrain(self.B, self.device, self.dtype, vesc, m_eq=cfg.drive_mass)
            if cfg.actuator == 'vesc' and cfg.vesc_dt > 0:
                self.dtr.pid_dt = float(cfg.vesc_dt)
        if cfg.actuator not in ('lag', 'vesc', 'none'):
            raise ValueError(f'actuator: {cfg.actuator}')
        self.dr = dict(dr_preset(cfg.dr, cfg.dr_sigma), **cfg.dr_terms)
        self.dr_level = float(cfg.dr_level)
        self.start_level = 1.0               # 시작 조건 커리큘럼 배율 (train.py 가 dr_level 과 같이 올린다)
        z = lambda *s: torch.zeros(*s, device=self.device, dtype=self.dtype)  # noqa: E731
        self.x = z(self.B, NX)
        self.ws_ref, self.prev_a = z(self.B), z(self.B, 2)
        self.u_applied = z(self.B, 2)        # 차가 지금 받고 있는 (ws_ref, δ_ref) — 지연용
        self.s, self.n, self.mu = z(self.B), z(self.B), z(self.B)          # 참값 (보상·경계)
        self.s_m, self.n_m, self.mu_m = z(self.B), z(self.B), z(self.B)    # 측정값 (관측)
        self.idx = torch.full((self.B,), -1, device=self.device, dtype=torch.long)
        self.idx_m = torch.full((self.B,), -1, device=self.device, dtype=torch.long)
        self.steps = torch.zeros(self.B, device=self.device, dtype=torch.long)
        self.t = z(self.B)
        self.slow_t = z(self.B)              # 저속으로 있은 시간
        self.progress = z(self.B)            # 리셋 뒤 누적 진행 m
        self.lap_t0, self.lap_p0 = z(self.B), z(self.B)
        self.latency = z(self.B)             # 환경마다 명령 지연 s
        self.drop_p = None                   # 평가용 끊김 확률 덮어쓰기 (B,) — eval_vec
        self.steer_gain, self.steer_off = z(self.B) + 1.0, z(self.B)
        self.drop_left = torch.zeros(self.B, device=self.device, dtype=torch.long)   # 남은 끊김 주기
        self.dropped = torch.zeros(self.B, device=self.device, dtype=torch.bool)
        self.lvl = z(self.B) + self.dr_level  # 환경마다의 랜덤화 배율 (리셋 때 정한다)
        self.meas_noise = z(self.B, 7)       # 마지막 측정 노이즈 (pos x, y, yaw, vx, vy, r, ws) — 그림용
        self.obs_dim = 10 + 3 * cfg.lookahead_n
        self.act_dim = 2
        # 관측 속도 (obs_vel) — 센서 틱 버퍼와 칼만 필터
        if cfg.obs_vel not in ('true', 'delay', 'kf', 'gyro'):
            raise ValueError(f'obs_vel: {cfg.obs_vel}')
        self.vo = z(self.B, 4)               # 관측에 넣는 (vx, vy, r, ws) — 노이즈 포함
        self.allids = torch.arange(self.B, device=self.device)
        if cfg.obs_vel != 'true':
            dmax = max(float(v) for v in cfg.obs_delay.values())
            self.n_hist = int(round(dmax / cfg.mocap_dt)) + 2
            self.hist = z(self.n_hist, self.B, 4)      # 센서 틱마다 참값 (vx, vy, r, ws)
            self.hist_i = 0
            self.sense_acc = 0.0
            self.kf_s = z(self.B, 3, 2)                # x, y, yaw 의 [위치, 속도] (뒤축 점)
            self.kf_K = self._kf_gain(cfg.mocap_dt, **cfg.kf)

    @staticmethod
    def _kf_gain(dt, q_pos, q_vel, r_meas):
        """PoseKF 정상상태 이득 (모든 환경·축이 같은 Q·R·dt 라 P 가 같다 — 실차도 1 s 안에 수렴)."""
        import numpy as np
        F = np.array([[1.0, dt], [0.0, 1.0]]); Q = np.diag([q_pos, q_vel]); P = np.eye(2)
        for _ in range(5000):
            P = F @ P @ F.T + Q * dt
            K = P[:, 0] / (P[0, 0] + r_meas)
            P = P - np.outer(K, P[0, :])
        return tuple(float(k) for k in K)

    # ── 랜덤 ───────────────────────────────────────────────────────────
    def _rand(self, n, lo=0.0, hi=1.0):
        return (torch.rand(n, generator=self.gen, dtype=torch.float64) * (hi - lo) + lo).to(self.device, self.dtype)

    def _randn(self, *shape):
        return torch.randn(*shape, generator=self.gen, dtype=torch.float64).to(self.device, self.dtype)

    def _sig(self, key, ids):
        """(n,) 환경마다의 크기 = 항목 크기 × 그 환경의 배율. 항목이 없으면 None."""
        v = float(self.dr.get(key, 0.0))
        if v <= 0:
            return None
        return v * self.lvl[ids]

    def set_dr_level(self, level):
        """커리큘럼. 다음 리셋부터 적용 (지금 달리는 환경은 그대로). 시작 조건도 같은 수준으로."""
        self.dr_level = float(min(max(level, 0.0), 1.0))
        self.start_level = self.dr_level

    def _mult(self, ids, key, *shape):
        s = self._sig(key, ids)
        if s is None:
            return None
        s = s.view(-1, *([1] * len(shape)))
        return torch.clamp(1 + s * self._randn(len(ids), *shape), min=0.2)

    def _randomize(self, ids):
        n = len(ids)
        nominal = self._rand(n) < self.cfg.dr_nominal_frac
        self.lvl[ids] = torch.where(nominal, torch.zeros(n, device=self.device, dtype=self.dtype),
                                    torch.full((n,), self.dr_level, device=self.device, dtype=self.dtype))
        P = self.veh.base.index(ids)
        m = self._mult(ids, 'friction')
        if m is not None:
            P.friction = P.friction * m
        m = self._mult(ids, 'vehicle', 10)
        if m is not None:
            P.p[:, 0:10] = P.p[:, 0:10] * m
            P.mass = P.mass * self._mult(ids, 'vehicle')
        m = self._mult(ids, 'tire', 20)
        if m is not None:
            P.p[:, 10:30] = P.p[:, 10:30] * m
        m = self._mult(ids, 'tau')
        if m is not None:
            P.steering_tau = P.steering_tau * m
            P.wheel_speed_tau = P.wheel_speed_tau * self._mult(ids, 'tau')
        self.P.assign(ids, P)
        g = self._mult(ids, 'steer_gain')
        self.steer_gain[ids] = g if g is not None else 1.0
        so = self._sig('steer_off', ids)
        self.steer_off[ids] = so * self._randn(n) if so is not None else 0.0
        lat = self._sig('latency', ids)
        self.latency[ids] = self._rand(n) * lat if lat is not None else 0.0
        self.drop_left[ids] = 0
        self.dropped[ids] = False
        if self.dtr is not None:
            for attr in ('kp_scale', 'ki_scale', 'kd_scale'):
                m = self._mult(ids, 'pid')
                getattr(self.dtr, attr)[ids] = m if m is not None else 1.0
            m = self._mult(ids, 'pid')
            self.dtr.m_eq[ids] = self.dtr.m_eq0 * (m if m is not None else 1.0)

    # ── 리셋·상태 ──────────────────────────────────────────────────────
    def reset(self, ids=None):
        """ids 환경을 센터라인 위 임의 위치로. 전체 배치의 관측을 돌려준다."""
        if ids is None:
            ids = torch.arange(self.B, device=self.device)
        n = len(ids)
        if n == 0:
            return self.observation()
        cfg = self.cfg
        s0 = self._rand(n, 0.0, self.track.length)
        g = cfg.start_spread * self.start_level
        if g > 0:
            wl, wr = self.track.walls(s0)
            n0 = self._rand(n, -1.0, 1.0) * cfg.start_n * g * torch.minimum(wl, wr)
            x, y, h = self.track.pose_at(s0, n0)
            h = h + self._rand(n, -1.0, 1.0) * cfg.start_mu * g
            v0 = self._rand(n, cfg.reset_v[0], cfg.reset_v[1] + cfg.start_dv * g)
            v0 = torch.where(self._rand(n) < cfg.start_still, torch.zeros_like(v0), v0)
        else:
            x, y, h = self.track.pose_at(s0)
            v0 = self._rand(n, *cfg.reset_v)
        st = torch.zeros(n, NX, device=self.device, dtype=self.dtype)
        st[:, 0], st[:, 1], st[:, 2], st[:, 3], st[:, 6] = x, y, h, v0, v0
        self.x[ids] = st
        self.ws_ref[ids] = v0
        self.prev_a[ids] = 0.0
        self.u_applied[ids] = torch.stack([v0, torch.zeros_like(v0)], dim=1)
        self.steps[ids] = 0
        self.t[ids] = 0.0
        self.slow_t[ids] = 0.0
        self.progress[ids] = 0.0
        self.lap_t0[ids] = 0.0
        self.lap_p0[ids] = 0.0
        self._randomize(ids)
        if self.dtr is not None:
            self.dtr.reset(ids)
            self.dtr.start_moving(ids, v0)
        self.idx[ids] = -1
        self.idx_m[ids] = -1
        self._frenet(ids)
        self._sense_reset(ids)
        self._measure(ids)
        return self.observation()

    def set_state(self, x, ws_ref=None, prev_a=None, ids=None):
        """다른 시뮬레이터 (env/simulator.py) 의 상태를 넣어 관측만 계산할 때. x (n, 8). 노이즈는 dr_level 대로."""
        if ids is None:
            ids = torch.arange(self.B, device=self.device)
        self.x[ids] = torch.as_tensor(x, device=self.device, dtype=self.dtype)
        if ws_ref is not None:
            self.ws_ref[ids] = torch.as_tensor(ws_ref, device=self.device, dtype=self.dtype)
        if prev_a is not None:
            self.prev_a[ids] = torch.as_tensor(prev_a, device=self.device, dtype=self.dtype)
        self._frenet(ids)
        self._sense_reset(ids)               # ★ obs_vel ≠ true 이어도 여기서는 지연·KF 가 없다 (센서 틱이 안 돈다)
        self._measure(ids)

    def _frenet(self, ids):
        x = self.x[ids]
        s, n, mu, i = self.track.frenet(x[:, 0], x[:, 1], x[:, 2], self.idx[ids])
        self.s[ids], self.n[ids], self.mu[ids], self.idx[ids] = s, n, mu, i

    def front_axle(self, x=None):
        """무게중심 상태 → 앞축 중심 (모캡 강체 기준점) 의 (x, y)."""
        x = self.x if x is None else x
        lf = self.P.wheelbase - self.P.p[:, 1]
        return x[:, 0] + lf * torch.cos(x[:, 2]), x[:, 1] + lf * torch.sin(x[:, 2])

    def _measure(self, ids):
        """모캡 측정 (앞축 점 + yaw, 노이즈) → mocap_bridge 와 같은 변환으로 무게중심 추정 → Frenet 관측."""
        x = self.x[ids]
        n = len(ids)
        lf = (self.P.wheelbase - self.P.p[:, 1])[ids]
        fx, fy = x[:, 0] + lf * torch.cos(x[:, 2]), x[:, 1] + lf * torch.sin(x[:, 2])
        noise = torch.zeros(n, 7, device=self.device, dtype=self.dtype)
        for j, key in enumerate(('pos', 'pos', 'yaw', 'vel', 'vel', 'yawrate', 'ws')):
            s = self._sig(key, ids)
            if s is not None:
                noise[:, j] = s * self._randn(n)
        self.meas_noise[ids] = noise
        mode = self.cfg.obs_vel
        if mode in ('kf', 'gyro'):
            # 위치·헤딩도 실차처럼 KF 출력 (뒤축 점) → 무게중심
            lr = self.P.p[ids, 1]
            yaw_m = self.kf_s[ids, 2, 0]
            cx = self.kf_s[ids, 0, 0] + lr * torch.cos(yaw_m)
            cy = self.kf_s[ids, 1, 0] + lr * torch.sin(yaw_m)
        else:
            yaw_m = x[:, 2] + noise[:, 2]
            cx = fx + noise[:, 0] - lf * torch.cos(yaw_m)
            cy = fy + noise[:, 1] - lf * torch.sin(yaw_m)
        s, nn, mu, i = self.track.frenet(cx, cy, yaw_m, self.idx_m[ids])
        self.s_m[ids], self.n_m[ids], self.mu_m[ids], self.idx_m[ids] = s, nn, mu, i
        self.vo[ids] = self._vel_obs(ids, noise)

    # ── 관측 속도 (obs_vel) ─────────────────────────────────────────────
    def _hist_back(self, ids, col, delay):
        """delay s 전 센서 틱의 참값 (ids, ) — 마지막 틱 = 지금."""
        k = min(int(round(float(delay) / self.cfg.mocap_dt)), self.n_hist - 1)
        return self.hist[(self.hist_i - 1 - k) % self.n_hist, ids, col]

    def _kf_body(self, ids):
        """KF 출력 → 차체 (vx, vy_뒤축, r) — mocap_bridge 와 같은 식 (늦은 전역 속도를 KF yaw 로 돌린다)."""
        s = self.kf_s[ids]
        yaw, vxg, vyg = s[:, 2, 0], s[:, 0, 1], s[:, 1, 1]
        c, sn = torch.cos(yaw), torch.sin(yaw)
        return vxg * c + vyg * sn, -vxg * sn + vyg * c, s[:, 2, 1]

    def _vel_obs(self, ids, noise):
        x, mode, d = self.x[ids], self.cfg.obs_vel, self.cfg.obs_delay
        if mode == 'true':
            return x[:, 3:7] + noise[:, 3:7]
        ws = x[:, 6] + noise[:, 6]
        if mode == 'delay':
            v = torch.stack([self._hist_back(ids, j, d[k]) for j, k in enumerate(('vx', 'vy', 'r'))], 1)
            return torch.cat([v + noise[:, 3:6], ws[:, None]], 1)
        vx, vy_rear, r = self._kf_body(ids)
        lr = self.P.p[ids, 1]
        if mode == 'gyro':
            r = self._hist_back(ids, 2, d['gyro']) + noise[:, 5]
            vx = self._hist_back(ids, 3, d['gyro']) + noise[:, 6]
        if self.cfg.obs_vy == 'rot':
            sk = self.kf_s[ids]
            a = sk[:, 2, 0] - r * self.cfg.vy_lag
            vy_rear = -sk[:, 0, 1] * torch.sin(a) + sk[:, 1, 1] * torch.cos(a)
        return torch.stack([vx, vy_rear + lr * r, r, ws], 1)    # vy: 뒤축 → 무게중심

    def _mocap_pose(self):
        """센서 틱의 mocap 측정 — 앞축 점 + 노이즈, mocap_bridge offset 으로 뒤축 점 (전체 환경)."""
        x, ids = self.x_sense, self.allids
        L = self.P.wheelbase
        lf = L - self.P.p[:, 1]
        yaw = x[:, 2]
        sp, sy = self._sig('pos', ids), self._sig('yaw', ids)
        if sy is not None:
            yaw = yaw + sy * self._randn(self.B)
        fx, fy = x[:, 0] + lf * torch.cos(x[:, 2]), x[:, 1] + lf * torch.sin(x[:, 2])
        if sp is not None:
            fx, fy = fx + sp * self._randn(self.B), fy + sp * self._randn(self.B)
        return torch.stack([fx - L * torch.cos(yaw), fy - L * torch.sin(yaw), yaw], 1)

    def _sense_reset(self, ids):
        """리셋 환경의 센서 버퍼를 지금 참값으로 채우고 KF 를 참 상태 (뒤축 점·속도) 에서 시작."""
        if self.cfg.obs_vel == 'true':
            return
        x = self.x[ids]
        self.hist[:, ids] = x[:, 3:7][None]
        lr = self.P.p[ids, 1]
        c, sn = torch.cos(x[:, 2]), torch.sin(x[:, 2])
        vy_r = x[:, 4] - lr * x[:, 5]
        s = torch.zeros(len(ids), 3, 2, device=self.device, dtype=self.dtype)
        s[:, 0, 0], s[:, 1, 0], s[:, 2, 0] = x[:, 0] - lr * c, x[:, 1] - lr * sn, x[:, 2]
        s[:, 0, 1], s[:, 1, 1], s[:, 2, 1] = x[:, 3] * c - vy_r * sn, x[:, 3] * sn + vy_r * c, x[:, 5]
        self.kf_s[ids] = s

    def _sense_tick(self, x):
        """센서 틱 (mocap_dt) — 참값 버퍼 + KF 한 번 (전체 환경)."""
        self.hist[self.hist_i % self.n_hist] = x[:, 3:7]
        self.hist_i += 1
        if self.cfg.obs_vel in ('kf', 'gyro'):
            self.x_sense = x
            zm = self._mocap_pose()
            s = self.kf_s
            dt = self.cfg.mocap_dt
            p = s[..., 0] + dt * s[..., 1]
            inn = zm - p
            inn[:, 2] = torch.remainder(inn[:, 2] + math.pi, 2 * math.pi) - math.pi     # yaw unwrap (mocap_bridge 와 같다)
            k0, k1 = self.kf_K
            self.kf_s = torch.stack([p + k0 * inn, s[..., 1] + k1 * inn], -1)

    # ── 관측 ───────────────────────────────────────────────────────────
    def observation(self):
        k = self.cfg.obs_scale
        x, v = self.x, self.vo
        kap, wl, wr = self.track.lookahead(self.s_m, self.offsets)
        delta_ref = self.prev_a[:, 0] * self.max_steer
        return torch.cat([
            torch.stack([v[:, 0] / k['vx'], v[:, 1] / k['vy'], self.mu_m / k['mu'],
                         self.n_m / k['n'], v[:, 2] / k['r'], x[:, 7] / self.max_steer,
                         delta_ref / self.max_steer, self.prev_a[:, 1], self.ws_ref / k['ws'],
                         v[:, 3] / k['ws']], dim=1),
            kap / k['kappa'], wl / k['w'], wr / k['w']], dim=1)

    # ── 한 주기 ────────────────────────────────────────────────────────
    def action_to_input(self, a):
        """행동 → (ws_ref, delta_ref) — ws_ref 를 적분한다 (self.ws_ref 갱신). δ_ref 에 서보 이득·오프셋 오차."""
        a = torch.clamp(a, -1.0, 1.0)
        delta_ref = a[:, 0] * self.max_steer * self.steer_gain + self.steer_off
        delta_ref = torch.clamp(delta_ref, -self.max_steer, self.max_steer)
        self.ws_ref = torch.clamp(self.ws_ref + a[:, 1] * self.cfg.acc_max * self.cfg.dt, 0.0, self.v_max)
        return torch.stack([self.ws_ref, delta_ref], dim=1), a

    def _drop_mask(self):
        """이번 주기에 명령이 끊긴 환경 (B,) bool. 연속 끊김 (평균 drop_len 주기)."""
        p = self._sig('drop', torch.arange(self.B, device=self.device))
        if self.drop_p is not None:             # 평가용: 환경마다 끊김 확률을 직접 (랜덤화 수준과 따로)
            p = self.drop_p
        if p is None:
            return torch.zeros(self.B, device=self.device, dtype=torch.bool)
        L = max(float(self.dr.get('drop_len', 3.0)), 1.0)
        start = (self.drop_left == 0) & (self._rand(self.B) < p)
        u = torch.clamp(self._rand(self.B), min=1e-6)
        k = torch.clamp(torch.ceil(-(L - 0.5) * torch.log(u)), min=1).long()     # 평균 ≈ L 인 기하분포 근사
        self.drop_left = torch.where(start, k, self.drop_left)
        dropped = self.drop_left > 0
        self.drop_left = torch.clamp(self.drop_left - 1, min=0)
        return dropped

    def _integrate(self, u_new, dropped=None):
        cfg = self.cfg
        sub = cfg.substep
        if cfg.actuator == 'vesc':
            # ★ 바퀴 (1 ms PID·반암시적) 와 차체를 같은 1 ms 로. 차체만 5 ms 로 나눠 적분하면 바퀴↔타이어 결합이 늦어
            #   Simulator(wheel_model='vesc') 와 최대 0.37 m/s 어긋난다 (1 ms 면 소수 셋째 자리까지 같다 — test ⑥)
            sub = min(sub, self.dtr.pid_dt)
        n = max(1, int(math.ceil(cfg.dt / sub - 1e-9)))
        h = cfg.dt / n
        k_lat = torch.round(self.latency / h).long()            # 이 substep 수만큼 직전 명령
        u_old = self.u_applied
        if dropped is not None:                         # 끊긴 환경은 직전 명령 그대로 (새 명령이 안 왔다)
            u_new = torch.where(dropped[:, None], u_old, u_new)
        x = self.x
        act = cfg.actuator
        dtr = self.dtr
        for j in range(n):
            new = (k_lat <= j)
            u = torch.where(new[:, None], u_new, u_old)
            if act == 'vesc':
                dtr_cmd = (k_lat == j) if dropped is None else (k_lat == j) & ~dropped
                if bool(dtr_cmd.any()):
                    self._vesc_command(u_new[:, 0], x[:, 6], dtr_cmd)
                m = max(1, int(round(h / dtr.pid_dt)))
                hp = h / m
                for _ in range(m):
                    rpm = x[:, 6] * dtr.gain
                    dtr.pid(rpm)
                    F = dtr.k_force * dtr.current(rpm)
                    R = self.veh.long_reaction(x, self.P)
                    if cfg.vesc_dR == 'approx':
                        dR = self.veh.long_stiffness(x, self.P)
                    else:
                        eps = 1e-3
                        xe = x.clone()
                        xe[:, 6] = xe[:, 6] + eps
                        dR = torch.clamp((self.veh.long_reaction(xe, self.P) - R) / eps, min=0.0)
                    ws = x[:, 6] + hp * (F - R) / (dtr.m_eq + hp * dR)
                    x = x.clone()
                    x[:, 6] = torch.clamp(ws, min=0.0)
            lim = None
            if act == 'lag' and dtr is not None:
                lim = dtr.accel_limits(x[:, 6], self.P.mass)
            if act == 'vesc' and cfg.vesc_body == 'rk2':
                x = self.veh.rk2(x, u, self.P, h, act)
            elif act == 'vesc' and cfg.vesc_body == 'euler':
                x = self.veh.euler(x, u, self.P, h, act)
            else:
                x = self.veh.rk4(x, u, self.P, h, h, act, lim)
            if cfg.obs_vel != 'true':
                self.sense_acc += h
                if self.sense_acc >= cfg.mocap_dt - 1e-9:
                    self.sense_acc -= cfg.mocap_dt
                    self._sense_tick(x)
        self.u_applied = u_new
        return x

    def _vesc_command(self, speed, ws, mask):
        """mask 인 환경만 /drive 가 도착 (지연 반영)."""
        d = self.dtr
        keep = {k: getattr(d, k).clone() for k in ('mode', 'running', 'braking', 'set_rpm', 'cmd_rpm', 'iq_set')}
        d.command(speed, ws)
        for k, old in keep.items():
            setattr(d, k, torch.where(mask, getattr(d, k), old))

    def step(self, action):
        action = torch.as_tensor(action, device=self.device, dtype=self.dtype)
        u, a = self.action_to_input(action)
        s_old = self.s.clone()
        self.dropped = self._drop_mask()
        self.x = self._integrate(u, self.dropped)
        self.prev_a = a
        self.steps += 1
        self.t += self.cfg.dt
        allids = torch.arange(self.B, device=self.device)
        self._frenet(allids)
        L = self.track.length
        ds = torch.remainder(self.s - s_old + L / 2, L) - L / 2
        self.progress += ds
        wl, wr = self.track.walls(self.s)
        out = (self.n > wl) | (self.n < -wr)
        nonfinite = ~torch.isfinite(self.x).all(dim=1)
        self.slow_t = torch.where(self.x[:, 3] < self.cfg.stuck_v, self.slow_t + self.cfg.dt, torch.zeros_like(ds))
        stuck = (self.slow_t > self.cfg.stuck_time) & ~out & (self.cfg.stuck_time > 0)
        terminated = out | nonfinite | stuck
        # 벽·멈춤 모두 −1. ★ 멈춤에 페널티가 없으면 어려운 코너에서 '멈춰서 종료당하기' 가 벽보다 나은 탈출구가 된다
        #   (2026-11-20 실제로 학습됨 — 결정적 정책이 헤어핀에서 섰다)
        reward = torch.where(terminated, torch.full_like(ds, -1.0), ds)
        truncated = (self.steps >= self.cfg.max_steps) & ~terminated
        # 랩 — 리셋 지점부터 트랙 길이만큼 진행할 때마다
        lap = (self.progress - self.lap_p0 >= L) & ~terminated
        lap_time = torch.where(lap, self.t - self.lap_t0, torch.full_like(ds, float('nan')))
        self.lap_t0 = torch.where(lap, self.t, self.lap_t0)
        self.lap_p0 = torch.where(lap, self.lap_p0 + L, self.lap_p0)
        if nonfinite.any():                 # 관측이 NaN 이 되지 않게 (리셋 전까지)
            self.x[nonfinite] = 0.0
            self._sense_reset(nonfinite.nonzero().squeeze(1))
        self._measure(allids)
        info = dict(out=out, nonfinite=nonfinite, stuck=stuck, dropped=self.dropped.clone(), lap_time=lap_time, progress=ds, n=self.n.clone(),
                    off=torch.clamp(torch.maximum(self.n - wl, -self.n - wr), min=0.0))
        return self.observation(), reward, terminated, truncated, info
