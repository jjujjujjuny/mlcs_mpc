"""
estimation — 실차가 /mpc/state 를 만드는 방식을 시뮬레이터에서 그대로 (ROS 없음). main.py --realtime 이 쓴다.

  실차 (io/mocap_bridge.py): mocap 100 Hz 프레임마다 강체 원점 → 뒤축 점 (offset) → x·y·yaw 를 따로
  정속도 칼만 필터 (PoseKF) → 전역 속도를 KF yaw 로 돌려 차체 vx·vy, r = yaw KF 속도.
  11-10 연구실 로그 실측: vx·r 이 80~90 ms 늦고, 코너에서 늦은 전역 속도를 지금 yaw 로 돌려 가짜 vy (평균 −0.22 m/s).

  mode
    true  참값 (mocap 프레임 시각의 참 상태 — 타이밍 지연만 있고 추정 오차는 없다)
    kf    지금 실차 (mocap_bridge 그대로)
    gyro  연구원님 결정 (2026-11-25): vx = 바퀴 속도 (/sensors/core erpm, VESC 50 Hz), r = 자이로 (VESC IMU),
          둘 다 VESC 샘플 시각 기준 gyro_delay 만큼 늦음. 위치·yaw·vy 는 kf 와 같다 (vy 는 아직 정하지 않음 — KF)

  같은 식의 torch 배치 판은 env/vec/race_env.py (obs_vel) — RL 학습용.
"""
import math
from dataclasses import dataclass

import numpy as np


class PoseKF:
    """io/mocap_bridge.PoseKF 와 같은 식 (축 하나 [위치, 속도], 정속도 모델, 위치만 관측)."""

    def __init__(self, q_pos=0.01, q_vel=1.0, r_meas=1e-4):
        self.x = np.zeros(2)
        self.P = np.eye(2)
        self.Q = np.diag([q_pos, q_vel])
        self.R = r_meas
        self.init = False

    def reset(self, z, v=0.0):
        self.x = np.array([z, v])
        self.P = np.eye(2)
        self.init = True

    def update(self, z, dt):
        if not self.init:
            self.reset(z)
            return self.x.copy()
        F = np.array([[1.0, dt], [0.0, 1.0]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt
        H = np.array([1.0, 0.0])
        y = z - H @ self.x
        S = H @ self.P @ H + self.R
        K = self.P @ H / S
        self.x = self.x + K * y
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P
        return self.x.copy()


@dataclass
class EstimatorConfig:
    mode: str = 'kf'                # true | kf | gyro
    q_pos: float = 0.01             # config/mocap.yaml 과 같다
    q_vel: float = 1.0
    r_meas: float = 1e-4
    pos_noise: float = 0.0005       # m  mocap 위치 노이즈 σ (강체 원점, OptiTrack sub-mm)
    yaw_noise: float = 0.002        # rad
    vy_mode: str = 'kf'             # kf = 늦은 전역 속도를 지금 KF yaw 로 (지금 실차 — 코너에서 가짜 vy)
                                    # rot = 같은 속도를 지연만큼 되돌린 yaw (yaw − r·vy_lag) 로 — 시뮬에서 오차 rms 0.31 → 0.10
                                    # zero = 뒤축 옆 속도 0 (기구학, KF 안 씀 → 지연 없음). MPC 무게중심 vy = lr·r (자이로)
                                    #        11-10 실제 로그: 실제 뒤축 vy rms 0.04~0.08 → 이 방식의 오차가 그만큼
    vy_lag: float = 0.10            # s  KF 속도 지연. 11-10 실제 로그 4 주행: 0.09~0.12 에서 vy 오차 rms 0.18~0.26 → 0.03~0.04
    gyro_delay: float = 0.02        # s  gyro 모드: 자이로·바퀴 속도가 VESC 샘플 시각보다 늦는 정도
    gyro_noise: float = 0.02        # rad/s
    ws_noise: float = 0.01          # m/s

    def check(self):
        if self.vy_mode not in ('kf', 'rot', 'zero'):
            raise ValueError(f'vy_mode: kf | rot | zero ({self.vy_mode!r})')
        if self.mode not in ('true', 'kf', 'gyro'):
            raise ValueError(f'estimator mode: true | kf | gyro ({self.mode!r})')


class StateEstimator:
    """시뮬레이터 상태 → 실차 노드가 받는 (뒤축 pose, 뒤축 twist).

    mocap(t, sim)   mocap 프레임 하나 (시각 t 의 시뮬 참 상태를 잰다)
    vesc(t, sim)    VESC 샘플 하나 (바퀴 속도·자이로)
    state()         지금 발행할 값 — (pose, twist, wheel_speed)
    """

    def __init__(self, cfg=None, rng=None, mocap_offset=None):
        self.cfg = cfg or EstimatorConfig()
        self.cfg.check()
        c = self.cfg
        self.rng = rng or np.random.default_rng(0)
        self.kf = [PoseKF(c.q_pos, c.q_vel, c.r_meas) for _ in range(3)]
        self.t_prev = None
        self.yaw_unw = None
        self.prev_yaw = None
        self.pose = (0.0, 0.0, 0.0)
        self.twist = (0.0, 0.0, 0.0)
        self.true_twist = (0.0, 0.0, 0.0)
        self.ws = 0.0
        self.vesc_hist = []             # (t, ws, r) — gyro 지연용
        self.offset = mocap_offset      # None = 뒤축 점을 바로 잰다 (offset 보정 뒤와 같다)

    def reset(self, sim):
        """참 상태에서 시작 (실차도 정지 중 KF 가 수렴한 뒤 출발한다)."""
        px, py, yaw = sim.pose()
        vx, vy, r = sim.twist()
        c, s = math.cos(yaw), math.sin(yaw)
        for kf, z, v in zip(self.kf, (px, py, yaw), (vx * c - vy * s, vx * s + vy * c, r)):
            kf.reset(z, v)
            kf.P = np.diag([1e-4, 1e-2])
        self.t_prev, self.yaw_unw, self.prev_yaw = sim.t, yaw, yaw
        self.pose, self.twist, self.true_twist = (px, py, yaw), (vx, vy, r), (vx, vy, r)
        self.ws = sim.wheel_speed_meas
        self.vesc_hist = [(sim.t, self.ws, r)]

    # ── 측정 ───────────────────────────────────────────────────────────
    def mocap(self, t, sim):
        c = self.cfg
        px, py, yaw = sim.pose()
        self.true_twist = sim.twist()
        if c.mode == 'true':
            self.pose, self.twist = (px, py, yaw), self.true_twist
            self.t_prev = t
            return
        yaw_m = yaw + c.yaw_noise * self.rng.standard_normal()
        px += c.pos_noise * self.rng.standard_normal()
        py += c.pos_noise * self.rng.standard_normal()
        dt = 0.01 if self.t_prev is None else max(t - self.t_prev, 1e-4)
        self.t_prev = t
        if self.prev_yaw is None:
            self.yaw_unw = yaw_m
        else:
            self.yaw_unw += (yaw_m - self.prev_yaw + math.pi) % (2 * math.pi) - math.pi
        self.prev_yaw = yaw_m
        sx, sy, syaw = (kf.update(z, dt) for kf, z in zip(self.kf, (px, py, self.yaw_unw)))
        fyaw = syaw[0]
        cs, sn = math.cos(fyaw), math.sin(fyaw)
        vx = sx[1] * cs + sy[1] * sn
        vy = -sx[1] * sn + sy[1] * cs
        r = syaw[1]
        if c.mode == 'gyro':
            vx, r = self._vesc_at(t - c.gyro_delay)
        if c.vy_mode == 'rot':
            a = fyaw - r * c.vy_lag
            vy = -sx[1] * math.sin(a) + sy[1] * math.cos(a)
        elif c.vy_mode == 'zero':
            vy = 0.0
        self.pose = (sx[0], sy[0], (fyaw + math.pi) % (2 * math.pi) - math.pi)
        self.twist = (vx, vy, r)

    def vesc(self, t, sim):
        c = self.cfg
        self.ws = sim.wheel_speed_meas + c.ws_noise * self.rng.standard_normal()
        r = sim.twist()[2] + c.gyro_noise * self.rng.standard_normal()
        # vx 는 바퀴 속도를 환산 오차 (speed_scale 1.044) 까지 보정한 값 = 실제 바퀴 둘레 속도 (미끄럼만 오차)
        self.vesc_hist.append((t, sim.wheel_speed + c.ws_noise * self.rng.standard_normal(), r))
        if len(self.vesc_hist) > 50:
            del self.vesc_hist[:-50]
        if c.mode == 'gyro':
            vx, r = self._vesc_at(t - c.gyro_delay)
            self.twist = (vx, self.twist[1], r)

    def _vesc_at(self, t):
        """시각 t 이전 마지막 VESC 샘플의 (바퀴 속도 → vx, 자이로 r)."""
        best = self.vesc_hist[0]
        for h in self.vesc_hist:
            if h[0] <= t + 1e-9:
                best = h
            else:
                break
        return best[1], best[2]

    def state(self):
        return self.pose, self.twist, self.ws
