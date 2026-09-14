#!/usr/bin/env python3
"""
vehicle_model — MPC 가 쓰는 차량 동역학 모델.

■ 왜 운동학(kinematic) 모델부터인가

  HyperMPC 논문(arXiv:2508.06181)의 차량 모델은 동역학 단일트랙 + Pacejka
  타이어이고 상태가 9차원이다:
      [s, n, psi, v_x, v_y, r, friction, wheel_speed, delta]
  이 모델은 타이어 계수(mu, B, C, D)와 질량·관성 실측이 **선행되어야**
  의미가 있다. 지금 이 차량은 VESC 캘리브레이션조차 안 된 상태다
  (speed_to_erpm_gain, 조향 gain/offset 미측정).

  잘못 잰 타이어 모델로 도는 동역학 MPC 는 운동학 MPC 보다 **더** 나쁘다.
  선형 타이어 영역(저속)에서는 두 모델의 예측이 거의 같은데, 동역학 모델은
  v_x 가 0 근처일 때 슬립각 alpha = atan(v_y/v_x) 가 발산해서 솔버가 깨진다.
  그래서 순서는 이렇다:

      ① 운동학 MPC 로 주행 + 데이터 로깅       ← 지금 여기
      ② 로그로 타이어/질량 파라미터 동정
      ③ 동역학 모델로 교체 (params 만 갈아끼움)
      ④ HyperPM 신경망으로 시변 파라미터 예측  ← HyperMPC 본체

  이 파일은 ①~③ 이 같은 인터페이스를 쓰도록 짜여 있다. dynamics() 는
  파라미터 벡터 p 를 받고, MPC 는 그 p 를 매 스텝 바꿔 넣을 수 있다 —
  이게 HyperMPC 의 핵심(시변 파라미터 주입)이 들어갈 자리다.

■ 상태 / 입력

  x = [X, Y, psi, v, delta]     전역좌표 기반 (mocap 이 전역 pose 를 직접 준다)
  u = [a, delta_dot]            가속도 / 조향각속도

  조향각 delta 를 **상태로** 두고 그 변화율을 입력으로 잡는 이유: 서보에는
  물리적 속도 한계가 있어서, delta 를 직접 입력으로 두면 MPC 가 한 스텝에
  -0.36 → +0.36 같은 불가능한 명령을 낸다. 변화율을 제한하면 서보가 실제로
  따라갈 수 있는 궤적만 나온다. f1tenth_stack 의 throttle_interpolator 가
  하던 스무딩을 MPC 가 제약으로 직접 아는 셈이다.
"""

import numpy as np


class VehicleParams:
    """차량 물리 파라미터.

    ★ 여기 값들은 26-jetson 2호차 실측값을 '출발점'으로 빌려온 것이다.
      이 차량으로 재측정하기 전까지는 전부 잠정값이다.
      측정 절차는 docs/CALIBRATION.md 참고.
    """

    def __init__(self,
                 wheelbase=0.33,     # 미측정 — f1tenth 표준 섀시 기준
                 lf=0.176,           # 미측정 — 무게중심~앞축
                 lr=0.144,           # 미측정 — 무게중심~뒤축
                 mass=3.5,           # 미측정
                 max_speed=4.0,
                 min_speed=0.0,
                 max_accel=3.0,
                 max_decel=4.0,
                 max_steer=0.36,     # 미측정 — 기구 스토퍼로 확인 필요
                 max_steer_rate=3.2, # rad/s, 서보 속도 한계
                 mu=0.35):           # 미측정 — 노면 마찰
        self.wheelbase = wheelbase
        self.lf = lf
        self.lr = lr
        self.mass = mass
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
        self.mu = mu

    @classmethod
    def from_dict(cls, d):
        """YAML 에서 읽은 dict → VehicleParams. 모르는 키는 무시한다."""
        known = cls().__dict__.keys()
        return cls(**{k: v for k, v in d.items() if k in known})

    def as_vector(self):
        """HyperMPC 스타일 파라미터 벡터.

        MPC 가 예측 구간의 매 스텝마다 다른 값을 넣을 수 있도록 벡터로 뽑는다.
        지금은 전 구간 동일(=상수 파라미터 MPC)이지만, HyperPM 을 붙이면
        이 벡터가 스텝마다 달라진다.
        """
        return np.array([self.wheelbase, self.mu, self.max_accel], dtype=float)


# 상태/입력 인덱스 — 숫자 리터럴을 코드에 흩뿌리지 않는다
X, Y, PSI, V, DELTA = 0, 1, 2, 3, 4
A, DELTA_DOT = 0, 1
NX, NU = 5, 2


def dynamics(x, u, p):
    """연속시간 운동학 자전거 모델. xdot = f(x, u, p)

    x : [X, Y, psi, v, delta]
    u : [a, delta_dot]
    p : 파라미터 벡터 (VehicleParams.as_vector() 형식)

    numpy 배열과 CasADi 심볼 양쪽에서 동작한다 — 연산이 +,*,sin,cos,tan 뿐이라
    CasADi 심볼을 넣으면 심볼 그래프가, ndarray 를 넣으면 숫자가 나온다.
    acados 로 넘어갈 때 이 함수를 그대로 재사용하기 위한 설계다.
    """
    wheelbase = p[0]

    # CasADi/numpy 양쪽에서 도는 삼각함수 선택
    if hasattr(x[PSI], 'is_symbolic') or type(x).__module__.startswith('casadi'):
        import casadi as ca
        sin, cos, tan = ca.sin, ca.cos, ca.tan
        out = ca.vertcat(
            x[V] * cos(x[PSI]),
            x[V] * sin(x[PSI]),
            x[V] * tan(x[DELTA]) / wheelbase,
            u[A],
            u[DELTA_DOT],
        )
        return out

    return np.array([
        x[V] * np.cos(x[PSI]),
        x[V] * np.sin(x[PSI]),
        x[V] * np.tan(x[DELTA]) / wheelbase,
        u[A],
        u[DELTA_DOT],
    ], dtype=float)


def step_rk4(x, u, p, dt):
    """RK4 이산화.

    오일러가 아니라 RK4 인 이유: 20Hz(dt=0.05) 에서 오일러는 곡선 구간에서
    위치 오차가 눈에 띄게 누적된다. yaw 가 예측 구간(1초) 동안 상당히 도는데
    오일러는 그 회전을 직선으로 근사하기 때문이다. RK4 는 연산이 4배지만
    5차원 모델이라 여전히 μs 단위다 — 정확도를 살 가치가 충분하다.
    """
    k1 = dynamics(x, u, p)
    k2 = dynamics(x + dt / 2.0 * k1, u, p)
    k3 = dynamics(x + dt / 2.0 * k2, u, p)
    k4 = dynamics(x + dt * k3, u, p)
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def normalize_angle(a):
    """각도를 [-pi, pi] 로 감는다.

    ★ MPC 비용에서 이걸 빼먹으면 치명적이다. psi=+3.13 인 차가 목표
      psi=-3.13 을 추종할 때 감지 않으면 오차가 6.26 rad 으로 보이고,
      MPC 는 한 바퀴를 거꾸로 돌려 그 오차를 없애려 든다.
    """
    return (a + np.pi) % (2.0 * np.pi) - np.pi
