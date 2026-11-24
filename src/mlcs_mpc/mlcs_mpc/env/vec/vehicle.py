"""
vec.vehicle — env/vehicle_model.py 를 torch 로 옮긴 배치 (벡터화) 차 모델. 강화학습 환경 (vec/race_env.py) 이 쓴다.

  vm = VecVehicle.from_model(VehicleModel.from_yaml(...), num_envs=400)
  x = vm.rk4(x, u, dt=0.05, substep=0.001)       x (B, 8), u (B, 2) — 전부 torch 텐서, CPU·CUDA·MPS 같은 코드

■ 식은 VehicleModel.cartesian() 과 한 줄씩 같다 (Pacejka 단일 궤적 + 액추에이터 1차 지연 + 저속 운동학 섞기).
  test/test_vec_vehicle.py 가 casadi 식과 숫자로 대조한다 (float64 에서 1e-9 수준).
  벡터화 방식은 codemoo/vectorized-vehicle-control 을 따랐다 — 상태를 (환경 수, 상태 수) 텐서로 두고,
  고정 간격 RK4 substep 을 배치 전체에 한 번에 돌리며, 환경은 스스로 리셋하지 않는다 (reset(env_ids)).

■ 차마다 다른 값 (도메인 랜덤화) — 아래는 모두 (B,) 또는 (B, 30) 텐서라 환경마다 다르게 줄 수 있다.
      p (B, 30)       차량 10 + 앞타이어 10 + 뒤타이어 10 (vehicle_model 과 같은 순서)
      friction (B,)   타이어 힘에 곱한다
      mass (B,), wheelbase (B,), steering_tau (B,), wheel_speed_tau (B,)
  casadi 모델에서 mass·L·tau 는 생성 코드에 굳은 상수지만 여기서는 바꿀 수 있다.

■ 상태 x = [X, Y, yaw, vx, vy, r, ws, delta] (X, Y 무게중심), 입력 u = [ws_ref, delta_ref] — Simulator 와 같다.
"""
import math
from dataclasses import dataclass

import torch

G = 9.81
DIV_EPS = 1e-6          # vehicle_model.DIV_EPS 와 같다
KIN_TAU = 0.05          # vehicle_model.body_kinematic 의 tau


@dataclass
class VecParams:
    """환경마다의 차 값. 모두 같은 device·dtype 의 텐서."""
    p: torch.Tensor                 # (B, 30)
    friction: torch.Tensor          # (B,)
    mass: torch.Tensor              # (B,)
    wheelbase: torch.Tensor         # (B,)
    steering_tau: torch.Tensor      # (B,)
    wheel_speed_tau: torch.Tensor   # (B,)

    def index(self, ids):
        return VecParams(*(getattr(self, f)[ids] for f in self.__dataclass_fields__))

    def assign(self, ids, other):
        for f in self.__dataclass_fields__:
            getattr(self, f)[ids] = getattr(other, f)

    def clone(self):
        return VecParams(*(getattr(self, f).clone() for f in self.__dataclass_fields__))


class VecVehicle:

    def __init__(self, base: VecParams, slip_eps=1e-12, v_lo=0.2, v_hi=0.4):
        self.base = base                # 랜덤화의 기준 (B 개 모두 같은 값)
        self.slip_eps = float(slip_eps)
        self.v_lo, self.v_hi = float(v_lo), float(v_hi)

    @classmethod
    def from_model(cls, model, num_envs, device='cpu', dtype=torch.float32, v_lo=0.2, v_hi=0.4):
        """env/vehicle_model.VehicleModel → 같은 값을 B 개 복사."""
        def full(v):
            return torch.full((num_envs,), float(v), device=device, dtype=dtype)
        p = torch.as_tensor(model.p, device=device, dtype=dtype).expand(num_envs, -1).clone()
        base = VecParams(p, full(model.friction), full(model.mass), full(model.L),
                         full(model.steering_tau), full(model.wheel_speed_tau))
        return cls(base, slip_eps=model.slip_eps, v_lo=v_lo, v_hi=v_hi)

    # ── 식 ─────────────────────────────────────────────────────────────
    def tire(self, alpha, sigma, t):
        """VehicleModel.tire — t (B, 10). 반환 (Fx, Fy) 정규화 힘."""
        sx_p, a_p, by, cy, dy, ey, bx, cx, dx, ex = t.unbind(-1)
        sx_n = sigma / sx_p
        a_n = alpha * 180.0 / math.pi / a_p
        s2 = sx_n ** 2 + a_n ** 2
        s = torch.sqrt(s2 + self.slip_eps) if self.slip_eps > 0 else torch.sqrt(s2)
        sx_m, a_m = s * sx_p, s * a_p
        fy = (a_n / s) * dy * torch.sin(cy * torch.atan(by * a_m + ey * (by * a_m - torch.atan(by * a_m))))
        fx = (sx_n / s) * dx * torch.sin(cx * torch.atan(bx * sx_m + ex * (bx * sx_m - torch.atan(bx * sx_m))))
        return fx, -fy

    def body(self, vx, vy, r, ws, delta, P: VecParams):
        """VehicleModel.body — (vx, vy, r)'."""
        p = P.p
        m, L, fric = P.mass, P.wheelbase, P.friction
        I_z, lr, cd0, cd2, cd1 = p[:, 0], p[:, 1], p[:, 2], p[:, 3], p[:, 4]
        lf = L - lr
        f_drag = cd0 * torch.sign(vx) + cd1 * vx + cd2 * vx * vx
        a_f = torch.atan((vy + lf * r) / (vx + DIV_EPS)) - delta
        a_r = torch.atan((vy - lf * r) / (vx + DIV_EPS))        # 원저자 식 그대로 (lf)
        v_f = vx * torch.cos(delta) + (vy + r * lr) * torch.sin(delta)
        s_f = (ws - v_f) / (v_f + DIV_EPS)
        s_r = (ws - vx) / (vx + DIV_EPS)
        fx_f, fy_f = self.tire(a_f, s_f, p[:, 10:20])
        fx_r, fy_r = self.tire(a_r, s_r, p[:, 20:30])
        fz_f, fz_r = m * G * lr / L, m * G * lf / L
        fxf, fyf = fz_f * fx_f * fric, fz_f * fy_f * fric
        fxr, fyr = fz_r * fx_r * fric, fz_r * fy_r * fric
        sd, cd = torch.sin(delta), torch.cos(delta)
        vx_d = (fxr + fxf * cd - fyf * sd - f_drag + m * vy * r) / m
        vy_d = (fxf * sd + fyr + fyf * cd - m * vx * r) / m
        r_d = ((fxf * sd + fyf * cd) * lf - fyr * lr) / I_z
        return vx_d, vy_d, r_d

    def body_kinematic(self, vx, vy, r, ws, delta, P: VecParams):
        lr, L, tan_d = P.p[:, 1], P.wheelbase, torch.tan(delta)
        return (ws - vx) / KIN_TAU, (vx * tan_d * lr / L - vy) / KIN_TAU, (vx * tan_d / L - r) / KIN_TAU

    def long_reaction(self, x, P: VecParams):
        """바퀴를 거꾸로 미는 힘 R (N) — 타이어 종방향 힘 합 (저속은 운동학 모델의 m·(ws − vx)/τ 와 섞음).
        env/drivetrain.reaction_function 과 같은 식 (actuator='vesc' 의 바퀴 방정식 m_eq·ws' = F − R)."""
        X, Y, yaw, vx, vy, r, ws, delta = x.unbind(-1)
        p = P.p
        m, L, fric = P.mass, P.wheelbase, P.friction
        lr = p[:, 1]
        lf = L - lr
        a_f = torch.atan((vy + lf * r) / (vx + DIV_EPS)) - delta
        a_r = torch.atan((vy - lf * r) / (vx + DIV_EPS))
        v_f = vx * torch.cos(delta) + (vy + r * lr) * torch.sin(delta)
        s_f = (ws - v_f) / (v_f + DIV_EPS)
        s_r = (ws - vx) / (vx + DIV_EPS)
        fx_f, _ = self.tire(a_f, s_f, p[:, 10:20])
        fx_r, _ = self.tire(a_r, s_r, p[:, 20:30])
        R = (m * G * lr / L) * fx_f * fric + (m * G * lf / L) * fx_r * fric
        if self.v_hi > self.v_lo:
            w = torch.clamp((vx.abs() - self.v_lo) / (self.v_hi - self.v_lo), 0.0, 1.0)
            R = w * R + (1 - w) * m * (ws - vx) / KIN_TAU
        return R

    def long_stiffness(self, x, P: VecParams):
        """dR/dws 근사 — 타이어 선형 영역 기울기 Σ fz·μ·Dx·Cx·Bx / v (저속은 운동학 m/τ 와 섞음).
        반암시적 바퀴 적분의 안정화 분모에만 쓰므로 근사로 충분하다 (long_reaction 두 번 → 한 번)."""
        vx = x[:, 3]
        p, m, L, fric = P.p, P.mass, P.wheelbase, P.friction
        lr = p[:, 1]
        lf = L - lr
        kf = p[:, 18] * p[:, 17] * p[:, 16]          # 앞 Dx·Cx·Bx
        kr = p[:, 28] * p[:, 27] * p[:, 26]          # 뒤
        k = ((m * G * lr / L) * kf + (m * G * lf / L) * kr) * fric / (vx.abs() + 0.05)
        if self.v_hi > self.v_lo:
            w = torch.clamp((vx.abs() - self.v_lo) / (self.v_hi - self.v_lo), 0.0, 1.0)
            k = w * k + (1 - w) * m / KIN_TAU
        return k

    def euler(self, x, u, P: VecParams, h, actuator='lag'):
        """오일러 한 걸음 — vesc 1 ms 차체용 (vesc_body='euler')."""
        x = x + h * self.deriv(x, u, P, actuator)
        x = x.clone()
        x[:, 2] = torch.remainder(x[:, 2] + math.pi, 2 * math.pi) - math.pi
        return x

    def deriv(self, x, u, P: VecParams, actuator='lag', ws_lim=None):
        """VehicleModel.cartesian (v_lo~v_hi 운동학 섞기 포함) — x (B, 8), u (B, 2) → x' (B, 8).

        actuator 'lag'  = 식 그대로 (ws, δ 가 1차 지연으로 따라감, 논문 식 1). ws_lim = (구동 상한, 제동 크기) m/s² 를
                          주면 ws' 를 그 안으로 자른다 (모터 전류·배터리 전류 한계 — vec/drivetrain.accel_limits)
                 'none' = 액추에이터 모델 없음 (논문 Table I 'RL w/o am') — ws, δ 는 매 substep 명령값으로 둔다
                 'vesc' = ws 는 바깥 (vec/drivetrain + 바퀴 방정식) 에서 정한다 → 여기서는 ws' = 0, δ 는 1차 지연
        """
        X, Y, yaw, vx, vy, r, ws, delta = x.unbind(-1)
        dyn = self.body(vx, vy, r, ws, delta, P)
        if self.v_hi > self.v_lo:
            kin = self.body_kinematic(vx, vy, r, ws, delta, P)
            w = torch.clamp((vx.abs() - self.v_lo) / (self.v_hi - self.v_lo), 0.0, 1.0)
            dyn = tuple(w * a + (1 - w) * b for a, b in zip(dyn, kin))
        if actuator == 'none':
            ws_d = d_d = torch.zeros_like(ws)
        else:
            d_d = (u[:, 1] - delta) / P.steering_tau
            if actuator == 'vesc':
                ws_d = torch.zeros_like(ws)
            else:
                ws_d = (u[:, 0] - ws) / P.wheel_speed_tau
                if ws_lim is not None:
                    ws_d = torch.maximum(torch.minimum(ws_d, ws_lim[0]), -ws_lim[1])
        c, s = torch.cos(yaw), torch.sin(yaw)
        return torch.stack((vx * c - vy * s, vx * s + vy * c, r, dyn[0], dyn[1], dyn[2], ws_d, d_d), dim=-1)

    def rk2(self, x, u, P: VecParams, h, actuator='lag', ws_lim=None):
        """중점법 한 걸음 (h). actuator='vesc' 의 1 ms 차체 적분용 — RK4 의 절반 계산 (test ⑥ 으로 정확도 확인)."""
        if actuator == 'none':
            x = x.clone()
            x[:, 6], x[:, 7] = u[:, 0], u[:, 1]
        k1 = self.deriv(x, u, P, actuator, ws_lim)
        x = x + h * self.deriv(x + 0.5 * h * k1, u, P, actuator, ws_lim)
        x = x.clone()
        x[:, 2] = torch.remainder(x[:, 2] + math.pi, 2 * math.pi) - math.pi
        return x

    def rk4(self, x, u, P: VecParams, dt, substep=0.001, actuator='lag', ws_lim=None):
        """dt 동안 u 를 붙잡고 고정 간격 RK4 (Simulator 와 같은 방식). yaw 는 (−π, π] 로 감는다."""
        n = max(1, int(math.ceil(dt / substep - 1e-9)))
        h = dt / n
        if actuator == 'none':                  # 명령이 곧 실제 (지연 없음)
            x = x.clone()
            x[:, 6], x[:, 7] = u[:, 0], u[:, 1]
        for _ in range(n):
            k1 = self.deriv(x, u, P, actuator, ws_lim)
            k2 = self.deriv(x + 0.5 * h * k1, u, P, actuator, ws_lim)
            k3 = self.deriv(x + 0.5 * h * k2, u, P, actuator, ws_lim)
            k4 = self.deriv(x + h * k3, u, P, actuator, ws_lim)
            x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        x = x.clone()
        x[:, 2] = torch.remainder(x[:, 2] + math.pi, 2 * math.pi) - math.pi
        return x
