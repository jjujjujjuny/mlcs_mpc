"""
drivetrain — 실차의 저수준 구동 경로를 그대로 옮긴 모델 (시뮬레이터 wheel_model='vesc'). ROS 없음.

  /drive (speed m/s)
    → AckermannToVesc     vesc_ackermann 노드 + 우리 패치 (third_party/patches/vesc-stop-brake.patch)
                          ERPM 명령, 또는 정지 명령이면 제동 전류 (빠르면 속도 0 → VESC PID 제동, 0.5 m/s 아래에서 10 A)
    → VescSpeedController VESC 펌웨어 FOC 속도 PID (bldc master, motor/foc_math.c foc_run_pid_control_speed,
                          motor/mcpwm_foc.c 의 전류 제한·제동 부분). 설정은 docs/vesc/vesc6_f1tenth_1124.xml
    → 모터 q축 전류 iq     전류 루프 (30 kHz) 는 이상적으로 본다 — iq = 제한을 거친 iq 목표
    → 바퀴 힘 F = 1.5·λ·iq·(gain·2π/60)   [N]   (gain = speed_to_erpm_gain, erpm = gain·ws)
    → 바퀴 속도 ws         m_eq·ws' = F − (Fx_앞 + Fx_뒤)   m_eq = 모터 회전자·바퀴 관성을 바퀴 둘레로 옮긴 질량 (★ 추정)

■ 펌웨어 속도 PID (sp_pid_loop_rate 마다, 우리 설정 5 → 1000 Hz)
    set_rpm 을 command_rpm 쪽으로 s_pid_ramp_erpms_s·dt 만큼 옮긴다 (램프), [l_min_erpm, l_max_erpm] 로 자른다
    error = set_rpm − rpm
    |set_rpm| < s_pid_min_erpm 이면 i·이전 오차 초기화, iq = 0, 그리고 전류 루프가 duty 0 으로 묶는다 (상 단락 제동)
    p = error·kp/20,  d = (error − prev)·(kd/dt)/20 → EMA (s_pid_kd_filter)
    out = clip(p + i + d, ±1)            ← i 는 갱신 전 값 (펌웨어 순서 그대로)
    i += error·ki·dt/20, clip ±1, ki ≈ 0 이면 i = 0
    s_pid_allow_braking 이 0 이면 회전 방향과 반대 출력을 0 으로
    iq_set = out · l_current_max · l_current_max_scale

■ 전류 루프의 제한 (mcpwm_foc.c)
    제동 전류 모드  iq = −sign(rpm)·min(|I|, |l_current_min|)    (0 근처는 brake_rpm_eps 로 부드럽게 — ★ 근사)
    duty 0 (속도 모드, |set_rpm| < min_erpm)   vq = 0 → 정상상태 iq = −ω_e·λ/R  (상 단락, 속도에 비례하는 제동)
    입력 전류  mod_q = 1.5·vq/V_bus, vq ≈ R·iq(지난 실제값) + ω_e·λ → iq ∈ [I_in_min, I_in_max]/mod_q  (회생 −5 A 가 고속 제동을 묶는다)
    상 전류    mod_q > 0 이면 iq ∈ [l_current_min, l_current_max], 아니면 [−l_current_max, −l_current_min]

■ 센서리스 개루프 출발 (mcpwm_foc.c, ★ 근사 — 각도 오차·관측기는 모델링하지 않고 '보조 전류' 만)
    속도 모드로 돌고 있고 (제동·duty 0·놓임이면 타이머 0) |rpm| < 개루프 rpm 이면 hyst 타이머 += dt (빠르면 −= dt).
    개루프 rpm = map(|iq|, 0 → l_current_max, foc_openloop_rpm_low·foc_openloop_rpm → foc_openloop_rpm).
    hyst 타이머 > foc_sl_openloop_hyst 이면 foc_sl_openloop_time 동안 개루프 — 그동안 구동 방향 q축 전류를
    최소 foc_sl_openloop_boost_q A 로 (출발 떨림 대책, 11-24 설정 5 A). 보조 0 이면 개루프는 전류에 영향이 없다.
    전류 루프 이득 (foc_current_kp/ki) 과 인덕턴스 L 은 쓰지 않는다 (전류 루프를 이상적으로 본다).

■ 모델에 없는 것 — 센서리스 관측기·PLL 의 저속 속도 추정 오차 (foc_sl_erpm 2500 = 0.52 m/s 아래), 개루프의 각도 오차,
  온도·전력 감액 (l_temp_*, l_watt_*), d축 전류·약계자, 전류 루프 동특성, 배터리 전압 처짐, VESC 명령 timeout.
  후진 (vx < 0) 은 다루지 않는다 — 공용 차 모델의 슬립 (ws − vx)/(vx + ε) 이 vx > 0 에서만 맞아서
  후진하면 바퀴가 발산한다 (MPC 도 전진만 쓴다).
  펌웨어는 bldc master 기준이다 — 차의 펌웨어 버전이 다르면 PID 순서 등이 다를 수 있다.
"""
import math
import os
import xml.etree.ElementTree as ET

import casadi as cs

from mlcs_mpc.env.vehicle_model import DIV_EPS, G

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..'))
VESC_XML = os.path.join(REPO, 'docs', 'vesc', 'vesc6_f1tenth_1124.xml')          # 2026-11-24 최신 (예전: vesc6_upennmy_foc.xml)
PID_RATES_HZ = (25, 50, 100, 250, 500, 1000, 2500, 5000, 10000)    # datatypes.h PID_RATE enum 순서

# docs/vesc/vesc6_f1tenth_1124.xml (2026-11-24 최신 — 속도 PID 0.003/0.0002/0.0002, 모터 재측정 R 7.6 mΩ·L 2.89 µH,
# 개루프 출발 1800 erpm·0.1 s·보조 5 A) 의 값. XML 이 없을 때 쓴다. 예전 (11-08, docs/vesc/vesc6_upennmy_foc.xml):
# PID 0.004/0.002/0.0008, R 11.1 mΩ, λ 0.000726, 개루프 1200 erpm·0.01 s·보조 0 (= 개루프 효과 없음).
FOC_DEFAULTS = dict(
    l_current_max=60.0, l_current_min=-60.0, l_in_current_max=60.0, l_in_current_min=-5.0,
    l_current_max_scale=1.0, l_current_min_scale=1.0, l_max_erpm=30000.0, l_min_erpm=-30000.0,
    s_pid_kp=0.003, s_pid_ki=0.0002, s_pid_kd=0.0002, s_pid_kd_filter=0.1, s_pid_min_erpm=500.0,
    s_pid_allow_braking=1.0, s_pid_ramp_erpms_s=25000.0, sp_pid_loop_rate=5.0,
    foc_motor_r=0.0076, foc_motor_flux_linkage=0.000728, cc_min_current=0.2, si_motor_poles=14.0,
    si_battery_cells=3.0,
    foc_openloop_rpm=1800.0, foc_openloop_rpm_low=1.0, foc_sl_openloop_hyst=0.1, foc_sl_openloop_time=0.1,
    foc_sl_openloop_boost_q=5.0)


class FocConfig:
    """VESC 모터 설정 (VESC Tool XML 의 이름 그대로)."""

    def __init__(self, **over):
        v = dict(FOC_DEFAULTS)
        v.update({k: float(x) for k, x in over.items() if k in FOC_DEFAULTS})
        self.__dict__.update(v)
        self.source = 'defaults'

    @classmethod
    def from_xml(cls, path=VESC_XML):
        if not path or not os.path.isfile(path):
            return cls()
        root = ET.parse(path).getroot()
        vals = {}
        for k in FOC_DEFAULTS:
            e = root.find(k)
            if e is not None and e.text is not None:
                vals[k] = float(e.text)
        c = cls(**vals)
        c.source = path
        return c

    @property
    def pid_rate(self):
        return float(PID_RATES_HZ[int(self.sp_pid_loop_rate)])


# ── /drive → VESC 명령 ──────────────────────────────────────────────────
class AckermannToVesc:
    """vesc_ackermann ackermann_to_vesc 노드 + vesc-stop-brake.patch. 명령 하나가 들어올 때마다 command()."""

    def __init__(self, vesc, brake_current=10.0, brake_speed_threshold=0.05, brake_handoff_speed=0.5):
        self.vesc = vesc
        self.brake_current = float(brake_current)
        self.threshold = float(brake_speed_threshold)
        self.handoff = float(brake_handoff_speed)
        self.braking = False

    def reset(self):
        self.braking = False

    def command(self, speed, wheel_speed, state_fresh=True):
        """→ ('speed', erpm) 또는 ('brake', 전류 A). erpm 은 vesc_driver 의 speed_min/max 로 자른 값."""
        stop = self.brake_current > 0.0 and abs(speed) < self.threshold
        if not stop:
            self.braking = False
        elif not self.braking:
            # 빠르면 VESC PID 제동 (강하다). 느리거나 상태를 모르면 제동 전류로 넘긴다
            if self.handoff <= 0.0 or not state_fresh or abs(wheel_speed) < self.handoff:
                self.braking = True
        if stop and self.braking:
            return 'brake', self.brake_current
        v = self.vesc
        erpm = v.speed_to_erpm_gain * speed + v.speed_to_erpm_offset
        return 'speed', min(max(erpm, v.speed_min), v.speed_max)


# ── VESC 펌웨어 ─────────────────────────────────────────────────────────
class VescSpeedController:
    """FOC 속도 모드·제동 전류 모드. set_pid_speed / set_brake_current → pid(dt, rpm) → motor_current(rpm)."""

    def __init__(self, cfg=None, v_bus=None, brake_rpm_eps=100.0):
        self.cfg = cfg or FocConfig.from_xml()
        self.v_bus = float(v_bus) if v_bus else 3.8 * self.cfg.si_battery_cells    # 3S 쉬는 전압 약 11.4 V
        self.brake_rpm_eps = float(brake_rpm_eps)
        self.reset()

    def reset(self):
        self.mode = 'current'           # 시작 = 놓인 상태 (전류 0)
        self.running = False
        self.set_rpm = self.command_rpm = 0.0
        self.i_term = self.prev_error = self.d_filter = 0.0
        self.iq_set = 0.0
        self.iq = 0.0                   # 지난 실제 전류 (mod_q 계산용 — 펌웨어도 측정값을 거른 mod_q_filter 를 쓴다)
        self.ol_hyst = 0.0              # 개루프 hyst 타이머 s
        self.ol_left = 0.0              # 개루프 남은 시간 s (> 0 이면 개루프)

    # mcpwm_foc_set_pid_speed
    def set_pid_speed(self, rpm, rpm_now):
        c = self.cfg
        if c.s_pid_ramp_erpms_s > 0.0:
            if self.mode != 'speed' or not self.running:
                self.set_rpm = rpm_now
            self.command_rpm = rpm
        else:
            self.set_rpm = rpm
        self.mode = 'speed'
        if not self.running and abs(rpm) >= c.s_pid_min_erpm:
            self.running = True

    # mcpwm_foc_set_brake_current
    def set_brake_current(self, current):
        self.mode = 'brake'
        self.iq_set = current
        if abs(current) < self.cfg.cc_min_current:
            return
        self.running = True

    def openloop_rpm(self):
        c = self.cfg
        f = min(abs(self.iq) / max(c.l_current_max, 1e-9), 1.0)
        return c.foc_openloop_rpm * (c.foc_openloop_rpm_low + (1.0 - c.foc_openloop_rpm_low) * f)

    def openloop_step(self, dt, rpm):
        """개루프 타이머 (PID 주기마다). 속도 모드로 돌고 있지 않으면 0."""
        c = self.cfg
        if not (self.running and self.mode == 'speed' and not self.duty_zero):
            self.ol_hyst = self.ol_left = 0.0
            return
        if abs(rpm) < self.openloop_rpm():
            self.ol_hyst += dt
        elif self.ol_hyst > 0.0:
            self.ol_hyst = max(self.ol_hyst - dt, 0.0)
        if self.ol_hyst > c.foc_sl_openloop_hyst and self.ol_left <= 1e-4:
            self.ol_left = c.foc_sl_openloop_time
        if self.ol_left > 0.0:
            self.ol_left = max(self.ol_left - dt, 0.0)

    @property
    def openloop(self):
        return self.ol_left > 0.0

    # foc_run_pid_control_speed
    def pid(self, dt, rpm):
        c = self.cfg
        self.openloop_step(dt, rpm)
        if self.mode != 'speed':
            self.i_term = self.prev_error = self.d_filter = 0.0
            return
        if c.s_pid_ramp_erpms_s > 0.0:
            step = c.s_pid_ramp_erpms_s * dt
            d = self.command_rpm - self.set_rpm
            self.set_rpm = self.command_rpm if abs(d) <= step else self.set_rpm + math.copysign(step, d)
            # 센서리스는 encoder_index_found() 가 true 라 foc_openloop_rpm 으로 자르지 않는다
            self.set_rpm = min(max(self.set_rpm, c.l_min_erpm), c.l_max_erpm)
        error = self.set_rpm - rpm
        if abs(self.set_rpm) < c.s_pid_min_erpm:
            self.i_term = 0.0
            self.prev_error = error
            self.iq_set = 0.0
            return
        p_term = error * c.s_pid_kp / 20.0
        d_term = (error - self.prev_error) * (c.s_pid_kd / dt) / 20.0
        self.d_filter += c.s_pid_kd_filter * (d_term - self.d_filter)        # ema_step
        d_term = self.d_filter
        self.prev_error = error
        out = min(max(p_term + self.i_term + d_term, -1.0), 1.0)
        self.i_term = min(max(self.i_term + error * c.s_pid_ki * dt / 20.0, -1.0), 1.0)
        if c.s_pid_ki < 1e-9:
            self.i_term = 0.0
        if not c.s_pid_allow_braking:
            if rpm > 20.0 and out < 0.0:
                out = 0.0
            if rpm < -20.0 and out > 0.0:
                out = 0.0
        self.iq_set = out * c.l_current_max * c.l_current_max_scale

    @property
    def duty_zero(self):
        """속도 모드인데 set_rpm 이 min_erpm 아래 → duty 0 (상 단락) 으로 묶인다."""
        return self.mode == 'speed' and abs(self.set_rpm) < self.cfg.s_pid_min_erpm

    def motor_current(self, rpm):
        """전류 루프가 만드는 q축 전류 A (이상적인 전류 제어 + 펌웨어 제한)."""
        c = self.cfg
        self.iq = self._current(rpm)
        return self.iq

    def _current(self, rpm):
        c = self.cfg
        if not self.running:
            return 0.0
        we = rpm * 2.0 * math.pi / 60.0
        i_abs_max = max(abs(c.l_current_max), abs(c.l_current_min))
        if self.duty_zero:
            iq = -we * c.foc_motor_flux_linkage / c.foc_motor_r
            return min(max(iq, -i_abs_max), i_abs_max)
        iq = self.iq_set
        b = c.foc_sl_openloop_boost_q
        if self.mode == 'speed' and b > 0 and self.ol_left > 0.0 and self.set_rpm != 0.0:
            sgn = math.copysign(1.0, self.set_rpm)
            if iq * sgn >= 0.0:                     # 구동 방향일 때만 (PID 가 제동 중이면 그대로)
                iq = sgn * max(abs(iq), b)
        if self.mode == 'brake':
            mag = min(abs(iq), -c.l_current_min * c.l_current_min_scale)
            iq = -min(max(rpm / self.brake_rpm_eps, -1.0), 1.0) * mag
        elif self.mode != 'speed':
            return 0.0
        vq = c.foc_motor_r * self.iq + we * c.foc_motor_flux_linkage
        mod_q = 1.5 * vq / self.v_bus
        if mod_q > 0.001:
            iq = min(max(iq, c.l_in_current_min / mod_q), c.l_in_current_max / mod_q)
        elif mod_q < -0.001:
            iq = min(max(iq, c.l_in_current_max / mod_q), c.l_in_current_min / mod_q)
        lo, hi = c.l_current_min * c.l_current_min_scale, c.l_current_max * c.l_current_max_scale
        if mod_q > 0.0:
            iq = min(max(iq, lo), hi)
        else:
            iq = min(max(iq, -hi), -lo)
        return min(max(iq, -i_abs_max), i_abs_max)


# ── 타이어 반력 (바퀴 축) ─────────────────────────────────────────────
def wheel_forces(model, vx, vy, r, friction, ws, delta, p):
    """(fxf, fyf, fxr, fyr) N — VehicleModel.body() 와 같은 식 (앞 Fx 는 바퀴 좌표). test_drivetrain 이 대조한다."""
    m, L = model.mass, model.L
    lr = p[1]
    lf = L - lr
    a_f = cs.atan((vy + lf * r) / (vx + DIV_EPS)) - delta
    a_r = cs.atan((vy - lf * r) / (vx + DIV_EPS))
    v_f = vx * cs.cos(delta) + (vy + r * lr) * cs.sin(delta)
    s_f = (ws - v_f) / (v_f + DIV_EPS)
    s_r = (ws - vx) / (vx + DIV_EPS)
    fx_f, fy_f = model.tire(a_f, s_f, [p[10 + i] for i in range(10)])
    fx_r, fy_r = model.tire(a_r, s_r, [p[20 + i] for i in range(10)])
    fz_f, fz_r = m * G * lr / L, m * G * lf / L
    return fz_f * fx_f * friction, fz_f * fy_f * friction, fz_r * fx_r * friction, fz_r * fy_r * friction


def reaction_function(model, v_lo, v_hi, tau_kin=0.05):
    """casadi (x8, p30) → (R, dR/dws). R = 바퀴를 거꾸로 미는 힘 N.

    동적 모델: 타이어 종방향 힘 합 (Fx_앞 + Fx_뒤). 저속 운동학 모델 (simulator 의 섞기와 같은 무게 w):
    vx' = (ws − vx)/tau_kin 이므로 차를 미는 힘 m·(ws − vx)/tau_kin.
    """
    x, p = cs.SX.sym('x', 8), cs.SX.sym('p', 30)
    vx, vy, r, ws, delta = x[3], x[4], x[5], x[6], x[7]
    fxf, _, fxr, _ = wheel_forces(model, vx, vy, r, model.friction, ws, delta, p)
    R = fxf + fxr
    if v_hi > v_lo:
        w = cs.fmin(cs.fmax((cs.fabs(vx) - v_lo) / (v_hi - v_lo), 0.0), 1.0)
        R = w * R + (1 - w) * model.mass * (ws - vx) / tau_kin
    return cs.Function('f_reaction', [x, p], [R, cs.jacobian(R, ws)])


# ── 묶음 ────────────────────────────────────────────────────────────────
class Drivetrain:
    """시뮬레이터가 쓰는 구동 경로. command() 는 /drive 가 올 때, force() 는 적분 substep 마다."""

    # ★ 추정값: 모터 회전자 ~3e-5 kg·m² × (기어 3 / 바퀴 반지름 0.0415)² ≈ 0.16 kg + 바퀴 4개 ≈ 0.17 kg
    M_EQ_DEFAULT = 0.3

    def __init__(self, vesc, cfg=None, m_eq=M_EQ_DEFAULT, v_bus=None, **brake):
        self.vesc = vesc
        self.cfg = cfg or FocConfig.from_xml()
        self.ack = AckermannToVesc(vesc, **brake)
        self.ctrl = VescSpeedController(self.cfg, v_bus=v_bus)
        self.m_eq = float(m_eq)
        self.k_force = 1.5 * self.cfg.foc_motor_flux_linkage * vesc.speed_to_erpm_gain * 2.0 * math.pi / 60.0  # N/A
        self.pid_period = 1.0 / self.cfg.pid_rate
        self._pid_t = 0.0
        self.iq = 0.0
        self.last_cmd = ('speed', 0.0)

    def reset(self):
        self.ack.reset()
        self.ctrl.reset()
        self._pid_t = 0.0
        self.iq = 0.0

    def rpm(self, ws):
        return self.vesc.erpm(ws) - self.vesc.speed_to_erpm_offset

    def command(self, speed, ws):
        kind, val = self.ack.command(speed, ws)
        if kind == 'brake':
            self.ctrl.set_brake_current(val)
        else:
            self.ctrl.set_pid_speed(val, self.rpm(ws))
        self.last_cmd = (kind, val)

    def force(self, h, ws):
        """substep h 동안의 바퀴 힘 N (PID 는 자기 주기로, 전류는 지금 회전수로)."""
        rpm = self.rpm(ws)
        self._pid_t += h
        while self._pid_t >= self.pid_period - 1e-12:
            self.ctrl.pid(self.pid_period, rpm)
            self._pid_t -= self.pid_period
        self.iq = self.ctrl.motor_current(rpm)
        return self.k_force * self.iq
