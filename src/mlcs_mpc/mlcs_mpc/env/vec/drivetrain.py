"""
vec.drivetrain — env/drivetrain.py (실차 저수준 구동 경로) 의 배치 torch 판. RL 환경의 actuator='vesc' 와 모터 한계.

  /drive 속도 (20 Hz) → AckermannToVesc (정지 제동 패치) → VESC FOC 속도 PID (펌웨어, 1 kHz) → q축 전류 iq
  → 바퀴 힘 F = k_force·iq → 바퀴 속도  m_eq·ws' = F − R(타이어 종방향 반력)   (반암시적: dR/dws 로 안정화)

  식·순서는 env/drivetrain.py (numpy, test/test_drivetrain.py 로 검증) 를 한 줄씩 옮겼다 — 상태가 (B,) 텐서인 것만 다르다.
  분기 (속도 모드/제동 모드/duty 0) 는 torch.where 로 환경마다 따로.

■ 모터 한계 (actuator='lag' 에도 쓴다 — accel_limits())
  구동  iq ≤ min(l_current_max, l_in_current_max / mod_q)       mod_q = 1.5·ω_e·λ / V_bus
  제동  |iq| ≤ min(|l_current_min|, |l_in_current_min| / mod_q)  → 회생 −5 A 때문에 빠를수록 제동이 약하다
  가속도 = k_force·iq / (질량 + m_eq),  최고 속도 = ERPM 상한 (vesc.yaml speed_max → 4.86 m/s)
"""
import math

import torch

from mlcs_mpc.env.drivetrain import FOC_DEFAULTS, Drivetrain, FocConfig


class VecDrivetrain:

    def __init__(self, B, device, dtype, vesc, cfg=None, m_eq=Drivetrain.M_EQ_DEFAULT, v_bus=None,
                 brake_current=10.0, brake_speed_threshold=0.05, brake_handoff_speed=0.5, brake_rpm_eps=100.0):
        try:
            c = cfg or FocConfig.from_xml()
        except Exception:                       # XML 이 없으면 기본값
            c = FocConfig(**FOC_DEFAULTS)
        self.c = c
        self.B, self.device, self.dtype = B, device, dtype
        self.gain = float(vesc.speed_to_erpm_gain)
        self.erpm_lo, self.erpm_hi = float(vesc.speed_min), float(vesc.speed_max)
        self.k_force = 1.5 * c.foc_motor_flux_linkage * self.gain * 2.0 * math.pi / 60.0      # N/A (≈ 0.545)
        self.v_bus = float(v_bus) if v_bus else 3.8 * c.si_battery_cells
        self.pid_dt = 1.0 / c.pid_rate
        self.brake_current, self.brake_thr, self.handoff = brake_current, brake_speed_threshold, brake_handoff_speed
        self.brake_rpm_eps = brake_rpm_eps
        # 환경마다 다르게 줄 수 있는 값 (DR) — PID 이득 배율, m_eq
        z = lambda v=0.0: torch.full((B,), float(v), device=device, dtype=dtype)  # noqa: E731
        self.kp_scale, self.ki_scale, self.kd_scale = z(1.0), z(1.0), z(1.0)
        self.m_eq = z(m_eq)
        self.m_eq0 = float(m_eq)
        # 상태
        self.mode = torch.zeros(B, device=device, dtype=torch.long)     # 0 속도, 1 제동 전류, 2 놓임
        self.running = torch.zeros(B, device=device, dtype=torch.bool)
        self.braking = torch.zeros(B, device=device, dtype=torch.bool)
        self.set_rpm, self.cmd_rpm = z(), z()
        self.i_term, self.prev_err, self.d_filt = z(), z(), z()
        self.iq_set, self.iq = z(), z()
        self.ol_hyst, self.ol_left = z(), z()      # 개루프 출발 타이머 (env/drivetrain.py openloop_step)

    def reset(self, ids):
        self.mode[ids] = 2
        self.running[ids] = False
        self.braking[ids] = False
        for t in (self.set_rpm, self.cmd_rpm, self.i_term, self.prev_err, self.d_filt, self.iq_set, self.iq,
                  self.ol_hyst, self.ol_left):
            t[ids] = 0.0

    def start_moving(self, ids, ws):
        """리셋 때 이미 달리고 있는 차 (속도 ws) — 속도 모드로 그 속도를 유지하던 것처럼."""
        rpm = ws * self.gain
        self.mode[ids] = 0
        self.running[ids] = rpm.abs() >= self.c.s_pid_min_erpm
        self.set_rpm[ids] = rpm
        self.cmd_rpm[ids] = rpm

    # ── AckermannToVesc + mcpwm_foc_set_pid_speed / set_brake_current (명령이 올 때) ────────
    def command(self, speed, ws):
        c = self.c
        erpm = torch.clamp(self.gain * speed, self.erpm_lo, self.erpm_hi)
        stop = speed.abs() < self.brake_thr
        if self.brake_current > 0:
            self.braking = torch.where(stop, self.braking | (ws.abs() < self.handoff), torch.zeros_like(stop))
            brake = stop & self.braking
        else:
            brake = torch.zeros_like(stop)
        rpm_now = ws * self.gain
        # 속도 명령
        seed = (self.mode != 0) | ~self.running
        new_set = torch.where(seed, rpm_now, self.set_rpm) if c.s_pid_ramp_erpms_s > 0 else erpm
        self.set_rpm = torch.where(brake, self.set_rpm, new_set)
        self.cmd_rpm = torch.where(brake, self.cmd_rpm, erpm)
        start = (~self.running) & (erpm.abs() >= c.s_pid_min_erpm) & ~brake
        # 제동 전류
        bstart = brake & (abs(self.brake_current) >= c.cc_min_current)
        self.iq_set = torch.where(brake, torch.full_like(self.iq_set, self.brake_current), self.iq_set)
        self.running = self.running | start | bstart
        self.mode = torch.where(brake, torch.ones_like(self.mode), torch.zeros_like(self.mode))

    # ── foc_run_pid_control_speed (1 kHz) ─────────────────────────────
    def openloop_step(self, rpm):
        """개루프 타이머 — env/drivetrain.VescSpeedController.openloop_step 과 같은 식."""
        c, dt = self.c, self.pid_dt
        act = self.running & (self.mode == 0) & (self.set_rpm.abs() >= c.s_pid_min_erpm)
        f = torch.clamp(self.iq.abs() / max(c.l_current_max, 1e-9), max=1.0)
        ol_rpm = c.foc_openloop_rpm * (c.foc_openloop_rpm_low + (1.0 - c.foc_openloop_rpm_low) * f)
        low = rpm.abs() < ol_rpm
        hy = torch.where(low, self.ol_hyst + dt, torch.clamp(self.ol_hyst - dt, min=0.0))
        left = torch.where((hy > c.foc_sl_openloop_hyst) & (self.ol_left <= 1e-4),
                           torch.full_like(hy, c.foc_sl_openloop_time), self.ol_left)
        left = torch.clamp(left - dt, min=0.0)
        zero = torch.zeros_like(hy)
        self.ol_hyst = torch.where(act, hy, zero)
        self.ol_left = torch.where(act, left, zero)

    def pid(self, rpm):
        c, dt = self.c, self.pid_dt
        self.openloop_step(rpm)
        speed = self.mode == 0
        if c.s_pid_ramp_erpms_s > 0:
            step = c.s_pid_ramp_erpms_s * dt
            sr = self.set_rpm + torch.clamp(self.cmd_rpm - self.set_rpm, -step, step)
            sr = torch.clamp(sr, c.l_min_erpm, c.l_max_erpm)
            self.set_rpm = torch.where(speed, sr, self.set_rpm)
        err = self.set_rpm - rpm
        low = self.set_rpm.abs() < c.s_pid_min_erpm
        p = err * c.s_pid_kp * self.kp_scale / 20.0
        d = (err - self.prev_err) * (c.s_pid_kd * self.kd_scale / dt) / 20.0
        dfilt = self.d_filt + c.s_pid_kd_filter * (d - self.d_filt)
        out = torch.clamp(p + self.i_term + dfilt, -1.0, 1.0)
        i_new = torch.clamp(self.i_term + err * c.s_pid_ki * self.ki_scale * dt / 20.0, -1.0, 1.0)
        if c.s_pid_ki < 1e-9:
            i_new = torch.zeros_like(i_new)
        if not c.s_pid_allow_braking:
            out = torch.where((rpm > 20.0) & (out < 0), torch.zeros_like(out), out)
            out = torch.where((rpm < -20.0) & (out > 0), torch.zeros_like(out), out)
        iq_set = out * c.l_current_max * c.l_current_max_scale
        act = speed & ~low
        zero = torch.zeros_like(err)
        self.i_term = torch.where(act, i_new, zero)
        self.prev_err = torch.where(speed, err, zero)
        self.d_filt = torch.where(act, dfilt, torch.where(speed, self.d_filt, zero))
        self.iq_set = torch.where(act, iq_set, torch.where(speed, zero, self.iq_set))

    # ── 전류 루프 제한 (mcpwm_foc.c) ─────────────────────────────────
    def current(self, rpm):
        c = self.c
        we = rpm * 2.0 * math.pi / 60.0
        i_abs = max(abs(c.l_current_max), abs(c.l_current_min))
        duty0 = (self.mode == 0) & (self.set_rpm.abs() < c.s_pid_min_erpm)
        iq_short = torch.clamp(-we * c.foc_motor_flux_linkage / c.foc_motor_r, -i_abs, i_abs)
        mag = torch.clamp(self.iq_set.abs(), max=-c.l_current_min * c.l_current_min_scale)
        iq_brake = -torch.clamp(rpm / self.brake_rpm_eps, -1.0, 1.0) * mag
        iq_spd = self.iq_set
        if c.foc_sl_openloop_boost_q > 0:          # 개루프 보조 전류 (구동 방향일 때만)
            sgn = torch.sign(self.set_rpm)
            boost = (self.ol_left > 0) & (sgn != 0) & (iq_spd * sgn >= 0)
            iq_spd = torch.where(boost, sgn * torch.clamp(iq_spd.abs(), min=c.foc_sl_openloop_boost_q), iq_spd)
        iq = torch.where(self.mode == 1, iq_brake, torch.where(self.mode == 0, iq_spd, torch.zeros_like(rpm)))
        vq = c.foc_motor_r * self.iq + we * c.foc_motor_flux_linkage
        mq = 1.5 * vq / self.v_bus
        pos, neg = mq > 0.001, mq < -0.001
        safe = torch.where(mq.abs() > 0.001, mq, torch.ones_like(mq))
        iq = torch.where(pos, torch.clamp(iq, c.l_in_current_min / safe, c.l_in_current_max / safe), iq)
        iq = torch.where(neg, torch.clamp(iq, c.l_in_current_max / safe, c.l_in_current_min / safe), iq)
        lo, hi = c.l_current_min * c.l_current_min_scale, c.l_current_max * c.l_current_max_scale
        iq = torch.where(mq > 0, torch.clamp(iq, lo, hi), torch.clamp(iq, -hi, -lo))
        iq = torch.where(duty0, iq_short, iq)
        iq = torch.where(self.running, iq, torch.zeros_like(iq))
        self.iq = torch.clamp(iq, -i_abs, i_abs)
        return self.iq

    # ── 모터 한계 (lag 모델에 씌운다) ────────────────────────────────
    def accel_limits(self, ws, mass):
        """(구동 가속도 상한, 제동 감속도 크기) m/s² — ws 에서 모터가 낼 수 있는 최대."""
        c = self.c
        we = (ws * self.gain).abs() * 2.0 * math.pi / 60.0
        mq = torch.clamp(1.5 * we * c.foc_motor_flux_linkage / self.v_bus, min=1e-3)
        i_drv = torch.clamp(c.l_in_current_max / mq, max=c.l_current_max * c.l_current_max_scale)
        i_brk = torch.clamp(-c.l_in_current_min / mq, max=-c.l_current_min * c.l_current_min_scale)
        m = mass + self.m_eq
        return self.k_force * i_drv / m, self.k_force * i_brk / m
