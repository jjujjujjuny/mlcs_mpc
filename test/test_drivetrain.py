#!/usr/bin/env python3
"""
test_drivetrain.py — 저수준 구동 경로 (mlcs_mpc.env.drivetrain) 와 시뮬레이터 wheel_model='vesc' 시험. ROS·acados 불필요.

  python3 test/test_drivetrain.py

  ① wheel_forces 가 VehicleModel.body() 와 같은 힘인가 (body 의 vx·vy·r 미분을 그 힘으로 다시 만든다)
  ② ackermann_to_vesc 정지 처리 (패치): 빠르면 속도 0, 0.5 m/s 아래면 제동 전류, 한 번 넘어가면 출발 명령까지 유지
  ③ 펌웨어 PID: 램프, min_erpm 아래 duty 0, 적분 와인드업 순서 (출력은 갱신 전 i), 제동 끔이면 반대 출력 0
  ④ 전류 제한: 회생 입력 전류 −5 A 가 고속 제동 전류를 묶는다, 제동 전류 모드는 회전 반대 방향
  ⑤ 시뮬레이터 vesc 모드: 출발·정지, 유한, 정지 후 제자리
  ⑥ 센서리스 개루프 출발 보조 전류 (11-24 설정), docs/vesc/vesc6_f1tenth_1124.xml 이 그대로 읽히는가
"""
import math
import os
import sys

import casadi as cs
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.env.drivetrain import (AckermannToVesc, Drivetrain, FocConfig, VescSpeedController,  # noqa: E402
                                     wheel_forces)
from mlcs_mpc.env.simulator import Simulator  # noqa: E402
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402
from mlcs_mpc.env.vesc import Vesc  # noqa: E402

CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')


def test_wheel_forces_match_body():
    m = VehicleModel.from_yaml(CFG)
    p = m.p
    rng = np.random.default_rng(0)
    for _ in range(50):
        vx, vy, r = rng.uniform(0.3, 4.0), rng.uniform(-0.5, 0.5), rng.uniform(-3, 3)
        ws, d = vx * rng.uniform(0.7, 1.3), rng.uniform(-0.38, 0.38)
        fxf, fyf, fxr, fyr = (float(v) for v in wheel_forces(m, vx, vy, r, m.friction, ws, d, p))
        vx_d, vy_d, r_d = (float(v) for v in m.body(vx, vy, r, m.friction, ws, d, p))
        I_z, lr, cd0, cd2, cd1 = p[0], p[1], p[2], p[3], p[4]
        lf = m.L - lr
        drag = cd0 * np.sign(vx) + cd1 * vx + cd2 * vx * vx
        sd, cd = math.sin(d), math.cos(d)
        assert abs((fxr + fxf * cd - fyf * sd - drag + m.mass * vy * r) / m.mass - vx_d) < 1e-9
        assert abs((fxf * sd + fyr + fyf * cd - m.mass * vx * r) / m.mass - vy_d) < 1e-9
        assert abs(((fxf * sd + fyf * cd) * lf - fyr * lr) / I_z - r_d) < 1e-9


def test_ackermann_stop_brake():
    a = AckermannToVesc(Vesc())
    assert a.command(1.0, 1.0)[0] == 'speed'
    k, v = a.command(0.0, 2.0)            # 빠르다 → 속도 0 (VESC 제동)
    assert k == 'speed' and v == 0.0
    assert a.command(0.0, 0.4) == ('brake', 10.0)     # 0.5 아래 → 제동 전류
    assert a.command(0.0, 0.9) == ('brake', 10.0)     # 유지 (래치)
    assert a.command(0.01, 0.9) == ('brake', 10.0)    # threshold 0.05 아래도 정지 명령
    assert a.command(0.3, 0.0)[0] == 'speed'           # 출발 명령 → 풀림
    assert a.command(0.0, 2.0, state_fresh=False) == ('brake', 10.0)  # 상태 끊김 → 제동 전류
    k, v = a.command(10.0, 0.0)
    assert v == Vesc().speed_max                       # vesc_driver 상한


def test_pid_firmware_order():
    c = VescSpeedController(FocConfig())
    c.set_pid_speed(10000.0, 0.0)
    assert c.running and c.set_rpm == 0.0 and c.command_rpm == 10000.0
    c.pid(0.001, 0.0)
    assert abs(c.set_rpm - 25.0) < 1e-9       # 램프 25000 erpm/s × 1 ms
    assert c.duty_zero                         # 25 < min_erpm 500 → duty 0
    assert c.iq_set == 0.0
    for _ in range(100):
        c.pid(0.001, 0.0)
    assert not c.duty_zero
    # 출력은 갱신 전 i 로 계산: i 를 직접 넣고 한 번
    c2 = VescSpeedController(FocConfig(s_pid_kd=0.0, s_pid_ramp_erpms_s=0.0))
    c2.set_pid_speed(2000.0, 0.0)
    c2.i_term = 0.1
    c2.pid(0.001, 1000.0)
    p = 1000.0 * c2.cfg.s_pid_kp / 20.0
    assert abs(c2.iq_set - (p + 0.1) * 60.0) < 1e-9
    assert abs(c2.i_term - (0.1 + 1000.0 * c2.cfg.s_pid_ki * 0.001 / 20.0)) < 1e-12
    # 제동 끔: 앞으로 도는데 감속 출력이면 0
    c3 = VescSpeedController(FocConfig(s_pid_allow_braking=0.0, s_pid_ramp_erpms_s=0.0))
    c3.set_pid_speed(1000.0, 5000.0)
    c3.pid(0.001, 5000.0)
    assert c3.iq_set == 0.0


def test_current_limits():
    c = VescSpeedController(FocConfig(s_pid_ramp_erpms_s=0.0))
    c.set_pid_speed(600.0, 19000.0)           # 약 4 m/s 에서 거의 0 으로 → 강한 제동 요청
    c.pid(0.001, 19000.0)
    assert c.iq_set == -60.0
    iq = c.motor_current(19000.0)
    we = 19000.0 * 2 * math.pi / 60
    mod_q = 1.5 * we * c.cfg.foc_motor_flux_linkage / c.v_bus
    assert abs(iq - (-5.0 / mod_q)) < 1e-6, iq   # 회생 입력 전류 한계
    assert -60.0 < iq < -10.0
    b = VescSpeedController(FocConfig())
    b.set_brake_current(10.0)
    assert b.motor_current(2000.0) == -10.0 and b.motor_current(-2000.0) == 10.0
    assert abs(b.motor_current(0.0)) < 1e-12
    d = VescSpeedController(FocConfig())
    d.set_pid_speed(0.0, 300.0)
    d.running = True
    d.pid(0.001, 300.0)
    assert d.duty_zero
    lam, R = d.cfg.foc_motor_flux_linkage, d.cfg.foc_motor_r
    assert abs(d.motor_current(300.0) - (-(300 * 2 * math.pi / 60) * lam / R)) < 1e-9


def test_sim_vesc_start_stop():
    m = VehicleModel.from_yaml(CFG)
    s = Simulator(m, wheel_model='vesc')
    s.reset(0.0, 0.0, 0.0)
    for _ in range(250):
        s.step(0.0, 2.0)
    vx = s.twist()[0]
    assert np.all(np.isfinite(s.x)) and 1.8 < vx < 2.2, vx
    for _ in range(200):
        s.step(0.0, 0.0)
    assert abs(s.twist()[0]) < 0.02 and s.drivetrain.ctrl.mode == 'brake'
    x0 = s.pose()[0]
    for _ in range(100):
        s.step(0.0, 0.0)
    assert abs(s.pose()[0] - x0) < 0.01       # 제동 전류로 서 있다 (튀거나 굴러가지 않음)
    # 회전 방향: 왼쪽 조향이면 왼쪽으로 돈다
    s.reset(0.0, 0.0, 0.0)
    for _ in range(200):
        s.step(0.2, 1.0)
    assert s.twist()[2] > 0.5


def test_drivetrain_force_constant():
    v = Vesc()
    dt = Drivetrain(v)
    assert abs(dt.k_force - 1.5 * dt.cfg.foc_motor_flux_linkage * v.speed_to_erpm_gain * 2 * math.pi / 60) < 1e-12
    assert 0.5 < dt.k_force < 0.6           # 약 0.545 N/A — 60 A 면 33 N
    assert dt.pid_period == 0.001            # sp_pid_loop_rate 5 → 1000 Hz
    c = FocConfig.from_xml()
    assert c.source.endswith('vesc6_f1tenth_1124.xml')
    # 11-24 설정이 그대로 읽히는가
    assert (c.s_pid_kp, c.s_pid_ki, c.s_pid_kd) == (0.003, 0.0002, 0.0002)
    assert (c.foc_motor_r, c.foc_openloop_rpm, c.foc_sl_openloop_time, c.foc_sl_openloop_boost_q) == (0.0076, 1800, 0.1, 5)


def test_openloop_boost():
    """개루프 출발: 저속 (|rpm| < 1800) 이 hyst 0.1 s 를 넘으면 0.1 s 씩 개루프, 그동안 구동 전류 ≥ 보조 5 A.
    천천히 구르며 (400 erpm ≈ 0.08 m/s) 목표 600 erpm — PID 만으로는 5 A 아래. 보조 0 (예전 설정) 이면 PID 그대로."""
    out = {}
    for boost in (5.0, 0.0):
        c = VescSpeedController(FocConfig(foc_sl_openloop_boost_q=boost))
        c.set_pid_speed(600.0, 400.0)
        iqs, ol = [], []
        for _ in range(300):
            c.pid(0.001, 400.0)
            iqs.append(c.motor_current(400.0))
            ol.append(c.openloop)
        out[boost] = (np.array(iqs), np.array(ol))
    iq5, ol5 = out[5.0]
    iq0, _ = out[0.0]
    k = int(np.argmax(ol5))
    assert 0.1 < k * 0.001 < 0.12, k             # 램프 400→500 erpm 4 ms + hyst 0.1 s
    assert np.all(iq5[ol5] >= 5.0 - 1e-9) and np.max(iq0[ol5]) < 5.0
    assert np.max(np.abs(iq5[~ol5] - iq0[~ol5])) < 1e-9
    # 빨라지면 (rpm ≥ 1800) hyst 가 줄어 개루프가 멎는다 / 제동 모드면 타이머 0
    c = VescSpeedController(FocConfig())
    c.set_pid_speed(5000.0, 3000.0)
    for _ in range(300):
        c.pid(0.001, 3000.0)
    assert not c.openloop and c.ol_hyst == 0.0
    c.set_brake_current(10.0)
    c.pid(0.001, 0.0)
    assert c.ol_hyst == 0.0 and not c.openloop
    print(f'  ✓ 개루프 {k} ms 에 시작 ({ol5.sum()} ms 동안), 전류 PID {iq0[ol5].max():.2f} A → 보조 ≥ 5 A, 보조 0 이면 PID 그대로')


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    for t in tests:
        t()
        print(f'  ✓ {t.__name__}')
    print(f'{len(tests)}개 통과')
