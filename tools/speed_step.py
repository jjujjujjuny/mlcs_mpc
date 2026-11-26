#!/usr/bin/env python3
"""
speed_step.py — 바퀴 속도 계단 응답 (거치대 전용). 메인 PC 11-25 할 일 4: 새 VESC PID (11-24, ki 1/10) 에서
wheel_speed_tau 0.4 가 아직 맞는지.

  ./drive.sh speedstep            0 → 1.5 → 3.0 → 1.5 → 0 m/s (각 2.5 s) × 2

■ ★ 거치대 위, 바퀴가 공중에서 자유롭게 돌 때만. 3 m/s 로 바퀴가 돈다 — 손·선이 닿지 않게.
  바닥 하중·노면이 없어서 가속은 실제보다 빠를 수 있다 (감속은 제동 전류라 비슷). 땅 위 값은 주행 로그로
  `./drive.sh actuator` (tools/actuator_id.py) 가 따로 맞춘다.

■ 하는 일
  /drive (AckermannDriveStamped, 조향 0) 를 50 Hz 로 — 실제 주행과 같은 경로 (ackermann_mux → ackermann_to_vesc,
  속도 0 은 제동 전류 패치 포함). /sensors/core 의 erpm 을 바퀴 선속도 (÷ speed_to_erpm_gain) 로 기록.
  끝나면 계단마다 지연·10→90 % 시간·최대 가감속을 찍고, 1 차 τ 와 '1 차 + 가감속 상한' 을 맞춘다.
  ~/mlcs_data/speedstep_<시각>.csv
"""
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:
    VescStateStamped = None

STEPS = [0.0, 1.5, 3.0, 1.5, 0.0]
HOLD = 2.5
REPEAT = 2
RATE = 50.0


class SpeedStep(Node):
    def __init__(self, vesc_yaml):
        super().__init__('speed_step')
        p = next(iter(yaml.safe_load(open(vesc_yaml)).values()))['ros__parameters']
        self.gain, self.off = float(p['speed_to_erpm_gain']), float(p.get('speed_to_erpm_offset', 0.0))
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        if VescStateStamped is None:
            raise SystemExit('vesc_msgs 가 없습니다 — source ~/f1tenth_ws/install/setup.bash')
        self.create_subscription(VescStateStamped, '/sensors/core', self._core, 10)
        self.v, self.cur, self.v_t = float('nan'), float('nan'), -1e9
        self.seq = [a for _ in range(REPEAT) for a in STEPS]
        self.t0 = time.monotonic() + 3.0
        self.rows = []
        self.done = False
        self.create_timer(1.0 / RATE, self._tick)
        print(f'speed_step: erpm = {self.gain} × v + {self.off}. 3 초 뒤 시작, {len(self.seq) * HOLD:.0f} s. '
              '★ 거치대 — 바퀴가 3 m/s 로 돈다', flush=True)

    def _core(self, m):
        self.v = (m.state.speed - self.off) / self.gain
        self.cur = m.state.current_motor
        self.v_t = time.monotonic()

    def _send(self, v):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed, msg.drive.steering_angle = float(v), 0.0
        self.pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        k = int((now - self.t0) // HOLD) if now >= self.t0 else -1
        if k >= len(self.seq):
            self._send(0.0)
            self.done = True
            return
        cmd = 0.0 if k < 0 else self.seq[k]
        self._send(cmd)
        fresh = now - self.v_t < 0.1
        self.rows.append((now, cmd, self.v if fresh else float('nan'), self.cur if fresh else float('nan')))


def fit(t, u, v):
    """1 차 (τ) 와 1 차 + 가감속 상한. 측정 v 에서 출발해 명령으로만 적분한 오차."""
    dt = np.median(np.diff(t))
    ok = np.isfinite(v)

    def sim(tau, acc, dec, delay=0.0):
        k = int(round(delay / dt))
        y = np.zeros_like(v)
        y[0] = v[0] if np.isfinite(v[0]) else 0.0
        for i in range(1, len(v)):
            g = u[max(i - 1 - k, 0)]
            dy = (g - y[i - 1]) / tau * dt
            y[i] = y[i - 1] + np.clip(dy, -dec * dt, acc * dt)
        return np.sqrt(np.mean((y - v)[ok] ** 2))
    r1 = min((sim(tau, 1e3, 1e3, de), tau, de) for tau in np.arange(0.05, 1.01, 0.05) for de in (0.0, 0.02, 0.04, 0.06))
    r2 = min((sim(tau, a, b, de), tau, a, b, de) for tau in (0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.6)
             for a in (2.0, 3.0, 4.0, 6.0, 8.0, 1e3) for b in (1.5, 2.5, 3.5, 5.0, 8.0, 1e3) for de in (0.0, 0.04))
    print(f'\n맞춤 (거치대): 1 차 → 지연 {r1[2] * 1000:.0f} ms + τ {r1[1]:.2f} s (rms {r1[0]:.3f} m/s)', flush=True)
    print(f'            1 차 + 상한 → 지연 {r2[4] * 1000:.0f} ms + τ {r2[1]:.2f} s, 가속 ≤ {r2[2]:g}, 감속 ≤ {r2[3]:g} m/s² '
          f'(rms {r2[0]:.3f} m/s)', flush=True)
    print('  (지금 MPC: wheel_speed_tau 0.4, ws_dec_max 2.5, ws_acc_max 0 = 끔)', flush=True)


def analyse(rows):
    a = np.array(rows, float)
    t, u, v = a[:, 0], a[:, 1], a[:, 2]
    if np.isfinite(v).sum() < 50:
        print('바퀴 속도 기록이 없습니다 — /sensors/core 확인', flush=True)
        return
    edges = np.where(np.diff(u) != 0)[0] + 1
    print('\n 계단            | 지연   | 10→90 % | 최대 가/감속 | 정상상태 (명령 대비)', flush=True)
    for e in edges:
        seg = slice(e, min(e + int(HOLD * RATE), len(t)))
        tt, y = t[seg] - t[e], v[seg]
        okm = np.isfinite(y)
        if okm.sum() < 20:
            continue
        tt, y = tt[okm], y[okm]
        y0 = np.nanmean(v[max(e - 10, 0):e])
        yf = np.mean(y[-15:])
        dy = yf - y0
        if abs(dy) < 0.2:
            continue
        fr = (y - y0) / dy
        t10, t90 = tt[np.argmax(fr >= 0.1)], tt[np.argmax(fr >= 0.9)]
        acc = np.max(np.abs(np.gradient(y, tt)))
        print(f' {u[e - 1]:.1f} → {u[e]:.1f} m/s   | {t10 * 1000:4.0f} ms | {(t90 - t10) * 1000:5.0f} ms | {acc:5.1f} m/s²   | '
              f'{yf:.2f} m/s ({yf - u[e]:+.2f})', flush=True)
    fit(t, u, v)


def main():
    vy = os.path.expanduser('~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')
    rclpy.init()
    n = SpeedStep(vy)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    for _ in range(5):
        n._send(0.0)
        time.sleep(0.02)
    path = os.path.expanduser(f'~/mlcs_data/speedstep_{time.strftime("%Y%m%d_%H%M%S")}.csv')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write('t,cmd_mps,wheel_mps,motor_current\n')
        for r in n.rows:
            f.write(','.join('' if x != x else f'{x:.4f}' for x in r) + '\n')
    print(f'기록 → {path}', flush=True)
    analyse(n.rows)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
