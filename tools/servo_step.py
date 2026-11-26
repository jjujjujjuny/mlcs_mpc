#!/usr/bin/env python3
"""
servo_step.py — 서보 계단 응답 (거치대 전용). 메인 PC 11-25 부탁: 시뮬 서보 모델 (τ / 최대 각속도 / 순수 지연) 정하기.

  ./drive.sh servostep                       명령만 기록 (영상으로 볼 때)
  ./drive.sh servostep /wheel/pose           앞바퀴 강체 (Motive) 로 실제 조향각까지 — 권장

■ 준비
  1. 차를 거치대에 (바퀴가 땅에 안 닿게). 속도 명령은 내지 않는다
  2. Motive: 차체 강체 (/car/pose, 이미 있음) + **오른쪽 (또는 왼쪽) 앞바퀴 바깥 면에 마커 3 개로 강체 하나**
     → natnet 이 /wheel/pose 로 내게 (이름은 Motive 강체 이름). 바퀴가 돌 때 같이 도는 자리 (허브 바깥) 에
  3. 실행하면 3 초 뒤 시작. 0 → +0.2 → 0 → −0.2 → 0 → +0.4 → 0 → −0.4 → 0 (rad, 각 1.5 s) 를 3 번

■ 하는 일
  /commands/servo/position 에 서보 값을 **직접** 50 Hz 로 낸다 (hmpc_node 의 조향 변화율 묶음 3.2 rad/s 를 거치지 않는
  물리 서보 응답). 값 = vesc.yaml steering_angle_to_servo_gain × 각 + offset (ackermann_to_vesc 와 같은 식).
  바퀴 강체가 있으면 실제 조향각 = 바퀴 yaw − 차체 yaw (처음 1 s 평균을 0 으로).
  끝나면 계단마다: 순수 지연 (명령 → 10 % 움직임), 10→90 % 시간, 최대 각속도, 최종 각 (명령 대비) 를 찍고
  ~/mlcs_data/servostep_<시각>.csv 로 남긴다.
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

STEPS = [0.0, 0.2, 0.0, -0.2, 0.0, 0.4, 0.0, -0.4, 0.0]
HOLD = 1.5
REPEAT = 3
RATE = 50.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ServoStep(Node):
    def __init__(self, vesc_yaml, wheel_topic):
        super().__init__('servo_step')
        p = next(iter(yaml.safe_load(open(vesc_yaml)).values()))['ros__parameters']
        self.gain, self.off = float(p['steering_angle_to_servo_gain']), float(p['steering_angle_to_servo_offset'])
        self.smin, self.smax = float(p.get('servo_min', 0.0)), float(p.get('servo_max', 1.0))
        self.pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.body = self.wheel = None
        self.rows = []
        if wheel_topic:
            self.create_subscription(PoseStamped, '/car/pose', lambda m: self._pose(m, 'body'), qos)
            self.create_subscription(PoseStamped, wheel_topic, lambda m: self._pose(m, 'wheel'), qos)
        self.has_wheel = bool(wheel_topic)
        self.seq = [a for _ in range(REPEAT) for a in STEPS]
        self.t0 = time.monotonic() + 3.0
        self.cmd = 0.0
        self.create_timer(1.0 / RATE, self._tick)
        print(f'servo_step: 서보 = {self.gain:+.4f} × 각 + {self.offset_str()}, 바퀴 강체 {"있음 " + wheel_topic if wheel_topic else "없음 (명령만)"}'
              f'\n3 초 뒤 시작, {len(self.seq) * HOLD:.0f} s', flush=True)
        self.done = False

    def offset_str(self):
        return f'{self.off:.4f}'

    def _pose(self, m, which):
        setattr(self, which, (time.monotonic(), yaw_of(m.pose.orientation)))

    def _tick(self):
        now = time.monotonic()
        k = int((now - self.t0) // HOLD) if now >= self.t0 else -1
        if k >= len(self.seq):
            self.pub.publish(Float64(data=float(np.clip(self.off, self.smin, self.smax))))
            self.done = True
            return
        self.cmd = 0.0 if k < 0 else self.seq[k]
        servo = float(np.clip(self.gain * self.cmd + self.off, self.smin, self.smax))
        self.pub.publish(Float64(data=servo))
        rel = ''
        if self.has_wheel and self.body and self.wheel and now - self.body[0] < 0.05 and now - self.wheel[0] < 0.05:
            rel = (self.wheel[1] - self.body[1] + math.pi) % (2 * math.pi) - math.pi
        self.rows.append((now, self.cmd, servo, rel))


def analyse(rows):
    t = np.array([r[0] for r in rows]); cmd = np.array([r[1] for r in rows])
    rel = np.array([np.nan if r[3] == '' else r[3] for r in rows], float)
    if np.isfinite(rel).sum() < 50:
        print('바퀴 각이 없어 분석 생략 (명령만 기록) — 영상으로 보거나 /wheel/pose 강체로 다시', flush=True)
        return
    rel = rel - np.nanmean(rel[(t - t[0]) < 1.0 + 3.0][:50]) if np.isfinite(rel[:50]).any() else rel
    edges = np.where(np.diff(cmd) != 0)[0] + 1
    print('\n 계단            | 순수 지연 | 10→90 % | 최대 각속도 | 최종 각 (명령 대비)', flush=True)
    for e in edges:
        a0, a1 = cmd[e - 1], cmd[e]
        seg = slice(e, min(e + int(HOLD * RATE), len(t)))
        tt, y = t[seg] - t[e], rel[seg]
        ok = np.isfinite(y)
        if ok.sum() < 10:
            continue
        tt, y = tt[ok], y[ok]
        y0 = np.nanmean(rel[max(e - 10, 0):e]); yf = np.mean(y[-10:]); dy = yf - y0
        if abs(dy) < 0.02:
            continue
        frac = (y - y0) / dy
        t10 = tt[np.argmax(frac >= 0.1)]; t90 = tt[np.argmax(frac >= 0.9)]
        rate = np.max(np.abs(np.gradient(y, tt)))
        print(f' {a0:+.1f} → {a1:+.1f} rad   | {t10 * 1000:5.0f} ms  | {(t90 - t10) * 1000:5.0f} ms | {rate:5.2f} rad/s  | '
              f'{dy:+.3f} rad ({dy / (a1 - a0) * 100:4.0f} %)', flush=True)


def main():
    vy = os.path.expanduser('~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')
    wheel = sys.argv[1] if len(sys.argv) > 1 else ''
    rclpy.init()
    n = ServoStep(vy, wheel)
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    n.pub.publish(Float64(data=float(np.clip(n.off, n.smin, n.smax))))
    path = os.path.expanduser(f'~/mlcs_data/servostep_{time.strftime("%Y%m%d_%H%M%S")}.csv')
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write('t,cmd_rad,servo,wheel_rel_rad\n')
        for r in n.rows:
            f.write(f'{r[0]:.4f},{r[1]:.3f},{r[2]:.4f},{"" if r[3] == "" else f"{r[3]:.5f}"}\n')
    print(f'기록 → {path}', flush=True)
    analyse(n.rows)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
