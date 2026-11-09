#!/usr/bin/env python3
"""
servo_sweep.py — 조향 기계적 한계 찾기 (거치대 전용)

  ./drive.sh servo  로 실행한다 (브링업을 같이 띄워 준다).

  /commands/servo/position 에 서보 값을 **직접** 쏜다. ackermann 변환을
  거치지 않으므로 max_steer 에 안 막힌다. 다만 vesc_driver 가 vesc.yaml 의
  servo_min / servo_max 로 한 번 더 자르므로, 그 바깥을 보려면 그 두 값을
  잠깐 넓히고 브링업을 다시 띄워야 한다 (화면에 '잘림' 으로 표시된다).

  한계 판정 — 값을 조금씩 밀면서 바퀴를 본다:
    · 값을 올려도 바퀴가 더 안 돌아간다  (서보 세이버 스프링이 휜다)
    · 서보에서 '지잉' 하는 소리가 계속 난다 (스톨 — 오래 두면 서보가 탄다)
  → 그 지점이 한계. 거기서 [ 또는 ] 로 기록하고 바로 중앙으로 돌린다.

  키:
    a / d      ±0.01   (← 좌 / 우 →)
    A / D      ±0.002  (미세)
    c          중앙 (offset)
    [          지금 값을 '좌측 한계' 로 기록
    ]          지금 값을 '우측 한계' 로 기록
    q          중앙으로 돌리고 종료 + 권장값 출력
"""
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64

MARGIN = 0.02      # 기계 한계에서 이만큼 안쪽을 servo_min/max 로 권장


def load_vesc_cfg(path):
    with open(path) as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    return (float(p['steering_angle_to_servo_gain']),
            float(p['steering_angle_to_servo_offset']),
            float(p['servo_min']), float(p['servo_max']))


class ServoSweep(Node):
    def __init__(self, gain, offset):
        super().__init__('servo_sweep')
        self.gain, self.offset = gain, offset
        self.value = offset
        self.clipped = None
        self.pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.create_subscription(Float64, '/sensors/servo_position_command',
                                 self._on_clipped, 10)
        self.create_timer(0.05, self._tick)

    def _on_clipped(self, m):
        self.clipped = m.data

    def _tick(self):
        self.pub.publish(Float64(data=float(self.value)))

    def delta(self, servo):
        # servo = gain·δ + offset  →  δ = (servo − offset) / gain
        return (servo - self.offset) / self.gain


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')
    gain, offset, smin, smax = load_vesc_cfg(cfg)

    rclpy.init()
    node = ServoSweep(gain, offset)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # δ>0 = 좌. gain 이 음수이므로 좌 = 서보값 감소.
    left_dir = -1.0 if gain < 0 else 1.0
    marks = {'L': None, 'R': None}

    print(__doc__.split('키:')[1])
    print(f'  vesc.yaml  gain={gain}  offset={offset}  '
          f'servo_min={smin}  servo_max={smax}')
    print(f'  → 지금 도달 가능한 δ:  좌 {node.delta(smin if left_dir < 0 else smax):+.4f}'
          f'  우 {node.delta(smax if left_dir < 0 else smin):+.4f} rad\n')

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            v = node.value
            d = node.delta(v)
            c = node.clipped
            clip_note = ''
            if c is not None and abs(c - v) > 1e-6:
                clip_note = f'  \033[33m잘림→{c:.3f}\033[0m'
            side = '좌' if d > 1e-6 else ('우' if d < -1e-6 else '중')
            sys.stdout.write(
                f'\r  servo {v:.3f}   δ {d:+.4f} rad ({d*57.2958:+6.2f}°) {side}'
                f'{clip_note}          ')
            sys.stdout.flush()

            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            k = sys.stdin.read(1)
            step = {'a': 0.01, 'A': 0.002, 'd': -0.01, 'D': -0.002}.get(k)
            if step is not None:
                # a/A = 좌. left_dir(= gain 부호) 로 서보 방향을 맞춘다.
                node.value = min(1.0, max(0.0, v + step * left_dir))
            elif k == 'c':
                node.value = offset
            elif k == '[':
                marks['L'] = v
                print(f'\n  ★ 좌측 한계 기록: servo {v:.3f}  δ {d:+.4f} rad')
            elif k == ']':
                marks['R'] = v
                print(f'\n  ★ 우측 한계 기록: servo {v:.3f}  δ {d:+.4f} rad')
            elif k == 'q':
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.value = offset
        time.sleep(0.3)          # 중앙 명령이 실제로 나가게

    print('\n\n  ── 결과 ──')
    if marks['L'] is None or marks['R'] is None:
        print('  좌/우 한계를 둘 다 기록하지 않아 권장값을 못 냅니다.')
    else:
        lo, hi = sorted((marks['L'], marks['R']))
        new_min, new_max = lo + MARGIN, hi - MARGIN
        dl = abs(node.delta(marks['L']))
        dr = abs(node.delta(marks['R']))
        # 새 servo 범위 안에서 좌우 모두 낼 수 있는 δ
        dmax = min(abs(node.delta(new_min)), abs(node.delta(new_max)))
        print(f'  기계 한계   좌 servo {marks["L"]:.3f} (δ≈{dl:.4f})   '
              f'우 servo {marks["R"]:.3f} (δ≈{dr:.4f})')
        print(f'  권장 vesc.yaml   servo_min: {new_min:.3f}   servo_max: {new_max:.3f}'
              f'   (한계에서 {MARGIN} 안쪽)')
        print(f'  권장 max_steer   {dmax:.3f} rad ({dmax*57.2958:.1f}°)  — 좌우 중 좁은 쪽')
        print('  ⚠ δ 는 ±0.2 에서 잰 gain 을 외삽한 값이다. 끝단은 비선형일 수 있으니')
        print(f'    ./drive.sh cal steer {dmax:.2f}  와  -{dmax:.2f}  로 실측 확인할 것.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
