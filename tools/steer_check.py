#!/usr/bin/env python3
"""
steer_check.py — mocap 없이 조향 점검 (대회장용). ./drive.sh rally 와 같이 띄운다.

  터미널 1   ./drive.sh rally                      (VESC·조이스틱. 스틱 끝 = 최대 속도·최대 조향, 십자키로 조절)
  터미널 2   python3 tools/steer_check.py           (빌드 필요 없음. source ~/f1tenth_ws/install/setup.bash 뒤)

  받는 것: /sensors/core (바퀴 ERPM), /sensors/imu/raw (VESC 자이로, deg/s), /commands/servo/position (서보 명령)
  하는 일: 조향을 고정하고 달리는 구간마다 실제 곡률 κ = 요레이트 / 속도 를 모아,
           조향 명령별로 모델 (hmpc.yaml 조향 맵 + 축거 0.33 m) 과 나란히 보여 준다.

■ 순서 (5 분)
  1. 차를 세워 두고 3 초 — 자이로 바이어스를 잰다 (화면에 '바이어스' 가 찍힐 때까지)
  2. 중립: 스틱 좌우를 놓고 (조향 0) 직선으로 0.8~1.5 m/s, 3~4 m 를 두세 번
  3. 조향 끝: 십자키로 최대 조향을 0.40 으로 두고, 스틱을 왼쪽 끝 → 원 한두 바퀴, 오른쪽 끝 → 원 한두 바퀴
     (속도 0.5~1.0 m/s. 빠르면 미끄러져 반경이 커진다)
  4. 필요하면 0.20 등 중간 조향도
  5. Ctrl+C — 요약과 권장값을 찍고 ~/mlcs_data/steercheck_<시각>.csv 에 남긴다

■ 통과 기준 (11-07~11-11 실측)
  중립 곡률 |κ| ≤ 0.025 1/m (4 m 에 6° 이내)
  최대 조향 (0.40 명령) 반경 0.78~0.89 m (모델 0.83 m), 좌우 차이 10 % 이내
"""
import math
import os
import re
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:
    VescStateStamped = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VESC_YAML = os.path.expanduser('~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml')
HMPC_YAML = os.path.join(ROOT, 'src/mlcs_mpc/config/hmpc.yaml')
L = 0.33
VX_SCALE = 1.042            # 바퀴 ×1.042 = 실제 (wheel_odom 과 같게)
V_MIN = 0.4                 # 이보다 느리면 곡률이 시끄럽다
HOLD = 0.4                  # 서보가 이만큼 그대로였던 구간만 (조향 과도 응답 빼기) s
BIN = 0.02                  # 조향 명령 묶음 rad


def read_num(path, key):
    with open(path) as f:
        for line in f:
            m = re.match(rf'^\s*{key}:\s*([-0-9.eE+]+)', line)
            if m:
                return float(m.group(1))
    raise SystemExit(f'{path} 에 {key} 가 없습니다')


def read_list(path, key):
    with open(path) as f:
        for line in f:
            m = re.match(rf'^\s*{key}:\s*\[([^\]]*)\]', line)
            if m:
                return [float(v) for v in m.group(1).split(',')]
    raise SystemExit(f'{path} 에 {key} 가 없습니다')


class SteerCheck(Node):
    def __init__(self):
        super().__init__('steer_check')
        self.gain = read_num(VESC_YAML, 'steering_angle_to_servo_gain')
        self.offset = read_num(VESC_YAML, 'steering_angle_to_servo_offset')
        self.erpm = read_num(VESC_YAML, 'speed_to_erpm_gain')
        self.map_cmd = read_list(HMPC_YAML, 'steer_map_cmd')
        self.map_act = read_list(HMPC_YAML, 'steer_map_act')
        self.v = 0.0
        self.v_t = -1e9
        self.servo = None
        self.servo_since = 0.0
        self.bias, self.bias_n, self.still_since = 0.0, 0, None
        self.bias_ok = False
        self.bins = {}              # 명령 (rad, BIN 단위) → [κ 들]
        self.rows = []
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        if VescStateStamped is None:
            raise SystemExit('vesc_msgs 가 없습니다 — source ~/f1tenth_ws/install/setup.bash 를 먼저')
        self.create_subscription(VescStateStamped, '/sensors/core', self._core, 10)
        self.create_subscription(Imu, '/sensors/imu/raw', self._imu, qos)
        self.create_subscription(Float64, '/commands/servo/position', self._servo, 10)
        self.create_timer(3.0, self._print)
        print(f'steer_check — 서보 = {self.gain:+.4f} × 조향 + {self.offset:.4f} (vesc.yaml), '
              f'조향 맵 {self.map_cmd} → {self.map_act}', flush=True)
        print('차를 세워 두세요 — 자이로 바이어스를 잽니다 …', flush=True)

    def now(self):
        return time.monotonic()

    def cmd_angle(self):
        return (self.servo - self.offset) / self.gain

    def model_kappa(self, cmd):
        act = math.copysign(float(np.interp(abs(cmd), self.map_cmd, self.map_act)), cmd)
        return math.tan(act) / L

    def _core(self, m):
        self.v = m.state.speed / self.erpm * VX_SCALE
        self.v_t = self.now()

    def _servo(self, m):
        if self.servo is None or abs(m.data - self.servo) > 0.002:
            self.servo_since = self.now()
        self.servo = m.data

    def _imu(self, m):
        t = self.now()
        r_raw = math.radians(m.angular_velocity.z)      # VESC 드라이버는 deg/s 를 그대로 넣는다
        if t - self.v_t > 0.2:
            return
        if abs(self.v) < 0.02:
            self.still_since = self.still_since or t
            if t - self.still_since > 0.5 and abs(r_raw - self.bias) < math.radians(2.0) or not self.bias_ok:
                self.bias_n += 1
                self.bias += (r_raw - self.bias) / min(self.bias_n, 200)
                if self.bias_n == 150 and not self.bias_ok:
                    self.bias_ok = True
                    print(f'✓ 자이로 바이어스 {math.degrees(self.bias):+.2f}°/s — 이제 달리세요', flush=True)
            return
        self.still_since = None
        if not self.bias_ok or self.servo is None or self.v < V_MIN or t - self.servo_since < HOLD:
            return
        cmd = self.cmd_angle()
        k = (r_raw - self.bias) / self.v
        key = round(cmd / BIN) * BIN
        self.bins.setdefault(key, []).append(k)
        self.rows.append((time.time(), cmd, self.v, r_raw - self.bias, k))

    def table(self):
        out = []
        for key in sorted(self.bins):
            ks = np.array(self.bins[key])
            if len(ks) < 25:        # 0.5 s 미만
                continue
            km = float(np.median(ks))
            mk = self.model_kappa(key)
            out.append((key, len(ks), km, mk))
        return out

    def _print(self):
        tb = self.table()
        if not tb:
            return
        print('\n 조향 명령 | 표본 | 실제 곡률 (반경)        | 모델 곡률 (반경)        | 실제/모델', flush=True)
        for key, n, km, mk in tb:
            rr = f'{1 / abs(km):5.2f} m' if abs(km) > 0.05 else '  직진 '
            rm = f'{1 / abs(mk):5.2f} m' if abs(mk) > 0.05 else '  직진 '
            ratio = f'{km / mk:5.2f}' if abs(mk) > 0.05 else '   - '
            print(f' {key:+6.2f} rad | {n:4d} | {km:+6.3f} 1/m ({rr}) | {mk:+6.3f} 1/m ({rm}) | {ratio}', flush=True)

    def summary(self):
        tb = self.table()
        print('\n════════ 요약 ════════', flush=True)
        if not tb:
            print('표본이 없습니다 — 조향을 고정하고 0.4 m/s 넘게 달렸는지 확인하세요', flush=True)
            return
        self._print()
        neu = [(n, km) for key, n, km, mk in tb if abs(key) < BIN / 2]
        neu_bad = False
        if neu:
            n, k0 = neu[0]
            d_err = math.atan(k0 * L)                       # 명령 0 일 때 실제 조향
            new_off = self.offset - self.gain * d_err       # 그 조향을 지우는 서보 중립
            ok = abs(k0) <= 0.025
            print(f'\n중립: 곡률 {k0:+.3f} 1/m ({"왼쪽" if k0 > 0 else "오른쪽"}으로 {abs(k0) * 4 * 57.3:.1f}°/4 m), '
                  f'실제 조향 {math.degrees(d_err):+.2f}° → {"✓ 통과" if ok else "✗ 기준 0.025 1/m 넘음"}', flush=True)
            neu_bad = not ok
            if not ok:
                print(f'  권장: vesc.yaml steering_angle_to_servo_offset {self.offset:.4f} → {new_off:.4f} '
                      f'(브링업 다시 띄운 뒤 이 점검을 한 번 더)', flush=True)
        else:
            print('\n중립: 표본 없음 — 스틱을 놓고 (조향 0) 직선으로 달려 주세요', flush=True)
        ends = [(key, km) for key, n, km, mk in tb if abs(key) >= 0.37]
        rl = [1 / km for key, km in ends if key > 0 and km > 0]
        rr = [1 / -km for key, km in ends if key < 0 and km < 0]
        for name, r in (('왼쪽', rl), ('오른쪽', rr)):
            if r:
                R = float(np.median(r))
                print(f'최대 조향 {name}: 반경 {R:.2f} m → {"✓" if 0.78 <= R <= 0.89 else "✗ 기준 0.78~0.89 m 밖"}',
                      flush=True)
            else:
                print(f'최대 조향 {name}: 표본 없음 (십자키로 0.40, 스틱 {name} 끝)', flush=True)
        if rl and rr:
            a, b = float(np.median(rl)), float(np.median(rr))
            print(f'좌우 차이 {abs(a - b) / ((a + b) / 2) * 100:.0f} % → {"✓" if abs(a - b) / ((a + b) / 2) <= 0.10 else "✗ 10 % 넘음 (서보 혼·링크 확인)"}',
                  flush=True)
            if neu_bad and abs(a - b) / ((a + b) / 2) > 0.10:
                print('  (중립이 틀어져 있으면 좌우 반경도 달라진다 — 중립을 먼저 고치고 다시 재세요)', flush=True)
        path = os.path.expanduser(f'~/mlcs_data/steercheck_{time.strftime("%Y%m%d_%H%M%S")}.csv')
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                f.write('wall_time,cmd_rad,v,r,kappa\n')
                for row in self.rows:
                    f.write(','.join(f'{v:.5f}' for v in row) + '\n')
            print(f'기록 → {path}', flush=True)
        except OSError as e:
            print(f'기록 못 함: {e}', flush=True)


def main():
    rclpy.init()
    n = SteerCheck()
    stop = {'v': False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(v=True))
    while rclpy.ok() and not stop['v']:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.summary()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
