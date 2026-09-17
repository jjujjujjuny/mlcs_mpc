#!/usr/bin/env python3
"""
data_logger — HyperPM 학습용 주행 데이터 수집.

  ros2 run mlcs_mpc data_logger --ros-args -p output_dir:=~/mlcs_data

■ 왜 지금부터 모으는가

  HyperMPC 논문(arXiv:2508.06181)은 **36분치 실주행 데이터**로 HyperPM 을
  학습시켰다. 전문가가 여러 트랙을 수동 주행하며 "차량 상태공간의 운용
  영역 전체를 덮도록" 모은 것이다.

  이건 나중에 몰아서 모으기 어렵다. 어차피 캘리브레이션·튜닝·경로 기록으로
  차를 계속 몰게 되는데, 그때 로깅을 안 해두면 같은 주행을 두 번 해야 한다.
  그래서 **1단계부터 켜 두고 쌓는다.**

■ 논문이 기록한 것 (6.4.3 Dataset)

  - 차량 내부 상태: 모터 RPM, 모터 q축 전류, 조향각
  - OptiTrack: 위치·자세
  - 속도: 위치·자세를 **미분**해서 차체 좌표계로 (논문은 Savitzky-Golay)
  - **전부 100Hz 로 동기 수집**

  이 노드는 같은 항목을 같은 주기로 남긴다.

■ ★ 시간 기준은 mocap 스탬프다

  여러 센서가 제각각 도착하는데, 로그의 시간축이 흔들리면 미분(속도·가속도)이
  통째로 오염된다. mocap 이 가장 빠르고(100Hz) 가장 정확하므로 그걸 기준으로
  삼고, 나머지는 "그 시점의 최신값"을 붙인다. 안 온 값은 빈칸으로 둔다 —
  0 으로 채우면 나중에 "정말 0" 과 구분이 안 된다.

  (같은 연구실 TurtleBot 장비의 multimodal_sample_api 가 쓰는 방식과 같다)

■ 속도를 두 벌 남기는 이유

  vx_kf / vy_kf   : mocap_bridge 의 칼만 필터 결과 (실시간 제어에 쓰는 값)
  x, y, yaw       : 원본 위치 (후처리로 다시 미분할 수 있게)

  논문은 Savitzky-Golay 로 오프라인 미분했다. 우리 KF 는 실시간용이라
  지연 특성이 다르므로, **원본을 남겨야 나중에 논문과 같은 방식으로
  다시 뽑을 수 있다.** 필터를 바꿀 때마다 재주행하지 않으려면 필수다.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

# VESC 상태는 선택적 — 패키지가 없어도 로거는 돌아야 한다
try:
    from vesc_msgs.msg import VescStateStamped
    HAVE_VESC = True
except ImportError:
    HAVE_VESC = False


COLUMNS = [
    'time',            # 첫 mocap 프레임 기준 경과 (초)
    'dt',              # 직전 프레임과의 간격
    'stamp',           # mocap 원본 스탬프 (절대)
    # ── mocap (ground truth) ─────────────────────────────
    'x', 'y', 'yaw',
    'vx_kf', 'vy_kf', 'yaw_rate_kf',
    # ── 명령 ─────────────────────────────────────────────
    'cmd_speed', 'cmd_steer',     # /drive (실제로 차에 나간 것)
    # ── VESC 내부 상태 ───────────────────────────────────
    'motor_rpm',       # 전기적 RPM (speed 필드)
    'motor_iq',        # q축 전류 — 논문이 기록한 항목
    'motor_current',   # 모터 전류 (avg)
    'duty_cycle',
    'voltage',
    'servo_position',  # 조향 서보 지령 (0~1)
]


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')

        self.declare_parameter('output_dir', '~/mlcs_data')
        self.declare_parameter('tag', '')          # 파일명에 붙일 메모
        self.declare_parameter('state_topic', '/mpc/state')
        self.declare_parameter('drive_topic', '/drive')
        # 정지 상태가 길게 이어지면 학습에 쓸모없는 행만 쌓인다.
        # 0 이면 전부 기록.
        self.declare_parameter('min_speed_to_log', 0.0)

        out_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        tag = self.get_parameter('tag').value
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        name = f'drive_{stamp}' + (f'_{tag}' if tag else '') + '.csv'
        self.path = os.path.join(out_dir, name)

        self.min_v = float(self.get_parameter('min_speed_to_log').value)

        # ── 최신값 보관 (mocap 틱에 붙인다) ──────────────────────────
        self.cmd = None          # (speed, steer)
        self.vesc = None         # dict
        self.servo = None

        self.t0 = None
        self.last_stamp = None
        self.rows = 0
        self.dropped = 0

        self.f = open(self.path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(COLUMNS)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(
            Odometry, self.get_parameter('state_topic').value,
            self._state_cb, sensor_qos)
        self.create_subscription(
            AckermannDriveStamped, self.get_parameter('drive_topic').value,
            self._drive_cb, 10)

        if HAVE_VESC:
            self.create_subscription(
                VescStateStamped, '/sensors/core', self._vesc_cb, 10)
        else:
            self.get_logger().warn(
                'vesc_msgs 를 못 찾았습니다 — 모터 RPM/전류 열이 빈칸이 됩니다.\n'
                '  (위치·명령은 정상 기록됩니다)')

        # 서보 지령은 Float64 — f1tenth_stack 이 /commands/servo/position 으로 낸다
        try:
            from std_msgs.msg import Float64
            self.create_subscription(
                Float64, '/commands/servo/position',
                lambda m: setattr(self, 'servo', m.data), 10)
        except ImportError:
            pass

        self.create_timer(5.0, self._status)
        self.get_logger().info(
            f'데이터 로깅 시작 → {self.path}\n'
            f'  시간 기준: mocap 스탬프  |  VESC: {"연결" if HAVE_VESC else "없음"}\n'
            '  Ctrl+C 로 종료하면 저장됩니다.')

    # ────────────────────────────────────────────────────────────────
    def _drive_cb(self, msg):
        self.cmd = (msg.drive.speed, msg.drive.steering_angle)

    def _vesc_cb(self, msg):
        s = msg.state
        self.vesc = {
            'motor_rpm': s.speed,
            'motor_iq': getattr(s, 'avg_iq', ''),
            'motor_current': s.current_motor,
            'duty_cycle': s.duty_cycle,
            'voltage': s.voltage_input,
        }

    def _state_cb(self, msg: Odometry):
        """mocap 틱마다 한 행. 이게 로그의 시간축이다."""
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.t0 is None:
            self.t0 = st
            dt = 0.0
        else:
            dt = st - self.last_stamp
            # 스탬프가 뒤로 가거나 크게 벌어지면 그 행은 버린다 —
            # 미분할 때 그 지점이 통째로 튄다.
            if dt <= 0.0 or dt > 0.5:
                self.dropped += 1
                self.last_stamp = st
                return
        self.last_stamp = st

        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        vx = msg.twist.twist.linear.x

        if self.min_v > 0.0 and abs(vx) < self.min_v:
            return

        v = self.vesc or {}
        c = self.cmd or ('', '')

        self.w.writerow([
            f'{st - self.t0:.4f}', f'{dt:.4f}', f'{st:.4f}',
            f'{p.position.x:.5f}', f'{p.position.y:.5f}', f'{yaw:.5f}',
            f'{vx:.4f}', f'{msg.twist.twist.linear.y:.4f}',
            f'{msg.twist.twist.angular.z:.4f}',
            c[0] if c[0] == '' else f'{c[0]:.4f}',
            c[1] if c[1] == '' else f'{c[1]:.4f}',
            v.get('motor_rpm', ''), v.get('motor_iq', ''),
            v.get('motor_current', ''), v.get('duty_cycle', ''),
            v.get('voltage', ''),
            '' if self.servo is None else f'{self.servo:.4f}',
        ])
        self.rows += 1

    def _status(self):
        if self.rows == 0:
            self.get_logger().warn(
                '아직 한 행도 기록되지 않았습니다 — /mpc/state 가 오나요?',
                throttle_duration_sec=10.0)
            return
        secs = self.rows / 100.0
        self.get_logger().info(
            f'  {self.rows}행 (~{secs:.0f}초, {secs/60:.1f}분)'
            + (f'  버린 프레임 {self.dropped}' if self.dropped else ''))

    def close(self):
        self.f.close()
        mins = self.rows / 100.0 / 60.0
        print(f'\n저장 완료: {self.path}')
        print(f'  {self.rows}행  ≈ {mins:.1f}분')
        if self.dropped:
            print(f'  ⚠ 스탬프 이상으로 버린 프레임: {self.dropped}')
        print()
        print(f'  HyperPM 학습 목표는 36분입니다 (논문 기준).')
        if mins < 36:
            print(f'  → {36 - mins:.0f}분 더 필요합니다.')
        print('  누적 확인:  python3 -c "import glob,csv;'
              'print(sum(sum(1 for _ in open(f))-1 for f in '
              'glob.glob(\'<폴더>/drive_*.csv\'))/6000, \'분\')"')


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
