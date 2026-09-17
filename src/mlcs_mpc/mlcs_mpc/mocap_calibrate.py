#!/usr/bin/env python3
"""
mocap_calibrate — 마커 강체 원점 → 차량 뒤축 중심 오프셋을 자동으로 잰다.

  ros2 run mlcs_mpc mocap_calibrate --ros-args -p mode:=spin

■ 왜 필요한가

  마커 구조물을 차량 형상과 무관한 자리(예: 앞바퀴 앞의 Y자 지그)에 붙이면,
  Motive 가 보고하는 위치는 **그 지그의 원점**이지 차량 기준점이 아니다.

  MPC 의 운동학 자전거 모델은 **뒤축 중심** 기준이다. 30cm 앞의 점을
  차량 위치로 착각하면:

      · 직진 중에는 그냥 30cm 앞으로 밀린 위치 (상수 오프셋)
      · 선회 중에는 **그 오프셋의 방향이 계속 바뀐다**
        → 반경 1m 선회에서 마커는 1.044m 를 돈다 (4.4% 큰 원)
      · 경로 추종 목표가 보통 5cm 인데 30cm 가 틀어진다

  ★ yaw 는 괜찮다. 강체의 각도·각속도는 기준점을 어디로 잡든 같다
    (강체운동의 성질). 그래서 조치가 필요한 것은 **위치(그리고 그로부터
    미분되는 속도)뿐**이다. yaw 는 "0도가 어느 방향이냐" 만 맞추면 된다.

■ 두 가지 측정

  mode:=spin   제자리 회전 → offset_x, offset_y 를 한 번에
               차를 세운 채 조향을 한쪽 끝까지 주고 천천히 원을 그린다.
               마커 궤적이 그리는 원의 **중심**이 회전 중심이고,
               그 중심에서 마커까지의 벡터가 곧 오프셋이다.

  mode:=straight  직진 → yaw_offset
               차를 앞으로 곧게 밀면, 진행 방향과 보고된 yaw 의 차이가
               그대로 yaw_offset 이다.

■ 어느 쪽을 회전 중심으로 볼 것인가

  Ackermann 차량이 조향을 고정하고 돌면 회전 중심은 **뒤축의 연장선 위**에
  있다. 뒤축 중심은 그 회전 중심에서 반경 R 만큼 떨어진 점이다.

  그래서 spin 모드는 오프셋을 **회전 중심 기준**으로 주고, 거기서
  뒤축 중심까지는 추가로 알아야 한다. 가장 확실한 방법은 조향을 양쪽으로
  각각 재는 것이다 — 두 원의 중심을 잇는 선의 수직이등분선 위에 뒤축이
  있다. 이 노드는 좌/우 두 번 측정을 받아 그것까지 계산해 준다.

  ⚠ 그게 번거로우면 **줄자가 더 빠르고 충분히 정확하다.** 마커 지그의
    원점(Motive 에서 Pivot 을 어디 뒀는지)과 뒤축 중심 사이 거리를 재서
    직접 적으면 된다. 이 노드는 그 값을 **검증**하는 용도로도 쓴다.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def fit_circle(x, y):
    """최소제곱 원 피팅 → (cx, cy, R). 실패하면 (None, None, None).

    (x-cx)^2 + (y-cy)^2 = R^2 을 전개하면 미지수에 대해 선형이 되어
    반복 없이 한 번에 풀린다.
    """
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if len(x) < 10:
        return None, None, None
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None, None, None
    cx, cy, c = sol
    r2 = c + cx ** 2 + cy ** 2
    if r2 <= 0:
        return None, None, None
    return float(cx), float(cy), float(math.sqrt(r2))


class MocapCalibrate(Node):

    def __init__(self):
        super().__init__('mocap_calibrate')

        self.declare_parameter('mocap_topic', '/car/pose')
        self.declare_parameter('mode', 'spin')      # spin | straight
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('min_samples', 100)

        self.topic = self.get_parameter('mocap_topic').value
        self.mode = self.get_parameter('mode').value
        self.duration = float(self.get_parameter('duration').value)
        self.min_n = int(self.get_parameter('min_samples').value)

        self.samples = []
        self.t0 = None
        self.done = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PoseStamped, self.topic, self._cb, qos)
        self.create_timer(0.1, self._tick)

        print()
        if self.mode == 'spin':
            print('=' * 66)
            print('  마커 오프셋 측정 — 제자리 회전 모드')
            print('=' * 66)
            print('  1) 차를 넓은 곳에 둡니다')
            print('  2) 조향을 **한쪽 끝까지** 주고 아주 천천히 원을 그립니다')
            print('     (손으로 밀어도 되고 조이스틱으로 저속 주행해도 됩니다)')
            print(f'  3) {self.duration:.0f}초 동안 **최소 한 바퀴** 이상 도세요')
            print()
            print('  ★ 반 바퀴만 돌면 원 피팅이 부정확합니다. 한 바퀴 이상.')
        else:
            print('=' * 66)
            print('  yaw 오프셋 측정 — 직진 모드')
            print('=' * 66)
            print('  차를 **앞으로 곧게** 2m 이상 미세요 (조향 중립).')
        print('=' * 66)
        print(f'  토픽: {self.topic}   측정 {self.duration:.0f}초')
        print()

    def _cb(self, msg: PoseStamped):
        if self.done:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((t, msg.pose.position.x, msg.pose.position.y,
                             yaw_from_quat(msg.pose.orientation)))

    def _tick(self):
        if self.done:
            return
        if not self.samples:
            self.get_logger().warn(
                f'{self.topic} 를 기다리는 중...', throttle_duration_sec=3.0)
            return
        if self.t0 is None:
            self.t0 = self.samples[0][0]
        el = self.samples[-1][0] - self.t0
        if el < self.duration:
            if len(self.samples) % 100 == 0:
                print(f'  ... {el:.1f}s / {self.duration:.0f}s  '
                      f'({len(self.samples)} 샘플)', end='\r')
            return
        self.done = True
        print()
        self._report()

    # ────────────────────────────────────────────────────────────────
    def _report(self):
        a = np.array(self.samples)
        if len(a) < self.min_n:
            print(f'\n✗ 샘플이 {len(a)}개뿐입니다 (최소 {self.min_n}). '
                  'mocap 이 끊기지 않았는지 확인하세요.\n')
            return
        _, xs, ys, yaws = a.T

        print('=' * 66)
        if self.mode == 'spin':
            self._report_spin(xs, ys, yaws)
        else:
            self._report_straight(xs, ys, yaws)
        print('=' * 66 + '\n')

    def _report_spin(self, xs, ys, yaws):
        cx, cy, R = fit_circle(xs, ys)
        if cx is None:
            print('  ✗ 원 피팅 실패 — 차가 거의 안 움직였습니다.')
            return

        # 얼마나 돌았는지 (unwrap 해서 총 회전각)
        uw = np.unwrap(yaws)
        swept = abs(uw[-1] - uw[0])
        rms = float(np.sqrt(np.mean(
            (np.hypot(xs - cx, ys - cy) - R) ** 2)))

        print('  결과 — 제자리 회전')
        print('=' * 66)
        print(f'  마커 궤적 반경 : {R:.4f} m')
        print(f'  회전 중심      : ({cx:+.4f}, {cy:+.4f})')
        print(f'  총 회전각      : {math.degrees(swept):.0f}°'
              f'  {"✓" if swept > 5.5 else "⚠ 한 바퀴 미만 — 부정확합니다"}')
        print(f'  원 피팅 잔차   : {rms*1000:.1f} mm'
              f'  {"✓" if rms < 0.01 else "⚠ 마커가 흔들렸을 수 있습니다"}')
        print()

        # 각 시점에서 "회전중심 → 마커" 벡터를 차체 좌표계로 옮긴다.
        # 회전하는 내내 이 값이 일정해야 하고, 그게 곧 오프셋이다.
        dx, dy = xs - cx, ys - cy
        bx = np.cos(-yaws) * dx - np.sin(-yaws) * dy
        by = np.sin(-yaws) * dx + np.cos(-yaws) * dy

        print('  회전중심 기준 마커 위치 (차체 좌표계, x=전방 y=좌)')
        print(f'    x = {bx.mean():+.4f} ± {bx.std():.4f} m')
        print(f'    y = {by.mean():+.4f} ± {by.std():.4f} m')
        if max(bx.std(), by.std()) > 0.02:
            print('    ⚠ 산포가 큽니다 — 회전이 일정하지 않았거나 마커가 흔들립니다')
        print()
        print('  ── 이 값의 의미 ──')
        print('  Ackermann 차량이 조향을 고정하고 돌면 회전 중심은 **뒤축의')
        print('  연장선 위**에 있습니다. 즉 위 x 값은 "뒤축 → 마커" 의 전후')
        print('  거리와 같고, y 는 회전 반경 성분이 섞여 있습니다.')
        print()
        print(f'  → config/mocap.yaml 에 넣을 값 (마커 → 뒤축):')
        print(f'       offset_x: {-bx.mean():+.4f}')
        print(f'       offset_y: 0.0      # 좌우 대칭으로 붙였다면 0')
        print()
        print('  ⚠ offset_y 는 좌/우 양쪽으로 각각 돌려서 비교하는 것이')
        print('    가장 확실합니다. 두 측정의 x 가 같고 y 부호만 반대여야 합니다.')
        print('    한쪽만 쟀다면 줄자로 교차 검증하세요.')

    def _report_straight(self, xs, ys, yaws):
        # 이동 방향
        dx, dy = xs[-1] - xs[0], ys[-1] - ys[0]
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            print(f'  ✗ {dist:.2f} m 밖에 안 움직였습니다. 2m 이상 미세요.')
            return

        heading = math.atan2(dy, dx)
        yaw_mean = float(np.arctan2(np.sin(yaws).mean(), np.cos(yaws).mean()))
        off = math.atan2(math.sin(heading - yaw_mean),
                         math.cos(heading - yaw_mean))

        # 직진이 얼마나 곧았는지 — 시작→끝 직선에서 벗어난 정도
        ux, uy = dx / dist, dy / dist
        lat = np.abs(-(xs - xs[0]) * uy + (ys - ys[0]) * ux)

        print('  결과 — 직진')
        print('=' * 66)
        print(f'  이동 거리      : {dist:.3f} m')
        print(f'  직진 이탈      : 최대 {lat.max()*1000:.0f} mm'
              f'  {"✓" if lat.max() < 0.05 else "⚠ 곧게 밀지 못했습니다"}')
        print(f'  진행 방향      : {math.degrees(heading):+.2f}°')
        print(f'  보고된 yaw     : {math.degrees(yaw_mean):+.2f}°')
        print(f'  yaw 산포       : {math.degrees(float(np.std(np.unwrap(yaws)))):.2f}°')
        print()
        print(f'  → config/mocap.yaml 에 넣을 값:')
        print(f'       yaw_offset: {off:+.4f}      # {math.degrees(off):+.2f}°')
        print()
        if abs(off) < math.radians(2):
            print('  ✓ 거의 0 입니다 — Motive 에서 이미 잘 맞춰져 있습니다.')
        else:
            print('  ⚠ 값이 큽니다. Motive 에서 RigidBody Orientation 을')
            print('    "Reset to Current" 로 맞추는 편이 더 깔끔합니다.')


def main(args=None):
    rclpy.init(args=args)
    node = MocapCalibrate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
        if not node.done and len(node.samples) > node.min_n:
            node._report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
