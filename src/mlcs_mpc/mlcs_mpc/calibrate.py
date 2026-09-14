#!/usr/bin/env python3
"""
calibrate — mocap 을 자로 삼아 VESC 캘리브레이션 값을 측정한다.

■ 왜 이 도구가 먼저인가

  MPC 는 "속도 2.0 m/s 를 명령하면 차가 2.0 m/s 로 간다" 를 전제로 예측한다.
  speed_to_erpm_gain 이 5% 틀리면 예측이 5% 틀리고, 그 오차는 예측 구간
  1초 동안 누적되어 코너 진입 지점이 어긋난다. 조향 gain 이 틀리면 더
  직접적으로 — MPC 가 계산한 조향각이 그대로 안 나간다.

  26-jetson repo 의 2호차는 이 값이 4.5% 틀린 채로 달렸고, 그 결과
  "지령보다 4.5% 느리고 odom 은 4.8% 과대보고" 하는 상태였다. 그 차는
  줄자와 주행로그 두 가지 방법으로 뒤늦게 잡아냈다.

  ★ 우리는 mocap 이 있다. 줄자보다 훨씬 정확하고 빠르다 — 이게 이 장비의
    가장 큰 이점이므로 처음부터 제대로 쓴다.

■ 세 가지 측정

  speed   : 일정 속도 명령 → mocap 실제 속도. 비율이 gain 보정계수.
  steer   : 일정 조향 명령 → mocap 선회반경 → 실제 조향각 atan(L/R).
  neutral : 조향 0 명령으로 직진 → 휘면 그만큼이 중립 오프셋.

  사용:
    ros2 run mlcs_mpc calibrate --ros-args -p mode:=speed -p value:=1.5
    ros2 run mlcs_mpc calibrate --ros-args -p mode:=steer -p value:=0.2
    ros2 run mlcs_mpc calibrate --ros-args -p mode:=neutral

  ⚠ 안전: 반드시 넓은 공간에서, 킬스위치를 손에 들고 하세요.
     steer 모드는 차가 원을 그리며 돕니다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped


class Calibrate(Node):

    def __init__(self):
        super().__init__('calibrate')

        self.declare_parameter('mode', 'speed')       # speed | steer | neutral
        self.declare_parameter('value', 1.5)          # 명령할 속도 또는 조향각
        self.declare_parameter('duration', 5.0)       # 측정 시간 (초)
        self.declare_parameter('settle', 1.5)         # 과도응답 제외 시간
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('speed_for_steer', 1.0)

        self.mode = self.get_parameter('mode').value
        self.value = float(self.get_parameter('value').value)
        self.duration = float(self.get_parameter('duration').value)
        self.settle = float(self.get_parameter('settle').value)
        self.wb = float(self.get_parameter('wheelbase').value)
        self.v_steer = float(self.get_parameter('speed_for_steer').value)

        self.samples = []
        self.t0 = None
        self.done = False
        self.got_state = False

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/mpc/state', self._cb, 10)
        self.create_timer(0.02, self._loop)

        self.get_logger().info(
            f'캘리브레이션 시작: mode={self.mode} value={self.value} '
            f'duration={self.duration}s')
        self.get_logger().warn('★ 킬스위치를 손에 드세요. 차가 움직입니다.')

    def _cb(self, msg: Odometry):
        self.got_state = True
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((t, p.position.x, p.position.y, yaw,
                             msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z))

    def _loop(self):
        if self.done:
            return
        if not self.got_state:
            self.get_logger().warn('mocap 상태 대기 중... (/mpc/state)',
                                   throttle_duration_sec=2.0)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        el = now - self.t0

        if el >= self.duration:
            self._publish(0.0, 0.0)
            self.done = True
            self._report()
            return

        if self.mode == 'speed':
            self._publish(self.value, 0.0)
        elif self.mode == 'steer':
            self._publish(self.v_steer, self.value)
        elif self.mode == 'neutral':
            self._publish(self.v_steer, 0.0)
        else:
            self.get_logger().error(f'알 수 없는 mode: {self.mode}')
            self.done = True

    def _publish(self, speed, steer):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steer)
        self.drive_pub.publish(m)

    # ────────────────────────────────────────────────────────────────
    def _report(self):
        if len(self.samples) < 20:
            self.get_logger().error(
                f'샘플이 {len(self.samples)}개뿐 — mocap 이 들어왔는지 확인하세요')
            return

        a = np.array(self.samples)
        t = a[:, 0] - a[0, 0]
        keep = t >= self.settle          # 과도응답 구간 제외
        if keep.sum() < 10:
            self.get_logger().error('정상상태 샘플 부족 — duration 을 늘리세요')
            return
        a = a[keep]
        t, xs, ys, yaws, vs, ws = a.T

        print('\n' + '=' * 62)
        print(f'  캘리브레이션 결과 — mode={self.mode}')
        print('=' * 62)

        if self.mode == 'speed':
            # mocap 위치로 실제 이동거리/시간 → 실제 속도
            dist = np.sum(np.hypot(np.diff(xs), np.diff(ys)))
            dur = t[-1] - t[0]
            v_actual = dist / dur
            v_filt = vs.mean()
            ratio = self.value / v_actual if v_actual > 1e-6 else float('nan')

            print(f'  명령 속도      : {self.value:.3f} m/s')
            print(f'  실제 속도      : {v_actual:.3f} m/s  (이동 {dist:.2f}m / {dur:.2f}s)')
            print(f'  필터 속도      : {v_filt:.3f} m/s  (교차검증용)')
            print(f'  오차           : {(v_actual/self.value - 1)*100:+.2f} %')
            print()
            print(f'  ★ speed_to_erpm_gain 을 현재값 × {ratio:.4f} 로 고치세요.')
            print(f'    예) 현재 4614.0 이면 → {4614.0 * ratio:.1f}')
            print()
            print('    이유: erpm = gain × speed 이므로, 차가 명령보다 느리면')
            print('    같은 speed 에 더 큰 erpm 이 필요하다 = gain 을 키운다.')
            if abs(v_filt - v_actual) > 0.1:
                print(f'  ⚠ 필터속도와 위치적분 속도가 {abs(v_filt-v_actual):.3f} m/s 다릅니다.')
                print('    mocap 스탬프나 필터 설정을 확인하세요.')

        elif self.mode == 'steer':
            # 원 궤적 피팅 → 반경 → 실제 조향각
            cx, cy, R = fit_circle(xs, ys)
            if R is None or R > 1e4:
                print('  ✗ 원 피팅 실패 — 차가 거의 직진했습니다.')
                print('    value 를 더 크게 주거나 duration 을 늘리세요.')
                return
            delta_actual = math.atan(self.wb / R)
            # 회전 방향 부호
            sgn = 1.0 if np.mean(ws) > 0 else -1.0
            delta_actual *= sgn
            ratio = delta_actual / self.value if abs(self.value) > 1e-9 else float('nan')

            print(f'  명령 조향각    : {self.value:+.4f} rad ({math.degrees(self.value):+.2f}°)')
            print(f'  선회반경       : {R:.3f} m  (중심 {cx:.2f}, {cy:.2f})')
            print(f'  실제 조향각    : {delta_actual:+.4f} rad '
                  f'({math.degrees(delta_actual):+.2f}°)')
            print(f'  평균 요레이트  : {np.mean(ws):+.3f} rad/s')
            print()
            print(f'  ★ 실제/명령 비 = {ratio:.4f}')
            print(f'    steering_angle_to_servo_gain 을 현재값 ÷ {ratio:.4f} 로 고치세요.')
            print(f'    예) 현재 -1.2135 이면 → {-1.2135 / ratio:.4f}')
            print()
            print('    ⚠ 반드시 좌/우 양쪽(+value, -value)을 재서 비교하세요.')
            print('      좌우가 다르면 중립 오프셋이 틀린 것입니다 (neutral 모드).')

        elif self.mode == 'neutral':
            # 조향 0 으로 직진 명령 → 실제로 얼마나 휘는가
            cx, cy, R = fit_circle(xs, ys)
            yaw_drift = float(np.mean(ws))
            v = float(np.mean(vs))
            # 요레이트 → 등가 조향각: omega = v tan(delta)/L
            if abs(v) > 0.1:
                delta_eq = math.atan(yaw_drift * self.wb / v)
            else:
                delta_eq = float('nan')

            print(f'  명령 조향각    : 0.0 (직진)')
            print(f'  평균 속도      : {v:.3f} m/s')
            print(f'  평균 요레이트  : {yaw_drift:+.4f} rad/s')
            if R is not None and R < 1e4:
                print(f'  실측 선회반경  : {R:.2f} m  (완전 직진이면 매우 커야 함)')
            print(f'  등가 조향각    : {delta_eq:+.4f} rad '
                  f'({math.degrees(delta_eq):+.2f}°)')
            print()
            print(f'  ★ steering_angle_to_servo_offset 을 이만큼 보정하세요:')
            print(f'    새 offset = 현재 offset + (gain × {delta_eq:+.4f})')
            print(f'    예) gain=-1.2135, offset=0.5304 이면')
            print(f'        → {0.5304 + (-1.2135) * delta_eq:.4f}')
            print()
            print('    부호가 헷갈리면: 차가 왼쪽으로 휘면 오른쪽으로 트림을 줘야 합니다.')
            print('    한 번 고치고 다시 재서 요레이트가 0 에 가까워지는지 확인하세요.')

        print('=' * 62)
        print('  값을 config/vesc.yaml 과 config/vehicle.yaml 에 반영하세요.')
        print('  자세한 절차: docs/CALIBRATION.md')
        print('=' * 62 + '\n')


def fit_circle(x, y):
    """최소제곱 원 피팅. (cx, cy, R) 반환, 실패하면 (None, None, None).

    (x-cx)^2 + (y-cy)^2 = R^2 을 전개하면
        2x·cx + 2y·cy + (R^2 - cx^2 - cy^2) = x^2 + y^2
    로 미지수에 대해 **선형**이 된다. 그래서 반복 없이 한 번에 풀린다.
    """
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    if len(x) < 5:
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


def main(args=None):
    rclpy.init(args=args)
    node = Calibrate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
