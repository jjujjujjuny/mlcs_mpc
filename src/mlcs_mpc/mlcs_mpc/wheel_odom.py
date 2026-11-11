#!/usr/bin/env python3
"""
wheel_odom — 바퀴 속도 + IMU 자이로로 오도메트리를 만든다. Cartographer 위치추정의 예측용 (loc_odom: ekf).

  구독: /sensors/core (VESC, 바퀴 속도 ERPM), /sensors/imu/raw (VESC IMU, ★ 각속도가 deg/s 그대로 들어온다)
  발행: /odometry/filtered (nav_msgs/Odometry, odom → base_link). TF 는 내지 않는다
        (odom→base_link TF 는 지금처럼 vesc_to_odom 이 낸다. Cartographer 는 이 메시지를 예측에만 쓴다)

  적분: yaw += r·dt (자이로 − 바이어스), 위치 += v·(cos yaw, sin yaw)·dt, 옆 미끄러짐 0.
  바퀴 속도 = (ERPM − offset) / gain × vx_scale. ★ vx_scale 1.042 — mocap 직선·정속 4264행에서
    mocap ÷ 바퀴 = 1.042 (IQR 1.030~1.052, 11-08~11-10 모든 세션 1.04~1.06). vesc.yaml gain 은 그대로 둔다
    (명령·v_max·MPC 바퀴 상태·학습 모델이 같이 바뀌므로).
  슬립: Cartographer 는 오도메트리의 분산을 보지 않고 자세만 쓴다 → 속도 자체를 묶는다.
    바퀴가 잠기면 (11-09 급제동, 바퀴 0 인데 차 1.5 m/s) 추정 속도가 max_decel 보다 빨리 줄지 않게,
    출발 때 헛돌면 max_accel 보다 빨리 늘지 않게.
  자이로 바이어스: 바퀴가 서 있고 (|v| < 0.02) 자이로가 조용할 때 (|r − b| < 2°/s) 지수 평균으로 갱신한다.

  ros2 run mlcs_mpc wheel_odom --ros-args -p gyro_sign:=1.0
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

try:
    from vesc_msgs.msg import VescStateStamped
except ImportError:      # 시험 환경
    VescStateStamped = None


class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')
        d = self.declare_parameter
        d('speed_to_erpm_gain', 4781.5)     # vesc.yaml 과 같게
        d('speed_to_erpm_offset', 0.0)
        d('vx_scale', 1.042)
        d('gyro_sign', 1.0)                 # 왼쪽 회전 (반시계) 이 + 가 되게. 장착 방향 따라 −1
        d('gyro_deg', True)                 # VESC 드라이버는 deg/s 를 그대로 넣는다
        d('max_accel', 4.0)                 # 추정 속도 변화 상한 m/s² (헛돎)
        d('max_decel', 3.0)                 # m/s² (잠김 — 실측 잠김 감속 p10 −2.7)
        d('bias_tau', 2.0)                  # 정지 중 바이어스 지수 평균 시정수 s
        d('bias_gate_dps', 2.0)             # |자이로 − 바이어스| 가 이 이하일 때만 갱신 °/s
        d('odom_frame', 'odom')
        d('base_frame', 'base_link')
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.gain, self.offset = float(gp('speed_to_erpm_gain')), float(gp('speed_to_erpm_offset'))
        self.scale, self.gsign = float(gp('vx_scale')), float(gp('gyro_sign'))
        self.gk = math.pi / 180.0 if bool(gp('gyro_deg')) else 1.0
        self.amax, self.dmax, self.btau = float(gp('max_accel')), float(gp('max_decel')), float(gp('bias_tau'))
        self.bias_gate = math.radians(float(gp('bias_gate_dps')))
        self.odom_frame, self.base_frame = gp('odom_frame'), gp('base_frame')
        self.v_wheel = 0.0
        self.v = 0.0                # 슬립을 묶은 추정 속도
        self.bias = None            # 자이로 바이어스 rad/s
        self.x = self.y = self.yaw = 0.0
        self.t_last = None
        self.n_slip = 0
        self.n_msg = 0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        if VescStateStamped is not None:
            self.create_subscription(VescStateStamped, '/sensors/core', self._core_cb, 10)
        self.create_subscription(Imu, '/sensors/imu/raw', self._imu_cb, qos)
        self.pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'wheel_odom: vx×{self.scale}, 자이로 부호 {self.gsign:+.0f}, '
                               f'{"deg/s → rad/s" if self.gk != 1.0 else "rad/s"}, 슬립 묶음 +{self.amax}/−{self.dmax} m/s²')

    def _core_cb(self, m):
        self.v_wheel = (m.state.speed - self.offset) / self.gain * self.scale

    def _imu_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        r_raw = self.gsign * m.angular_velocity.z * self.gk
        if self.t_last is None:
            self.t_last = t
            self.bias = r_raw
            return
        dt = t - self.t_last
        if dt <= 0.0 or dt > 0.5:
            self.t_last = t
            return
        self.t_last = t
        # 서 있고 자이로도 조용할 때만 바이어스 갱신. 바퀴만 보면 손으로 돌리거나 제자리에서 미끄러질 때
        # 그 회전을 바이어스로 배워 버린다 (손 90° 시험: +70° 까지 갔다가 2 s 시정수로 +33° 로 되돌아감)
        if abs(self.v_wheel) < 0.02 and abs(r_raw - self.bias) < self.bias_gate:
            a = min(dt / self.btau, 1.0)
            self.bias += a * (r_raw - self.bias)
        r = r_raw - self.bias
        # 슬립 묶음: 바퀴 속도를 따르되 가감속 상한 안에서만
        dv = self.v_wheel - self.v
        lim = (self.amax if dv > 0 else self.dmax) * dt
        clamped = abs(dv) > lim
        if clamped:
            self.n_slip += 1
            dv = math.copysign(lim, dv)
        self.v += dv
        if abs(self.v_wheel) < 0.02 and abs(self.v) < 0.05:
            self.v = 0.0
        # 적분 (반 스텝 yaw 로)
        ym = self.yaw + 0.5 * r * dt
        self.x += self.v * math.cos(ym) * dt
        self.y += self.v * math.sin(ym) * dt
        self.yaw = math.atan2(math.sin(self.yaw + r * dt), math.cos(self.yaw + r * dt))
        o = Odometry()
        o.header.stamp = m.header.stamp if m.header.stamp.sec > 0 else self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = self.odom_frame, self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = r
        # vx 분산: 평소 0.05², 가감속 묶음이 걸린 순간 0.5² (data_logger 가 odom_clamped 로 기록)
        o.twist.covariance[0] = 0.25 if clamped else 0.0025
        self.pub.publish(o)
        self.n_msg += 1

    def _report(self):
        b = 0.0 if self.bias is None else math.degrees(self.bias)
        self.get_logger().info(f'{self.n_msg / 5.0:.0f} Hz, 자이로 바이어스 {b:+.2f}°/s, 슬립 묶음 {self.n_slip}번, '
                               f'yaw {math.degrees(self.yaw):+.1f}°')
        self.n_msg = 0


def main(args=None):
    rclpy.init(args=args)
    n = WheelOdom()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
