#!/usr/bin/env python3
"""
mocap_bridge — OptiTrack Motive(NatNet) pose → 컨트롤러용 상태 추정.

■ 이 노드가 하는 일

  natnet_ros2 가 발행하는 PoseStamped(위치+자세만) 를 받아서
  MPC 가 필요로 하는 **속도까지 포함한** 완전한 상태로 바꾼다:

      /mocap/pose (PoseStamped)  →  /mpc/state (Odometry)
                                 →  /odom      (Odometry, 호환용)
                                 →  tf: map → base_link

  라이다가 없으므로 AMCL 이 없고, 이 노드가 위치추정 전체를 담당한다.

■ 왜 단순 차분이 아니라 필터를 쓰는가

  mocap 은 위치는 mm 급으로 정확하지만 **속도는 직접 주지 않는다.**
  100Hz 위치를 그냥 차분하면:
      v = dx/dt = (1mm 노이즈)/(0.01s) = 0.1 m/s 노이즈
  MPC 는 이 속도로 가속 명령을 만들기 때문에 노이즈가 그대로 스로틀
  떨림이 된다. Desktop/CLAUDE.md 의 TurtleBot 장비에서도 같은 이유로
  "속도·가속도는 차분이라 노이즈가 커지므로 평활화가 전제" 라고 적혀 있다.

  여기서는 **정속도 모델 칼만 필터**를 쓴다. 단순 저역통과(LPF)보다 나은
  이유는 지연이다. LPF 는 차단주파수를 낮출수록 위상지연이 커지고, MPC 는
  지연된 속도를 현재 속도로 믿고 제어해서 진동한다. KF 는 모델(등속 운동)
  로 예측하고 측정으로 보정하므로, 같은 노이즈 억제를 훨씬 적은 지연으로
  얻는다.

■ ★ mocap 끊김 = 즉시 정지

  이 구성에서 mocap 이 끊기면 차는 자기 위치를 **전혀** 모른다 (라이다도
  휠 오도메트리 기반 대체 위치추정도 없다). 네트워크 한 번 끊기는 것이
  곧 제어 불능이므로, timeout 을 넘기면 /mocap/valid=False 를 내려
  컨트롤러가 정지하게 한다. Desktop/CLAUDE.md 의 low-level bridge 가
  command_timeout 으로 하는 것과 같은 안전장치다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class PoseKF:
    """정속도 모델 칼만 필터 (축 하나당 [위치, 속도]).

    x축/y축/yaw 를 각각 독립으로 돌린다. 실제로는 비홀로노믹 구속 때문에
    완전 독립이 아니지만, mocap 측정 노이즈가 워낙 작아서 축간 상관을
    모델링해 얻는 이득보다 구현/튜닝 단순함이 크다.
    """

    def __init__(self, q_pos=0.01, q_vel=1.0, r_meas=1e-4):
        self.x = np.zeros(2)          # [위치, 속도]
        self.P = np.eye(2) * 1.0
        self.Q = np.diag([q_pos, q_vel])
        self.R = r_meas
        self.init = False

    def reset(self, z):
        self.x = np.array([z, 0.0])
        self.P = np.eye(2) * 1.0
        self.init = True

    def update(self, z, dt):
        if not self.init:
            self.reset(z)
            return self.x.copy()

        F = np.array([[1.0, dt], [0.0, 1.0]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt

        # 측정 갱신 (위치만 관측)
        H = np.array([1.0, 0.0])
        y = z - H @ self.x
        S = H @ self.P @ H + self.R
        K = self.P @ H / S
        self.x = self.x + K * y
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P
        return self.x.copy()


class MocapBridge(Node):

    def __init__(self):
        super().__init__('mocap_bridge')

        # ── 토픽/프레임 ────────────────────────────────────────────────
        # natnet_ros2 는 보통 /natnet_ros/<RigidBody이름>/pose 로 낸다.
        # RigidBody 이름은 Motive 에서 정하므로 파라미터로 뺀다.
        self.declare_parameter('mocap_topic', '/car/pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # ── 안전 ──────────────────────────────────────────────────────
        # 100Hz 스트림 기준. 0.2초면 20프레임을 연속으로 놓친 것이다.
        self.declare_parameter('timeout', 0.2)

        # ── 필터 ──────────────────────────────────────────────────────
        self.declare_parameter('q_pos', 0.01)
        self.declare_parameter('q_vel', 1.0)
        self.declare_parameter('r_meas', 1e-4)
        # mocap 강체 원점이 차량 기준점(뒤축 중심)과 다를 때 보정.
        # Desktop/CLAUDE.md 의 mocap_to_model 캘리브레이션과 같은 개념.
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('yaw_offset', 0.0)

        self.mocap_topic = self.get_parameter('mocap_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.timeout = float(self.get_parameter('timeout').value)
        self.off_x = float(self.get_parameter('offset_x').value)
        self.off_y = float(self.get_parameter('offset_y').value)
        self.yaw_off = float(self.get_parameter('yaw_offset').value)

        q_pos = float(self.get_parameter('q_pos').value)
        q_vel = float(self.get_parameter('q_vel').value)
        r_meas = float(self.get_parameter('r_meas').value)
        self.kf_x = PoseKF(q_pos, q_vel, r_meas)
        self.kf_y = PoseKF(q_pos, q_vel, r_meas)
        self.kf_yaw = PoseKF(q_pos, q_vel, r_meas)

        self.last_stamp = None
        self.last_rx = None
        self.yaw_unwrapped = 0.0
        self.prev_yaw_raw = None
        self.warned = False

        # ★ mocap 은 센서 스트림이므로 BEST_EFFORT.
        #   RELIABLE 로 두면 무선 구간에서 재전송이 쌓여 **오래된 pose** 가
        #   순서대로 밀려온다. 제어에는 늦은 정확한 값보다 최신 값이 낫다.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(
            PoseStamped, self.mocap_topic, self._pose_cb, sensor_qos)

        self.state_pub = self.create_publisher(Odometry, '/mpc/state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10)   # 기존 도구 호환
        self.valid_pub = self.create_publisher(Bool, '/mocap/valid', 10)

        self.tf_bc = TransformBroadcaster(self) if \
            self.get_parameter('publish_tf').value else None

        # 워치독 — 콜백이 아예 안 오는 경우를 잡으려면 별도 타이머가 필요하다
        self.create_timer(0.05, self._watchdog)

        self.get_logger().info(
            f'mocap_bridge: {self.mocap_topic} 구독, timeout={self.timeout}s')

    # ────────────────────────────────────────────────────────────────
    def _pose_cb(self, msg: PoseStamped):
        now = self.get_clock().now().nanoseconds * 1e-9

        # ★ 시간은 mocap 스탬프를 쓴다 (수신 시각이 아니라).
        #   네트워크 지터가 dt 에 그대로 들어가면 속도 추정이 흔들린다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is None:
            dt = 0.01
        else:
            dt = stamp - self.last_stamp
            # 스탬프가 뒤로 가거나(재시작) 너무 벌어지면 필터를 리셋한다
            if dt <= 0.0 or dt > 0.5:
                self.get_logger().warn(
                    f'mocap 스탬프 이상 (dt={dt:.3f}s) — 필터 리셋')
                self.kf_x.init = self.kf_y.init = self.kf_yaw.init = False
                self.prev_yaw_raw = None
                dt = 0.01
        self.last_stamp = stamp
        self.last_rx = now

        raw_x = msg.pose.position.x
        raw_y = msg.pose.position.y
        raw_yaw = yaw_from_quat(msg.pose.orientation)

        # 강체 원점 → 차량 기준점 보정
        yaw = raw_yaw + self.yaw_off
        px = raw_x + math.cos(yaw) * self.off_x - math.sin(yaw) * self.off_y
        py = raw_y + math.sin(yaw) * self.off_x + math.cos(yaw) * self.off_y

        # ★ yaw 는 감기 전에 unwrap 해야 한다.
        #   +pi → -pi 로 넘어가는 순간 차분하면 -2pi 짜리 가짜 각속도가
        #   튀고, KF 가 그걸 실제 회전으로 믿어 한동안 요동친다.
        if self.prev_yaw_raw is None:
            self.yaw_unwrapped = yaw
        else:
            d = yaw - self.prev_yaw_raw
            d = (d + math.pi) % (2.0 * math.pi) - math.pi
            self.yaw_unwrapped += d
        self.prev_yaw_raw = yaw

        sx = self.kf_x.update(px, dt)
        sy = self.kf_y.update(py, dt)
        syaw = self.kf_yaw.update(self.yaw_unwrapped, dt)

        fx, vx_g = sx[0], sx[1]
        fy, vy_g = sy[0], sy[1]
        fyaw, yaw_rate = syaw[0], syaw[1]

        # 전역 속도 → 차체 전진속도. MPC 의 v 는 차체 x 방향 속도다.
        speed = vx_g * math.cos(fyaw) + vy_g * math.sin(fyaw)
        lateral = -vx_g * math.sin(fyaw) + vy_g * math.cos(fyaw)

        self._publish(msg.header.stamp, fx, fy, fyaw,
                      speed, lateral, yaw_rate)
        self.valid_pub.publish(Bool(data=True))
        self.warned = False

    # ────────────────────────────────────────────────────────────────
    def _watchdog(self):
        if self.last_rx is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_rx > self.timeout:
            self.valid_pub.publish(Bool(data=False))
            if not self.warned:
                self.get_logger().error(
                    f'✗ mocap 끊김 ({now - self.last_rx:.2f}s) — '
                    '컨트롤러 정지 신호 발행')
                self.warned = True

    def _publish(self, stamp, x, y, yaw, vx, vy, yaw_rate):
        qx, qy, qz, qw = quat_from_yaw(yaw)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.map_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = vx
        od.twist.twist.linear.y = vy
        od.twist.twist.angular.z = yaw_rate
        self.state_pub.publish(od)
        self.odom_pub.publish(od)

        pc = PoseWithCovarianceStamped()
        pc.header = od.header
        pc.pose.pose = od.pose.pose
        self.pose_pub.publish(pc)

        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
