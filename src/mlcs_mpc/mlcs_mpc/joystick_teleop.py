#!/usr/bin/env python3
"""
joystick_teleop — 로지텍 F710 조이스틱 수동 조종.

  sensor_msgs/Joy  →  /teleop (AckermannDriveStamped)
                   →  /mpc/manual (Bool)   ← "사람이 조종 중" 신호

조작법 (Logitech F710, 뒷면 **Mode 버튼 OFF** — LED 꺼진 상태):

    RT 당김           전진 (당긴 만큼 선형, 놓으면 즉시 정지)
    RB 누른 채 RT     후진
    L스틱 좌/우       조향
    LB                수동 ↔ 자율 토글
    START(버튼 7)     비상 정지 — 자율 해제 + 속도 0

■ ★ "안 보내는 것" 으로는 차가 서지 않는다

  이 노드에서 가장 중요한 부분은 워치독이다. 26-jetson 저장소는 2026-08-17
  하루에 **세 번** "차가 혼자 직진하는" 사고를 겪었고, 셋 다 구조가 같았다:

      ① 다른 창의 joy_teleop 이 살아 있는데 트리거를 건드림
      ② 주행 중 조이스틱 USB 를 뽑음
      ③ 무선 수신기가 끊김

  셋 다 **/joy 가 끊긴 것**이고, 그러면 보통의 노드는 발행을 멈춘다.
  그런데 **VESC 는 새 명령이 없으면 마지막 속도를 무한히 유지한다.**
  ackermann_mux 의 타임아웃도 소용없다 — mux 가 채널을 내려도 이미 VESC 에
  들어간 erpm 은 남아 있기 때문이다.

  그래서 멈추려면 **0 을 계속 보내야 한다.** joy_timeout 을 넘기면 20Hz 로
  속도 0 을 쏜다. 조향은 마지막 값을 유지한다 — 급조향이 더 위험하다.

  ※ 이 노드가 통째로 죽는 경우는 못 막는다. 하드웨어 킬스위치의 영역이다.

■ 트리거 부호 — 자동 캘리브레이션

  같은 RT 트리거인데 읽는 경로마다 휴지값이 다르다:

      커널 /dev/input/js0 : 안 누름 = -1  또는  0
      ROS2 joy 의 /joy    : 안 누름 = +1        ← 보통 이쪽

  yaml 로 고정하면 드라이버 버전이 바뀔 때 조용히 틀어지고, 그러면
  **RT 를 안 눌렀는데 최고 속도가 나간다.** 그래서 기동 직후 0.4초간 RT 축
  평균을 재서 휴지값을 스스로 잡는다. 캘리 중에는 속도가 0 으로 묶인다.

  ★ 기동할 때 RT 에서 손을 떼고 가만히 두세요.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class JoystickTeleop(Node):

    def __init__(self):
        super().__init__('joystick_teleop')

        # ── 축·버튼 배치 (F710, Mode OFF / X-input 기준) ─────────────
        self.declare_parameter('axis_steer', 0)       # L스틱 좌우
        self.declare_parameter('axis_speed', 5)       # RT 트리거
        self.declare_parameter('btn_reverse', 5)      # RB — 누르는 동안 후진
        self.declare_parameter('btn_toggle', 4)       # LB — 수동↔자율
        self.declare_parameter('btn_estop', 7)        # START — 비상 정지

        # ── 모드 ────────────────────────────────────────────────────
        # allow_toggle=False 면 수동 전용. 자율로 넘길 대상(mpc_node)이
        # 없는데 토글되면 "전환했다는 로그만 뜨고 차는 가만히" 있게 된다 —
        # 26-jetson 이 실제로 겪은 조용한 실패라 아예 끌 수 있게 뒀다.
        self.declare_parameter('allow_toggle', True)
        self.declare_parameter('start_auto', False)   # ★ 기본은 수동으로 시작

        # ── 부호 / 감도 ──────────────────────────────────────────────
        self.declare_parameter('steer_invert', False)
        self.declare_parameter('speed_invert', False)
        self.declare_parameter('trigger_invert', True)    # 초기 추정값
        self.declare_parameter('trigger_deadzone', 0.05)
        self.declare_parameter('trigger_expo', 1.0)       # 1.0 = 완전 선형
        self.declare_parameter('steer_deadzone', 0.08)

        # ── 상한 ────────────────────────────────────────────────────
        self.declare_parameter('max_speed', 1.5)   # ★ 캘리브레이션 전 저속
        self.declare_parameter('max_steer', 0.36)

        # ── 안전 ────────────────────────────────────────────────────
        # 0.3s = joy 20~50Hz 기준 6~15 프레임. 무선 순간 끊김에는 안 걸리고
        # 진짜 단절에는 확실히 걸리는 값.
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('debug', False)

        def p(n):
            return self.get_parameter(n).value

        self.ax_steer = int(p('axis_steer'))
        self.ax_speed = int(p('axis_speed'))
        self.btn_rev = int(p('btn_reverse'))
        self.btn_tog = int(p('btn_toggle'))
        self.btn_estop = int(p('btn_estop'))
        self.allow_tog = bool(p('allow_toggle'))
        self.auto_mode = bool(p('start_auto'))
        self.inv_steer = bool(p('steer_invert'))
        self.inv_speed = bool(p('speed_invert'))
        self.inv_trigger = bool(p('trigger_invert'))
        self.trig_dz = min(max(float(p('trigger_deadzone')), 0.0), 0.9)
        self.trig_expo = max(float(p('trigger_expo')), 0.1)
        self.steer_dz = min(max(float(p('steer_deadzone')), 0.0), 0.9)
        self.max_speed = float(p('max_speed'))
        self.max_steer = float(p('max_steer'))
        self.joy_timeout = float(p('joy_timeout'))
        self.debug = bool(p('debug'))

        # 트리거 양 끝점 — 아래 자동 캘리로 덮인다
        self.trig_neutral = 1.0 if self.inv_trigger else -1.0
        self.trig_full = -self.trig_neutral
        self._rest_samples = []
        self._rest_calibrated = False

        self.prev_lb = False
        self.prev_estop = False
        self._last_log = 0.0

        # 워치독 상태
        self._last_joy = None
        self._wd_fired = False
        self._wd_steer = 0.0
        self._wd_speed = 0.0     # 마지막으로 명령한 속도 (경고 수준 판단용)

        # 자율 전환 후 실제로 /drive 가 오는지 감시
        self._drive_seen = 0.0
        self._auto_since = None

        # ── 통신 ────────────────────────────────────────────────────
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(
            AckermannDriveStamped, '/drive', self._drive_cb, 10)

        self.pub = self.create_publisher(
            AckermannDriveStamped, '/teleop', 10)

        # ── ★ /mpc/enabled 를 직접 쓰지 않는다 ─────────────────────────
        #   safety_node 도 같은 토픽에 쓴다. 둘이 같이 쓰면 **마지막에 쓴
        #   쪽이 이기므로**, 경계 이탈로 safety 가 정지시킨 직후 운전자가
        #   LB 를 누르면 그대로 다시 출발해 버린다. 안전장치가 사람의
        #   실수 한 번에 풀리면 안전장치가 아니다.
        #
        #   그래서 "사람이 조종을 가져갔다/돌려줬다" 는 **사실만** 별도
        #   토픽으로 알리고, /mpc/enabled 를 실제로 내릴지 올릴지는
        #   mpc_node 가 자기 상태(safety 포함)와 합쳐서 판단한다.
        #   26-jetson 이 mission_manager 로 같은 문제를 푼 방식이다.
        self.pub_manual = self.create_publisher(Bool, '/mpc/manual', LATCHED)
        self.pub_manual.publish(Bool(data=not self.auto_mode))

        self.create_timer(1.0, self._check_auto)
        if self.joy_timeout > 0.0:
            self.create_timer(0.05, self._joy_watchdog)

        self.get_logger().info(
            f'JoystickTeleop 준비  [{"자율" if self.auto_mode else "수동"} 모드로 시작]\n'
            f'  RT(축{self.ax_speed})      → 전진 (놓으면 즉시 정지)\n'
            f'  RB(버튼{self.btn_rev}) + RT  → 후진\n'
            f'  L스틱(축{self.ax_steer})    → 조향\n'
            + (f'  LB(버튼{self.btn_tog})      → 수동↔자율 토글\n'
               if self.allow_tog else '  LB 토글 없음 — 수동 전용\n')
            + f'  START(버튼{self.btn_estop})   → 비상 정지\n'
            f'  max_speed={self.max_speed} m/s  max_steer={self.max_steer} rad\n'
            '  ★ 트리거 자동 캘리 중 — RT 에서 손 떼고 가만히 두세요')

    # ────────────────────────────────────────────────────────────────
    def _drive_cb(self, msg):
        self._drive_seen = time.time()

    def _check_auto(self):
        """자율로 넘겼는데 /drive 가 안 오면 알려준다.

        자율 모드는 "/teleop 발행을 멈춰 mux 를 /drive 채널로 넘기는" 것뿐이다.
        /drive 를 내는 mpc_node 가 안 떠 있으면 아무 일도 안 일어나는데,
        로그만 "자율" 이라 원인을 찾기 어렵다.
        """
        if not self.auto_mode or self._auto_since is None:
            return
        now = time.time()
        if now - self._auto_since < 1.5 or now - self._drive_seen < 0.5:
            return
        self.get_logger().warn(
            '자율 모드인데 /drive 가 오지 않습니다 — 차는 움직이지 않습니다.\n'
            '  mpc_node 가 떠 있는지, 웨이포인트가 로드됐는지 확인하세요:\n'
            '    ros2 topic hz /drive\n'
            '    ros2 node list | grep mpc',
            throttle_duration_sec=5.0)

    # ── 트리거 휴지값 자동 캘리 ──────────────────────────────────────
    def _calibrate_rest(self, raw):
        """True 를 반환하는 동안은 캘리 중 — 속도를 0 으로 묶는다."""
        if self._rest_calibrated:
            return False
        self._rest_samples.append(float(raw))
        if len(self._rest_samples) < 20:      # joy 20~50Hz 기준 0.4~1초
            return True

        m = sum(self._rest_samples) / len(self._rest_samples)
        if m >= 0.4:
            self.trig_neutral, self.trig_full = 1.0, -1.0
            mode = '휴지=+1, 당김=-1 (ROS joy 표준)'
        elif m <= -0.4:
            self.trig_neutral, self.trig_full = -1.0, 1.0
            mode = '휴지=-1, 당김=+1'
        else:
            # 휴지≈0, 당김≈-1 (F710 커널 드라이버에서 흔하다)
            self.trig_neutral, self.trig_full = 0.0, -1.0
            mode = '휴지≈0, 당김=-1 (커널 드라이버)'

        self._rest_calibrated = True
        self.get_logger().info(
            f'트리거 캘리 완료: 휴지 평균={m:+.3f} → {mode} — 이제 RT 사용 가능')
        return False

    # ── raw → 0.0~1.0 스로틀 ────────────────────────────────────────
    def _throttle(self, raw):
        if abs(self.trig_neutral) < 0.2:
            # 휴지≈0 인 경우: raw 0 → 0, raw -1 → 1
            u = max(0.0, min(1.0, -float(raw)))
        else:
            span = self.trig_full - self.trig_neutral
            if abs(span) < 1e-6:
                return 0.0
            u = min(max((raw - self.trig_neutral) / span, 0.0), 1.0)

        if u <= self.trig_dz:
            return 0.0
        u = (u - self.trig_dz) / (1.0 - self.trig_dz)
        return u ** self.trig_expo

    # ── 스틱 중립 유격 제거 ─────────────────────────────────────────
    def _stick(self, raw):
        mag = abs(raw)
        if mag <= self.steer_dz:
            return 0.0
        mag = (mag - self.steer_dz) / (1.0 - self.steer_dz)
        return min(mag, 1.0) * (1.0 if raw > 0 else -1.0)

    # ────────────────────────────────────────────────────────────────
    def _joy_cb(self, msg: Joy):
        self._last_joy = self.get_clock().now().nanoseconds * 1e-9

        def btn(i):
            return len(msg.buttons) > i and bool(msg.buttons[i])

        # ── 비상 정지 (상승 엣지) ───────────────────────────────────
        es = btn(self.btn_estop)
        if es and not self.prev_estop:
            self.auto_mode = False
            self._auto_since = None
            self.pub_manual.publish(Bool(data=True))
            self._publish(0.0, 0.0)
            self.get_logger().error(
                '★ 비상 정지 — 자율 해제, 속도 0. '
                '다시 출발하려면 LB 로 자율 전환하세요.')
        self.prev_estop = es

        # ── 모드 토글 (LB 상승 엣지) ────────────────────────────────
        lb = btn(self.btn_tog)
        if lb and not self.prev_lb:
            if not self.allow_tog:
                self.get_logger().info(
                    'LB 토글은 이 모드에서 꺼져 있습니다 (수동 전용).',
                    throttle_duration_sec=5.0)
            else:
                self.auto_mode = not self.auto_mode
                self._auto_since = time.time() if self.auto_mode else None
                self.pub_manual.publish(Bool(data=not self.auto_mode))
                if self.auto_mode:
                    self.get_logger().info(
                        '모드 전환 → [자율]  MPC 가 지금 위치에서 이어받습니다')
                else:
                    self.get_logger().warn(
                        '모드 전환 → [수동]  MPC 정지, 사람이 조종합니다')
                    self._publish(0.0, 0.0)
        self.prev_lb = lb

        # 자율 모드에서는 /teleop 을 안 낸다 → mux 가 /drive 채널을 통과시킨다
        if self.auto_mode:
            return

        # ── 수동 조종 ───────────────────────────────────────────────
        raw_trig = (msg.axes[self.ax_speed]
                    if len(msg.axes) > self.ax_speed else self.trig_neutral)
        raw_steer = (msg.axes[self.ax_steer]
                     if len(msg.axes) > self.ax_steer else 0.0)
        reverse = btn(self.btn_rev)

        if self._calibrate_rest(raw_trig):
            self._publish(0.0, 0.0)
            return

        u = self._throttle(raw_trig)
        speed = u * self.max_speed
        if reverse:
            speed = -speed
        if self.inv_speed:
            speed = -speed

        steer = self._stick(raw_steer)
        if self.inv_steer:
            steer = -steer
        steer *= self.max_steer

        speed = float(max(-self.max_speed, min(self.max_speed, speed)))
        steer = float(max(-self.max_steer, min(self.max_steer, steer)))

        if self.debug:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_log > 0.5:
                self._last_log = now
                self.get_logger().info(
                    f'RT raw {raw_trig:+.3f} → {u*100:5.1f}% → {speed:+.2f} m/s'
                    f' | 조향 {steer:+.3f} rad ({math.degrees(steer):+.1f}°)')

        self._publish(steer, speed)

    # ── /joy 워치독 — 이 노드에서 가장 중요한 부분 ───────────────────
    def _joy_watchdog(self):
        if self._last_joy is None:
            return              # 아직 한 번도 안 받음 — 정상 기동 중
        gap = self.get_clock().now().nanoseconds * 1e-9 - self._last_joy

        if gap < self.joy_timeout:
            if self._wd_fired:
                self._wd_fired = False
                self.get_logger().info(f'/joy 복구 ({gap*1000:.0f}ms) — 조종 재개')
            return

        if self.auto_mode:
            return              # 자율 주행 중이면 /teleop 을 낼 일이 없다

        # ★ 정지 상태에서는 경고하지 않는다 (2026-09-14 젯슨 실측)
        #   joy_node 는 autorepeat_rate 가 붙기 전 기동 직후 한 박자 쉰다.
        #   그때 워치독이 "끊겼다" 고 에러를 뱉었다가 1ms 뒤 "복구" 로 이어졌다:
        #       /joy 가 0.33s 끊겼습니다 — ★ 속도 0 강제 발행
        #       /joy 복구 (1ms) — 조종 재개
        #   차는 어차피 서 있었으므로 위험한 상황이 아닌데 빨간 에러가 떠서,
        #   진짜 폭주 위험과 구분이 안 된다. 경고가 잦으면 무시하게 되고
        #   그게 이 워치독을 무력화한다.
        #
        #   ⚠ 단, **발행은 계속한다.** 0 을 계속 보내는 것이 이 워치독의
        #     본체다 (VESC 는 마지막 명령을 유지한다). 조용히 할 뿐이다.
        moving = abs(self._wd_speed) > 1e-6
        if not self._wd_fired:
            self._wd_fired = True
            if moving:
                self.get_logger().error(
                    f'/joy 가 {gap:.2f}s 끊겼습니다 — ★ 속도 0 강제 발행.\n'
                    '  조이스틱 연결·배터리를 확인하세요.\n'
                    '  (발행을 멈추는 것으로는 안 섭니다: VESC 는 마지막 명령을 유지합니다)')
            else:
                self.get_logger().info(
                    f'/joy 끊김 {gap:.2f}s (정지 상태) — 0 을 계속 발행합니다',
                    throttle_duration_sec=10.0)
        self._publish(self._wd_steer, 0.0)
        return

    def _publish(self, steer, speed):
        self._wd_steer = float(steer)    # 워치독이 유지할 마지막 조향
        self._wd_speed = float(speed)    # 위험도 판단용 (정지 중이면 조용히)
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.steering_angle = float(steer)
        m.drive.speed = float(speed)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령을 남긴다 — Ctrl+C 로 죽였는데 마지막 속도가
        # VESC 에 남아 차가 계속 가는 상황을 막는다.
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
