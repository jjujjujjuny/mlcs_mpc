#!/usr/bin/env python3
"""
relocalize — Cartographer 위치추정의 시작 위치를 잡고, 근처에서 맵에 가장 잘 맞는 자세로 다시 맞춘다.

  RViz "2D Pose Estimate" (/initialpose)  → 찍은 곳 근처(±click_xy, ±click_yaw)에서 스캔 매칭
  조이스틱 LT (/mpc/relocalize)           → 지금 위치 근처(±align_xy, ±align_yaw)에서 스캔 매칭
                    ↓
  찾은 자세로 Cartographer 궤적을 다시 시작한다 (finish_trajectory → start_trajectory, initial_pose)
                    ↓
  /localization/ready = true (래치)  → lidar_bridge 가 그때부터 /mocap/valid 를 true 로 낸다

■ 왜 필요한가 (2026-11-08 주행 두 번)

  시작 위치를 안 주면 Cartographer 는 원점에서 새 궤적을 시작하고 맵 전체에서 맞는 곳을 찾는다.
  첫 주행은 27 초 동안 최대 180 m 엉뚱한 곳에 있었고, 맞춘 뒤에도 전역 제약이 붙을 때마다 수 m 씩 튀었다.

■ 스캔 매칭

  /map (OccupancyGrid) 의 점유 셀까지 거리 d 로 우도 exp(-d²/2σ²) 를 만들고, 스캔 점들의 평균 우도가
  가장 큰 (x, y, yaw) 를 격자 탐색한다 (거친 → 고운). 차는 멈춰 있어야 한다 — 스캔과 자세가 같은 순간이라고 본다.
"""
import math
import threading
import time

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros

from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def likelihood_field(grid, sigma):
    """OccupancyGrid → (우도 배열 [row=y, col=x], 원점 x, y, 해상도)."""
    info = grid.info
    a = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    occ = a >= 65
    if not occ.any():
        return None
    d = distance_transform_edt(~occ) * info.resolution
    return (np.exp(-0.5 * (d / sigma) ** 2).astype(np.float32),
            info.origin.position.x, info.origin.position.y, info.resolution)


def score_poses(field, pts, xs, ys, yaw):
    """yaw 하나에 대해 (len(ys), len(xs)) 점수 격자. pts: (P, 2) base_link 좌표."""
    L, ox, oy, res = field
    c, s = math.cos(yaw), math.sin(yaw)
    px = c * pts[:, 0] - s * pts[:, 1]
    py = s * pts[:, 0] + c * pts[:, 1]
    gx = ((xs[None, :, None] + px[None, None, :]) - ox) / res      # (1, X, P)
    gy = ((ys[:, None, None] + py[None, None, :]) - oy) / res      # (Y, 1, P)
    ix = np.floor(gx).astype(np.int32)
    iy = np.floor(gy).astype(np.int32)
    ix, iy = np.broadcast_arrays(ix, iy)
    ok = (ix >= 0) & (ix < L.shape[1]) & (iy >= 0) & (iy < L.shape[0])
    v = np.zeros(ix.shape, np.float32)
    v[ok] = L[iy[ok], ix[ok]]
    return v.mean(axis=2)


def match(field, pts, x0, y0, yaw0, rxy, ryaw, sxy, syaw):
    """(x0, y0, yaw0) 주변 ±rxy, ±ryaw 를 sxy, syaw 간격으로 탐색. → (x, y, yaw, 점수)."""
    xs = x0 + np.arange(-rxy, rxy + 1e-9, sxy)
    ys = y0 + np.arange(-rxy, rxy + 1e-9, sxy)
    best = (x0, y0, yaw0, -1.0)
    for yaw in yaw0 + np.arange(-ryaw, ryaw + 1e-12, syaw):
        g = score_poses(field, pts, xs, ys, yaw)
        k = int(np.argmax(g))
        if g.flat[k] > best[3]:
            iy, ix = divmod(k, len(xs))
            best = (float(xs[ix]), float(ys[iy]), float(yaw), float(g.flat[k]))
    return best


def refine(field, pts, x0, y0, yaw0, rxy, ryaw):
    """거친 탐색 뒤 두 번 좁혀서 고운 탐색."""
    x, y, w, sc = match(field, pts, x0, y0, yaw0, rxy, ryaw, 0.02, math.radians(1.0))
    x, y, w, sc = match(field, pts, x, y, w, 0.03, math.radians(1.5), 0.005, math.radians(0.25))
    return x, y, w, sc


class Relocalize(Node):

    def __init__(self):
        super().__init__('relocalize')
        d = self.declare_parameter
        d('configuration_directory', '')
        d('configuration_basename', 'localization_2d.lua')
        d('base_frame', 'base_link')
        d('click_xy', 0.5)          # RViz 로 찍은 곳에서 찾는 범위 m
        d('click_yaw_deg', 25.0)
        d('align_xy', 0.3)          # LT: 지금 위치에서 찾는 범위 m
        d('align_yaw_deg', 15.0)
        d('sigma', 0.05)            # 우도 폭 m
        d('max_points', 400)
        d('min_score', 0.35)        # 이보다 낮으면 맞은 곳이 없다고 보고 적용하지 않는다
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg_dir = gp('configuration_directory')
        self.cfg_base = gp('configuration_basename')
        self.base_frame = gp('base_frame')
        self.click = (float(gp('click_xy')), math.radians(float(gp('click_yaw_deg'))))
        self.align = (float(gp('align_xy')), math.radians(float(gp('align_yaw_deg'))))
        self.sigma = float(gp('sigma'))
        self.max_points = int(gp('max_points'))
        self.min_score = float(gp('min_score'))

        self.scan = self.grid = self.pose = None
        self.busy = threading.Lock()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.laser_tf = None

        # ★ /scan(40 Hz)·/map·/tracked_pose 는 **요청할 때만** 구독한다. 늘 받아 두면 파이썬
        #   역직렬화만으로 CPU 45% 를 썼다 (2026-11-08, 주행 중 Cartographer 와 경쟁).
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._click_cb, 1)
        self.create_subscription(Bool, '/mpc/relocalize', self._align_cb, 10)
        self.ready_pub = self.create_publisher(Bool, '/localization/ready', LATCHED)
        self.ready_pub.publish(Bool(data=False))
        self.cli_states = self.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.cli_finish = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.cli_start = self.create_client(StartTrajectory, '/start_trajectory')
        self.get_logger().info(
            'relocalize 준비 — RViz "2D Pose Estimate" 로 시작 위치를 찍으세요. '
            'LT = 지금 위치 근처에서 다시 맞춤. 그 전에는 출발하지 않습니다.')

    # ── 입력 ───────────────────────────────────────────────────────
    def _click_cb(self, m):
        p = m.pose.pose
        self._spawn('RViz', p.position.x, p.position.y, yaw_of(p.orientation), *self.click)

    def _align_cb(self, m):
        if not m.data:
            return
        self._spawn('LT', None, None, None, *self.align)

    def _grab(self, msg_type, topic, qos, timeout=2.0):
        """topic 에서 메시지 하나를 받아 온다 (스레드에서 부른다). 없으면 None."""
        box = []
        sub = self.create_subscription(msg_type, topic, lambda m: box.append(m) if not box else None, qos)
        end = time.monotonic() + timeout
        while not box and time.monotonic() < end:
            time.sleep(0.01)
        self.destroy_subscription(sub)
        return box[0] if box else None

    def _spawn(self, who, x, y, yaw, rxy, ryaw):
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn(f'{who} — 이전 맞춤이 진행 중입니다')
            return
        threading.Thread(target=self._run, args=(who, x, y, yaw, rxy, ryaw), daemon=True).start()

    # ── 본 작업 (스레드) ────────────────────────────────────────────
    def _points(self):
        s = self.scan
        if self.laser_tf is None:
            t = self.tf_buf.lookup_transform(self.base_frame, s.header.frame_id, rclpy.time.Time())
            tr, q = t.transform.translation, t.transform.rotation
            self.laser_tf = (tr.x, tr.y, yaw_of(q))
        r = np.asarray(s.ranges, dtype=np.float64)
        a = s.angle_min + s.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > max(s.range_min, 0.05)) & (r < min(s.range_max, 15.0))
        r, a = r[ok], a[ok]
        if len(r) > self.max_points:
            k = np.linspace(0, len(r) - 1, self.max_points).astype(int)
            r, a = r[k], a[k]
        lx, ly, lw = self.laser_tf
        px, py = r * np.cos(a + lw), r * np.sin(a + lw)
        return np.stack([px + lx, py + ly], axis=1)

    def _run(self, who, x, y, yaw, rxy, ryaw):
        log = self.get_logger()
        try:
            if x is None:       # LT: 지금 위치에서
                self.pose = self._grab(PoseStamped, '/tracked_pose', BEST_EFFORT)
                if self.pose is None:
                    log.warn('LT — /tracked_pose 가 없습니다. RViz 로 먼저 찍으세요.')
                    return
                p = self.pose.pose
                x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
            self.scan = self._grab(LaserScan, '/scan', BEST_EFFORT)
            self.grid = self._grab(OccupancyGrid, '/map', 1, timeout=3.0)
            if self.scan is None or self.grid is None:
                log.error(f'{who} — /scan 또는 /map 을 못 받았습니다')
                return
            t0 = time.monotonic()
            field = likelihood_field(self.grid, self.sigma)
            if field is None:
                log.error(f'{who} — /map 에 벽이 없습니다')
                return
            pts = self._points()
            before = float(score_poses(field, pts, np.array([x]), np.array([y]), yaw)[0, 0])
            bx, by, bw, sc = refine(field, pts, x, y, yaw, rxy, ryaw)
            dt = time.monotonic() - t0
            msg = (f'{who} — 맞춤 {dt:.1f} s, 점수 {before:.2f} → {sc:.2f}, '
                   f'이동 {math.hypot(bx - x, by - y) * 100:.1f} cm, '
                   f'yaw {math.degrees((bw - yaw + math.pi) % (2 * math.pi) - math.pi):+.1f}°')
            if sc < self.min_score:
                log.error(msg + f' — 점수가 {self.min_score} 미만이라 적용하지 않습니다. 다시 찍으세요.')
                return
            log.info(msg)
            if not self._restart(bx, by, bw):
                return
            self.ready_pub.publish(Bool(data=True))
            log.info(f'✓ 위치 설정 ({bx:.3f}, {by:.3f}, {math.degrees(bw):.1f}°) — 조이스틱 B 로 출발')
        except Exception as e:          # noqa: BLE001 — 스레드에서 죽으면 아무도 모른다
            log.error(f'{who} — 실패: {e!r}')
        finally:
            self.busy.release()

    def _call(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{cli.srv_name} 서비스가 없습니다 (Cartographer 가 떠 있나?)')
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                raise RuntimeError(f'{cli.srv_name} 응답 없음')
            time.sleep(0.01)
        return fut.result()

    def _restart(self, x, y, yaw):
        st = self._call(self.cli_states, GetTrajectoryStates.Request()).trajectory_states
        ids, states = list(st.trajectory_id), list(st.trajectory_state)
        frozen = [i for i, s in zip(ids, states) if s == st.FROZEN]
        if not frozen:
            self.get_logger().error(f'고정된 맵 궤적이 없습니다 (상태 {list(zip(ids, states))})')
            return False
        for i, s in zip(ids, states):
            if s == st.ACTIVE:
                r = self._call(self.cli_finish, FinishTrajectory.Request(trajectory_id=i))
                if r.status.code != 0:
                    self.get_logger().warn(f'궤적 {i} 종료: {r.status.message}')
        req = StartTrajectory.Request()
        req.configuration_directory = self.cfg_dir
        req.configuration_basename = self.cfg_base
        req.use_initial_pose = True
        req.initial_pose.position.x, req.initial_pose.position.y = x, y
        req.initial_pose.orientation.z = math.sin(yaw / 2)
        req.initial_pose.orientation.w = math.cos(yaw / 2)
        req.relative_to_trajectory_id = frozen[0]
        r = self._call(self.cli_start, req)
        if r.status.code != 0:
            self.get_logger().error(f'궤적 시작 실패: {r.status.message}')
            return False
        self.get_logger().info(f'Cartographer 궤적 {r.trajectory_id} 시작 (맵 궤적 {frozen[0]} 기준)')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Relocalize()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
