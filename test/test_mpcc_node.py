#!/usr/bin/env python3
"""
test_mpcc_node.py — mpcc_node 안전 동작 시험. 차·VESC·mocap 불필요.

  python3 test/test_mpcc_node.py [--only 이름]

  별도 ROS_DOMAIN_ID 에서 진짜 노드와 진짜 acados 워커를 띄우고, 이 스크립트가 가짜 차
  (/mpc/state, /mocap/valid, /sensors/core) 노릇을 한다. 실제 차에는 닿지 않는다.

  멈춘 solve 는 세 가지로 만든다: 진짜로 돌아오지 않는 solve (--no-guard + 슬립 0),
  SIGSTOP, 프로세스 kill. 비정상 해는 가짜 워커로 만든다.
"""
import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '87'

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Bool, Float32  # noqa: E402
from vesc_msgs.msg import VescStateStamped  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = os.path.expanduser('~/f1tenth_ws/install/mlcs_mpc/lib/mlcs_mpc/mpcc_node')
CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
L, LR = 0.33, 0.1537

FAKE_WORKER = r'''
import argparse, json, os, sys
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument('--fd-in', type=int); ap.add_argument('--fd-out', type=int); ap.add_argument('--config')
a = ap.parse_args(); N = json.loads(a.config)['N']
os.write(a.fd_out, b'R')
while True:
    buf = b''
    while len(buf) < 8 * (3 + 9 + 2 * (N + 1) + N):
        c = os.read(a.fd_in, 4096)
        if not c: sys.exit(0)
        buf += c
    rid = np.frombuffer(buf, dtype=np.float64)[0]
    st, u = (4.0, 0.5) if os.environ['FAKE'] == 'status' else (0.0, float('nan'))
    r = np.concatenate([[rid, st, 1.0, u, 0.1], np.zeros(9), np.zeros(N + 1)])
    os.write(a.fd_out, r.tobytes())
'''


class FakeCar:
    """100 Hz 운동학 차. /drive 를 받아 움직이고 기록한다."""

    def __init__(self, x, y, yaw, v=0.0, frozen=False):
        self.n = rclpy.create_node('fake_car')
        self.x, self.y, self.yaw, self.v = x, y, yaw, v
        self.delta = 0.0
        self.frozen = frozen    # True 면 움직이지 않고 상태를 그대로 낸다
        self.speed_cmd = self.steer_cmd = 0.0
        self.log = []           # (시각, 속도, 조향)
        self.solve = []
        self.send_state = self.send_vesc = True
        self.valid = True
        self.nan = False
        self.st = self.n.create_publisher(Odometry, '/mpc/state', 10)
        self.vp = self.n.create_publisher(Bool, '/mocap/valid', 10)
        self.vs = self.n.create_publisher(VescStateStamped, '/sensors/core', 10)
        self.en = self.n.create_publisher(Bool, '/mpc/enabled', LATCHED)
        self.ma = self.n.create_publisher(Bool, '/mpc/manual', LATCHED)
        self.go = self.n.create_publisher(Bool, '/mpc/go', 10)
        self.n.create_subscription(AckermannDriveStamped, '/drive', self._drive, 50)
        self.n.create_subscription(Float32, '/mpc/solve_time', lambda m: self.solve.append(m.data), 50)
        self.n.create_timer(0.01, self._tick)

    def _drive(self, m):
        self.speed_cmd, self.steer_cmd = m.drive.speed, m.drive.steering_angle
        self.log.append((time.monotonic(), m.drive.speed, m.drive.steering_angle))

    def _tick(self):
        dt = 0.01
        if not self.frozen:
            self.v += (self.speed_cmd - self.v) * dt / 0.3
            self.delta += float(np.clip(self.steer_cmd - self.delta, -0.032, 0.032))
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v * math.tan(self.delta) / L * dt
        self.vp.publish(Bool(data=self.valid))
        if self.send_vesc:
            m = VescStateStamped()
            m.state.speed = self.v * 4781.5
            self.vs.publish(m)
        if self.send_state:
            o = Odometry()
            o.pose.pose.position.x = float('nan') if self.nan else self.x
            o.pose.pose.position.y = self.y
            o.pose.pose.orientation.z = math.sin(self.yaw / 2)
            o.pose.pose.orientation.w = math.cos(self.yaw / 2)
            o.twist.twist.linear.x = self.v
            o.twist.twist.angular.z = 0.0 if self.frozen else self.v * math.tan(self.delta) / L
            self.st.publish(o)

    def spin(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.005)

    def enable(self, on):
        self.en.publish(Bool(data=on))

    def since(self, t):
        return [e for e in self.log if e[0] >= t]

    def stop_latency(self, t):
        """t 이후 마지막 비정지 명령까지의 시간. 없으면 0."""
        moving = [e[0] for e in self.since(t) if e[1] != 0.0]
        return (moving[-1] - t) if moving else 0.0

    def close(self):
        self.n.destroy_node()


class Harness:
    live = []

    def __init__(self, car, *params, env=None):
        self.car = car
        Harness.live.append(self)
        args = [EXE, '--ros-args', '--params-file', CFG, '-p', f'repo_root:={ROOT}']
        for p in params:
            args += ['-p', p]
        e = dict(os.environ, **(env or {}))
        log = open(os.environ.get('MPCC_TEST_LOG', '/tmp/mlcs_mpcc_test_node.log'), 'ab')
        self.proc = subprocess.Popen(args, env=e, stdout=log, stderr=log, start_new_session=True)

    def worker_pid(self):
        out = subprocess.run(['pgrep', '-P', str(self.proc.pid)], capture_output=True, text=True)
        return [int(x) for x in out.stdout.split()]

    def wait_ready(self, secs=40.0):
        """워커가 답할 때까지 기다린다. enable 후 첫 움직임 또는 solve_time."""
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.car.spin(0.1)
            if self.car.solve:
                return True
        return False

    def close(self):
        Harness.live.remove(self)
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            os.killpg(self.proc.pid, signal.SIGKILL)
            self.proc.wait()
        self.car.close()


def track_start():
    d = np.genfromtxt(os.path.join(ROOT, 'src/mlcs_mpc/mlcs_mpc/env/track/data/s_track/prep_s_track.csv'),
                      delimiter=',', names=True)
    h = d['heading'][0]
    # 무게중심이 첫 점에 오도록 뒤축을 놓는다
    return d['x'][0] - LR * math.cos(h), d['y'][0] - LR * math.sin(h), h


def check(name, ok, detail=''):
    print(f'  {"✓" if ok else "✗"} {name}  {detail}', flush=True)
    return ok


def test_gates():
    x, y, h = track_start()
    car = FakeCar(x, y, h)
    n = Harness(car)
    r = True
    car.spin(3.0)
    t = time.monotonic()
    car.spin(1.0)
    w = car.since(t)
    gap = np.diff([e[0] for e in w]).max() if len(w) > 1 else 9.9
    # ackermann_mux timeout 0.2 s 안에 다음 명령이 와야 한다.
    r &= check('출발 전 속도 0 을 계속 보냄', gap < 0.2 and all(e[1] == 0.0 for e in w),
               f'{len(w)} Hz, 최대 간격 {gap * 1000:.0f} ms')
    r &= check('B 전에는 정지', not any(e[1] > 0 for e in car.since(t)))
    car.go.publish(Bool(data=True))     # 조이스틱 B
    if not n.wait_ready():
        n.close()
        return check('B 뒤 워커 준비', False)
    t = time.monotonic()
    car.spin(12.0)
    w = car.since(t)
    sp = np.array([e[1] for e in w])
    r &= check('주행: v_max 를 넘지 않음', sp.max() <= 0.7 + 1e-6 and sp.max() > 0.5, f'최대 {sp.max():.3f}')
    r &= check('주행: 30 Hz 근처', 20 <= len(w) / 12.0 <= 35, f'{len(w) / 12.0:.1f} Hz')
    ms = np.array(car.solve[10:])
    print(f'    풀이 평균 {ms.mean():.2f} p95 {np.percentile(ms, 95):.2f} 최대 {ms.max():.2f} ms, v {car.v:.2f}')
    pid = n.worker_pid()[0]
    allowed = [ln for ln in open(f'/proc/{pid}/status') if 'Cpus_allowed_list' in ln][0].split()[1]
    r &= check('워커 코어 지정', allowed == '5', allowed)

    for name, off, on in (
            ('mocap 무효', lambda: setattr(car, 'valid', False), lambda: setattr(car, 'valid', True)),
            ('상태 끊김', lambda: setattr(car, 'send_state', False), lambda: setattr(car, 'send_state', True)),
            ('VESC 끊김', lambda: setattr(car, 'send_vesc', False), lambda: setattr(car, 'send_vesc', True)),
            ('상태 NaN', lambda: setattr(car, 'nan', True), lambda: setattr(car, 'nan', False)),
            ('수동 조종', lambda: car.ma.publish(Bool(data=True)), lambda: car.ma.publish(Bool(data=False))),
            ('출발 해제', lambda: car.enable(False), lambda: car.enable(True))):
        t = time.monotonic()
        off()
        car.spin(1.0)
        lat = car.stop_latency(t)
        w = [e for e in car.since(t + 0.5)]
        r &= check(f'{name}: 정지', lat < 0.45 and len(w) >= 10 and all(e[1] == 0.0 for e in w),
                   f'마지막 비정지 {lat * 1000:.0f} ms, 0 명령 {len(w) * 2} Hz')
        t = time.monotonic()
        on()
        car.spin(1.5)
        moved = any(e[1] > 0 for e in car.since(t))
        if name in ('mocap 무효', '상태 끊김', '상태 NaN'):
            # 달리던 중 mocap 불량은 혼자 다시 출발하지 않는다.
            r &= check(f'{name}: 복구돼도 혼자 출발 안 함', not moved)
            if name == 'mocap 무효':
                # 끊긴 동안 LB 를 두 번 눌러도 복구 후 혼자 출발하면 안 된다.
                off()
                car.spin(0.3)
                car.ma.publish(Bool(data=True))
                car.spin(0.2)
                car.ma.publish(Bool(data=False))
                car.spin(0.3)
                t = time.monotonic()
                on()
                car.spin(1.5)
                r &= check('끊긴 동안 LB 두 번: 복구돼도 출발 안 함',
                           not any(e[1] > 0 for e in car.since(t)))
            car.ma.publish(Bool(data=True))
            car.spin(0.2)
            car.ma.publish(Bool(data=False))
            t = time.monotonic()
            car.spin(1.5)
            moved = any(e[1] > 0 for e in car.since(t))
        r &= check(f'{name}: 복귀', moved)
    n.close()
    return r


def hang_case(name, car, n, trigger):
    r = True
    t = time.monotonic()
    trigger()
    car.spin(2.0)
    lat = car.stop_latency(t)
    w = car.since(t + 0.5)
    r &= check(f'{name}: 정지', lat < 0.35 and len(w) >= 20 and all(e[1] == 0.0 for e in w),
               f'마지막 비정지 {lat * 1000:.0f} ms, 0 명령 {len(w) / 1.5:.0f} Hz')
    return r


def test_stopped_worker():
    x, y, h = track_start()
    car = FakeCar(x, y, h)
    n = Harness(car)
    car.enable(True)
    r = check('워커 준비', n.wait_ready())
    car.spin(2.0)
    old = n.worker_pid()[0]
    r &= hang_case('SIGSTOP', car, n, lambda: os.kill(old, signal.SIGSTOP))
    car.spin(4.0)
    new = n.worker_pid()
    r &= check('멈춘 워커를 죽이고 새로 띄움', new and old not in new, f'{old} → {new}')
    t = time.monotonic()
    car.spin(1.0)
    r &= check('고장이 래치됨 (enable 이 켜져 있어도 안 달림)', all(e[1] == 0.0 for e in car.since(t)))
    car.enable(False)
    car.spin(0.3)
    car.enable(True)
    t = time.monotonic()
    car.spin(3.0)
    r &= check('false→true 로 재개', any(e[1] > 0 for e in car.since(t)))
    r &= hang_case('kill', car, n, lambda: os.kill(n.worker_pid()[0], signal.SIGKILL))
    n.close()
    return r


def test_real_hang():
    # 진짜 돌아오지 않는 solve: 저자 식 그대로, 슬립 0, 곡률 0
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'prep_line.csv')
    s = np.arange(0, 40, 0.2)
    np.savetxt(p, np.c_[s, s, 0 * s, 0 * s, 0 * s, 0 * s + 0.5], delimiter=',',
               header='s,x,y,heading,curvature,track_width', comments='')
    car = FakeCar(1.0, 0.0, 0.0, v=1.0, frozen=True)
    n = Harness(car, f'track_csv:={p}', 'worker_extra_args:=["--no-guard"]', 'v_max:=2.0')
    car.spin(6.0)   # 워커 준비를 기다린다 (requests 전)
    t = time.monotonic()
    # 뒤축 x, CoG x=x+lr. 슬립이 정확히 0: vy=r=0, wheel=vx=1
    car.enable(True)
    car.spin(4.0)
    w = car.since(t)
    moving = [e for e in w if e[1] != 0.0]
    gap = np.diff([e[0] for e in w]).max()
    r = check('슬립 0 solve 멈춤: 속도 명령 없음', not moving, f'비정지 {len(moving)}개')
    r &= check('멈춘 동안에도 0 을 계속 보냄', gap < 0.1, f'최대 간격 {gap * 1000:.0f} ms')
    r &= check('solve 답이 없었음 (진짜 멈춤)', not car.solve, f'답 {len(car.solve)}개')
    n.close()
    return r


def test_bad_solution():
    r = True
    for mode in ('status', 'nan'):
        x, y, h = track_start()
        car = FakeCar(x, y, h)
        d = tempfile.mkdtemp()
        f = os.path.join(d, 'fake.py')
        open(f, 'w').write(FAKE_WORKER)
        n = Harness(car, f'worker_script:={f}', f'worker_python:={sys.executable}', env={'FAKE': mode})
        car.spin(3.0)
        t = time.monotonic()
        car.enable(True)
        car.spin(2.0)
        w = car.since(t)
        r &= check(f'{mode}: 비정상 해를 내보내지 않음', bool(w) and all(e[1] == 0.0 for e in w), f'{len(w)}개 모두 0')
        n.close()
    return r


def test_bench():
    x, y, h = track_start()
    car = FakeCar(x, y, h)
    n = Harness(car, 'bench:=true')
    car.enable(True)
    r = check('워커 준비', n.wait_ready())
    t = time.monotonic()
    car.spin(3.0)
    w = car.since(t)
    r &= check('bench: 속도는 항상 0', all(e[1] == 0.0 for e in w), f'{len(w)}개')
    n.close()
    return r


def test_bridge_frozen_pose():
    # 진짜 mocap_bridge 에 가짜 /car/pose. 마커를 가렸을 때처럼 pose 만 멈춘다.
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import ReliabilityPolicy
    exe = EXE.replace('mpcc_node', 'mocap_bridge')
    log = open('/tmp/mlcs_mpcc_test_node.log', 'ab')
    proc = subprocess.Popen([exe, '--ros-args', '--params-file',
                             os.path.join(ROOT, 'src/mlcs_mpc/config/mocap.yaml')],
                            stdout=log, stderr=log, start_new_session=True)
    n = rclpy.create_node('fake_natnet')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = n.create_publisher(PoseStamped, '/car/pose', qos)
    valid = []
    n.create_subscription(Bool, '/mocap/valid', lambda m: valid.append((time.monotonic(), m.data)), 50)
    rng = np.random.default_rng(0)

    def feed(secs, frozen=None):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            m = PoseStamped()
            m.header.stamp = n.get_clock().now().to_msg()
            m.pose.position.x = frozen if frozen is not None else float(rng.normal(0, 2e-5))
            m.pose.orientation.w = 1.0
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.01)
    try:
        feed(5.0)
        r = check('정상 pose: valid', bool(valid) and valid[-1][1], f'{len(valid)}개')
        t = time.monotonic()
        feed(1.0, frozen=0.0)
        first = next((e[0] for e in valid if e[0] >= t and not e[1]), None)
        r &= check('멈춘 pose: 무효', first is not None, f'{(first - t) * 1000:.0f} ms' if first else '')
        r &= check('멈춘 동안 계속 무효', not any(e[1] for e in valid if first and e[0] > first))
        t = time.monotonic()
        feed(1.0)
        back = next((e[0] for e in valid if e[0] >= t and e[1]), None)
        r &= check('복구는 0.3 s 뒤', back is not None and back - t > 0.25,
                   f'{(back - t) * 1000:.0f} ms' if back else '')
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
        n.destroy_node()
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only')
    a = ap.parse_args()
    rclpy.init()
    tests = [test_gates, test_stopped_worker, test_real_hang, test_bad_solution, test_bench,
             test_bridge_frozen_pose]
    bad = []
    for f in tests:
        if a.only and a.only != f.__name__:
            continue
        print(f'■ {f.__name__}', flush=True)
        # 시험이 예외로 끝나도 노드와 워커를 남기지 않는다.
        try:
            if not f():
                bad.append(f.__name__)
        finally:
            for h in list(Harness.live):
                h.close()
    rclpy.shutdown()
    print('모두 통과' if not bad else f'실패: {bad}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
