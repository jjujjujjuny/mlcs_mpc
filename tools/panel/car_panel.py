#!/usr/bin/env python3
"""노트북 제어판 — 젯슨 차의 상태를 보고, 실행·정지·설정을 버튼으로.

  ./car.sh panel        (vesc_msgs 빌드·ROS 환경을 car.sh 가 준비한다)

두 통로를 쓴다 (2026-11-10):
  · 보기   — Zenoh 브리지로 오는 토픽 (/sensors/core /scan /tracked_pose /mpc/state
             /mpc/solve_time /hmpc/status). 젯슨 쪽 비용은 브리지뿐이다.
  · 명령   — SSH 연결 하나(ControlMaster) 위에서 tools/panel/remote.sh. 상태 조회는 5초에
             한 번, 젯슨 코어 0-3·nice 19 에서 약 0.05 s CPU (실측).

★ 노트북 → 젯슨 실시간 제어는 없다. 정지는 젯슨에 직결된 조이스틱(LB·START)이 한다.
  노트북 연결이 끊겨도 주행은 계속된다 (사용자 결정). 여기서 보내는 것은
  실행/정지/설정 명령과 '출발'(/mpc/go, 조이스틱 B 와 같음)뿐이다.
"""
import os
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque

os.environ['ROS_LOCALHOST_ONLY'] = '1'      # 노트북 브리지와 이 노드는 노트북 안에서만 만난다

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    sys.exit('PyQt5 가 없습니다:  sudo apt install -y python3-pyqt5')

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
ZB = os.path.join(ROOT, 'tools', 'zenoh', 'x86_64', 'zenoh-bridge-ros2dds')
HOST_FILE = os.path.join(ROOT, 'tools', 'zenoh', '.car_host')
REMOTE = '~/mlcs_mpc/tools/panel/remote.sh'
USER = 'mlcs'

STATUS_PERIOD_MS = 5000      # 젯슨 상태 조회 주기 (remote.sh status ≈ 0.055 s CPU → 코어 하나의 약 1 %)
SCREEN_PERIOD_MS = 2000      # '화면' 탭을 보고 있을 때만
SOLVE_BUDGET_MS = 33.0       # 제어 주기 (30 Hz)

GREEN, RED, YELLOW, GREY = '#2e7d32', '#c62828', '#f9a825', '#555'
CHIP = 'padding:3px 8px; border-radius:4px; color:white; background:{};'

# 쉬고 있는 LiPo 셀 전압 → 잔량 [%] (bat.sh 와 같은 표). 주행 중엔 처져서 낮게 나온다
LIPO_SOC = [(3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25), (3.77, 30),
            (3.79, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55), (3.87, 60), (3.91, 65),
            (3.95, 70), (3.98, 75), (4.02, 80), (4.08, 85), (4.11, 90), (4.15, 95), (4.20, 100)]
CELLS = 3

# remote.sh logs 의 tail 머리줄 → 탭
LOG_TABS = [('screen', '화면'), ('hmpc', 'hmpc'), ('hmpc_worker', '솔버'),
            ('cartographer', 'Cartographer'), ('lidar', '라이다'), ('bringup', '드라이버'),
            ('bridge', '브리지'), ('relocalize', '위치 다시잡기'), ('panel_job', '맵 저장·트랙'),
            ('local', '제어판')]
TAIL_HEAD = re.compile(r'^==> /tmp/mlcs_(\w+)\.log <==$')
MAX_LOG_LINES = 2000


def battery_percent(volts):
    cell = volts / CELLS
    if cell <= LIPO_SOC[0][0]:
        return 0
    for (v0, p0), (v1, p1) in zip(LIPO_SOC, LIPO_SOC[1:]):
        if cell < v1:
            return int(p0 + (p1 - p0) * (cell - v0) / (v1 - v0))
    return 100


def percentile(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))] if s else 0.0


# ─────────────────────────────────────────────────────────────────────
#  ROS — 브리지로 온 토픽을 받아 둔다 (rclpy 가 없으면 보기만 빠진다)
# ─────────────────────────────────────────────────────────────────────
class RosLink:
    def __init__(self):
        self.ok = False
        self.err = ''
        self.lock = threading.Lock()
        self.volts = None
        self.vesc_t = 0.0
        self.fault = 0
        self.scan_t = deque(maxlen=50)
        self.pose = None
        self.pose_t = deque(maxlen=60)
        self.speed = None
        self.state_t = 0.0
        self.solve = deque(maxlen=150)
        self.solve_t = 0.0
        self.hstatus = ''
        self.hstatus_t = 0.0
        self.go_pub = None
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Odometry
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import LaserScan
            from std_msgs.msg import Bool, Float32, String
        except ImportError as e:
            self.err = f'rclpy 없음 ({e}) — ROS 환경을 source 하세요'
            return
        self.Bool = Bool
        rclpy.init()
        n = self.node = rclpy.create_node('car_panel')
        be = qos_profile_sensor_data           # best effort: 놓친 건 다시 안 보낸다
        n.create_subscription(LaserScan, '/scan', self._scan, be)
        n.create_subscription(PoseStamped, '/tracked_pose', self._pose, be)
        n.create_subscription(Odometry, '/mpc/state', self._state, be)
        n.create_subscription(Float32, '/mpc/solve_time', self._solve, be)
        n.create_subscription(String, '/hmpc/status', self._hstatus, 10)
        try:
            from vesc_msgs.msg import VescStateStamped
            n.create_subscription(VescStateStamped, '/sensors/core', self._core, be)
        except ImportError:
            self.err = 'vesc_msgs 없음 — 배터리 표시 꺼짐 (./car.sh panel 로 실행하면 빌드한다)'
        self.go_pub = n.create_publisher(Bool, '/mpc/go', 10)
        self.ok = True
        threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    # 콜백 — 시각만 적고 바로 나간다
    def _scan(self, _m):
        with self.lock:
            self.scan_t.append(time.monotonic())

    def _pose(self, m):
        with self.lock:
            self.pose = (m.pose.position.x, m.pose.position.y)
            self.pose_t.append(time.monotonic())

    def _state(self, m):
        with self.lock:
            v = m.twist.twist.linear
            self.speed = (v.x ** 2 + v.y ** 2) ** 0.5
            self.state_t = time.monotonic()

    def _solve(self, m):
        with self.lock:
            self.solve.append(float(m.data))
            self.solve_t = time.monotonic()

    def _hstatus(self, m):
        with self.lock:
            self.hstatus = m.data
            self.hstatus_t = time.monotonic()

    def _core(self, m):
        with self.lock:
            self.volts = m.state.voltage_input
            self.fault = m.state.fault_code
            self.vesc_t = time.monotonic()

    def go(self):
        if self.go_pub is not None:
            self.go_pub.publish(self.Bool(data=True))

    @staticmethod
    def rate(ts, now, window=2.0):
        n = sum(1 for t in ts if now - t < window)
        return n / window

    def shutdown(self):
        if self.ok:
            import rclpy
            try:
                self.node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────
#  SSH — 연결 하나를 열어 두고(ControlMaster) 그 위에서 명령을 보낸다
# ─────────────────────────────────────────────────────────────────────
class Ssh(QtCore.QObject):
    def __init__(self, host, parent):
        super().__init__(parent)
        self.host = host
        cp = os.path.expanduser('~/.ssh/cm-carpanel-%r@%h:%p')
        self.base = ['-o', 'ControlMaster=auto', '-o', f'ControlPath={cp}',
                     '-o', 'ControlPersist=120', '-o', 'ServerAliveInterval=5',
                     '-o', 'ServerAliveCountMax=2', '-o', 'ConnectTimeout=4',
                     '-o', 'BatchMode=yes', f'{USER}@{host}']
        self.busy = set()

    def run(self, args, done=None, tag=None):
        """remote.sh <args...> 를 비동기로. done(ok, text). tag 가 같은 호출은 겹치지 않게."""
        if tag and tag in self.busy:
            return
        p = QtCore.QProcess(self)
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        if tag:
            self.busy.add(tag)

        def fin(code, _st):
            out = bytes(p.readAll()).decode('utf-8', 'replace')
            if tag:
                self.busy.discard(tag)
            if done:
                done(code == 0, out)
            p.deleteLater()
        p.finished.connect(fin)
        p.start('ssh', self.base + [REMOTE] + list(args))
        return p

    def stream(self, args):
        p = QtCore.QProcess(self)
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        p.start('ssh', self.base + [REMOTE] + list(args))
        return p

    def close(self):
        subprocess.run(['ssh'] + self.base[:4] + ['-O', 'exit', f'{USER}@{self.host}'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)


def nofocus(w):
    w.setFocusPolicy(QtCore.Qt.NoFocus)
    return w


# ─────────────────────────────────────────────────────────────────────
#  제어판
# ─────────────────────────────────────────────────────────────────────
class Panel(QtWidgets.QWidget):
    def __init__(self, host):
        super().__init__()
        self.host = host
        self.setWindowTitle(f'젯슨 제어판 — {host}')
        self.st = {}                 # 마지막 remote.sh status
        self.st_t = 0.0              # 그때 시각 (SSH 살아 있음 판단)
        self.ssh_fail = 0
        self.maps, self.tracks = [], []
        self.yaml_loaded = False
        self.ssh = Ssh(host, self)
        self.ros = RosLink()
        self.bridge = None
        self.rviz = None
        self.log_proc = None
        self.tail_tab = 'local'

        root = QtWidgets.QVBoxLayout(self)
        root.addLayout(self._chip_row())
        mid = QtWidgets.QHBoxLayout()
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._run_box())
        left.addWidget(self._map_box())
        left.addStretch(1)
        mid.addLayout(left, 3)
        mid.addWidget(self._cfg_box(), 2)
        root.addLayout(mid)
        root.addWidget(self._log_tabs(), 1)

        self.start_bridge()
        if not self.ros.ok or self.ros.err:
            self.note(self.ros.err)
        self.reload_lists()
        self.poll_status()
        self.start_logs()
        QtCore.QTimer(self, interval=STATUS_PERIOD_MS, timeout=self.poll_status).start()
        QtCore.QTimer(self, interval=500, timeout=self.refresh_chips).start()
        QtCore.QTimer(self, interval=SCREEN_PERIOD_MS, timeout=self.poll_screen).start()
        self.resize(1100, 820)

    # ───────── 위젯 ─────────
    def _chip_row(self):
        row = QtWidgets.QHBoxLayout()
        self.chips = {}
        for k, tip in [('ssh', 'SSH 명령 통로'), ('link', '브리지로 오는 토픽'),
                       ('bat', '배터리 (VESC 전압, 3S). 주행 중엔 처져서 낮게 나온다'),
                       ('scan', '라이다 /scan 수신 (브리지가 10 Hz 로 솎아 보낸다)'),
                       ('pose', '위치 /tracked_pose (Cartographer)'),
                       ('speed', '속도 /mpc/state'), ('solve', '솔버 시간 (최근 150 풀이)'),
                       ('hmpc', 'hmpc 상태 (/hmpc/status — 바뀔 때만 온다)')]:
            lb = QtWidgets.QLabel('…')
            lb.setToolTip(tip)
            lb.setStyleSheet(CHIP.format(GREY))
            row.addWidget(lb)
            self.chips[k] = lb
        row.addStretch(1)
        return row

    def _btn(self, text, slot, color=None, tip=''):
        b = nofocus(QtWidgets.QPushButton(text))
        b.clicked.connect(slot)
        if color:
            b.setStyleSheet(f'background:{color}; color:white; font-weight:bold; padding:6px;')
        if tip:
            b.setToolTip(tip)
        return b

    def _run_box(self):
        box = QtWidgets.QGroupBox('실행 (젯슨)')
        g = QtWidgets.QGridLayout(box)
        self.state_lbl = QtWidgets.QLabel('젯슨 상태 확인 중…')
        f = self.state_lbl.font()
        f.setPointSize(f.pointSize() + 3)
        f.setBold(True)
        self.state_lbl.setFont(f)
        self.proc_lbl = QtWidgets.QLabel('')
        self.proc_lbl.setTextFormat(QtCore.Qt.RichText)
        g.addWidget(self.state_lbl, 0, 0, 1, 3)
        g.addWidget(self.proc_lbl, 1, 0, 1, 3)
        self.b_bench = self._btn('bench 시작', lambda: self.start('bench'),
                                 tip='race.sh 를 bench 로 (속도 0, 조향만). race.yaml 의 drive 는 안 바꾼다')
        self.b_race = self._btn('주행 시작', lambda: self.start('race'), RED,
                                tip='race.sh 실제 주행. 출발은 조이스틱 B (또는 아래 출발)')
        self.b_mapping = self._btn('맵핑 시작', lambda: self.start('mapping'),
                                   tip='drive.sh mapping — 조이스틱으로 천천히 1~2 바퀴')
        self.b_int = self._btn('정지 (Ctrl+C)', self.interrupt, YELLOW,
                               tip='젯슨 화면에 Ctrl+C — drive.sh 가 차를 세우고 정리하며 내린다')
        self.b_kill = self._btn('강제 종료', self.force_stop,
                                tip='drive.sh stop — Ctrl+C 로 안 내려갈 때')
        self.b_go = self._btn('출발 (B)', self.go, GREEN,
                              tip='/mpc/go — 조이스틱 B 와 같다. 정지는 조이스틱 LB/START')
        self.b_rviz = self._btn('RViz 열기', self.open_rviz, tip='노트북 RViz (2D Pose Estimate 는 여기서)')
        self.b_term = self._btn('젯슨 터미널', self.open_term, tip='젯슨 tmux 화면에 붙기 (Ctrl+b d 로 빠지기)')
        for i, b in enumerate([self.b_bench, self.b_race, self.b_mapping,
                               self.b_int, self.b_kill, self.b_go,
                               self.b_rviz, self.b_term]):
            g.addWidget(b, 2 + i // 3, i % 3)
        return box

    def _map_box(self):
        box = QtWidgets.QGroupBox('맵 만들기')
        g = QtWidgets.QGridLayout(box)
        self.map_name = QtWidgets.QLineEdit()
        self.map_name.setPlaceholderText('새 맵 이름 (영문·숫자·_-), 예: 1110_1')
        self.b_savemap = self._btn('맵 저장', self.save_map, tip='맵핑 중에만. 저장 후 맵핑은 Ctrl+C 로 끈다')
        self.b_maptrack = self._btn('트랙 만들기', self.make_track,
                                    tip='drive.sh maptrack <맵> — 맵에서 트랙(<맵>_track)을 만든다')
        g.addWidget(self.map_name, 0, 0, 1, 2)
        g.addWidget(self.b_savemap, 1, 0)
        g.addWidget(self.b_maptrack, 1, 1)
        hint = QtWidgets.QLabel('순서: 맵핑 시작 → 조이스틱으로 1~2 바퀴 → 맵 저장 → 정지 → 트랙 만들기 → 설정에서 고르기')
        hint.setWordWrap(True)
        hint.setStyleSheet('color:#888;')
        g.addWidget(hint, 2, 0, 1, 2)
        return box

    def _cfg_box(self):
        box = QtWidgets.QGroupBox('설정 (젯슨 race.yaml)')
        g = QtWidgets.QGridLayout(box)
        self.cb_map = nofocus(QtWidgets.QComboBox())
        self.cb_track = nofocus(QtWidgets.QComboBox())
        self.cb_map.currentTextChanged.connect(self.map_changed)
        self.sp_vmax = QtWidgets.QDoubleSpinBox()
        self.sp_vmax.setRange(0.3, 4.8)
        self.sp_vmax.setSingleStep(0.1)
        self.sp_vmax.setDecimals(1)
        self.cb_obs = nofocus(QtWidgets.QComboBox())
        self.cb_obs.addItems(['off', 'watch', 'lidar'])
        self.le_log = QtWidgets.QLineEdit()
        self.le_log.setPlaceholderText('비우면 기록 안 함')
        rows = [('맵', self.cb_map), ('트랙', self.cb_track), ('최고 속도 [m/s]', self.sp_vmax),
                ('장애물', self.cb_obs), ('기록 태그', self.le_log)]
        for r, (t, w) in enumerate(rows):
            g.addWidget(QtWidgets.QLabel(t), r, 0)
            g.addWidget(w, r, 1, 1, 2)
        self.cfg_warn = QtWidgets.QLabel('')
        self.cfg_warn.setWordWrap(True)
        self.cfg_warn.setStyleSheet('color:#e65100; font-weight:bold;')
        g.addWidget(self.cfg_warn, len(rows), 0, 1, 3)
        g.addWidget(self._btn('적용', self.apply_cfg, GREEN,
                              tip='race.yaml 에 쓴다. 실행 중이면 다음 시작부터 적용'), len(rows) + 1, 0, 1, 2)
        g.addWidget(self._btn('되돌리기', self.reload_cfg, tip='젯슨의 지금 값으로 다시 읽기'),
                    len(rows) + 1, 2)
        g.addWidget(self._btn('목록 새로고침', self.reload_lists), len(rows) + 2, 0, 1, 3)
        g.setRowStretch(len(rows) + 3, 1)
        return box

    def _log_tabs(self):
        self.tabs = nofocus(QtWidgets.QTabWidget())
        self.logs = {}
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        for key, label in LOG_TABS:
            t = QtWidgets.QPlainTextEdit(readOnly=True)
            t.setFont(font)
            t.setMaximumBlockCount(MAX_LOG_LINES)
            t.setFocusPolicy(QtCore.Qt.NoFocus)
            self.logs[key] = t
            self.tabs.addTab(t, label)
        self.tabs.currentChanged.connect(lambda _i: self.poll_screen())
        return self.tabs

    # ───────── 로그 ─────────
    def note(self, text):
        if text:
            self.logs['local'].appendPlainText(time.strftime('%H:%M:%S ') + text.rstrip())

    def start_logs(self):
        """remote.sh logs (tail -F) 하나를 계속 받아 탭으로 나눈다. 끊기면 5초 뒤 다시."""
        if self.log_proc is not None:
            return
        p = self.log_proc = self.ssh.stream(['logs'])
        buf = ['']

        def read():
            buf[0] += bytes(p.readAll()).decode('utf-8', 'replace')
            *lines, buf[0] = buf[0].split('\n')
            for line in lines:
                m = TAIL_HEAD.match(line)
                if m:
                    self.tail_tab = m.group(1) if m.group(1) in self.logs else 'local'
                    continue
                if line:
                    self.logs[self.tail_tab].appendPlainText(line)

        def fin(*_):
            self.log_proc = None
            QtCore.QTimer.singleShot(5000, self.start_logs)
        p.readyRead.connect(read)
        p.finished.connect(fin)

    def poll_screen(self):
        if self.tabs.currentWidget() is not self.logs['screen']:
            return

        def done(ok, out):
            if ok:
                t = self.logs['screen']
                bar = t.verticalScrollBar()
                at_end = bar.value() >= bar.maximum() - 2
                t.setPlainText(out.rstrip())
                if at_end:
                    bar.setValue(bar.maximum())
        self.ssh.run(['screen'], done, tag='screen')

    # ───────── 젯슨 상태 ─────────
    def poll_status(self):
        def done(ok, out):
            kv = dict(l.split('=', 1) for l in out.splitlines() if '=' in l)
            if ok and 'session' in kv:
                self.st, self.st_t, self.ssh_fail = kv, time.monotonic(), 0
                if not self.yaml_loaded:
                    self.fill_cfg()
            else:
                self.ssh_fail += 1
                if self.ssh_fail in (1, 10):
                    self.note('SSH 상태 조회 실패: ' + (out.strip().splitlines() or ['응답 없음'])[-1]
                              + '  (./car.sh key 를 했는지, 젯슨 주소가 맞는지)')
            self.refresh_run()
        self.ssh.run(['status'], done, tag='status')

    def refresh_run(self):
        s = self.st
        alive = time.monotonic() - self.st_t < 3 * STATUS_PERIOD_MS / 1000
        mode = s.get('mode', '')
        ended = mode == 'ended'                 # 끝나고 화면만 남음 → 새로 시작할 수 있다
        session = alive and s.get('session') == '1' and not ended
        mapping = alive and s.get('carto') == '1' and s.get('carto_mode', '').startswith('mapping')
        names = {'bench': 'bench 실행 중 (속도 0)', 'race': '★ 주행 모드 실행 중',
                 'mapping': '맵핑 중', 'term': '터미널 세션 열림'}
        if not alive:
            self.state_lbl.setText('젯슨 연결 안 됨 (SSH)')
        elif session:
            self.state_lbl.setText('젯슨: ' + names.get(mode, mode or '실행 중'))
        elif ended:
            self.state_lbl.setText('젯슨: 끝남 (화면 탭에서 마지막 출력 확인)')
        else:
            self.state_lbl.setText('젯슨: 대기 (실행 중인 것 없음)')
        if alive:
            dots = []
            for k, t in [('bringup', '드라이버'), ('lidar', '라이다'), ('carto', 'Cartographer'),
                         ('hmpc', 'hmpc'), ('bridge', '브리지'), ('job', '맵 저장·트랙 작업')]:
                c = GREEN if s.get(k) == '1' else '#999'
                dots.append(f'<span style="color:{c}">●</span> {t}')
            dots.append(f'&nbsp;&nbsp;부하 {s.get("load", "?")}')
            self.proc_lbl.setText('&nbsp;&nbsp;'.join(dots))
        for b in (self.b_bench, self.b_race, self.b_mapping):
            b.setEnabled(alive and not session)
        self.b_int.setEnabled(session)
        self.b_kill.setEnabled(alive)
        self.b_go.setEnabled(alive and s.get('hmpc') == '1' and self.ros.ok)
        self.b_term.setEnabled(alive)
        self.b_savemap.setEnabled(mapping and s.get('job') != '1')
        self.b_maptrack.setEnabled(alive and not mapping and s.get('job') != '1')

    # ───────── 상태 칩 (토픽) ─────────
    def chip(self, k, text, color):
        self.chips[k].setText(text)
        self.chips[k].setStyleSheet(CHIP.format(color))

    def refresh_chips(self):
        now = time.monotonic()
        ssh_ok = now - self.st_t < 3 * STATUS_PERIOD_MS / 1000
        self.chip('ssh', 'SSH ✓' if ssh_ok else 'SSH ✗', GREEN if ssh_ok else RED)
        r = self.ros
        if not r.ok:
            self.chip('link', '토픽 ✗ (rclpy)', RED)
            return
        with r.lock:
            scan_hz = r.rate(r.scan_t, now)
            pose_hz = r.rate(r.pose_t, now)
            pose_age = now - r.pose_t[-1] if r.pose_t else None
            pose, volts, vesc_age = r.pose, r.volts, now - r.vesc_t
            speed, state_age = r.speed, now - r.state_t
            solve = list(r.solve)
            solve_age = now - r.solve_t
            hst, hst_age = r.hstatus, now - r.hstatus_t
        newest = min(vesc_age, state_age, solve_age,
                     pose_age if pose_age is not None else 1e9,
                     now - r.scan_t[-1] if r.scan_t else 1e9)
        bridge_up = self.bridge is not None and self.bridge.poll() is None
        if not bridge_up:
            self.chip('link', '브리지 ✗', RED)
        elif newest < 2.0:
            self.chip('link', '토픽 ✓', GREEN)
        else:
            self.chip('link', '토픽 안 옴', YELLOW)

        if volts is not None and vesc_age < 3.0:
            p = battery_percent(volts)
            c = GREEN if p >= 40 else YELLOW if p >= 20 else RED
            self.chip('bat', f'배터리 {p}%  {volts:.1f} V', c)
        else:
            self.chip('bat', '배터리 —', GREY)
        if scan_hz > 0:
            self.chip('scan', f'라이다 {scan_hz:.0f} Hz', GREEN if scan_hz >= 7 else YELLOW)
        else:
            self.chip('scan', '라이다 —', GREY)
        if pose is not None and pose_age is not None and pose_age < 1.0:
            self.chip('pose', f'위치 ({pose[0]:+.2f}, {pose[1]:+.2f})  {pose_hz:.0f} Hz', GREEN)
        elif pose is not None:
            self.chip('pose', f'위치 끊김 {pose_age:.0f} s', RED)
        else:
            self.chip('pose', '위치 —', GREY)
        if speed is not None and state_age < 1.0:
            self.chip('speed', f'{speed:.2f} m/s', '#1565c0')
        else:
            self.chip('speed', '속도 —', GREY)
        if solve and solve_age < 2.0:
            p95, mx = percentile(solve, 0.95), max(solve)
            c = GREEN if p95 < SOLVE_BUDGET_MS else YELLOW if p95 < 1.3 * SOLVE_BUDGET_MS else RED
            self.chip('solve', f'솔버 p95 {p95:.0f} / 최대 {mx:.0f} ms', c)
        else:
            self.chip('solve', '솔버 —', GREY)
        if hst:
            c = GREEN if hst == '주행' else YELLOW
            self.chip('hmpc', f'hmpc: {hst}  ({hst_age:.0f} s 전)', c)
        else:
            self.chip('hmpc', 'hmpc —', GREY)

    # ───────── 설정 ─────────
    def reload_lists(self):
        def done(ok, out):
            if not ok:
                return
            kv = dict(l.split('=', 1) for l in out.splitlines() if '=' in l)
            self.maps = [x for x in kv.get('maps', '').split(',') if x]
            self.tracks = [x for x in kv.get('tracks', '').split(',') if x]
            self.fill_cfg(keep_edits=True)
        self.ssh.run(['lists'], done, tag='lists')

    def reload_cfg(self):
        self.yaml_loaded = False
        self.poll_status()

    def fill_cfg(self, keep_edits=False):
        s = self.st
        if not s:
            return
        cur_map = self.cb_map.currentText() if keep_edits and self.yaml_loaded else s.get('map', '')
        cur_track = self.cb_track.currentText() if keep_edits and self.yaml_loaded else s.get('track', '')
        for cb, items, cur in [(self.cb_map, self.maps, cur_map), (self.cb_track, self.tracks, cur_track)]:
            cb.blockSignals(True)
            cb.clear()
            cb.addItems(items if cur in items or not cur else [cur] + items)
            cb.setCurrentText(cur)
            cb.blockSignals(False)
        if not (keep_edits and self.yaml_loaded):
            try:
                self.sp_vmax.setValue(float(s.get('v_max', '1.0')))
            except ValueError:
                pass
            self.cb_obs.setCurrentText(s.get('obstacles', 'off'))
            lg = s.get('log', '')
            self.le_log.setText('' if lg in ("''", '""') else lg)
            self.yaml_loaded = True
        self.check_cfg()

    def map_changed(self, m):
        # 같은 맵에서 만든 트랙(<맵>_track)이 있으면 자동으로 고른다
        if f'{m}_track' in self.tracks:
            self.cb_track.setCurrentText(f'{m}_track')
        self.check_cfg()

    def check_cfg(self):
        m, t = self.cb_map.currentText(), self.cb_track.currentText()
        w = ''
        if t and m and not t.startswith(m):
            w = f'트랙 {t} 은(는) 맵 {m} 에서 만든 것이 아닐 수 있어요 (이름이 {m}_ 로 시작하지 않음).'
        if t.endswith('_mocap'):
            w = f'{t} 는 mocap 좌표 트랙이에요. 라이다 주행이면 맞지 않아요.'
        self.cfg_warn.setText(w)

    def apply_cfg(self):
        want = {'map': self.cb_map.currentText(), 'track': self.cb_track.currentText(),
                'v_max': f'{self.sp_vmax.value():.1f}', 'obstacles': self.cb_obs.currentText(),
                'log': self.le_log.text().strip()}
        changes = {k: v for k, v in want.items() if (self.st.get(k, '') or '') != v
                   and not (k == 'log' and v == '' and self.st.get('log') in ("''", '""'))}
        if not changes:
            self.note('설정: 바뀐 값이 없습니다')
            return
        for k, v in changes.items():
            if k in ('map', 'track', 'log') and not re.fullmatch(r'[A-Za-z0-9_.-]*', v):
                return self.warn(f'{k} 값에 쓸 수 없는 글자가 있어요: {v}')
        text = '\n'.join(f'  {k}: {self.st.get(k, "")} → {v}' for k, v in changes.items())
        if self.st.get('session') == '1':
            text += '\n\n지금 실행 중인 것에는 적용되지 않아요. 다음 시작부터 적용돼요.'
        if not self.ask(f'race.yaml 을 이렇게 바꿀까요?\n\n{text}'):
            return
        for k, v in changes.items():
            self.ssh.run(['set', k, v or "''"], lambda ok, out: self.note(out.strip()))
        QtCore.QTimer.singleShot(1500, self.reload_cfg)

    # ───────── 명령 ─────────
    def warn(self, text):
        QtWidgets.QMessageBox.warning(self, '젯슨 제어판', text)

    def ask(self, text):
        return QtWidgets.QMessageBox.question(self, '젯슨 제어판', text) == QtWidgets.QMessageBox.Yes

    def remote(self, args, then_poll=True):
        def done(ok, out):
            self.note(out.strip() or ('완료' if ok else '실패'))
            if then_poll:
                QtCore.QTimer.singleShot(800, self.poll_status)
        self.note('→ ' + ' '.join(args))
        self.ssh.run(args, done)

    def start(self, what):
        if what == 'race' and not self.ask(
                '★ 실제로 달립니다.\n\n트랙이 비었고, 조이스틱(LB·START)이 손에 있나요?\n'
                '출발은 조이스틱 B 또는 이 창의 "출발".'):
            return
        if what == 'mapping' and self.st.get('session') == '1':
            return self.warn('이미 실행 중인 것이 있어요. 먼저 정지하세요.')
        self.remote(['start', what])
        self.tabs.setCurrentWidget(self.logs['screen'])

    def interrupt(self):
        self.remote(['int'])

    def force_stop(self):
        if self.ask('drive.sh stop — 젯슨의 주행·맵핑을 전부 내려요.\n(맵핑 중 저장 안 한 맵은 사라져요)'):
            self.remote(['stop'])

    def go(self):
        if not self.ask('출발할까요? (조이스틱 B 와 같아요)\n정지는 조이스틱 LB 또는 START.'):
            return
        self.ros.go()
        self.note('출발 (/mpc/go) 보냄')

    def save_map(self):
        n = self.map_name.text().strip()
        if not re.fullmatch(r'[A-Za-z0-9_-]+', n):
            return self.warn('맵 이름은 영문·숫자·_- 만 (예: 1110_1)')
        if n in self.maps:
            return self.warn(f'{n} 맵이 이미 있어요. 다른 이름을 쓰세요.')
        self.remote(['savemap', n])
        self.tabs.setCurrentWidget(self.logs['panel_job'])
        QtCore.QTimer.singleShot(15000, self.reload_lists)

    def make_track(self):
        n = self.map_name.text().strip() or self.cb_map.currentText()
        if n not in self.maps:
            return self.warn(f'맵 {n or "(없음)"} 이 젯슨에 없어요. 이름을 확인하거나 목록을 새로고침하세요.')
        if f'{n}_track' in self.tracks and not self.ask(f'{n}_track 이 이미 있어요. 다시 만들까요?'):
            return
        self.remote(['maptrack', n])
        self.tabs.setCurrentWidget(self.logs['panel_job'])
        QtCore.QTimer.singleShot(30000, self.reload_lists)

    # ───────── 노트북 쪽 프로그램 ─────────
    def start_bridge(self):
        if subprocess.run(['pgrep', '-f', ZB], stdout=subprocess.DEVNULL).returncode == 0:
            self.note('노트북 브리지가 이미 떠 있습니다 — 그대로 씁니다')
            self.bridge = _Running()
            return
        if not os.access(ZB, os.X_OK):
            self.note(f'노트북 브리지 실행 파일이 없습니다: {ZB} (tools/zenoh/README.md)')
            return
        log = open(os.path.join(os.environ.get('TMPDIR', '/tmp'), 'mlcs_panel_bridge.log'), 'w')
        self.bridge = subprocess.Popen(
            [ZB, '-c', os.path.join(ROOT, 'tools', 'zenoh', 'laptop.json5'),
             '-e', f'tcp/{self.host}:7447'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.note(f'노트북 브리지 → tcp/{self.host}:7447')

    def open_rviz(self):
        if self.rviz is not None and self.rviz.poll() is None:
            return self.note('RViz 가 이미 떠 있습니다')
        self.rviz = subprocess.Popen(['rviz2', '-d', os.path.join(ROOT, 'tools', 'zenoh', 'laptop.rviz')],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     start_new_session=True)

    def open_term(self):
        cmd = f'ssh -t {USER}@{self.host} "tmux new -A -s race"'
        for term in (['gnome-terminal', '--', 'bash', '-c', cmd],
                     ['x-terminal-emulator', '-e', 'bash', '-c', cmd]):
            try:
                subprocess.Popen(term, start_new_session=True)
                return
            except FileNotFoundError:
                continue
        self.warn(f'터미널을 못 열었어요. 직접:\n{cmd}')

    def closeEvent(self, ev):
        # 젯슨은 건드리지 않는다 — 노트북 창을 닫아도 주행은 계속된다
        if self.log_proc is not None:
            self.log_proc.kill()
        for p in (self.rviz, self.bridge):
            if isinstance(p, subprocess.Popen) and p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
        self.ssh.close()
        self.ros.shutdown()
        ev.accept()


class _Running:
    """이미 떠 있던 브리지 (우리가 띄운 게 아니라 닫을 때 안 죽인다)."""
    def poll(self):
        return None


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else (
        open(HOST_FILE).read().strip() if os.path.exists(HOST_FILE) else '192.168.1.7')
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    QtCore.QTimer(app, interval=300, timeout=lambda: None).start()   # Ctrl+C 가 먹게
    w = Panel(host)
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
