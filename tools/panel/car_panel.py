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
# [ERROR] [1794235267.64] [relocalize]: RViz — 맞춤 3.1 s, 점수 0.01 → 0.24, ...
RELOC_LINE = re.compile(r'^\[\w+\] \[(\d+\.\d+)\] \[relocalize\]: (.*)$')
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
#  명령어 탭 — 스크립트 맨 위 사용법 주석을 읽어 목적별로 묶는다
#
#  ★ 명령 문구는 여기 옮겨 적지 않는다 (명령이 바뀌면 주석만 고치면 되게). 여기 있는 것은
#    '명령 이름 → 묶음' 표뿐이고, 표에 없는 새 명령은 '기타' 로 그대로 보인다.
# ─────────────────────────────────────────────────────────────────────
WHERE = {'car.sh': '노트북', 'drive.sh': '젯슨', 'race.sh': '젯슨', 'bat.sh': '젯슨'}
GROUP_ORDER = ['노트북 준비·연결', '주행', '맵 → 트랙', '점검·확인', '정지',
               '보정·데이터', '기타', '환경변수']
GROUP_OF = {
    'car.sh': {'ip': '노트북 준비·연결', 'key': '노트북 준비·연결', 'panel': '노트북 준비·연결',
               'rviz': '노트북 준비·연결', 'term': '노트북 준비·연결',
               'bench': '주행', 'race': '주행', 'check': '점검·확인', 'bat': '점검·확인',
               'stop': '정지'},
    'drive.sh': dict(
        **{k: '주행' for k in ('joy', 'rally', 'bench', 'mpc', 'stanley', 'hmpc', 'go', 'halt', 'log')},
        **{k: '맵 → 트랙' for k in ('mapping', 'savemap', 'maptrack', 'localize', 'drawtrack',
                                    'stracks', 'preptrack', 'preview', 'trackinfo', 'record',
                                    'laptrack', 'align', 'mocaptrack', 'racereport')},
        **{k: '점검·확인' for k in ('topics', 'checkstate', 'lidar', 'rviz', 'bridge', 'mocap',
                                    'natnet')},
        **{k: '보정·데이터' for k in ('servo', 'cal', 'mocapcal', 'cleanlog', 'hpmdata',
                                      'actuator')},
        stop='정지'),
    'race.sh': {'': '주행'},
    'bat.sh': {'': '점검·확인'},
}
_CMD = re.compile(r'^(\s*)((?:[A-Z_]+=\S+\s+)*\./(\w+\.sh)\b.*)$')


def parse_usage(script, text):
    """사용법 주석 → (묶음, 위치, 명령, 설명) 목록과 따로 붙은 안내문 목록."""
    entries, notes = [], []
    env = False
    cur = None
    for raw in text.splitlines():
        if not raw.startswith('#') or set(raw.strip('# ')) <= set('=─'):
            continue                                      # 구분선
        line = raw[1:].rstrip()
        body = line.strip()
        if not body:                                      # 빈 줄 = 구역 끝
            cur, env = None, False
            continue
        if body == '사용법:':
            cur = None
            continue
        if body.startswith('──'):                         # ── 라이다 맵 → 트랙 ── 같은 구분 줄
            cur = None
            continue
        if body == '환경변수:':
            env, cur = True, None
            continue
        m = _CMD.match(line)
        if m and m.group(3) == script:
            indent, rest = len(m.group(1)), m.group(2)
            parts = re.split(r'\s{2,}', rest, maxsplit=1)
            cmd, desc = parts[0].strip(), (parts[1].strip() if len(parts) > 1 else '')
            sub = re.match(r'(?:[A-Z_]+=\S+\s+)*\./\w+\.sh\s*(\S*)', cmd).group(1)
            table = GROUP_OF.get(script, {})
            group = table.get(sub.lstrip('-'), table.get('', '기타')) if not sub.startswith('<') \
                else table.get('', '기타')
            cur = [group, WHERE.get(script, '?'), cmd, desc, indent]
            entries.append(cur)
            continue
        if env and re.match(r'[A-Z_]+=', body):
            parts = re.split(r'\s{2,}', body, maxsplit=1)
            entries.append(['환경변수', WHERE.get(script, '?'), parts[0],
                            parts[1] if len(parts) > 1 else '', 0])
            continue
        indent = len(line) - len(line.lstrip())
        if cur is not None and indent > cur[4]:           # 위 명령의 옵션·이어지는 설명
            cur[3] = (cur[3] + '\n' + body).strip()
            continue
        cur = None
        notes.append(body)
    return [e[:4] for e in entries], notes


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
                     '-o', 'HostKeyAlias=mlcs-car',   # ★ IP 가 바뀌어도 같은 젯슨 (car.sh 와 같이)
                     '-o', 'ControlPersist=120', '-o', 'ServerAliveInterval=5',
                     '-o', 'ServerAliveCountMax=2', '-o', 'ConnectTimeout=4',
                     '-o', 'BatchMode=yes', f'{USER}@{host}']
        self.busy = set()
        self.closed = False

    def run(self, args, done=None, tag=None):
        """remote.sh <args...> 를 비동기로. done(ok, text). tag 가 같은 호출은 겹치지 않게."""
        if self.closed or (tag and tag in self.busy):
            return None
        p = QtCore.QProcess(self)
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        if tag:
            self.busy.add(tag)

        def fin(code, _st):
            try:
                out = bytes(p.readAll()).decode('utf-8', 'replace')
            except RuntimeError:            # 창을 닫는 중에 QProcess 가 먼저 지워졌다
                return
            if self.closed:
                return
            if tag:
                self.busy.discard(tag)
            if done:
                done(code == 0, out)
            p.deleteLater()
        p.finished.connect(fin)
        p.start('ssh', self.base + [REMOTE] + list(args))
        return p

    def stream(self, args):
        if self.closed:
            return None
        p = QtCore.QProcess(self)
        p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        p.start('ssh', self.base + [REMOTE] + list(args))
        return p

    def close(self):
        self.closed = True      # ★ 이 뒤로는 새 ssh 를 띄우지 않는다 (아래 설명)
        # 돌고 있는 ssh 를 먼저 끝낸다 — 안 그러면 창을 닫을 때
        # 'QProcess: Destroyed while process ("ssh") is still running.' (노트북 실측)
        for p in self.findChildren(QtCore.QProcess):
            try:
                p.finished.disconnect()         # 닫는 중에 콜백(로그 재접속 등)이 돌지 않게
            except TypeError:
                pass
            p.kill()
            p.waitForFinished(500)
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
        # 위치 다시잡기 결과 — 거부돼도 RViz 에는 아무 변화가 없어서 여기 띄운다 (노트북 시험 11-10)
        self.reloc = QtWidgets.QLabel('')
        self.reloc.setWordWrap(True)
        self.reloc.setVisible(False)
        root.addWidget(self.reloc)
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
        self.timers = [QtCore.QTimer(self, interval=STATUS_PERIOD_MS, timeout=self.poll_status),
                       QtCore.QTimer(self, interval=500, timeout=self.refresh_chips),
                       QtCore.QTimer(self, interval=SCREEN_PERIOD_MS, timeout=self.poll_screen)]
        for t in self.timers:
            t.start()
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
        self.b_quiet = self._btn('젯슨 VS Code·크롬 끄기', self.quiet,
                                 tip='./drive.sh quiet — 젯슨의 VS Code·크롬·RViz 를 닫아 CPU 를 주행에 몰아준다\n'
                                     '(주행 시작·맵핑 시작은 자동으로 한다. ROS 노드는 건드리지 않음)')
        for i, b in enumerate([self.b_bench, self.b_race, self.b_mapping,
                               self.b_int, self.b_kill, self.b_go,
                               self.b_rviz, self.b_term, self.b_quiet]):
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
        self.cb_obs.addItems(['off', 'watch', 'lidar', 'learn'])
        self.cb_obs.currentTextChanged.connect(lambda _t: self.check_cfg())
        self.cb_obs.setToolTip('off = 예선 단독 / lidar = 상대 차를 임시 벽으로 피함 (결승) / watch = 찾기만\n'
                               'learn = 빈 트랙에서 2~3 바퀴: 찾기만 하고, 끝나면 맵 헛것 목록 자동 갱신')
        self.cb_odom = nofocus(QtWidgets.QComboBox())
        self.cb_odom.addItems(['off', 'ekf', 'strict'])
        self.cb_odom.setToolTip('ekf = 바퀴 속도 + 자이로로 Cartographer 예측 (직선 미끄러짐 대책), strict = ekf + 오도메트리 강하게 믿고 먼 곳 맞춤 끔 (위치 튐 대책)')
        self.cb_pin = nofocus(QtWidgets.QComboBox())
        self.cb_pin.addItems(['true', 'false'])
        self.cb_pin.setToolTip('true = 매 스캔을 저장한 맵에 맞춰 위치 보정 (랩이 쌓여도 맵이 안 변함)')
        self.sp_fric = QtWidgets.QDoubleSpinBox()
        self.sp_fric.setRange(0.20, 0.60)
        self.sp_fric.setSingleStep(0.01)
        self.sp_fric.setDecimals(2)
        self.sp_fric.setToolTip('MPC 가 믿는 접지력 (재빌드 없음). 낮추면 코너 앞에서 일찍 감속한다\n'
                                '0.36 = 11-11 로비 급커브 교착 대책, 0.41 = 연구실 실측')
        self.sp_steer = QtWidgets.QDoubleSpinBox()
        self.sp_steer.setRange(0.50, 1.00)
        self.sp_steer.setSingleStep(0.05)
        self.sp_steer.setDecimals(2)
        self.sp_steer.setToolTip('조향 여유 (재빌드 없음). 1.0 = 끔, 0.9 = 3 단계 뒤 계획부터 조향 상한 90 %\n'
                                 '급코너에 조향 최대로 들어가 교착하면 낮춘다 (11-14 대회장)')
        self.le_log = QtWidgets.QLineEdit()
        self.le_log.setPlaceholderText('비우면 기록 안 함')
        rows = [('맵', self.cb_map), ('트랙', self.cb_track), ('최고 속도 [m/s]', self.sp_vmax),
                ('장애물', self.cb_obs), ('오도메트리', self.cb_odom), ('맵 고정', self.cb_pin),
                ('마찰 계수', self.sp_fric), ('조향 여유', self.sp_steer),
                ('기록 태그', self.le_log)]
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
        # 명령어 탭 — 탭을 열 때만 읽는다 (젯슨 쪽 1회 0.02 s CPU, 주기 조회 없음)
        self.cmd_page = QtWidgets.QScrollArea()
        self.cmd_page.setWidgetResizable(True)
        self.tabs.addTab(self.cmd_page, '명령어')
        self.tabs.currentChanged.connect(self._tab_changed)
        return self.tabs

    def _tab_changed(self, _i):
        if self.tabs.currentWidget() is self.cmd_page:
            self.load_usage()
        else:
            self.poll_screen()

    # ───────── 명령어 탭 ─────────
    def load_usage(self):
        try:
            with open(os.path.join(ROOT, 'car.sh'), encoding='utf-8') as f:
                lines = f.read().splitlines()[1:]
            car = '\n'.join(lines[:next((i for i, l in enumerate(lines) if not l.startswith('#')),
                                         len(lines))])
        except OSError as e:
            car = ''
            self.note(f'car.sh 를 못 읽었습니다: {e}')

        def done(ok, out):
            blocks, cur = {}, None
            for l in out.splitlines() if ok else []:
                if l.startswith('@@@ '):
                    cur = l[4:].strip()
                    blocks[cur] = []
                elif cur:
                    blocks[cur].append(l)
            blocks = {'car.sh': car, **{k: '\n'.join(v) for k, v in blocks.items()}}
            self.show_usage(blocks, None if ok else (out.strip().splitlines() or ['응답 없음'])[-1])
        self.ssh.run(['usage'], done, tag='usage')

    def show_usage(self, blocks, ssh_err):
        page = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(page)
        warn = QtWidgets.QLabel(
            '⚠ 젯슨 tmux 안(젯슨 터미널)에서는 car.sh 나 tmux 명령을 치지 마세요 — tmux 가 겹쳐 창이 '
            '엉킵니다 (11-09 노트북 맵핑 시도). 노트북 명령은 노트북 터미널에서, 젯슨 명령은 젯슨 '
            '터미널에서. 이 탭은 보기·복사 전용입니다.')
        warn.setWordWrap(True)
        warn.setStyleSheet('color:white; background:#e65100; padding:6px; border-radius:4px; font-weight:bold;')
        v.addWidget(warn)
        top = QtWidgets.QHBoxLayout()
        src = f'{time.strftime("%H:%M:%S")} 에 읽음 — car.sh 는 노트북 레포, 나머지는 젯슨에서 (스크립트 맨 위 주석)'
        if ssh_err:
            src += f'   ✗ 젯슨에서 못 읽음: {ssh_err}'
        lb = QtWidgets.QLabel(src)
        lb.setStyleSheet('color:#888;')
        top.addWidget(lb, 1)
        top.addWidget(self._btn('젯슨 터미널 열기', self.open_term))
        top.addWidget(self._btn('다시 읽기', self.load_usage))
        v.addLayout(top)

        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        groups, notes = {}, {}
        for script in ('car.sh', 'drive.sh', 'race.sh', 'bat.sh'):
            if script in blocks and blocks[script]:
                ents, nts = parse_usage(script, blocks[script])
                for e in ents:
                    groups.setdefault(e[0], []).append(e)
                notes[script] = nts
        for g in GROUP_ORDER + sorted(set(groups) - set(GROUP_ORDER)):
            if g not in groups:
                continue
            box = QtWidgets.QGroupBox(g)
            grid = QtWidgets.QGridLayout(box)
            grid.setColumnStretch(2, 1)
            for r, (_g, where, cmd, desc) in enumerate(groups[g]):
                badge = QtWidgets.QLabel(where)
                badge.setStyleSheet(CHIP.format('#1565c0' if where == '노트북' else GREEN))
                badge.setToolTip('노트북 터미널에서' if where == '노트북' else '젯슨에서 (젯슨 터미널 열기)')
                c = QtWidgets.QLabel(cmd)
                c.setFont(mono)
                c.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                d = QtWidgets.QLabel(desc)
                d.setWordWrap(True)
                d.setStyleSheet('color:#aaa;')
                cp = nofocus(QtWidgets.QPushButton('복사'))
                cp.clicked.connect(lambda _=False, t=cmd: self.copy(t))
                grid.addWidget(badge, r, 0, QtCore.Qt.AlignTop)
                grid.addWidget(c, r, 1, QtCore.Qt.AlignTop)
                grid.addWidget(d, r, 2)
                grid.addWidget(cp, r, 3, QtCore.Qt.AlignTop)
            v.addWidget(box)
        extra = [f'[{k}] ' + ' '.join(n) for k, n in notes.items() if n]
        if extra:
            nb = QtWidgets.QLabel('\n\n'.join(extra))
            nb.setWordWrap(True)
            nb.setStyleSheet('color:#888;')
            v.addWidget(nb)
        v.addStretch(1)
        self.cmd_page.setWidget(page)

    def copy(self, text):
        QtWidgets.QApplication.clipboard().setText(text)
        self.note(f'복사: {text}')

    # ───────── 로그 ─────────
    def note(self, text):
        if text:
            self.logs['local'].appendPlainText(time.strftime('%H:%M:%S ') + text.rstrip())

    def start_logs(self):
        """remote.sh logs (tail -F) 하나를 계속 받아 탭으로 나눈다. 끊기면 5초 뒤 다시."""
        if self.log_proc is not None:
            return
        p = self.log_proc = self.ssh.stream(['logs'])
        if p is None:                       # 창을 닫는 중
            return
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
                    if self.tail_tab == 'relocalize':
                        self.reloc_line(line)

        def fin(*_):
            self.log_proc = None
            QtCore.QTimer.singleShot(5000, self.start_logs)
        p.readyRead.connect(read)
        p.finished.connect(fin)

    def reloc_line(self, line):
        """relocalize 로그 한 줄 → 배너. 시각은 로그의 ROS 시각이라, 재접속으로 옛 줄이
        다시 와도 '언제 일인지' 가 맞게 보인다."""
        m = RELOC_LINE.match(line)
        if not m:
            return
        stamp, msg = float(m.group(1)), m.group(2)
        when = time.strftime('%H:%M:%S', time.localtime(stamp))
        if '적용하지 않습니다' in msg:
            sc = re.search(r'점수 ([\d.]+) → ([\d.]+)', msg)
            mv = re.search(r'이동 ([\d.]+) cm, yaw ([-+\d.]+)°', msg)
            text = (f'✗ {when} 위치 다시잡기 거부 — 맞춤 점수 '
                    + (f'{sc.group(1)} → {sc.group(2)}' if sc else '?') + ' (기준 0.35 이상)'
                    + (f', 찍은 곳에서 {mv.group(1)} cm · {mv.group(2)}° 떨어진 곳이 그나마 맞음' if mv else '')
                    + '.  차가 이 맵 위의 그 자리·방향에 있는지 보고 다시 찍으세요.'
                    + ('  (찍은 자리 점수가 0 근처면 위치나 방향이 크게 틀린 것)' if sc and float(sc.group(1)) < 0.1 else ''))
            color = RED
        elif '✓ 위치 설정' in msg:
            text, color = f'✓ {when} {msg.split("✓ ", 1)[1]}', GREEN
        elif '진행 중' in msg:
            text, color = f'… {when} {msg}', YELLOW
        elif any(k in msg for k in ('실패', '없습니다')):
            text, color = f'✗ {when} 위치 다시잡기: {msg}', RED
        else:
            return
        self.reloc.setText(text)
        self.reloc.setStyleSheet(CHIP.format(color) + ' font-weight:bold;')
        self.reloc.setVisible(True)

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
            self.cb_odom.setCurrentText(s.get('loc_odom', 'off'))
            self.cb_pin.setCurrentText(s.get('map_pin', 'true'))
            try:
                self.sp_fric.setValue(float(s.get('friction', '0.41')))
            except ValueError:
                pass
            try:
                self.sp_steer.setValue(float(s.get('steer_plan_frac', '1.0')))
            except ValueError:
                pass
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
        if self.cb_obs.currentText() == 'learn':
            w = (w + '\n' if w else '') + ('learn: 트랙에 상대 차·사람이 없어야 해요. 차는 장애물을 안 피하고 달리고, '
                                             '끝나면 그 자리들이 맵 헛것 목록에 들어가요.')
        self.cfg_warn.setText(w)

    def apply_cfg(self):
        want = {'map': self.cb_map.currentText(), 'track': self.cb_track.currentText(),
                'v_max': f'{self.sp_vmax.value():.1f}', 'obstacles': self.cb_obs.currentText(),
                'loc_odom': self.cb_odom.currentText(), 'map_pin': self.cb_pin.currentText(),
                'friction': f'{self.sp_fric.value():.2f}', 'steer_plan_frac': f'{self.sp_steer.value():.2f}',
                'log': self.le_log.text().strip()}
        def same(k, v):
            cur = self.st.get(k, '') or ''
            if k in ('friction', 'steer_plan_frac'):     # 0.4 와 0.40 은 같다 (없으면 1.0)
                cur = cur or ('1.0' if k == 'steer_plan_frac' else cur)
                try:
                    return abs(float(cur) - float(v)) < 1e-9
                except ValueError:
                    return False
            return cur == v or (k == 'log' and v == '' and cur in ("''", '""'))
        changes = {k: v for k, v in want.items() if not same(k, v)}
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

    def quiet(self):
        if not self.ask('젯슨의 VS Code·크롬·RViz 를 모두 닫을까요?\n\n'
                        'VS Code 안의 Claude 작업도 같이 꺼집니다. 주행 노드는 그대로입니다.'):
            return
        self.remote(['quiet'], then_poll=False)

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
        # ★ SSH 터널 안으로 (link.sh). 젯슨 브리지는 와이파이에 열려 있지 않다 (2026-11-11)
        #   닫을 때 killpg 로 link.sh·터널·브리지가 같이 내려간다
        self.bridge = subprocess.Popen(
            [os.path.join(ROOT, 'tools', 'zenoh', 'link.sh'), self.host],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.note(f'노트북 브리지 → SSH 터널 → {self.host} (로그: mlcs_tunnel.log·mlcs_laptop_bridge.log)')

    def open_rviz(self):
        if self.rviz is not None and self.rviz.poll() is None:
            return self.note('RViz 가 이미 떠 있습니다')
        self.rviz = subprocess.Popen(['rviz2', '-d', os.path.join(ROOT, 'tools', 'zenoh', 'laptop.rviz')],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     start_new_session=True)

    def open_term(self):
        cmd = f'ssh -o HostKeyAlias=mlcs-car -t {USER}@{self.host} "tmux new -A -s race"'
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
        # ★ 타이머를 먼저 멈춘다. closeEvent 뒤에도 이벤트 루프가 잠깐 돌아서, 그사이 타이머가
        #   새 ssh 조회를 띄우면 앱이 내려가며 그 QProcess 가 지워지고, 끝날 때 콜백이
        #   지워진 객체를 읽어 RuntimeError 가 났다 (노트북 시험 2, 띄우고 1분 만에 닫았을 때)
        for t in self.timers:
            t.stop()
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
