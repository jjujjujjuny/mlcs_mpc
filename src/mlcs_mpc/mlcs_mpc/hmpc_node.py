#!/usr/bin/env python3
"""
hmpc_node — HyperMPC 정식화의 OCP 로 차를 모는 노드. 동역학 파라미터는 상수 (hmpc.yaml) 이거나
  HyperPM 시변 파라미터 (hyper_model 에 .npz 를 주면, hypermpc/hyper_params.py) 다.

  구독: /mpc/state, /mocap/valid, /mpc/enabled, /mpc/manual, /sensors/core, (opponent_topic)
  발행: /drive, /mpc/solve_time, /hmpc/status, /hmpc/opponent, /hmpc/opponent_markers

■ ★ 풀이는 다른 프로세스에서 한다 (hypermpc/ocp_worker.py)

  solve() 는 NaN 에서 돌아오지 않는다 (2026-11-05 실측). VESC 는 새 명령이 없으면
  마지막 속도를 유지하므로, 이 노드가 멈추면 차가 계속 달린다. 그래서

    · 이 노드는 solve 를 부르지 않는다. 타이머는 항상 돈다.
    · 답이 solve_timeout 안에 안 오면 워커를 SIGKILL 하고 속도 0 을 계속 보낸다.
    · 고장은 **래치**한다. /mpc/enabled 를 false → true 로 다시 줘야 풀린다.

  이 노드 자체가 죽는 경우는 막지 못한다 (Stanley·MPC 노드와 같다).

■ 정지 조건 — 하나라도 걸리면 speed=0 을 control_rate 로 계속 보낸다

  mocap 무효, 상태 오래됨, VESC 오래됨, 수동 조종, 출발 대기, 후진 중, 비정상 상태값,
  솔버 준비 안 됨, 솔버 고장. 비정상 해(status≠0, NaN)는 내보내지 않고,
  max_bad_solves 번 이어지면 래치한다.

  달리던 중 mocap 이 불량이면 복구돼도 **혼자 다시 출발하지 않는다.** 그동안 차가 어디로
  갔는지 모르고, 복구 직후 pose·속도도 믿기 어렵다. LB 를 두 번 누르거나 (수동 → 자율)
  /mpc/enabled 를 false → true 로 다시 줘야 한다.

■ 실제 조향과 서보 명령

  모델의 delta 는 실제 조향이다. 원선회로 잰 맵으로 명령으로 바꿔 내보낸다
  (최대 명령 0.40 이 실제 0.378). 조향 시정수는 계단 응답에 맞췄고 순수 지연은 따로
  넣지 않았다. 바퀴 시정수는 저자 값이다 (config/hmpc.yaml 참고).

■ 상대 차 (opponent_topic 을 주면 켜진다 — opponent.py)

  상대 pose 로 예측한 위치 옆 통로로 스테이지별 n 경계를 좁힌다 (추월). 양쪽이 막히면 스테이지별
  바퀴 속도 상한으로 뒤따른다. 솔버는 그대로다. 측면·상태는 /hmpc/opponent, 예측과 통로는
  /hmpc/opponent_markers (RViz). ★ 상대 pose 는 트랙과 같은 좌표계여야 한다 (mocap 주행이면
  mocap 트랙 + 상대 mocap RigidBody, 시뮬이면 opp_odom). 라이다 주행에서 mocap 상대를 쓰려면
  map↔mocap 변환이 필요하다 (아직 없음).
"""
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from dataclasses import fields

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, Float32, String
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from visualization_msgs.msg import Marker, MarkerArray

from mlcs_mpc.hmpc_core import Track, make_state, solution_ok, steer_to_command, to_command
from mlcs_mpc.opponent import LEFT, RIGHT, TRAIL, Opponent, OppParams

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Worker:
    """ocp_worker 프로세스. 파이프가 막혀도 부르는 쪽을 막지 않는다."""

    def __init__(self, python, script, config, extra, N, on_reply, log):
        self.N = N
        self.reply_len = 8 * (3 + 2 + 9 + N + 1)
        self.on_reply = on_reply
        self.ready = False
        self.alive = True
        p_in, self.to_child = os.pipe()
        self.from_child, p_out = os.pipe()
        os.set_blocking(self.to_child, False)
        env = dict(os.environ)
        home = os.path.expanduser('~')
        env.setdefault('ACADOS_SOURCE_DIR', home + '/acados')
        env['LD_LIBRARY_PATH'] = env.get('LD_LIBRARY_PATH', '') + ':' + home + '/acados/lib'
        self.proc = subprocess.Popen(
            [python, '-u', script, '--fd-in', str(p_in), '--fd-out', str(p_out),
             '--config', json.dumps(config)] + extra,
            pass_fds=(p_in, p_out), env=env, stdout=log, stderr=log, stdin=subprocess.DEVNULL)
        os.close(p_in)
        os.close(p_out)
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _read_loop(self):
        try:
            if os.read(self.from_child, 1) != b'R':
                self.alive = False
                return
            self.ready = True
            buf = b''
            while True:
                chunk = os.read(self.from_child, self.reply_len - len(buf))
                if not chunk:
                    break
                buf += chunk
                if len(buf) == self.reply_len:
                    self.on_reply(np.frombuffer(buf, dtype=np.float64))
                    buf = b''
        except OSError:
            pass
        self.alive = False

    DELAY_HIST = 8      # ocp_worker.DELAY_HIST 와 같아야 한다

    def send(self, rid, cmd, t, x, kappa, n_lo, n_hi, v_cap, dc=(0.0, math.nan, math.nan), hist=None):
        """dc = (지연 보상 켬, 상태 측정 시각, 목표 시각), hist = (시각, ws_ref, δ) × DELAY_HIST (NaN 은 빈 칸)."""
        if hist is None:
            hist = np.full((self.DELAY_HIST, 3), math.nan)
        data = np.concatenate([[rid, cmd, t], x, kappa, n_lo, n_hi, v_cap, dc,
                               np.asarray(hist, float).ravel()]).tobytes()
        return os.write(self.to_child, data) == len(data)

    def kill(self):
        self.alive = False
        self.ready = False
        try:
            self.proc.send_signal(signal.SIGKILL)
            self.proc.wait(timeout=1.0)
        except (OSError, subprocess.TimeoutExpired):
            pass
        for fd in (self.to_child,):
            try:
                os.close(fd)
            except OSError:
                pass


class HmpcNode(Node):

    def __init__(self):
        super().__init__('hmpc_node')
        d = self.declare_parameter
        d('repo_root', os.path.expanduser('~/mlcs_mpc'))
        d('worker_python', os.path.expanduser('~/hmpc-venv/bin/python'))
        d('worker_script', 'hypermpc/ocp_worker.py')
        d('worker_extra_args', [''])
        d('worker_log', '/tmp/mlcs_hmpc_worker.log')
        d('solver_core', 5)
        d('track_csv', 'lab_map/data/tracks/s_track/prep_s_track.csv')
        d('N', 80)
        d('dt', 0.033)
        d('rk_steps', 4)
        d('steering_tau', 0.13)
        d('wheel_speed_tau', 0.0461)
        # 바퀴 가속도 한계 m/s² (0 = 끔). tau 는 작게 두고 이것으로 '바로 못 선다' 를 알려 준다
        d('ws_acc_max', 0.0)
        d('ws_dec_max', 0.0)
        # 헤딩 돌출: 차체 앞·뒤 끝 (무게중심에서 m) 도 벽 경계 안에. 0 = 끔. 켜면 wall_margin 은 반폭+여유
        # 경계를 p 에 (상자 경계 constraints_set 대신). 주기당 ~5 ms 줄인다 (offline 42.9 → 38.3 ms)
        d('bounds_param', False)
        # ★ 지연 보상 (2026-11-10): 상태 측정 시각 (header stamp) → 지금 + 예상 풀이 시간 만큼, 그동안 낸
        #   명령으로 워커가 모델을 적분해 x0 를 당긴다. 라이다 센서→명령 60~100 ms. offline 1109_4 v2.0 80 ms:
        #   보상 없음 랩 6.60~7.52 / 벽밖 1.6% → 보상 6.24~6.30 / 0% (지연 0 은 6.24~6.47). 기본 끔
        d('delay_comp', False)
        # 스탬프에 안 보이는 추가 지연 s. Cartographer /tracked_pose 는 발행 시각으로 외삽해 찍어서
        #   recv − stamp 는 5~6 ms 뿐인데, mocap 과 맞춰 잰 실제 지연 (map_mocap_align time_lag) 은 15~51 ms (11-10).
        #   race.sh 가 라이다 주행이면 25 ms 를 넣는다 (주행별 실측 17~27 ms, 맵 정렬 값은 재현 안 됨)
        d('delay_extra', 0.0)
        # 액추에이터 지연 s: 조향 명령이 실제 곡률로 반영되는 데 모델 (τ 0.13) 보다 더 걸리는 시간.
        #   11-10 실차 1108_7: 명령→곡률 187 ms vs offline 107 ms → 66~80 ms. offline 에서 70 ms 명령 지연 차를
        #   0.07 보상으로 회복. 센서 지연 (delay_extra) 과 따로 두고 더한다
        d('act_delay', 0.0)
        # ★ 교착 탈출 후진 (2026-11-10): MPC 로 달리는데 recover_wait 동안 0.1 m 도 못 가고 평균 vx < 0.15 면
        #   (급코너 안에서 바깥을 향해 서면 어떻게 가도 경계 침범이라 MPC 가 서 있기를 고른다) 조향
        #   sign(μ)·최대로 짧게 후진 → 정지 → 다시 풂. 같은 자리 (s 0.5 m) 에서 2번 넘게 막히면 고장 래치.
        #   offline 1108_7·1109_4 급코너 −25° 5가지 모두 빠져나옴, 직선 정상 출발은 안 걸림. 기본 끔
        d('recover_reverse', False)
        d('recover_speed', 1.2)         # m/s — 짧고 세게. 0.45~0.5 는 정지에서 모터가 늦게 돌아 느렸다 (실차 11-10)
        d('recover_time', 1.2)          # s — 상한. 보통은 거리로 끝난다
        d('recover_dist', 0.2)          # m — 이만큼 물러나면 끝 (바로 정지·제동). 11-11 실트랙: 0.4 는 구름까지 0.6~0.7 m 라 길었다 → 절반
        d('recover_wait', 1.0)          # s
        d('recover_tries', 3)           # 같은 자리 (s 1.0 m 안) 에서 이만큼 후진해도 막히면 고장 래치
        d('q_delta', 2.0)               # ★ 조향 명령과 조향 상태 차이 비용 (저자 2.0). 생성 코드에 들어간다
        d('qp_cond_n', 0)               # ★ HPIPM 부분 응축 길이 (0 = acados 기본 N). 생성 코드에 들어간다
        d('wall_heading', 0.0)
        d('wall_heading_r', 0.25)
        d('trail_stop_speed', 0.3)      # 뒤따름 속도 상한이 이보다 낮으면 풀지 않고 세운다 m/s
        d('trail_go_speed', 0.4)        # 세운 뒤 상한이 이보다 높아지면 다시 푼다 m/s
        d('trail_stop_vx', 0.5)         # 우리 속도가 이보다 낮을 때만 풀지 않고 세운다 m/s (빠를 땐 MPC 가 조향하며 감속)
        d('track_hard', True)
        d('track_width', 0.12)
        d('track_slack', 1000.0)
        # ★ MPCC 경계. walls = 트랙 파일의 지점별 벽 폭 − wall_margin (선은 좌표계일 뿐, MPC 가 선을 고른다).
        #   fixed = 선에서 ±track_width (선을 따라가게 묶는다. 예전 방식)
        d('track_bounds', 'walls')
        d('wall_margin', 0.22)      # 차 반폭 0.15 + 여유. CoG 기준이라 차가 비스듬하면 모서리가 더 나간다
        d('control_rate', 30.0)
        d('solve_timeout', 0.15)
        d('restart_timeout', 0.5)
        d('slow_speed', 0.4)
        d('max_bad_solves', 3)
        d('v_max', 0.7)
        d('max_steer', 0.40)
        d('steer_map_cmd', [0.0, 0.2, 0.3, 0.4])
        d('steer_map_act', [0.0, 0.2021, 0.2972, 0.3776])
        d('max_steer_rate', 3.2)
        d('v_floor', 0.25)
        d('reverse_limit', -0.2)
        d('mass', 3.284)
        d('friction', 0.41)
        d('erpm_gain', 4781.5)
        d('vehicle_params', [0.089, 0.1537, 0.01, 0.01, 0.01, 0.2, 0.90064745,
                             0.304115174, 0.50421894, 0.05])
        d('tire_front', [0.05, 2.0, 0.35, 1.4, 1.0, 1.2, 30.0, 1.3, 1.0, 0.5])
        d('tire_rear', [0.05, 2.0, 0.35, 1.4, 1.0, 1.2, 30.0, 1.3, 1.0, 0.5])
        # HyperPM. 비우면 상수 파라미터. 상대 경로는 repo_root 기준 (config/hyper/*.npz)
        d('hyper_model', '')
        d('hyper_v_min', 0.0)
        d('hyper_blend', 0.0)
        d('state_timeout', 0.2)
        d('vesc_timeout', 0.3)
        d('require_enable', True)
        d('bench', False)
        # 상대 차. 비우면 끔. opponent_msg: pose = PoseStamped (mocap), odom = Odometry (시뮬 opp_odom)
        d('opponent_topic', '')
        d('opponent_msg', 'pose')
        for f in fields(OppParams):         # opp_clear, opp_length, ... (opponent.py 의 OppParams)
            d('opp_' + f.name, f.default)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        root = gp('repo_root')
        rel = lambda p: p if os.path.isabs(p) else os.path.join(root, p)  # noqa: E731
        self.N, self.dt = int(gp('N')), float(gp('dt'))
        self.v_max = float(gp('v_max'))
        self.max_steer = float(gp('max_steer'))
        self.map_cmd = list(gp('steer_map_cmd'))
        self.map_act = list(gp('steer_map_act'))
        self.period = 1.0 / float(gp('control_rate'))
        self.max_dsteer = float(gp('max_steer_rate')) * self.period
        self.lr = list(gp('vehicle_params'))[1]
        self.v_floor = float(gp('v_floor'))
        self.delay_comp = bool(gp('delay_comp'))
        self.delay_extra = float(gp('delay_extra'))
        self.act_delay = float(gp('act_delay'))
        self.recover_reverse = bool(gp('recover_reverse'))
        self.rec_speed, self.rec_time = float(gp('recover_speed')), float(gp('recover_time'))
        self.rec_dist, self.rec_wait = float(gp('recover_dist')), float(gp('recover_wait'))
        self.rec_tries = int(gp('recover_tries'))
        self.recovering = False
        self.rec_until = 0.0
        self.rec_end_t = -1e9           # 마지막 후진이 끝난 시각
        self.rec_start = None           # 후진 시작 (x, y)
        self.rec_steer = 0.0            # 후진 중 서보 명령
        self.rec_s = []                 # 후진을 시작한 s 들 (같은 자리 실패 판정)
        self.prog = deque()             # MPC 로 달리는 주기의 (시각, s, vx)
        self.t_meas = None                  # 지금 상태가 측정된 시각 (/mpc/state header stamp)
        self.cmd_hist = deque(maxlen=Worker.DELAY_HIST)     # 내보낸 명령 (시각, ws_ref, 실제 조향 δ)
        self.solve_est = 0.025              # 풀이 시간 평균 (지수) s — 목표 시각 = 지금 + 이것
        self.dc_skips = 0                   # 스탬프가 이상해 보상을 건너뛴 주기 수
        self._warned_age = False
        self.ws_dec_max = float(gp('ws_dec_max'))
        self.trail_stop_speed, self.trail_go_speed = float(gp('trail_stop_speed')), float(gp('trail_go_speed'))
        self.trail_stop_vx = float(gp('trail_stop_vx'))
        self.reverse_limit = float(gp('reverse_limit'))
        self.friction = float(gp('friction'))
        self.erpm_gain = float(gp('erpm_gain'))
        self.state_timeout = float(gp('state_timeout'))
        self.vesc_timeout = float(gp('vesc_timeout'))
        self.solve_timeout = float(gp('solve_timeout'))
        # 초기 추측을 다시 까는 첫 요청은 평소보다 오래 걸린다. 이때 차는 서 있다.
        self.restart_timeout = float(gp('restart_timeout'))
        self.slow_speed = float(gp('slow_speed'))
        self.limit = self.solve_timeout
        self.max_bad = int(gp('max_bad_solves'))
        self.bench = bool(gp('bench'))
        self.require_enable = bool(gp('require_enable'))

        self.track = Track(rel(gp('track_csv')))
        self.track_width = float(gp('track_width'))
        self.wall_margin = float(gp('wall_margin'))
        mode = str(gp('track_bounds'))
        if mode not in ('walls', 'fixed'):
            raise ValueError(f'track_bounds 는 walls 또는 fixed: {mode}')
        self.walls = mode == 'walls'
        if self.walls:
            lo, hi = self.track.bounds_at(self.track.s, self.wall_margin)
            self.get_logger().info(
                f'경계: 벽 (MPCC) — 여유 {self.wall_margin:.2f} m, n ∈ [{lo.max():+.2f}~{lo.min():+.2f}, '
                f'{hi.min():+.2f}~{hi.max():+.2f}] m'
                + ('' if self.track.has_walls else '  ★ 트랙 파일에 w_left/w_right 가 없어 track_width/2 사용'))
            if (hi - lo).min() < 0.05:
                self.get_logger().warn(f'벽 폭이 여유보다 좁은 곳이 있습니다 (최소 {(hi - lo).min():.2f} m)')
        else:
            self.get_logger().info(f'경계: 선에서 ±{self.track_width:.2f} m (fixed)')
        params = list(gp('vehicle_params')) + list(gp('tire_front')) + list(gp('tire_rear'))
        if gp('hyper_model'):
            hp = rel(gp('hyper_model'))
            if not os.path.isfile(hp):
                raise FileNotFoundError(f'hyper_model 이 없습니다: {hp}')
            self.get_logger().info(f'★ HyperPM 시변 파라미터: {hp} (v_min {gp("hyper_v_min")}). '
                                   '상수 파라미터는 출력이 비정상일 때만 쓴다')
        else:
            self.get_logger().info('동역학: 상수 파라미터 (hmpc.yaml)')
        self.worker_args = dict(
            python=gp('worker_python'), script=rel(gp('worker_script')),
            config=dict(N=self.N, dt=self.dt, rk_steps=int(gp('rk_steps')), steering_tau=float(gp('steering_tau')),
                        wheel_speed_tau=float(gp('wheel_speed_tau')),
                        ws_acc_max=float(gp('ws_acc_max')), ws_dec_max=float(gp('ws_dec_max')),
                        # 헤딩 돌출은 walls 경계에서만 (fixed 면 앞뒤 끝이 생성 기본 ±track_width 에 묶인다)
                        wall_heading=float(gp('wall_heading')) if self.walls else 0.0,
                        wall_heading_r=float(gp('wall_heading_r')),
                        bounds_param=bool(gp('bounds_param')), q_delta=float(gp('q_delta')),
                        qp_cond_N=int(gp('qp_cond_n')),
                        track_hard=bool(gp('track_hard')), track_width=float(gp('track_width')),
                        track_slack=float(gp('track_slack')),
                        mass=float(gp('mass')), friction=float(gp('friction')), v_max=self.v_max,
                        max_steer=self.map_act[-1], params=params, core=int(gp('solver_core')),
                        hyper=rel(gp('hyper_model')) if gp('hyper_model') else '',
                        hyper_v_min=float(gp('hyper_v_min')), hyper_blend=float(gp('hyper_blend'))),
            extra=[a for a in gp('worker_extra_args') if a], N=self.N)
        self.log_file = open(gp('worker_log'), 'ab')

        self.lock = threading.RLock()
        self.pose = self.twist = None
        self.t_state = self.t_vesc = None
        self.ws = 0.0
        self.mocap_valid = False
        self.enabled = not self.require_enable
        self.manual = False
        self.fault = None       # 래치된 솔버 고장
        self.mocap_lost = False # 달리던 중 mocap 불량. 사람이 다시 넘겨야 풀린다
        self.bad = 0
        self.rid = 0.0
        self.pending_t = None   # 답을 기다리는 요청 시각
        self.s_pred = None
        self.v_cap = np.full(self.N, self.v_max)
        self.delta = 0.0
        self.cmd_steer = 0.0
        self.trail_stopped = False      # 뒤따름 정지 중 (풀지 않고 세우는 중)
        self.trail_cmd = math.inf       # 그때 내는 속도 명령 (ws_dec_max 로 줄여 간다)
        self.running = False    # 다음 요청이 초기 추측을 다시 깔아야 하면 False
        self.stop_reason = 'init'
        self.worker = None

        self.create_subscription(Odometry, '/mpc/state', self._state_cb, 10)
        self.create_subscription(Bool, '/mocap/valid', self._valid_cb, 10)
        self.create_subscription(Bool, '/mpc/enabled', self._enable_cb, LATCHED)
        self.create_subscription(Bool, '/mpc/manual', self._manual_cb, LATCHED)
        # 조이스틱 B = 출발. 래치하지 않는다 — 지난번 누름이 재기동 때 남아 혼자 출발하면 안 된다.
        self.create_subscription(Bool, '/mpc/go', self._go_cb, 10)
        self.create_subscription(VescStateStamped, '/sensors/core', self._vesc_cb, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.solve_pub = self.create_publisher(Float32, '/mpc/solve_time', 1)
        self.status_pub = self.create_publisher(String, '/hmpc/status', 1)
        # 컨트롤러가 실제로 기준 삼는 중심선 (track_csv 그대로) — RViz 표시용.
        #   래치 + 2초마다 재발행: RViz·노트북 브리지를 나중에 켜도 뜬다.
        self.center_pub = self.create_publisher(Marker, '/hmpc/centerline', LATCHED)
        self._center_msg = self._make_centerline()
        self.center_pub.publish(self._center_msg)
        self.create_timer(2.0, lambda: self.center_pub.publish(self._center_msg))

        self.opp = None
        self.opp_mode = None
        topic = str(gp('opponent_topic')).strip()
        if topic:
            op = OppParams(**{f.name: type(f.default)(gp('opp_' + f.name)) for f in fields(OppParams)})
            self.opp = Opponent(self.track, op)
            kind = str(gp('opponent_msg'))
            if kind not in ('pose', 'odom'):
                raise ValueError(f'opponent_msg 는 pose 또는 odom: {kind}')
            self.create_subscription(PoseStamped if kind == 'pose' else Odometry, topic, self._opp_cb, 10)
            self.opp_pub = self.create_publisher(String, '/hmpc/opponent', LATCHED)
            self.opp_marker_pub = self.create_publisher(MarkerArray, '/hmpc/opponent_markers', 1)
            self.create_timer(1.0, self._opp_refit)
            self.create_timer(0.1, self._opp_markers)
            self.get_logger().info(f'상대 차: {topic} ({kind}) — 통로 여유 {op.clear:.2f} m, '
                                   f'GP {"켬" if op.gp else "끔"}')

        self._start_worker()
        self.create_timer(self.period, self._control_loop)
        self.get_logger().info(
            f'hmpc_node 시작 — N={self.N} dt={self.dt} v_max={self.v_max} '
            f'트랙 {self.track.length:.1f} m, 제한 {self.solve_timeout * 1000:.0f} ms')
        if self.bench:
            self.get_logger().warn('★ bench=true — 속도는 항상 0 으로 발행됩니다')

    # ── 콜백 ─────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _state_cb(self, m):
        p = m.pose.pose
        with self.lock:
            self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
            self.twist = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z)
            self.t_state = self._now()
            self.t_meas = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _valid_cb(self, m):
        self.mocap_valid = bool(m.data)

    def _manual_cb(self, m):
        with self.lock:
            if m.data and not self.manual:
                self.mocap_lost = False
            self.manual = bool(m.data)

    def _vesc_cb(self, m):
        self.ws = m.state.speed / self.erpm_gain
        self.t_vesc = self._now()

    def _enable_cb(self, m):
        with self.lock:
            was, self.enabled = self.enabled, bool(m.data)
            if self.enabled and not was:
                self.mocap_lost = False
            if was and not self.enabled and self.fault:
                self.get_logger().warn('고장 래치 해제 대기 — 다시 true 를 주면 재개합니다')
            if self.enabled and not was and self.fault:
                self.get_logger().info(f'고장 래치 해제: {self.fault}')
                self.fault = None
                self.bad = 0

    def _opp_cb(self, m):
        p = m.pose if isinstance(m, PoseStamped) else m.pose.pose
        q = p.orientation
        with self.lock:
            # 수신 시각을 쓴다. 다른 PC 의 stamp 와 이 노드 시계가 어긋나도 상관없게
            self.opp.measure(self._now(), p.position.x, p.position.y, yaw_from_quat(q))

    def _opp_refit(self):
        try:
            with self.lock:
                was = self.opp.model.tab is not None
                now = self.opp.model.refit()
        except Exception as e:  # noqa: BLE001 — GP 재학습 실패로 노드가 죽으면 안 된다 (제로샷 예측으로 계속)
            self.get_logger().error(f'상대 GP 재학습 실패 (주행은 계속): {e!r}', throttle_duration_sec=10.0)
            return
        if now != was:
            self.get_logger().info('상대 라인: ' + ('GP 학습됨 — 상대 선과 속도로 예측' if now
                                                   else '관측 부족 — 제로샷 예측'))

    def _opp_markers(self):
        # ★ 2026-11-11 실트랙: 시각화 타이머의 예외 (ColorRGBA 에 정수) 하나로 노드가 죽어 /drive 가 끊겼다.
        #   그림은 주행에 필요 없으니 어떤 예외도 삼키고 한 번만 알린다.
        try:
            self._opp_markers_impl()
        except Exception as e:  # noqa: BLE001
            if not getattr(self, '_warned_markers', False):
                self._warned_markers = True
                self.get_logger().error(f'상대 마커 그리기 실패 (주행은 계속): {e!r}')

    def _make_centerline(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type, m.action = 'centerline', 0, Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.03
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.9)
        m.pose.orientation.w = 1.0
        xs, ys = np.r_[self.track.x, self.track.x[0]], np.r_[self.track.y, self.track.y[0]]  # 닫힌 루프
        m.points = [Point(x=float(a), y=float(b)) for a, b in zip(xs, ys)]
        return m

    def _opp_markers_impl(self):
        with self.lock:
            last = self.opp.last
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def strip(mid, xs, ys, rgba, width):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id, m.type = 'opponent', mid, Marker.LINE_STRIP
            m.action = Marker.ADD if xs is not None else Marker.DELETE
            m.scale.x = width
            m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
            m.pose.orientation.w = 1.0
            if xs is not None:
                m.points = [Point(x=float(a), y=float(b)) for a, b in zip(xs, ys)]
            return m

        if last is None:
            arr.markers = [strip(0, None, None, (0, 0, 0, 0), 0.02), strip(1, None, None, (0, 0, 0, 0), 0.02)]
        else:
            pred, pl, mode = last
            xs, ys = self.track.xy(pred.s, pred.d)
            arr.markers.append(strip(0, xs, ys, (1.0, 0.4, 0.0, 0.9), 0.04))   # 상대 예측 경로
            h = pl.hit
            if mode in (LEFT, RIGHT) and h.any():
                c = self.opp.p.clear + pred.sig_d[h]
                n = pred.d[h] + c if mode == LEFT else pred.d[h] - c
                bx, by = self.track.xy(pred.s[h], n)
                arr.markers.append(strip(1, bx, by, (0.1, 0.8, 0.2, 0.9), 0.03))  # 통로 경계
            else:
                arr.markers.append(strip(1, None, None, (0, 0, 0, 0), 0.02))
        self.opp_marker_pub.publish(arr)

    def _go_cb(self, m):
        """B 버튼. /mpc/enabled=true 와 같고, 누를 때마다 고장·mocap 래치도 푼다."""
        if not m.data:
            return
        with self.lock:
            if not self.enabled:
                self.get_logger().info('B — 출발')
            self.enabled = True
            self.mocap_lost = False
            if self.fault:
                self.get_logger().info(f'B — 고장 래치 해제: {self.fault}')
                self.fault = None
                self.bad = 0

    # ── 워커 ─────────────────────────────────────────────────────────
    def _start_worker(self):
        self.worker = Worker(on_reply=self._reply_cb, log=self.log_file, **self.worker_args)

    def _trip(self, why):
        """워커를 버리고 고장을 래치한다. 부르면 바로 돌아온다."""
        with self.lock:
            self.fault = why
            self.pending_t = None
            self.running = False
            old = self.worker
            self.worker = None
        self.get_logger().error(f'✗ 솔버 고장: {why} — 속도 0. 조이스틱 B 로 재개')
        self._publish(0.0, 0.0)

        def replace():
            old.kill()
            with self.lock:
                self._start_worker()
        threading.Thread(target=replace, daemon=True).start()

    def _reply_cb(self, r):
        rid, st, ms = r[0], r[1], r[2]
        u, x1, s_pred = r[3:5], r[5:14], r[14:]
        with self.lock:
            if self.pending_t is None or rid != self.rid:
                return      # 고장 처리나 정지 뒤에 도착한 답
            self.pending_t = None
            self.solve_pub.publish(Float32(data=float(ms)))
            if math.isfinite(ms) and ms > 0:
                self.solve_est = 0.9 * self.solve_est + 0.1 * min(ms, 100.0) / 1000.0
            if not solution_ok(st, u, x1):
                self.bad += 1
                self.running = False
                self.get_logger().warn(f'비정상 해 버림 (status {int(st)}, {self.bad}번째)')
                self._publish(0.0, 0.0)
                if self.bad >= self.max_bad:
                    self._trip(f'비정상 해 {self.bad}번 연속 (status {int(st)})')
                return
            self.bad = 0
            self.s_pred = s_pred
            self.delta = float(x1[8])
            if self._blocked() is not None:
                return      # 답을 기다리는 사이에 정지 조건이 걸렸다
            speed, steer = to_command(u, self.v_max, self.max_steer, self.cmd_steer,
                                      self.max_dsteer, self.map_cmd, self.map_act)
            self.cmd_steer = steer
            self._publish(speed, steer)

    # ── 정지 조건 ────────────────────────────────────────────────────
    def _blocked(self):
        if self.fault:
            return f'솔버 고장: {self.fault}'
        if self.worker is None or not self.worker.ready:
            return '솔버 준비 중'
        if not self.worker.alive:
            return '솔버 프로세스 없음'
        if self.pose is None:
            return '상태 수신 전'
        now = self._now()
        mocap = None
        if not self.mocap_valid:
            mocap = 'mocap 무효'
        elif now - self.t_state > self.state_timeout:
            mocap = '상태 오래됨'
        elif not all(math.isfinite(v) for v in self.pose + self.twist):
            mocap = '상태값 비정상'
        if mocap is not None:
            # 끊긴 동안 LB 를 눌러 자율로 넘겨도 풀리지 않는다. 복구를 보고 다시 넘겨야 한다.
            if self.enabled and not self.manual:
                self.mocap_lost = True
            return mocap
        if self.mocap_lost:
            return 'mocap 불량 뒤 대기 — LB 두 번 또는 B'
        if self.t_vesc is None or now - self.t_vesc > self.vesc_timeout:
            return 'VESC 끊김'
        # 탈출 후진 중과 끝난 뒤 1 s (아직 뒤로 구르는 중) 은 후진 정지를 걸지 않는다
        if self.twist[0] < self.reverse_limit and not self.recovering and self._now() - self.rec_end_t > 1.0:
            return '후진 중'
        if self.manual:
            return '수동 조종 중'
        if not self.enabled:
            return '출발 대기 — 조이스틱 B'
        return None

    # ── 주기 루프 — 어떤 경우에도 막히면 안 된다 ──────────────────────
    def _control_loop(self):
        with self.lock:
            now = self._now()
            if self.pending_t is not None and now - self.pending_t > self.limit:
                self._trip(f'{(now - self.pending_t) * 1000:.0f} ms 동안 답이 없음')
            if self.worker is not None and self.worker.ready and not self.worker.alive \
                    and not self.fault:
                self._trip('솔버 프로세스가 죽음')

            reason = self._blocked()
            if reason is not None:
                if reason != self.stop_reason:
                    self.get_logger().warn(f'정지: {reason}')
                    self.status_pub.publish(String(data=reason))
                    self.stop_reason = reason
                self.pending_t = None
                self.running = False
                self.track.reset()
                self.delta = self.cmd_steer = 0.0
                self.trail_stopped, self.trail_cmd = False, math.inf
                self.recovering = False
                self.prog.clear()
                if reason != '후진 중':
                    self.rec_s = []         # 사람이 개입했다 (수동·B·고장 해제) — 같은 자리 횟수를 새로
                self._publish(0.0, 0.0)
                return
            if self.stop_reason is not None:
                self.get_logger().info('▶ 주행 조건 충족')
                self.status_pub.publish(String(data='주행'))
                self.stop_reason = None
            if self.recovering:
                self._recover_step(now)
                return
            if self.pending_t is not None:
                return      # 지난 요청의 답을 기다린다. 제한 시간은 위에서 본다

            x = make_state(self.track, self.pose, self.twist, self.ws, self.delta,
                           self.lr, self.v_floor, self.friction)
            if self.recover_reverse and self._check_stuck(now, x):
                return
            # 지연 보상: 워커가 x0 를 t_meas → t_tgt 로 당긴다. 경계·곡률을 읽는 s 도 그만큼 앞으로
            # 보상을 꺼도 측정 시각은 보낸다 — 워커 통계 age_ms 로 실제 상태 지연을 잰다
            dc = (0.0, self.t_meas if self.t_meas is not None else math.nan, math.nan)
            s_shift = 0.0
            if self.delay_comp and self.t_meas is not None:
                age = now - self.t_meas
                t_tgt = now + self.solve_est
                # 실제 측정 시각 (스탬프보다 센서 지연만큼 이르다) + 명령이 먹는 데 더 걸리는 시간만큼 더 앞으로
                t_eff = self.t_meas - self.delay_extra - self.act_delay
                d_pred = t_tgt - t_eff
                if 0.0 <= age <= 0.3 and d_pred <= 0.2:
                    dc = (1.0, t_eff, t_tgt)
                    s_shift = x[3] * d_pred
                else:
                    # 스탬프가 비었거나 시계가 다르거나 (mocap 다른 PC) 너무 오래됨 (합계 0.2 s 초과) — 보상 없이
                    self.dc_skips += 1
                    if not self._warned_age:
                        self.get_logger().warn(f'지연 보상: 상태 스탬프가 이상함 (지금 − 스탬프 {age:.3f} s) '
                                               '— 그 주기는 보상 안 함 (이후 횟수만 셈)')
                        self._warned_age = True
                    if self.dc_skips % 300 == 0:
                        self.get_logger().warn(f'지연 보상 건너뜀 {self.dc_skips}번')
            if self.running and self.s_pred is not None:
                # ★ 지난 풀이의 s 궤적을 지금 위치에 맞춘다 (2026-11-09). MERIT_BACKTRACKING 으로 스텝이
                #   일부만 들어가면 풀이의 스테이지 0 이 x0 와 어긋나, 예측 s 가 실제보다 최대 0.4 m 뒤처진다
                #   (뒤따름 급감속 때, 평소는 2 cm 이하). 그대로 쓰면 벽 경계·곡률·상대 간격을 엉뚱한 곳에서
                #   읽는다 — 정지 장애물 뒤에 0.4 m 까지 붙어 섰다 (offline 1109_3). 지금은 s_pred[1] ≈ x[0].
                s = self.s_pred + (x[0] + s_shift - self.s_pred[1])
            else:
                s = x[0] + s_shift + x[3] * self.dt * np.arange(self.N + 1)
            kappa = self.track.kappa_at(s)
            if self.walls:
                n_lo, n_hi = self.track.bounds_at(s, self.wall_margin)
            else:
                n_lo = np.full(self.N + 1, -self.track_width)
                n_hi = -n_lo
            v_cap = self.v_cap
            if self.opp is not None:
                # s_pred[k] 는 지난 풀이 시각 기준이라 지금 기준으로는 한 칸 앞이 맞다 (상대와 시각을 맞춘다)
                s_now = np.r_[s[1:], 2 * s[-1] - s[-2]] if self.running and self.s_pred is not None else s
                n_lo, n_hi, cap, mode = self.opp.step(now, s_now, x[1], n_lo, n_hi, self.v_max, self.N, self.dt)
                if cap is not None:
                    v_cap = cap
                if mode != self.opp_mode:
                    self.opp_mode = mode
                    self.get_logger().info(f'상대: {mode or "없음"}')
                    self.opp_pub.publish(String(data=mode or 'none'))
                # ★ 뒤따름 정지 (2026-11-09, 메인 PC 제안): 속도 상한이 trail_stop_speed 아래면 풀지 않고
                #   직접 세운다. 저속 + 상한 0 은 풀이가 status 4 로 깨져 래치된다 (offline 1109_3,
                #   정지 장애물 뒤 204/600). 감속은 ws_dec_max 안, 조향은 그대로. 상한이 다시 오르면 푼다.
                #   ★ 2026-11-11: 빠를 때 (커브 1.3 m/s) 조향을 고정한 채 세우면 벽 밖으로 미끄러졌다 (offline 1109_3).
                #   이미 느릴 때 (trail_stop_vx 아래) 만 직접 세우고, 그 전에는 MPC 가 상한을 지키며 조향·감속한다.
                stop_now = mode == TRAIL and cap is not None and (
                    (cap[0] < self.trail_stop_speed and (self.trail_stopped or self.twist[0] < self.trail_stop_vx))
                    or (self.trail_stopped and cap[0] < self.trail_go_speed))
                if stop_now:
                    if not self.trail_stopped:
                        self.get_logger().info(f'뒤따름 정지 — 상한 {cap[0]:.2f} m/s, 풀지 않고 세움')
                    self.trail_stopped = True
                    dec = self.ws_dec_max if self.ws_dec_max > 0 else 2.5
                    self.trail_cmd = max(min(self.trail_cmd, self.ws) - dec * self.dt, 0.0)
                    self.running = False        # 다시 풀 때 초기 추측을 지금 상태로
                    self._publish(self.trail_cmd, self.cmd_steer)
                    return
                if self.trail_stopped:
                    self.get_logger().info(f'뒤따름 정지 풀림 — 상한 {cap[0] if cap is not None else -1:.2f} m/s, mode {mode}')
                self.trail_stopped = False
                self.trail_cmd = math.inf
            self.rid += 1.0
            cmd = 0.0 if self.running else 1.0
            # 저속에서는 풀이가 110 ms 까지 간다 (실측). 느리니 더 기다려도 멀리 못 간다.
            slow = self.twist[0] < self.slow_speed
            self.limit = self.restart_timeout if not self.running or slow else self.solve_timeout
            try:
                hist = np.full((Worker.DELAY_HIST, 3), math.nan)
                if dc[0] > 0 and self.cmd_hist:
                    hist[:len(self.cmd_hist)] = np.array(self.cmd_hist)
                ok = self.worker.send(self.rid, cmd, now, x, kappa, n_lo, n_hi, v_cap, dc, hist)
            except OSError:
                ok = False
            if not ok:
                self._trip('요청을 못 보냄')
                return
            self.pending_t = now
            self.running = True

    # ── 교착 탈출 후진 ───────────────────────────────────────────────
    def _check_stuck(self, now, x):
        """MPC 로 달리는데 recover_wait 동안 못 나아가면 후진을 시작한다. 시작했으면 True."""
        if not self.running or self.bench or self.trail_stopped or self.opp_mode == TRAIL:
            self.prog.clear()       # 상대 뒤에서 일부러 느린 건 교착이 아니다
            return False
        if self.rec_s and x[0] - max(self.rec_s) > 1.0:
            self.rec_s = []             # 그 자리를 빠져나갔다
        # ★ 2026-11-11 실트랙 (1111_2 v_max 4): 후진 끝 바로 뒤 1 s 창에 아직 뒤로 구르는 거리와 MPC 재출발 지연이
        #   들어가 'ds < 0 → 또 교착' 이 9 번 이어지며 차가 트랙을 5 m 거꾸로 갔다. 후진 끝 1 s 는 창을 비워 둔다.
        if now - self.rec_end_t < 1.0:
            self.prog.clear()
            return False
        self.prog.append((now, float(x[0]), float(self.twist[0])))
        while self.prog and now - self.prog[0][0] > self.rec_wait + 0.2:
            self.prog.popleft()
        if not self.prog or now - self.prog[0][0] < self.rec_wait:
            return False
        ds = self.prog[-1][1] - self.prog[0][1]
        if ds >= 0.1 or np.mean([p[2] for p in self.prog]) >= 0.15:
            return False
        self.prog.clear()
        # 상대가 뒤쪽 1 m 안이면 후진 안 함 (라이다 뒤쪽 사각)
        if self.opp is not None:
            st = self.opp.tracker.state(now)
            if st is not None:
                back = (x[0] - st[0] + self.track.length / 2) % self.track.length - self.track.length / 2
                if 0.0 < back < 1.0:
                    self.get_logger().warn(f'교착이지만 상대가 뒤 {back:.2f} m — 후진 안 함')
                    return False
        # 같은 자리: 후진 0.4 m + 구름 + 재출발 실패까지 덮게 1.0 m (0.5 m 면 뒤로 밀리며 매번 '1번째' 가 됐다)
        self.rec_s = [q for q in self.rec_s if abs(q - x[0]) < 1.0]
        if len(self.rec_s) >= self.rec_tries:
            self._trip(f'같은 자리에서 후진 탈출 {self.rec_tries}번 뒤에도 교착')
            return True
        self.rec_s.append(float(x[0]))
        mu = float(x[2])
        act = math.copysign(self.map_act[-1], mu if mu != 0 else 1.0)    # 후진은 요 방향이 반대 → μ 가 줄어드는 쪽
        self.rec_steer = steer_to_command(act, self.map_cmd, self.map_act)
        self.recovering = True
        self.rec_until = now + self.rec_time
        self.rec_start = (self.pose[0], self.pose[1])
        self.running = False
        self.get_logger().warn(f'교착 (1 s 에 {ds:.2f} m) — 후진 탈출 {len(self.rec_s)}번째: '
                               f'{self.rec_speed} m/s, 조향 {act:+.2f} rad (μ {math.degrees(mu):+.0f}°)')
        self._publish(-self.rec_speed, self.rec_steer)
        return True

    def _recover_step(self, now):
        moved = math.hypot(self.pose[0] - self.rec_start[0], self.pose[1] - self.rec_start[1])
        if now < self.rec_until and moved < self.rec_dist:
            self._publish(-self.rec_speed, self.rec_steer)
            return
        self.recovering = False
        self.rec_end_t = now
        self.running = False            # 다시 풀 때 초기 추측을 지금 상태로 (cmd 1)
        self.prog.clear()
        self.cmd_steer = 0.0
        self._publish(0.0, 0.0)
        self.get_logger().info(f'후진 탈출 끝 ({moved:.2f} m) — MPC 다시')

    def _publish(self, speed, steer):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.drive.speed = 0.0 if self.bench else float(speed)
        m.drive.steering_angle = float(steer)
        self.drive_pub.publish(m)
        # 지연 보상용 기록: 모델 단위 (바퀴 속도, 실제 조향각 — 서보 명령을 조향 맵으로 되돌림)
        self.cmd_hist.append((self._now(), m.drive.speed,
                              float(np.interp(float(steer), self.map_cmd, self.map_act))))


def main(args=None):
    rclpy.init(args=args)
    node = HmpcNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as e:
        # 종료 신호가 메시지 변환 중에 오면 이렇게 올라온다. 어느 쪽이든 아래에서 세운다.
        node.get_logger().warn(f'spin 종료: {e}')
    finally:
        try:
            node._publish(0.0, 0.0)
            time.sleep(0.05)
        except Exception:
            pass
        if node.worker is not None:
            node.worker.kill()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
