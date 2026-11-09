"""
config — MPCC 설정. 값은 모두 config/mpcc.yaml 에서 온다 (race.yaml 의 mpcc: 가 덮어쓴다).

  mpcc_node, OCP 워커, 오프라인 시험 (offline.py), 시뮬레이터가 같은 파일을 같은 규칙으로 읽는다.
  여기 적힌 기본값은 파일에 그 키가 없을 때만 쓴다 — 바꿀 때는 mpcc.yaml 을 고친다.

    cfg = MpccConfig.from_yaml('config/mpcc.yaml', 'race_override.yaml')
    cfg = MpccConfig.from_params(ros_params_dict)       # mpcc_node → 워커

  ★ 표시는 생성 코드에 들어간다 — 바꾸면 다시 빌드한다 (약 20초, acados_solver 가 알아서).
"""
from dataclasses import asdict, dataclass, fields

AY_OFF = 100.0      # 횡가속도 제약을 끈 것과 같은 경계 m/s² (빌드 초기값, 상한 0 일 때)


@dataclass
class MpccConfig:
    # ── 예측 구간 ──────────────────────────────────────────────────────
    N: int = 60                 # ★ 스테이지 수
    dt: float = 0.033           # ★ 스테이지 간격 s
    rk_steps: int = 4           # ★ 구간당 RK4 횟수 (저속 안정성 — ocp/README.md)
    qp_warm_start: int = 1      # ★ HPIPM 을 직전 QP 해에서 시작 (0 = 끔, race_mpc 는 1)

    # ── 입력 경계 ──────────────────────────────────────────────────────
    v_max: float = 0.7          # 바퀴 속도 명령 상한 m/s (실행 중에 상대 차 뒤따르기로 스테이지별로 줄인다)
    v_min: float = 0.01         # 바퀴 속도 명령 하한 m/s
    max_steer_actual: float = 0.0     # 실제 조향 상한 rad. 0 이면 steer_map_act 의 끝값 (조향 맵)
    ws_acc_max: float = 0.0     # ★ 바퀴 가속도 한계 m/s² (0 = 끔)
    ws_dec_max: float = 0.0     # ★ 바퀴 감속도 한계 m/s² (0 = 끔)
    ws_rate_slack: float = 1000.0      # ★ 위 한계를 넘는 양의 L1 벌점
    # 횡가속도 |vx·r| 상한 m/s² (0 = 끔). 실차는 ay_max, 시뮬 (sim_mode) 은 ay_max_sim. 값은 실행 중에 넣는다
    #   (생성 코드에는 제약이 있는지만 — 둘 중 하나라도 > 0 이면 들어간다)
    ay_max: float = 3.5
    ay_max_sim: float = 6.0
    ay_slack: float = 1000.0    # ★ 넘는 양의 L1 벌점
    sim_mode: bool = False      # 시뮬로 띄울 때만 true (launch source:=sim, env/racing.py)

    # ── 트랙 경계 ──────────────────────────────────────────────────────
    track_hard: bool = True     # ★ true = |n| 상태 경계 (실행 중 스테이지별), false = 배리어 비용
    track_bounds: str = 'walls'  # walls = 트랙 파일의 지점별 벽 − wall_margin, fixed = 선에서 ±track_width
    wall_margin: float = 0.22
    track_width: float = 0.12   # fixed 일 때 (배리어면 비용의 폭)
    track_slack: float = 1000.0  # ★ 경계를 넘는 양의 L1 벌점 (0 = 순수 하드)

    # ── 비용 ★ — 성능 (무엇을 잘하게) ──────────────────────────────────
    cost: str = 'mpcc'          # mpcc (진행 −s 최대화) | tracking (목표 속도 추종, 예전 간이 비용)
    q_progress: float = 1.0     # 진행 (mpcc −s, tracking −ṡ)
    q_n: float = 0.02           # 기준 라인 횡방향 오차 n² (논문 값 — MPCC. 키우면 센터라인 추종)
    q_mu: float = 0.002         # 기준 라인 헤딩 오차 mu² (논문 값)
    q_mu_e: float = 5.0         # 끝 스테이지 헤딩 오차 mu²
    v_target: float = 1.2       # tracking: 목표 속도 (가중치 q_speed_diff)

    # ── 비용 ★ — 정규화 (상태·입력이 무리 없이) ────────────────────────
    q_delta: float = 2.0        # 조향 입력 변화 (명령 − 지금 조향)²
    q_speed_diff: float = 2.0   # 바퀴 입력 변화 (명령 − 지금 바퀴)²
    q_beta: float = 1.0         # 슬립각 (운동학 − 동역학)²
    q_slip: float = 0.1         # 슬립률 (바퀴 − v_x)²
    q_barrier: float = 1.0      # 트랙 배리어 (track_hard false 일 때)
    barrier_lambda: float = 200.0   # 배리어 날카로움
    q_e_scaler: float = 1.0     # 끝 비용 배율 (상태 항 × 이것)

    # 생성 코드의 입력 경계 초기값 (실행 중에 v_max, max_steer_actual 로 다시 넣는다)
    build_v_max: float = 7.0    # ★
    build_steer: float = 0.5    # ★

    def ay_on(self):
        """생성 코드에 횡가속도 제약을 넣나."""
        return self.ay_max > 0 or self.ay_max_sim > 0

    def ay_limit(self):
        """지금 쓸 횡가속도 상한 (0 = 끔)."""
        return self.ay_max_sim if self.sim_mode else self.ay_max

    @classmethod
    def names(cls):
        return [f.name for f in fields(cls)]

    @classmethod
    def from_params(cls, d, **over):
        """ROS 파라미터 / yaml 사전에서 아는 이름만. max_steer_actual 이 없으면 steer_map_act 끝값."""
        kw = {}
        for f in fields(cls):
            if f.name in d and d[f.name] is not None and d[f.name] != '':
                kw[f.name] = f.type(d[f.name]) if f.type in (int, float, bool, str) else d[f.name]
        if kw.get('max_steer_actual', 0.0) <= 0.0:
            kw['max_steer_actual'] = float(list(d['steer_map_act'])[-1]) if d.get('steer_map_act') else 0.3776
        if kw.get('cost') == 'author':          # 예전 이름
            kw['cost'] = 'mpcc'
        kw.update(over)
        return cls(**kw)

    @classmethod
    def from_yaml(cls, *paths, **over):
        return cls.from_params(read_params(*paths), **over)

    def to_dict(self):
        return asdict(self)

    def build_key(self):
        """생성 코드에 들어가는 값만 (★). acados_solver 가 이것으로 다시 빌드할지 정한다."""
        skip = {'v_max', 'v_min', 'max_steer_actual', 'track_bounds', 'wall_margin', 'track_width',
                'ay_max', 'ay_max_sim', 'sim_mode'}
        return tuple((k, v) for k, v in sorted(self.to_dict().items()) if k not in skip) + \
            (('track_width', self.track_width),) * (not self.track_hard) + (('ay_on', self.ay_on()),)


def read_params(*paths):
    """mpcc.yaml 형식 (/**: ros__parameters: ...) 파일들을 차례로 덮어 하나의 사전으로."""
    import yaml
    d = {}
    for path in paths:
        if path:
            y = yaml.safe_load(open(path, encoding='utf-8')) or {}
            d.update((y.get('/**') or next(iter(y.values()), {})).get('ros__parameters', {}))
    return d
