"""
racing — 튜닝 시뮬 환경 (race_mpc env/racing.py 자리). main.py · main_sim.py 가 같이 쓴다.

  params, track, ctrl, sim = setup(args)     설정 → 트랙 → 차 모델 → MPC (빌드) → 시뮬레이터
  cmd, tick = control_tick(ctrl, sim)       제어 한 주기 (MPC 풀이 → /drive 명령) + 기록
  info = sim.step(*cmd)                     차 (ODE) 한 적분 주기

  설정 우선순위: config/mpcc.yaml → race.yaml 의 mpcc: → --config 파일들 → --set (→ 대화형 extra)
  모르는 이름은 받지 않는다 (race.sh 와 같은 규칙).
"""
import os

import numpy as np

from mlcs_mpc.env.config import read_params
from mlcs_mpc.env.mpc.core import command_to_steer
from mlcs_mpc.env.mpc.mpcc import MpccController
from mlcs_mpc.env.simulator import Simulator
from mlcs_mpc.env.track import Track, find
from mlcs_mpc.env.vehicle_model import VehicleModel
from mlcs_mpc.env.vesc import Vesc

# env/ → mlcs_mpc/ → src/mlcs_mpc/ (패키지, config/) → src/ → 저장소 (race.yaml)
PKG = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
REPO = os.path.abspath(os.path.join(PKG, '..', '..'))
CFG = os.path.join(PKG, 'config', 'mpcc.yaml')
RACE = os.path.join(REPO, 'race.yaml')
CAR_KEYS = ('mass', 'friction', 'steering_tau', 'wheel_speed_tau')


def parse_kv(items):
    import yaml
    out = {}
    for it in items or []:
        k, _, v = it.partition('=')
        out[k.strip()] = yaml.safe_load(v)
    return out


def load_params(args, extra=None):
    """mpcc.yaml → race.yaml mpcc: → --config → --set → extra. race.sh 처럼 모르는 이름은 거부한다."""
    import yaml
    p = read_params(CFG)
    known = set(p)
    race = {}
    if args.race and os.path.isfile(args.race):
        race = yaml.safe_load(open(args.race, encoding='utf-8')) or {}
    over = dict(race.get('mpcc') or race.get('hmpc') or {})
    for f in args.config or []:
        over.update(read_params(f))
    over.update(parse_kv(args.set))
    over.update(extra or {})
    bad = [k for k in over if k not in known]
    if bad:
        raise KeyError(f'모르는 이름: {", ".join(bad)} (config/mpcc.yaml 에 있는 이름만)')
    for k, v in over.items():
        if isinstance(p[k], float) and isinstance(v, int) and not isinstance(v, bool):
            v = float(v)
        p[k] = v
    track = args.track or str(race.get('track') or '') or os.path.join(REPO, p['track_csv'])
    p['repo_root'] = REPO
    p['sim_mode'] = True        # 여기는 시뮬레이터 (main.py, main_sim.py) — 시뮬용 한계 (ay_max_sim)
    return p, over, find(track)


def setup(args, extra=None, car_extra=None, quiet=False):
    """설정 → 트랙 → 차 모델 → MPC → 시뮬레이터 → (params, track, ctrl, sim).

    args 는 add_args (main.py) 의 이름들을 가진 것 — track, race, config, set, car, start_s, vesc, build_dir.

    extra / car_extra = 대화형 튜닝에서 덧붙이는 값 (MPC·차 모두 / 차만). 생성 코드에 들어가는 값이
    바뀌었을 때만 다시 빌드한다 (acados_solver 가 캐시를 본다).
    """
    params, over, track_csv = load_params(args, extra)
    track = Track(track_csv)
    mpc_model = VehicleModel(**{k: params[k] for k in ('mass', 'friction', 'vehicle_params', 'tire_front',
                                                      'tire_rear', 'steering_tau', 'wheel_speed_tau')})
    car_over = parse_kv(args.car)
    car_over.update(car_extra or {})
    car_model = mpc_model.copy(**{k: float(v) for k, v in car_over.items() if k in CAR_KEYS})
    vesc = Vesc.load(args.vesc) if args.vesc else Vesc.load()
    ctrl = MpccController(mpc_model, track, params, args.build_dir, log=(lambda m: None) if quiet else print)
    sim = Simulator(car_model, Track(track_csv), dt=0.01,
                    steer_map_cmd=params['steer_map_cmd'], steer_map_act=params['steer_map_act'],
                    ws_acc=float(car_over.get('ws_acc_max', params['ws_acc_max'])),
                    ws_dec=float(car_over.get('ws_dec_max', params['ws_dec_max'])), vesc=vesc)
    sim.reset_to_s(args.start_s)
    if quiet:
        return params, track, ctrl, sim
    cfg = ctrl.cfg
    print(f'■ 트랙 {os.path.basename(track_csv)}  길이 {track.length:.2f} m')
    print(f'■ MPC  N {cfg.N}  dt {cfg.dt}  v_max {cfg.v_max}  비용 {cfg.cost}  경계 {cfg.track_bounds} '
          f'(여유 {cfg.wall_margin})  제어 {params["control_rate"]:.0f} Hz')
    changed = {k: v for k, v in over.items() if read_params(CFG).get(k) != v}
    if changed:
        print(f'  mpcc.yaml 과 다른 값 {changed}')
    if car_over:
        print(f'  ★ 차만 다른 값 {car_over}')
    print(f'■ VESC {vesc.source}  모터 상한 {vesc.v_motor:.2f} m/s')
    return params, track, ctrl, sim


def control_tick(ctrl, sim):
    """한 제어 주기: MPC 풀이 → 명령. 입력이 차까지 어떻게 들어가는지와 실제 상태를 같이 기록한다.

    tick = dict(t, status, ms, u=(바퀴 속도 ref, 실제 조향 ref) — MPC 해,
                cmd=(속도, 서보 조향 명령) — /drive, applied=(속도, 실제 조향 ref) — VESC·조향 맵을 지난 뒤 차가 받는 값,
                x=지금 실제 OCP 상태 (바퀴·조향은 시뮬 실제값), x1=MPC 가 예측한 다음 주기 상태)
    """
    speed, steer, r = ctrl.step(sim.pose(), sim.twist(), sim.wheel_speed, sim.t)
    x = np.array(r['x'], dtype=float)
    x[3], x[4] = sim.x[3], sim.x[4]               # v_floor 로 올리기 전의 실제 vx, 무게중심 vy
    x[7], x[8] = sim.wheel_speed, sim.steer       # 노드는 조향을 못 재서 예측값을 쓴다 — 여기서는 실제값
    sc = sim.vesc.steer(steer) if sim.vesc is not None else steer
    applied = (sim.vesc.speed(speed) if sim.vesc is not None else speed,
               command_to_steer(sc, sim.map_cmd, sim.map_act, sim.max_steer))
    tick = dict(t=sim.t, status=r['status'], ms=r['ms'], u=np.array(r['u'], float), cmd=(speed, steer),
                applied=applied, x=x, x1=np.array(r['x1'], float), tripped=r.get('tripped', False),
                terms=r.get('terms'),
                lap=len(sim.laps))
    return (steer, speed), tick
