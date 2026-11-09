"""
acados_solver — acados_ocp 의 OCP 에 솔버 옵션을 붙여 생성 코드를 만들고 (빌드), 이미 있으면 다시 쓴다.

  생성 코드는 build_dir/<태그>_trk<트랙 해시>_<설정 해시>/ 에 남는다 (설정마다 따로 — 바꿨다 되돌려도 다시 빌드 안 함). 트랙마다 따로 둔다 (곡률이 코드에 들어간다).
  이 파일들·차 모델·설정의 ★ 값·트랙 중 하나라도 바뀌면 다시 빌드한다 (첫 빌드 약 20초).
"""
import glob
import hashlib
import os
import time

import numpy as np
from acados_template import AcadosOcpSolver

from mlcs_mpc.env.mpc.acados_ocp import build_ocp


def set_solver_options(ocp, cfg):
    """원본 (HyperMPC mpc_formulation_car.py) 과 같은 조합 + QP warm start (config.qp_warm_start)."""
    o = ocp.solver_options
    o.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    o.hpipm_mode = 'BALANCE'
    o.hessian_approx = 'EXACT'
    o.regularize_method = 'PROJECT'
    o.globalization = 'MERIT_BACKTRACKING'
    o.integrator_type = 'ERK'
    o.sim_method_num_stages = 4
    o.sim_method_num_steps = cfg.rk_steps
    o.nlp_solver_type = 'SQP_RTI'
    o.qp_solver_warm_start = int(cfg.qp_warm_start)    # 직전 QP 해에서 시작 (2026-11-09 켬)


def _stamp(model, track_grid, cfg):
    import mlcs_mpc.env.mpc.acados_ocp as ao
    import mlcs_mpc.env.vehicle_model as vm
    src = b''.join(open(f, 'rb').read() for f in (__file__, ao.__file__, vm.__file__))
    baked = (model.mass, model.L, model.steering_tau, model.wheel_speed_tau, model.slip_eps)
    grid = b''.join(np.ascontiguousarray(g, dtype=float).tobytes() for g in track_grid)
    return hashlib.md5(src + grid + repr((cfg.build_key(), baked)).encode()).hexdigest()


def make_solver(model, track_grid, cfg, build_dir, log=print):
    """→ AcadosOcpSolver. track_grid = Track.kappa_grid() (s, κ)."""
    grid_id = hashlib.md5(b''.join(np.ascontiguousarray(g, dtype=float).tobytes()
                                   for g in track_grid)).hexdigest()[:8]
    tag = (f'N{cfg.N}_{cfg.cost}_{"guard" if model.slip_eps > 0 else "noguard"}_rk{cfg.rk_steps}'
           + ('_hard' if cfg.track_hard else '')
           + ('_slack' if cfg.track_hard and cfg.track_slack > 0 else '')
           + ('_wsrate' if cfg.ws_acc_max > 0 or cfg.ws_dec_max > 0 else '')
           + f'_trk{grid_id}')
    # ★ 설정 해시를 폴더·라이브러리 이름에 넣는다. 이름이 같으면 한 프로세스에서 다시 빌드해도 처음 불러온
    #   라이브러리가 계속 쓰인다 (dlopen 은 같은 경로를 다시 읽지 않는다 — 대화형 튜닝에서 가중치가 안 바뀌던 원인).
    stamp = _stamp(model, track_grid, cfg)
    tag += f'_{stamp[:8]}'
    build = os.path.join(build_dir, tag)
    os.makedirs(build, exist_ok=True)
    ocp = build_ocp(model, track_grid, cfg, name=f'mpcc_{stamp[:8]}')
    set_solver_options(ocp, cfg)
    ocp.code_export_directory = os.path.join(build, 'c_generated_code')
    json_file = os.path.join(build, 'ocp.json')
    stamp_file = os.path.join(build, 'source.md5')
    reuse = (os.path.exists(stamp_file) and open(stamp_file).read() == stamp
             and glob.glob(os.path.join(ocp.code_export_directory, 'libacados_ocp_solver_*.so')))
    log(f'■ {"이전 빌드 사용" if reuse else "코드 생성 + 빌드 중"}: {build}')
    t0 = time.perf_counter()
    # 우리 md5 스탬프가 같으면 acados 의 자체 해시 재검사를 건너뛴다. 그 해시가 어긋나
    # '이전 빌드 사용' 인데도 생성 4 s + 빌드 10 s 를 다시 해, 워커 재시작이 ~14 s 걸렸다 (젯슨 cProfile).
    solver = AcadosOcpSolver(ocp, json_file=json_file, build=not reuse, generate=not reuse, verbose=False,
                             check_reuse_possible=not reuse)
    with open(stamp_file, 'w') as f:
        f.write(stamp)
    log(f'  ✓ {time.perf_counter() - t0:.1f}초')
    return solver
