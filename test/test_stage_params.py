#!/usr/bin/env python3
"""
test_stage_params.py — 동역학 파라미터 P 가 호라이즌 스테이지마다 따로 들어가는가 (acados 빌드 필요, 이 PC 약 4초).

  python3 test/test_stage_params.py

  ① 스테이지마다 다른 P (k 가 커질수록 앞·뒤 타이어 Dy 를 1 → 0.3 배) 를 주면 솔버 안 p 가 스테이지별로 그대로인가
  ② 그 P 가 예측에 실제로 쓰이는가 — 상수 P 와 비교해 예측 궤적이 달라진다 (뒤 스테이지일수록)
  ③ 모양이 (N+1, 30) 이 아니면 받지 않는다
"""
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.utils.runenv import ensure_acados  # noqa: E402

ensure_acados()
from mlcs_mpc.env.config import MpccConfig  # noqa: E402
from mlcs_mpc.env.mpc import param_model as pmod  # noqa: E402
from mlcs_mpc.env.mpc.core import make_state  # noqa: E402
from mlcs_mpc.env.mpc.mpcc import MPCC  # noqa: E402
from mlcs_mpc.env.track import Track, find  # noqa: E402
from mlcs_mpc.env.vehicle_model import VehicleModel  # noqa: E402

CFG = os.path.join(ROOT, 'src/mlcs_mpc/config/mpcc.yaml')
DY = (14, 24)       # 앞·뒤 타이어 Dy (가로 힘 최대값)


class RampParams(pmod.ConstantParams):
    """스테이지 k 마다 타이어 Dy × (1 − 0.7·k/N) — 뒤로 갈수록 미끄러운 길."""
    name = 'ramp'

    def __init__(self, params, N):
        super().__init__(params, N)
        f = 1.0 - 0.7 * np.arange(N + 1) / N
        for i in DY:
            self.P[:, i] *= f


def solve(mpc, track, model, cfg):
    s0, v = 5.0, 2.0                        # 큰 코너 (s 6.1~) 로 들어가는 곳 — 가로 힘을 쓴다
    x, y = track.xy(s0, 0.0)
    h = track.heading_at(s0)
    lr = model.lr
    pose = (float(x) - lr * np.cos(h), float(y) - lr * np.sin(h), h)
    xs = make_state(track, pose, (v, 0.0, 0.0), v, 0.0, lr, 0.25, model.friction)
    lo, hi = track.bounds_at(xs[0] + v * cfg.dt * np.arange(cfg.N + 1), cfg.wall_margin)
    for _ in range(5):                      # RTI 몇 번 — 해를 수렴시킨다
        r = mpc.solve(xs, 0.0, lo, hi, reset=_ == 0)
    track.reset()
    return r


def test_stage_params():
    track = Track(find('comp1_track'))
    model = VehicleModel.from_yaml(CFG)
    cfg = MpccConfig.from_yaml(CFG, v_max=2.5)
    build = os.path.expanduser('~/mpcc_build')
    const = MPCC(model, track, cfg, build, log=lambda m: None)
    ramp = MPCC(model, track, cfg, build, param_model=RampParams(model.p, cfg.N), log=lambda m: None)
    rc, rr = solve(const, track, model, cfg), solve(ramp, track, model, cfg)
    assert rc['status'] == 0 and rr['status'] == 0, (rc['status'], rr['status'])

    # ① 솔버 안 값
    pin_c, pin_r = const.stage_params_in_solver(), ramp.stage_params_in_solver()
    assert pin_c.shape == (cfg.N + 1, 30)
    assert np.abs(pin_c - model.p).max() == 0.0
    assert np.abs(pin_r - ramp.pm.P).max() == 0.0
    print(f'  ① 상수: 61 스테이지 모두 mpcc.yaml 값    ramp: 스테이지별로 넣은 값 그대로')
    for k in (0, 15, 30, 45, 60):
        print(f'     스테이지 {k:2d}  앞 Dy {pin_r[k, DY[0]]:.4f}  뒤 Dy {pin_r[k, DY[1]]:.4f}   (상수 {pin_c[k, DY[0]]:.4f} / {pin_c[k, DY[1]]:.4f})')

    # ② 예측이 달라지나
    xy_c = np.array(track.xy(rc['xs'][:, 0], rc['xs'][:, 1])).T
    xy_r = np.array(track.xy(rr['xs'][:, 0], rr['xs'][:, 1])).T
    d = np.hypot(*(xy_c - xy_r).T)
    print('  ② 예측 위치 차 (상수 − ramp): ' + '  '.join(f'k{k} {d[k] * 100:.1f}cm' for k in (5, 15, 30, 45, 60)))
    print(f'     끝 속도 상수 {rc["xs"][-1, 3]:.2f} / ramp {rr["xs"][-1, 3]:.2f} m/s')
    assert d.max() > 0.01 and d[-1] > d[5], np.round(d, 4)     # 뒤 스테이지일수록 차이가 커야 한다

    # ③ 모양 검사
    class Bad(pmod.ConstantParams):
        def stage_params(self, t, x):
            return self.P[0], 0.0           # 한 줄만
    const.pm = Bad(model.p, cfg.N)
    try:
        solve(const, track, model, cfg)
        raise AssertionError('한 줄 P 를 받았다')
    except ValueError as e:
        print(f'  ③ 한 줄 P 는 받지 않음: {e}')


if __name__ == '__main__':
    test_stage_params()
    print('1개 통과')
