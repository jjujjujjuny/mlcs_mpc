"""
bounds — 실행 중에 스테이지별로 넣는 경계와 초기 추측. 워커와 오프라인 시험이 같이 쓴다.
"""
import numpy as np

from mlcs_mpc.env.config import AY_OFF


def hard_bounds(N, w, n0, relax=10):
    """스테이지 1..N 의 |n| 상한.

    현재 |n| 이 이미 w 를 넘었으면 첫 스테이지부터 w 를 요구하면 해가 없다 (한 스텝 0.033 s
    에 못 돌아온다). 그래서 relax 스테이지 동안 현재 값에서 w 로 줄여 간다.
    """
    k = np.arange(1, N + 1)
    return np.maximum(w, abs(n0) * np.clip(1.0 - k / relax, 0.0, 1.0)) + 1e-3


def set_hard_bounds(solver, N, w, n0):
    for k, b in zip(range(1, N + 1), hard_bounds(N, w, n0)):
        solver.constraints_set(k, 'lbx', np.array([-b]))
        solver.constraints_set(k, 'ubx', np.array([+b]))


def wall_bounds(N, lo, hi, n0, relax=10):
    """스테이지 1..N 의 n 범위 [lo_k, hi_k] (MPCC: 지점별 벽 폭 − 여유).

    lo, hi 는 스테이지 0..N 의 값 (길이 N+1). 현재 n0 가 범위 밖이면 hard_bounds 처럼 relax
    스테이지 동안 n0 에서 범위로 줄여 간다. 좁아서 lo > hi 면 가운데 한 점으로 둔다.
    """
    lo, hi = np.asarray(lo, float)[1:N + 1].copy(), np.asarray(hi, float)[1:N + 1].copy()
    bad = lo > hi
    lo[bad] = hi[bad] = 0.5 * (lo[bad] + hi[bad])
    f = np.clip(1.0 - np.arange(1, N + 1) / relax, 0.0, 1.0)
    return np.minimum(lo, n0 * f) - 1e-3, np.maximum(hi, n0 * f) + 1e-3


def set_wall_bounds(solver, N, lo, hi, n0):
    for k, a, b in zip(range(1, N + 1), *wall_bounds(N, lo, hi, n0)):
        solver.constraints_set(k, 'lbx', np.array([a]))
        solver.constraints_set(k, 'ubx', np.array([b]))


def set_input_bounds(solver, N, cfg, v_cap=None):
    """바퀴 속도 [v_min, v_max] (v_cap 이 있으면 스테이지별 상한), 실제 조향 ±max_steer_actual,
    횡가속도 |vx·r| ≤ cfg.ay_limit() (제약이 빌드돼 있을 때, 0 이면 끈 것과 같다)."""
    ay = (cfg.ay_limit() or AY_OFF) if cfg.ay_on() else None
    for k in range(N):
        v = cfg.v_max if v_cap is None else max(float(v_cap[k]), cfg.v_min)
        solver.constraints_set(k, 'lbu', np.array([cfg.v_min, -cfg.max_steer_actual]))
        solver.constraints_set(k, 'ubu', np.array([v, cfg.max_steer_actual]))
        if ay is not None:
            solver.constraints_set(k, 'lh', np.array([-ay]))
            solver.constraints_set(k, 'uh', np.array([+ay]))


def restart(solver, N, x0, cfg=None):
    """초기 추측을 x0 로 깐다. 0 으로 두면 v_x = 0 이라 슬립 분모가 eps 가 된다."""
    solver.reset()
    for k in range(N + 1):
        solver.set(k, 'x', x0)
    ws = x0[7] if cfg is None else float(np.clip(x0[7], cfg.v_min, cfg.v_max))
    for k in range(N):
        solver.set(k, 'u', np.array([ws, 0.0]))
    if cfg is not None:     # reset 이 경계까지 지우지는 않지만 다시 넣어 둔다
        set_input_bounds(solver, N, cfg)
