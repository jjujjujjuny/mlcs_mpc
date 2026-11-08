#!/usr/bin/env python3
"""test_opponent.py — opponent 시험 (ROS·acados 불필요).  python3 test/test_opponent.py

마지막 두 시험은 솔버 대신 간이 차 (통로 가운데로 가는 Frenet 운동학) 로 돌리는 폐루프다.
MPC 동역학은 보지 않는다. 측면 결정·통로·뒤따르기 로직이 끝까지 맞물리는지만 본다.
"""
import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
from mlcs_mpc.hmpc_core import Track  # noqa: E402
from mlcs_mpc.opponent import (FREE, LEFT, RIGHT, TRAIL, OppParams, Opponent,  # noqa: E402
                               OpponentModel, OpponentTracker, PeriodicGP, SideSelector,
                               apply_corridor, plan, trail_cap, wrap_s)

N, DT = 60, 0.033


def circle(R=3.0, width=1.4, n=300):
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'prep.csv')
    a = np.arange(n) * 2 * np.pi / n
    np.savetxt(p, np.c_[R * a, R * np.cos(a), R * np.sin(a), a + np.pi / 2, 0 * a + 1 / R, 0 * a + width],
               delimiter=',', header='s,x,y,heading,curvature,track_width', comments='')
    return Track(p), R


def xy_on(R, s, d):
    """반시계 원 위 (s, d) → (x, y, yaw). d 는 왼쪽(안쪽) +."""
    a = s / R
    r = R - d
    return r * math.cos(a), r * math.sin(a), a + math.pi / 2


def test_wrap():
    assert abs(wrap_s(9.0, 10.0) + 1.0) < 1e-12
    assert abs(wrap_s(-9.0, 10.0) - 1.0) < 1e-12
    assert abs(wrap_s(0.3, 10.0) - 0.3) < 1e-12


def test_track_xy():
    t, R = circle()
    for s, d in ((0.3, 0.2), (5.0, -0.4), (t.length - 0.05, 0.1)):
        x, y = t.xy(s, d)
        ex, ey, _ = xy_on(R, s, d)
        assert math.hypot(x - ex, y - ey) < 0.01, (s, d)


def test_tracker():
    t, R = circle()
    tr = OpponentTracker(t, OppParams())
    for i in range(200):
        tt = i * 0.01
        tr.update(tt, *xy_on(R, 1.0 + 1.5 * tt, 0.1))
    s, d, vs, vd = tr.state(1.99)
    assert abs(vs - 1.5) < 0.05, vs
    assert abs(d - 0.1) < 0.01 and abs(vd) < 0.05
    assert tr.state(1.99 + 0.5) is None        # 오래됨
    # 트랙 객체를 복사해서 쓴다: 원래 트랙의 연속 s 상태를 건드리지 않는다
    assert t.s_cont is None


def test_gp_periodic():
    L = 10.0
    x = np.arange(0, L, 0.1)
    y = 0.2 * np.sin(2 * np.pi * x / L * 2) + 0.005 * np.random.default_rng(0).standard_normal(len(x))
    g = PeriodicGP(L).fit(x, y)
    mu, sd = g.predict(np.array([0.05, 2.5, L - 0.01]))
    assert abs(mu[1] - 0.2 * np.sin(np.pi)) < 0.03
    assert abs(mu[0] - mu[2]) < 0.03           # 랩 이음매가 이어진다
    assert sd.max() < 0.05
    # 데이터 없는 구간은 표준편차가 커진다
    g2 = PeriodicGP(L).fit(x[x < 6], y[x < 6])
    _, sd2 = g2.predict(np.array([3.0, 8.0]))
    assert sd2[1] > 3 * sd2[0], sd2


def test_zero_shot_corner_speed():
    t, R = circle(R=2.0)
    m = OpponentModel(t, OppParams())
    pr = m.predict(0.0, 0.1, 3.0, 0.0, N, DT)
    vlim = math.sqrt(OppParams().a_lat * R)
    assert not pr.learned
    assert abs(pr.v[0] - vlim) < 1e-6 and abs(pr.s[-1] - vlim * N * DT) < 1e-6
    assert np.all(np.diff(pr.sig_d) >= 0) and pr.sig_d.max() <= 0.15 + 1e-12
    assert pr.sig_d[0] == 0.0


def test_lateral_damped_and_clipped():
    t, R = circle(width=1.0)
    m = OpponentModel(t, OppParams())
    pr = m.predict(0.0, 0.1, 1.0, 1.0, N, DT)       # 왼쪽으로 1 m/s
    assert pr.d[-1] <= 0.5 - 0.15 + 1e-9             # 벽 안으로 자른다
    assert np.all(np.diff(pr.d) >= -1e-12)


def test_learned_line():
    t, R = circle()
    p = OppParams()
    m = OpponentModel(t, p)
    line = lambda s: 0.25 * math.sin(2 * math.pi * s / t.length)  # noqa: E731
    assert not m.refit()
    for s in np.arange(0, t.length * 0.95, 0.02):
        m.observe(s, line(s), 1.2)
    assert m.refit()
    pr = m.predict(1.0, line(1.0), 1.2, 0.0, N, DT)
    assert pr.learned
    err = np.abs(pr.d - [line(s) for s in pr.s])
    assert err.max() < 0.03, err.max()
    assert abs(pr.v[-1] - 1.2) < 0.05


def _pred_const(t, s0, d, v):
    m = OpponentModel(t, OppParams())
    return m.predict(s0, d, v, 0.0, N, DT)


def test_corridor_left_and_only_hit_stages():
    t, R = circle(width=1.6)
    p = OppParams()
    pr = _pred_const(t, 1.0, -0.25, 1.0)
    s_ego = 0.0 + 2.5 * DT * np.arange(N + 1)
    lo, hi = t.bounds_at(s_ego, 0.22)
    pl = plan(s_ego, 0.0, lo, hi, pr, t.length, p)
    assert pl.hit.any() and not pl.hit.all() and not pl.alongside
    assert pl.gap_left > pl.gap_right
    sel = SideSelector(p)
    assert sel.update(pl) == LEFT
    lo2, hi2 = apply_corridor(lo, hi, pr, pl, LEFT, p)
    assert np.all(lo2[pl.hit] >= pr.d[pl.hit] + p.clear - 1e-12)
    assert np.array_equal(lo2[~pl.hit], lo[~pl.hit]) and np.array_equal(hi2, hi)


def test_blocked_trails():
    t, R = circle(width=1.0)
    p = OppParams()
    pr = _pred_const(t, 1.0, 0.0, 1.0)
    s_ego = 0.0 + 2.0 * DT * np.arange(N + 1)
    lo, hi = t.bounds_at(s_ego, 0.22)
    pl = plan(s_ego, 0.0, lo, hi, pr, t.length, p)
    sel = SideSelector(p)
    assert sel.update(pl) == TRAIL
    cap = trail_cap(pl, pr, 3.0, p)
    assert cap.shape == (N,) and np.all(cap >= 0) and abs(cap[0] - (1.0 + 1.0 * (1.0 - 0.8))) < 1e-9


def test_no_switch_alongside():
    t, R = circle(width=1.6)
    p = OppParams()
    sel = SideSelector(p)
    sel.mode = LEFT
    pr = _pred_const(t, 1.0, 0.3, 1.0)               # 상대가 왼쪽으로 옮겨 옴
    s_ego = 1.1 + 1.0 * DT * np.arange(N + 1)
    lo, hi = t.bounds_at(s_ego, 0.22)
    pl = plan(s_ego, 0.5, lo, hi, pr, t.length, p)
    assert pl.alongside and pl.gap_right > pl.gap_left
    for _ in range(30):
        assert sel.update(pl) == LEFT


def test_switch_needs_frames():
    t, R = circle(width=1.8)
    p = OppParams()
    sel = SideSelector(p)
    sel.mode = RIGHT
    pr = _pred_const(t, 2.0, -0.1, 1.0)               # 오른쪽도 되지만 왼쪽이 훨씬 넓다
    s_ego = 0.0 + 2.0 * DT * np.arange(N + 1)
    lo, hi = t.bounds_at(s_ego, 0.22)
    pl = plan(s_ego, 0.0, lo, hi, pr, t.length, p)
    assert pl.gap_right >= 0 and pl.gap_left > pl.gap_right + p.gap_switch
    modes = [sel.update(pl) for _ in range(p.switch_frames)]
    assert modes[:-1] == [RIGHT] * (p.switch_frames - 1) and modes[-1] == LEFT


def _toy_race(width, v_ego, v_opp, d_opp, secs=12.0):
    """간이 폐루프. 우리 차: s' = min(v_ego, cap), n 은 스테이지 1..10 경계 가운데로 1.5 m/s 까지."""
    t, R = circle(width=width)
    p = OppParams()
    opp = Opponent(t, p)
    s, n = 0.0, 0.0
    so = 2.0
    s_pred = None
    min_lat, modes, passed = math.inf, set(), False
    for i in range(int(secs / DT)):
        tt = i * DT
        for j in range(3):          # mocap 은 100 Hz
            tj = tt - DT + (j + 1) * DT / 3
            opp.measure(tj, *xy_on(R, so - v_opp * (DT - (j + 1) * DT / 3), d_opp))
        if s_pred is None:
            s_pred = s + v_ego * DT * np.arange(N + 1)
        lo, hi = t.bounds_at(s_pred, 0.22)
        lo, hi, cap, mode = opp.step(tt, s_pred, n, lo, hi, v_ego, N, DT)
        modes.add(mode)
        v = v_ego if cap is None else min(v_ego, cap[0])
        target = 0.5 * (lo[1:11].mean() + hi[1:11].mean())
        n += np.clip(target - n, -1.5 * DT, 1.5 * DT)
        s += v * DT
        so += v_opp * DT
        s_pred = s + v * DT * np.arange(N + 1)
        ds = float(wrap_s(s - so, t.length))
        if abs(ds) < 0.5:           # 앞뒤로 겹칠 때 횡간격
            min_lat = min(min_lat, abs(n - d_opp))
        if 1.0 < ds < 3.0:
            passed = True
    return passed, min_lat, modes


def test_toy_overtake():
    passed, min_lat, modes = _toy_race(width=1.6, v_ego=2.0, v_opp=1.0, d_opp=-0.25)
    assert passed, modes
    assert min_lat >= 0.40 - 0.02, min_lat
    assert LEFT in modes


def test_toy_trail_when_blocked():
    passed, min_lat, modes = _toy_race(width=1.0, v_ego=2.0, v_opp=1.0, d_opp=0.0)
    assert not passed and TRAIL in modes
    assert min_lat == math.inf, min_lat         # 앞뒤로 0.5 m 안에 들어간 적이 없다


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        f()
        print('ok ', f.__name__)
    print(f'{len(tests)}개 통과')
