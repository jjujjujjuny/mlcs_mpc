#!/usr/bin/env python3
"""
알고리즘 검증 — ROS 없이 순수 파이썬으로 돈다.

    python3 test/test_algorithms.py

README 의 "검증됨" 표가 이 스크립트의 결과다. 알고리즘을 고쳤으면
이걸 돌려서 여전히 통과하는지 확인하세요.

★ 이 테스트는 ROS 연동/실차 동작을 검증하지 않는다. 수학만 본다.
"""

import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'mlcs_mpc'))

from mlcs_mpc.vehicle_model import (          # noqa: E402
    VehicleParams, step_rk4, dynamics, normalize_angle, NX, NU)
from mlcs_mpc.mpc_solver import MPCSolver, MPCConfig    # noqa: E402
from mlcs_mpc.path_manager import PathManager          # noqa: E402

RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  — {detail}" if detail else ''))
    return ok


# ────────────────────────────────────────────────────────────────────
def test_jacobian():
    """해석적 야코비안이 오일러 이산화의 수치미분과 일치하는가."""
    print('\n■ 야코비안')
    p = VehicleParams()
    s = MPCSolver(p, MPCConfig(dt=0.05))
    pv = p.as_vector()
    dt = 0.05

    def num(x, u, eps=1e-7):
        f0 = x + dt * dynamics(x, u, pv)
        A = np.zeros((NX, NX))
        B = np.zeros((NX, NU))
        for i in range(NX):
            d = np.zeros(NX)
            d[i] = eps
            A[:, i] = ((x + d + dt * dynamics(x + d, u, pv)) - f0) / eps
        for i in range(NU):
            d = np.zeros(NU)
            d[i] = eps
            B[:, i] = ((x + dt * dynamics(x, u + d, pv)) - f0) / eps
        return A, B

    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(300):
        x = np.array([rng.uniform(-5, 5), rng.uniform(-5, 5),
                      rng.uniform(-np.pi, np.pi), rng.uniform(0.1, 4.0),
                      rng.uniform(-0.35, 0.35)])
        u = np.array([rng.uniform(-3, 3), rng.uniform(-3, 3)])
        An, Bn = num(x, u)
        Aa, Ba = s._linearize(x, u)
        worst = max(worst, np.abs(An - Aa).max(), np.abs(Bn - Ba).max())
    check('해석적 야코비안 == 수치미분', worst < 1e-5, f'최대오차 {worst:.2e}')


def test_angle():
    print('\n■ 각도 정규화')
    ok = True
    for a, want in [(0.0, 0.0), (np.pi - 0.01, np.pi - 0.01),
                    (3 * np.pi, -np.pi), (-3 * np.pi, -np.pi)]:
        got = normalize_angle(a)
        ok &= abs(normalize_angle(got - want)) < 1e-9
    # 감싸는 구간에서 차이가 작게 나오는가
    d = normalize_angle(3.13 - (-3.13))
    check('normalize_angle', ok and abs(d) < 0.03, f'+3.13 vs -3.13 → {d:.4f} rad')


def test_straight():
    print('\n■ 직선 추종 수렴')
    p = VehicleParams()
    cfg = MPCConfig()
    s = MPCSolver(p, cfg)
    x = np.array([0.0, 0.5, 0.2, 1.0, 0.0])
    for _ in range(150):
        ref = np.array([[x[0] + 0.1 * k, 0.0, 0.0, 2.0]
                        for k in range(cfg.N + 1)])
        u, _ = s.solve(x, ref)
        x = step_rk4(x, u, p.as_vector(), cfg.dt)
        x[4] = np.clip(x[4], -p.max_steer, p.max_steer)
    check('횡오차 → 0', abs(x[1]) < 0.02, f'y={x[1]:.2e} m')
    check('속도 → 목표', abs(x[3] - 2.0) < 0.05, f'v={x[3]:.4f} m/s')


def test_circle_path():
    print('\n■ 경로 관리 (원)')
    R = 2.0
    th = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    import tempfile
    f = os.path.join(tempfile.gettempdir(), '_mlcs_circle.csv')
    np.savetxt(f, np.c_[R * np.cos(th), R * np.sin(th)], delimiter=',')

    path = PathManager(f, target_speed=2.0, closed_loop=True,
                       params=VehicleParams())
    check('로드', path.ok)
    check('총 길이', abs(path.total_s - 2 * np.pi * R) < 0.05,
          f'{path.total_s:.3f} vs {2*np.pi*R:.3f} m')
    check('곡률', abs(np.abs(path.kappa).mean() - 1 / R) < 0.01,
          f'{np.abs(path.kappa).mean():.4f} vs {1/R:.4f}')

    worst = 0.0
    for a in np.linspace(0, 2 * np.pi, 17):
        st = np.array([R * np.cos(a), R * np.sin(a), a + np.pi / 2, 2.0, 0.0])
        ref = path.reference(st, 20, 0.05)
        worst = max(worst, np.abs(np.hypot(ref[:, 0], ref[:, 1]) - R).max())
    check('참조점이 경로 위에', worst < 0.02, f'최대 {worst*100:.2f} cm')

    # 폐곡선 감김 — 시작점 부근에서 참조가 점프하지 않아야 한다
    st = np.array([R * math.cos(-0.05), R * math.sin(-0.05),
                   -0.05 + np.pi / 2, 3.0, 0.0])
    ref = path.reference(st, 20, 0.05)
    gaps = np.hypot(np.diff(ref[:, 0]), np.diff(ref[:, 1]))
    check('폐곡선 이음매 연속', gaps.max() < 0.3,
          f'최대 간격 {gaps.max():.4f} m')
    os.remove(f)


def test_circle_fit():
    print('\n■ 캘리브레이션 원 피팅')
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'src', 'mlcs_mpc', 'mlcs_mpc',
                            'calibrate.py')).read()
    ns = {'np': np, 'math': math}
    exec(src[src.index('def fit_circle'):src.index('def main(')], ns)
    fit = ns['fit_circle']

    rng = np.random.default_rng(3)
    ok = True
    detail = []
    for cx0, cy0, R0 in [(0, 0, 2.0), (1.5, -2.3, 0.9), (-4, 7, 5.5)]:
        t = np.linspace(0, 1.4, 200)      # 부분 원호 (실측 상황)
        x = cx0 + R0 * np.cos(t) + rng.normal(0, 0.001, 200)
        y = cy0 + R0 * np.sin(t) + rng.normal(0, 0.001, 200)
        _, _, R = fit(x, y)
        ok &= abs(R - R0) < 0.01
        detail.append(f'{R0}→{R:.4f}')
    check('부분 원호에서 반경 복원', ok, ', '.join(detail))


def test_kalman():
    print('\n■ 속도 추정 칼만 필터')
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'src', 'mlcs_mpc', 'mlcs_mpc',
                            'mocap_bridge.py')).read()
    ns = {'np': np}
    exec(src[src.index('class PoseKF'):src.index('class MocapBridge')], ns)
    PoseKF = ns['PoseKF']

    dt = 0.01
    T = 600
    v_true = 2.0
    pos = v_true * np.arange(T) * dt
    rng = np.random.default_rng(1)
    meas = pos + rng.normal(0, 0.001, T)     # 1mm 노이즈

    kf = PoseKF()
    v_kf = np.array([kf.update(z, dt)[1] for z in meas])
    v_diff = np.gradient(meas, dt)

    s = 200
    ratio = v_diff[s:].std() / v_kf[s:].std()
    bias = abs(v_kf[s:].mean() - v_true)
    check('노이즈 감소', ratio > 3, f'단순차분 대비 {ratio:.1f}배')
    check('편향 없음', bias < 0.05, f'{bias:.4f} m/s')


def test_lap():
    print('\n■ Levine 트랙 한 바퀴 (종합)')
    wp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', 'src', 'mlcs_mpc', 'waypoints',
                      'levine_centerline.csv')
    if not os.path.isfile(wp):
        check('웨이포인트 존재', False, wp)
        return

    p = VehicleParams(max_speed=3.0, max_accel=2.0, max_decel=3.0)
    cfg = MPCConfig(N=20, dt=0.05)
    s = MPCSolver(p, cfg)
    path = PathManager(wp, target_speed=2.0, closed_loop=True, params=p)

    x0 = path.xy[0]
    x = np.array([x0[0] + 0.15, x0[1] - 0.1, path.heading[0] + 0.1, 0.5, 0.0])
    ts, errs = [], []
    for _ in range(1200):
        ref = path.reference(x, cfg.N, cfg.dt)
        u, _ = s.solve(x, ref)
        x = step_rk4(x, u, p.as_vector(), cfg.dt)
        x[4] = np.clip(x[4], -p.max_steer, p.max_steer)
        x[3] = np.clip(x[3], p.min_speed, p.max_speed)
        ts.append(s.last_solve_ms)
        errs.append(np.hypot(path.xy[:, 0] - x[0], path.xy[:, 1] - x[1]).min())
    ts, errs = np.array(ts), np.array(errs)

    check('경로 추종', errs.mean() < 0.10,
          f'평균 {errs.mean()*100:.1f}cm, 최대 {errs.max()*100:.1f}cm')
    check('실시간성 (20Hz=50ms)', np.percentile(ts, 95) < 50,
          f'평균 {ts.mean():.1f}ms, p95 {np.percentile(ts,95):.1f}ms')


def test_stanley():
    """Stanley 제어법칙이 경로로 수렴하는가 (ROS 없이 수식만).

    실차 조건(mocap 1mm 노이즈 + 60ms 액추에이터 지연)을 흉내낸다.
    """
    print('\n■ Stanley 경로 추종')
    wp = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', 'src', 'mlcs_mpc', 'waypoints', 'track_5.csv')
    if not os.path.isfile(wp):
        check('track_5 웨이포인트 존재', False, wp)
        return

    L, MAXD, DT = 0.33, 0.36, 0.02
    p = PathManager(wp, target_speed=1.0, closed_loop=True,
                    params=VehicleParams(wheelbase=L, max_steer=MAXD))
    check('트랙 로드', p.ok, f'{len(p.xy)}점 {p.total_s:.2f}m')

    # ★ 차량이 물리적으로 돌 수 있는 트랙인가
    R_car = L / math.tan(MAXD)
    r_min = 1.0 / np.abs(p.kappa).max()
    check('주행 가능 (최소반경 > 차량한계)', r_min > R_car,
          f'트랙 {r_min:.2f}m > 차량 {R_car:.2f}m')

    rng = np.random.default_rng(0)
    k_e, k_soft = 1.5, 0.5
    px, py = p.xy[0]
    ry = p.heading[0]
    x = px - math.sin(ry) * 0.3      # 횡오차 30cm
    y = py + math.cos(ry) * 0.3
    yaw, v, steer = ry + 0.4, 0.0, 0.0   # 헤딩오차 23°
    queue = [0.0] * 3                     # 60ms 지연
    errs = []
    for n in range(3000):
        mx = x + rng.normal(0, 0.001)
        my = y + rng.normal(0, 0.001)
        myaw = yaw + rng.normal(0, 0.002)
        fx, fy = mx + L * math.cos(myaw), my + L * math.sin(myaw)
        i = p.nearest_index(fx, fy)
        cx, cy = p.xy[i]
        cyaw = p.heading[i]
        e = -math.sin(cyaw) * (fx - cx) + math.cos(cyaw) * (fy - cy)
        cmd = normalize_angle(cyaw - myaw) + math.atan2(-k_e * e, k_soft + abs(v))
        md = 3.2 * DT
        steer = float(np.clip(np.clip(cmd, steer - md, steer + md), -MAXD, MAXD))
        queue.append(steer)
        applied = queue.pop(0)
        v += np.clip(float(p.v_ref[i]) - v, -3 * DT, 3 * DT)
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += v * math.tan(applied) / L * DT
        if n > 300:
            j = p.nearest_index(x + L * math.cos(yaw), y + L * math.sin(yaw))
            tx, ty = p.xy[j]
            tyaw = p.heading[j]
            errs.append(abs(-math.sin(tyaw) * (x + L * math.cos(yaw) - tx)
                            + math.cos(tyaw) * (y + L * math.sin(yaw) - ty)))
    errs = np.array(errs)
    check('횡오차 수렴', errs.mean() < 0.05,
          f'평균 {errs.mean()*100:.2f}cm 최대 {errs.max()*100:.2f}cm')


def test_packaging():
    """ROS 패키지 구성 — 빌드는 되는데 런치가 실패하는 경우를 잡는다.

    ★ setup.cfg 가 없으면 colcon build 는 성공하는데 실행파일이
      lib/ 가 아니라 bin/ 에 깔려서, ros2 launch 가 이렇게 죽는다:
          package 'mlcs_mpc' found at '...', but libexec directory
          '.../install/mlcs_mpc/lib/mlcs_mpc' does not exist
      빌드 로그에는 아무 경고도 안 나오므로 젯슨에서야 발견된다.
      (2026-09-14 실제로 겪음)
    """
    print('\n■ 패키지 구성')
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'src', 'mlcs_mpc')

    cfg = os.path.join(root, 'setup.cfg')
    ok = os.path.isfile(cfg)
    detail = ''
    if ok:
        txt = open(cfg).read()
        ok = 'lib/mlcs_mpc' in txt and 'install_scripts' in txt
        detail = 'install_scripts=$base/lib/mlcs_mpc'
    check('setup.cfg (실행파일이 lib/ 로 가게 한다)', ok, detail)

    # resource 마커 — 없으면 ros2 가 패키지를 아예 못 찾는다
    check('resource/mlcs_mpc 마커',
          os.path.isfile(os.path.join(root, 'resource', 'mlcs_mpc')))

    # setup.py 의 entry_points 와 실제 모듈이 일치하는가
    setup_txt = open(os.path.join(root, 'setup.py')).read()
    eps = re.findall(r"(\w+)\s*=\s*mlcs_mpc\.(\w+):main", setup_txt)
    missing = [m for _, m in eps
               if not os.path.isfile(os.path.join(root, 'mlcs_mpc', m + '.py'))]
    check(f'entry_points {len(eps)}개 ↔ 실제 모듈', not missing,
          f'없는 모듈: {missing}' if missing else f'{len(eps)}개 전부 존재')


def main():
    print('=' * 60)
    print('  mlcs_mpc 알고리즘 검증')
    print('=' * 60)

    test_packaging()
    test_jacobian()
    test_stanley()
    test_angle()
    test_straight()
    test_circle_path()
    test_circle_fit()
    test_kalman()
    test_lap()

    n = len(RESULTS)
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    print('\n' + '=' * 60)
    print(f'  {passed}/{n} 통과')
    print('=' * 60)
    if passed < n:
        print('  실패:')
        for name, ok, d in RESULTS:
            if not ok:
                print(f'    - {name}  {d}')
    return 0 if passed == n else 1


if __name__ == '__main__':
    sys.exit(main())
