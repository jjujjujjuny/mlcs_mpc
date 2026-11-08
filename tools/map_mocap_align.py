#!/usr/bin/env python3
"""
map_mocap_align.py — 라이다 맵(Cartographer map)과 mocap 좌표계를 잇는다. 연구실용.

  대회장에는 mocap 이 없어 라이다 맵만 쓴다. 연구실에서는 둘이 다 있으므로
  mocap 을 정답으로 라이다 위치추정을 검증할 수 있다. 그러려면 두 좌표계
  사이 변환부터 알아야 한다.

■ 보통은 drive.sh 로 쓴다

    ./drive.sh localize lab1      창 1 — 저장한 맵으로 위치추정 (조이스틱)
    ./drive.sh align              창 2 — 기록. Ctrl+C 하면 처음엔 fit, 그다음은 eval
    ./drive.sh mocaptrack lab1_track   트랙 → mocap 프레임

■ 순서 (직접 돌릴 때)

  ① 기록 (젯슨, 창 셋)
       ./drive.sh mapping            (또는 위치추정 모드) — Cartographer
       ./drive.sh natnet             mocap 드라이버만. mocap_bridge 는 띄우지 않는다
       python3 tools/map_mocap_align.py record ~/mlcs_data/align/a1.csv
     조이스틱으로 트랙을 2~3바퀴, 좌우 회전이 다 들어가게 몬다. Ctrl+C 로 끝.

  ② 변환 맞추기 (어디서든, ROS 불필요)
       python3 tools/map_mocap_align.py fit ~/mlcs_data/align/a1.csv maps/lab1
     → maps/lab1.mocap.yaml (+ .png)

  ③ 검증 — **다른 주행 로그**로 (맞춘 로그로 평가하면 오차가 작게 나온다)
       python3 tools/map_mocap_align.py eval ~/mlcs_data/align/a2.csv maps/lab1.mocap.yaml

  ④ 트랙 옮기기
       python3 tools/map_mocap_align.py apply maps/lab1.mocap.yaml lab1_track \\
           --to mocap --as lab1_track_mocap
     map 프레임 트랙(track_from_map 결과)을 mocap 프레임으로, 또는 그 반대로.
     mocap 으로 달리며 같은 트랙을 쓰거나, mocap 트랙을 라이다 위치추정으로 달릴 때.

■ 무엇을 맞추나

    p_mocap = R(θ) · (p_map + R(ψ_map) · d) + t      위치
    ψ_mocap = ψ_map + θ + δ                          방향

    θ, t   map → mocap 변환. 맵이 바뀌면 다시 맞춘다
    d, δ   Cartographer 의 base_link 가 mocap base_link 에서 벗어난 양 (차체 좌표).
           base_link→laser static TF 가 틀렸으면 여기 나온다. 0 에 가까워야 한다
    지연   라이다 자세가 mocap 보다 늦게 찍히는 시간. 요레이트 상호상관으로 잰다

■ 주의

  ★ mocap_bridge 를 같이 띄우면 안 된다. 그것도 map→base_link TF 를 내서
    Cartographer 와 겹친다 (drive.sh mapping 이 막는다). 여기서는 NatNet 원본
    (/car/pose) 을 받아 mocap.yaml 의 offset 을 직접 적용한다.
  ★ 두 스탬프가 같은 시계여야 지연이 의미 있다 (둘 다 젯슨에서 찍히는지 확인).
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MOCAP_YAML = os.path.join(REPO, 'src/mlcs_mpc/config/mocap.yaml')


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ─────────────────────────────────────────────────────────────────────
#  ① 기록
# ─────────────────────────────────────────────────────────────────────
def load_mocap_cfg(path=MOCAP_YAML):
    """mocap.yaml 의 토픽과 강체 → 차량 기준점 offset."""
    import yaml
    with open(path, encoding='utf-8') as f:
        y = yaml.safe_load(f)
    p = y['/**']['ros__parameters']
    return dict(topic=p.get('mocap_topic', '/car/pose'),
                off_x=float(p.get('offset_x', 0.0)),
                off_y=float(p.get('offset_y', 0.0)),
                yaw_off=float(p.get('yaw_offset', 0.0)))


def cmd_record(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped

    cfg = load_mocap_cfg()
    mtopic = a.mocap_topic or cfg['topic']
    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    f = open(out, 'w', newline='', encoding='utf-8')
    w = csv.writer(f)
    w.writerow(['src', 'stamp', 'recv', 'x', 'y', 'yaw'])
    cnt = {'m': 0, 'l': 0}

    class Rec(Node):
        def __init__(self):
            super().__init__('map_mocap_recorder')
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=50)
            self.create_subscription(PoseStamped, mtopic,
                                     lambda m: self.cb('m', m), qos)
            self.create_subscription(PoseStamped, a.lidar_topic,
                                     lambda m: self.cb('l', m), qos)
            self.create_timer(2.0, self.report)
            self.t0 = time.time()
            self.last = dict(cnt)

        def cb(self, src, msg):
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            q = msg.pose.orientation
            w.writerow([src, f'{st:.6f}', f'{now:.6f}',
                        f'{msg.pose.position.x:.5f}',
                        f'{msg.pose.position.y:.5f}',
                        f'{yaw_from_quat(q.x, q.y, q.z, q.w):.6f}'])
            cnt[src] += 1

        def report(self):
            f.flush()
            rm = (cnt['m'] - self.last['m']) / 2.0
            rl = (cnt['l'] - self.last['l']) / 2.0
            self.last = dict(cnt)
            warn = ''
            if rm < 50:
                warn += f'  ⚠ mocap {rm:.0f} Hz ({mtopic}, ./drive.sh natnet 확인)'
            if rl < 5:
                warn += (f'  ⚠ 라이다 자세 {rl:.0f} Hz ({a.lidar_topic}, '
                         'Cartographer 확인)')
            print(f'  {time.time() - self.t0:6.0f} s  mocap {cnt["m"]:6d} '
                  f'({rm:5.1f} Hz)  라이다 {cnt["l"]:5d} ({rl:5.1f} Hz){warn}',
                  flush=True)

    print(f'■ 기록 → {out}')
    print(f'   mocap {mtopic}   라이다 {a.lidar_topic}   (Ctrl+C 로 끝)')
    print('   트랙을 2~3바퀴, 좌우 회전이 다 들어가게 모세요')
    rclpy.init()
    node = Rec()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:                               # noqa: BLE001
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'\n✓ mocap {cnt["m"]}개, 라이다 {cnt["l"]}개 → {out}')


# ─────────────────────────────────────────────────────────────────────
#  읽기와 짝짓기
# ─────────────────────────────────────────────────────────────────────
def load_log(path, cfg, use_offset=True):
    """기록 CSV → mocap, 라이다 각각 dict(t, x, y, yaw). 스탬프순."""
    src, rows = [], []
    with open(os.path.expanduser(path), encoding='utf-8') as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            if len(row) < 6:
                continue
            src.append(row[0])
            rows.append([float(v) for v in row[1:6]])
    A = np.array(rows, dtype=float)
    src = np.array(src)
    out = {}
    for k in ('m', 'l'):
        B = A[src == k]
        B = B[np.argsort(B[:, 0])]
        _, keep = np.unique(B[:, 0], return_index=True)   # 같은 스탬프 중복 제거
        B = B[keep]
        t, x, y, yaw = B[:, 0], B[:, 2], B[:, 3], B[:, 4]
        if k == 'm' and use_offset:
            # mocap_bridge 와 같은 보정: 강체 원점 → 차량 기준점
            yaw = yaw + cfg['yaw_off']
            x = x + np.cos(yaw) * cfg['off_x'] - np.sin(yaw) * cfg['off_y']
            y = y + np.sin(yaw) * cfg['off_x'] + np.cos(yaw) * cfg['off_y']
        out[k] = dict(t=t, x=x, y=y, yaw=np.unwrap(yaw))
    return out['m'], out['l']


def describe(M, L):
    """기록 요약 — 주기, 끊김, 겹치는 시간."""
    lines = []
    for nm, D in (('mocap', M), ('라이다', L)):
        if len(D['t']) < 2:
            lines.append(f'   {nm}: {len(D["t"])}개 — 부족')
            continue
        dtv = np.diff(D['t'])
        lines.append(f'   {nm:4s} {len(D["t"]):6d}개  {1/np.median(dtv):5.1f} Hz  '
                     f'최대 끊김 {dtv.max():.3f} s  '
                     f'0.1 s 넘는 끊김 {int((dtv > 0.1).sum())}회')
    t0 = max(M['t'][0], L['t'][0])
    t1 = min(M['t'][-1], L['t'][-1])
    lines.append(f'   겹치는 시간 {max(t1 - t0, 0):.1f} s')
    return '\n'.join(lines), t0, t1


def estimate_lag(M, L, max_lag=0.5, dt=0.01):
    """요레이트 상호상관으로 지연을 잰다. + 면 라이다가 늦다.

    반환 (지연 s, 상관계수, 쓸만한가)
    """
    t0 = max(M['t'][0], L['t'][0]) + max_lag
    t1 = min(M['t'][-1], L['t'][-1]) - max_lag
    if t1 - t0 < 5.0:
        return 0.0, 0.0, False
    g = np.arange(t0 - max_lag, t1 + max_lag, dt)
    k = max(int(round(0.05 / dt)), 1)
    ker = np.ones(k) / k

    def rate(D):
        y = np.interp(g, D['t'], D['yaw'])
        return np.convolve(np.gradient(y, dt), ker, mode='same')
    rm, rl = rate(M), rate(L)
    n = int(round(max_lag / dt))
    core = slice(n, len(g) - n)
    a = rm[core] - rm[core].mean()
    if a.std() < 0.1:                         # 거의 직진만 했다
        return 0.0, 0.0, False
    cs = []
    for j in range(-n, n + 1):
        b = rl[n + j:len(g) - n + j]
        b = b - b.mean()
        cs.append(float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)))
    cs = np.array(cs)
    i = int(np.argmax(cs))
    off = 0.0
    if 0 < i < len(cs) - 1:                   # 포물선 보간으로 한 칸 아래까지
        den = cs[i - 1] - 2 * cs[i] + cs[i + 1]
        if abs(den) > 1e-12:
            off = 0.5 * (cs[i - 1] - cs[i + 1]) / den
    return (i - n + off) * dt, float(cs[i]), True


def pair(M, L, lag=0.0, max_gap=0.03):
    """라이다 샘플마다 같은 순간의 mocap 을 보간해 붙인다.

    라이다 스탬프 t 의 자세는 mocap 의 t - lag 와 같은 순간이다.
    mocap 앞뒤 샘플이 max_gap 보다 멀면 (끊김) 버린다.
    """
    tq = L['t'] - lag
    i = np.searchsorted(M['t'], tq)
    ok = (i > 0) & (i < len(M['t']))
    ii = np.clip(i, 1, len(M['t']) - 1)
    gap = M['t'][ii] - M['t'][ii - 1]
    ok &= (gap <= max_gap)
    tq = tq[ok]
    m = np.c_[np.interp(tq, M['t'], M['x']), np.interp(tq, M['t'], M['y'])]
    psm = np.interp(tq, M['t'], M['yaw'])
    c = np.c_[L['x'][ok], L['y'][ok]]
    psc = L['yaw'][ok]
    # 속도 (mocap 미분) — 오차를 속도별로 나눠 보려고
    vx = np.gradient(M['x'], M['t'])
    vy = np.gradient(M['y'], M['t'])
    v = np.hypot(np.interp(tq, M['t'], vx), np.interp(tq, M['t'], vy))
    return dict(t=L['t'][ok], c=c, psc=psc, m=m, psm=psm, v=v)


# ─────────────────────────────────────────────────────────────────────
#  ② 맞추기
# ─────────────────────────────────────────────────────────────────────
def umeyama2d(P, Q):
    """Q ≈ R(θ) P + t 의 최소제곱 해 (스케일 없음)."""
    pm, qm = P.mean(axis=0), Q.mean(axis=0)
    A, B = P - pm, Q - qm
    H = A.T @ B
    th = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    return th, qm - rot(th) @ pm


def fit_transform(c, psc, m, psm, lever=True, iters=50):
    """θ, t, d 를 번갈아 푼다. δ 는 마지막에 방향 차의 평균으로."""
    d = np.zeros(2)
    th, t = umeyama2d(c, m)
    for _ in range(iters):
        R_psi = np.stack([rot(a) for a in psc])                # (N,2,2)
        P = c + np.einsum('nij,j->ni', R_psi, d)
        th, t = umeyama2d(P, m)
        if not lever:
            break
        A = np.einsum('ij,njk->nik', rot(th), R_psi).reshape(-1, 2)
        b = (m - t - c @ rot(th).T).reshape(-1)
        d_new = np.linalg.lstsq(A, b, rcond=None)[0]
        if np.linalg.norm(d_new - d) < 1e-7:
            d = d_new
            break
        d = d_new
    dpsi = wrap(psm - psc - th)
    delta = float(math.atan2(np.sin(dpsi).mean(), np.cos(dpsi).mean()))
    return dict(theta=float(th), t=t, d=d, delta=delta)


def apply_T(T, c, psc, lever=True):
    """map 자세 → mocap 프레임. lever=False 면 θ, t 만 (MPC 가 받는 그대로)."""
    P = c.copy()
    if lever:
        P = P + np.einsum('nij,j->ni', np.stack([rot(a) for a in psc]), T['d'])
    q = P @ rot(T['theta']).T + T['t']
    ps = psc + T['theta'] + (T['delta'] if lever else 0.0)
    return q, ps


def errors(T, pr, lever=True):
    """mocap 대비 오차. 위치는 차체 앞뒤/좌우로도 나눈다."""
    q, ps = apply_T(T, pr['c'], pr['psc'], lever)
    e = q - pr['m']
    cs, sn = np.cos(pr['psm']), np.sin(pr['psm'])
    lon = e[:, 0] * cs + e[:, 1] * sn
    lat = -e[:, 0] * sn + e[:, 1] * cs
    return dict(pos=np.hypot(e[:, 0], e[:, 1]), lon=lon, lat=lat,
                yaw=wrap(ps - pr['psm']), q=q)


def fmt_stats(E):
    p = E['pos']
    return (f'위치 RMSE {np.sqrt(np.mean(p**2))*100:5.1f} cm  '
            f'p95 {np.percentile(p, 95)*100:5.1f}  최대 {p.max()*100:5.1f}  '
            f'| 앞뒤 RMSE {np.sqrt(np.mean(E["lon"]**2))*100:4.1f}  '
            f'좌우 {np.sqrt(np.mean(E["lat"]**2))*100:4.1f} cm  '
            f'| yaw RMSE {np.degrees(np.sqrt(np.mean(E["yaw"]**2))):4.2f}°')


def yaw_coverage(psc):
    """방향이 고르게 퍼진 정도 (0 = 한 방향, 1 = 고르게). d 를 풀 수 있나."""
    return float(1.0 - np.hypot(np.cos(psc).mean(), np.sin(psc).mean()))


def robust_fit(pr, lever=True, max_err=0.30):
    """한 번 맞추고, 크게 벗어난 점(스캔매칭 튐)을 빼고 다시 맞춘다."""
    T = fit_transform(pr['c'], pr['psc'], pr['m'], pr['psm'], lever)
    E = errors(T, pr, lever)
    med = np.median(E['pos'])
    thr = min(max_err, max(5.0 * 1.4826 * np.median(np.abs(E['pos'] - med))
                           + med, 0.05))
    keep = E['pos'] <= thr
    sub = {k: v[keep] for k, v in pr.items()}
    T = fit_transform(sub['c'], sub['psc'], sub['m'], sub['psm'], lever)
    return T, keep, thr


def save_plot(path, pr, E, title, M=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    try:
        import draw_track as dt
        dt.setup_font()
    except Exception:                                    # noqa: BLE001
        pass
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    a0 = ax[0, 0]
    a0.plot(pr['m'][:, 0], pr['m'][:, 1], '-', color='0.6', lw=2,
            label='mocap')
    a0.plot(E['q'][:, 0], E['q'][:, 1], '-', color='tab:blue', lw=0.8,
            label='라이다 (mocap 좌표로)')
    a0.set_aspect('equal')
    a0.legend(fontsize=8)
    a0.set_title('궤적 (mocap 프레임)')
    t = pr['t'] - pr['t'][0]
    a1 = ax[0, 1]
    a1.plot(t, E['pos'] * 100, lw=0.6, label='위치')
    a1.plot(t, E['lat'] * 100, lw=0.6, label='좌우')
    a1.plot(t, E['lon'] * 100, lw=0.6, label='앞뒤')
    a1.set_ylabel('cm')
    a1.legend(fontsize=8)
    a1.set_title('위치 오차')
    a2 = ax[1, 0]
    a2.plot(t, np.degrees(E['yaw']), lw=0.6)
    a2.set_ylabel('deg')
    a2.set_xlabel('s')
    a2.set_title('yaw 오차')
    a3 = ax[1, 1]
    a3.plot(pr['v'], E['pos'] * 100, '.', ms=2, alpha=0.4)
    a3.set_xlabel('속도 m/s (mocap)')
    a3.set_ylabel('위치 오차 cm')
    a3.set_title('속도별 오차')
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def speed_table(pr, E):
    out = []
    for lo, hi in ((0, 0.3), (0.3, 1.0), (1.0, 2.0), (2.0, 99)):
        k = (pr['v'] >= lo) & (pr['v'] < hi)
        if k.sum() < 20:
            continue
        p = E['pos'][k]
        out.append(f'   {lo:.1f}~{hi if hi < 99 else "":<4} m/s  {k.sum():6d}개  '
                   f'위치 RMSE {np.sqrt(np.mean(p**2))*100:5.1f} cm  '
                   f'좌우 {np.sqrt(np.mean(E["lat"][k]**2))*100:4.1f}  '
                   f'yaw {np.degrees(np.sqrt(np.mean(E["yaw"][k]**2))):4.2f}°')
    return '\n'.join(out)


def cmd_fit(a):
    import yaml
    cfg = load_mocap_cfg(a.mocap_yaml)
    M, L = load_log(a.log, cfg, not a.no_offset)
    desc, t0, t1 = describe(M, L)
    print(f'■ 기록  {a.log}\n{desc}')
    if t1 - t0 < 10:
        print('✗ 겹치는 시간이 10 s 보다 짧습니다'); sys.exit(1)

    lag, rho, ok = estimate_lag(M, L, a.max_lag)
    if a.lag is not None:
        lag, ok = a.lag, True
        print(f'\n■ 지연  {lag*1000:+.0f} ms (지정)')
    else:
        print(f'\n■ 지연  {lag*1000:+.0f} ms  (요레이트 상관 {rho:.3f})'
              + ('' if ok else '  ⚠ 회전이 적어 못 쟀습니다 — 0 으로 둡니다'))
        if not ok:
            lag = 0.0
    pr = pair(M, L, lag)
    cov = yaw_coverage(pr['psc'])
    lever = not a.no_lever and cov > 0.3
    if not a.no_lever and not lever:
        print(f'   ⚠ 방향 분포 {cov:.2f} — 한쪽으로만 달려 base_link 오차(d)를 못 '
              '풉니다. 0 으로 둡니다')
    T, keep, thr = robust_fit(pr, lever)
    print(f'\n■ 변환 (짝 {len(pr["t"])}개, {thr*100:.0f} cm 넘는 {int((~keep).sum())}개 제외)')
    print(f'   map → mocap   x {T["t"][0]:+.4f}  y {T["t"][1]:+.4f} m  '
          f'yaw {T["theta"]:+.5f} rad ({math.degrees(T["theta"]):+.2f}°)')
    print(f'   base_link 차이  앞 {T["d"][0]*100:+.1f}  왼 {T["d"][1]*100:+.1f} cm  '
          f'yaw {math.degrees(T["delta"]):+.2f}°'
          + ('' if lever else '  (위치는 안 풀었음)'))
    if np.hypot(*T['d']) > 0.03 or abs(T['delta']) > math.radians(1.5):
        print('   ⚠ 3 cm / 1.5° 보다 큽니다. base_link→laser static TF 나 '
              'mocap.yaml offset 이 틀렸을 수 있습니다')
    E_fit = errors(T, pr, lever)
    E_raw = errors(T, pr, False)
    print(f'\n■ 맞춘 로그에서의 오차 (참고용 — 검증은 다른 로그로 eval)')
    print(f'   보정 포함   {fmt_stats(E_fit)}')
    print(f'   변환만      {fmt_stats(E_raw)}')

    base = a.out
    if base.endswith('.yaml'):
        base = base[:-5]
    out = base + '.mocap.yaml' if not base.endswith('.mocap') else base + '.yaml'
    os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)
    tf_cmd = (f'ros2 run tf2_ros static_transform_publisher '
              f'--x {T["t"][0]:.4f} --y {T["t"][1]:.4f} --yaw {T["theta"]:.5f} '
              f'--frame-id mocap --child-frame-id map')
    doc = {
        'map_to_mocap': {'x': float(T['t'][0]), 'y': float(T['t'][1]),
                         'yaw': float(T['theta'])},
        'lidar_base_link_error': {'x': float(T['d'][0]), 'y': float(T['d'][1]),
                                  'yaw': float(T['delta']),
                                  'solved_position': bool(lever)},
        'time_lag_s': float(lag),
        'fit': {
            'pairs': int(len(pr['t'])), 'used': int(keep.sum()),
            'duration_s': float(pr['t'][-1] - pr['t'][0]),
            'pos_rmse_m': float(np.sqrt(np.mean(E_fit['pos'] ** 2))),
            'pos_p95_m': float(np.percentile(E_fit['pos'], 95)),
            'yaw_rmse_rad': float(np.sqrt(np.mean(E_fit['yaw'] ** 2))),
            'yaw_coverage': cov,
        },
        'source': {'log': os.path.abspath(os.path.expanduser(a.log)),
                   'mocap_offsets': cfg if not a.no_offset else None,
                   'created': time.strftime('%Y-%m-%dT%H:%M:%S')},
        'static_tf': tf_cmd,
    }
    with open(out, 'w', encoding='utf-8') as f:
        f.write('# map_mocap_align.py fit 결과\n')
        f.write('#   p_mocap = R(yaw)·p_map + [x, y]   (map_to_mocap)\n')
        f.write('#   lidar_base_link_error: Cartographer base_link 이 mocap '
                'base_link 에서 벗어난 양 (차체 좌표, 0 에 가까워야 한다)\n')
        f.write('#   time_lag_s: + 면 라이다 자세가 mocap 보다 늦게 찍힌다\n')
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
    png = out[:-5] + '.png'
    save_plot(png, pr, E_fit, f'fit — {os.path.basename(a.log)}')
    print(f'\n✓ {out}\n   그림 {png}')
    print(f'   RViz 에서 같이 보려면 (mocap_bridge 의 map_frame 을 mocap 으로 바꾼 뒤):\n'
          f'     {tf_cmd}')


def load_T(path):
    import yaml
    with open(path, encoding='utf-8') as f:
        y = yaml.safe_load(f)
    e = y['lidar_base_link_error']
    return dict(theta=float(y['map_to_mocap']['yaw']),
                t=np.array([y['map_to_mocap']['x'], y['map_to_mocap']['y']]),
                d=np.array([e['x'], e['y']]), delta=float(e['yaw']),
                lag=float(y.get('time_lag_s', 0.0)), raw=y)


def cmd_eval(a):
    cfg = load_mocap_cfg(a.mocap_yaml)
    T = load_T(a.align)
    if os.path.abspath(os.path.expanduser(a.log)) == T['raw']['source']['log']:
        print('⚠ 변환을 맞춘 로그와 같습니다 — 오차가 실제보다 작게 나옵니다')
    M, L = load_log(a.log, cfg, not a.no_offset)
    desc, t0, t1 = describe(M, L)
    print(f'■ 기록  {a.log}\n{desc}')
    lag, rho, ok = estimate_lag(M, L)
    print(f'   이 로그의 지연 {lag*1000:+.0f} ms (상관 {rho:.3f}), '
          f'맞출 때 {T["lag"]*1000:+.0f} ms')

    pr0 = pair(M, L, 0.0)
    E0 = errors(T, pr0, False)
    pr1 = pair(M, L, lag if ok else 0.0)
    E1 = errors(T, pr1, True)
    print(f'\n■ 위치추정 오차 (변환 {os.path.basename(a.align)})')
    print(f'   MPC 가 받는 값 (지연 그대로, 보정 없음)')
    print(f'     {fmt_stats(E0)}')
    print(f'   지연·base_link 보정 후 (위치추정 자체의 정확도)')
    print(f'     {fmt_stats(E1)}')
    print(f'\n■ 속도별 (MPC 가 받는 값)\n{speed_table(pr0, E0)}')
    gaps = np.diff(L['t'])
    print(f'\n■ 라이다 자세 끊김  최대 {gaps.max():.3f} s, '
          f'0.1 s 초과 {int((gaps > 0.1).sum())}회, '
          f'0.3 s 초과 {int((gaps > 0.3).sum())}회')
    png = os.path.splitext(os.path.expanduser(a.log))[0] + '_eval.png'
    save_plot(png, pr0, E0, f'eval — {os.path.basename(a.log)} (MPC 가 받는 값)')
    print(f'\n   그림 {png}')


# ─────────────────────────────────────────────────────────────────────
#  ④ 트랙 옮기기
# ─────────────────────────────────────────────────────────────────────
def transform_csv(src, dst, th, t, note):
    """x,y 열을 옮기고 heading 열이 있으면 같이 돌린다. 나머지 열은 그대로."""
    with open(src, encoding='utf-8') as f:
        lines = f.read().splitlines()
    out, xi, yi, hi = [], 0, 1, None
    header_seen = False
    R = rot(th)
    for ln in lines:
        s = ln.strip()
        if not s:
            out.append(ln)
            continue
        if s.startswith('#'):
            out.append(ln)
            continue
        cells = s.split(',')
        try:
            vals = [float(v) for v in cells]
        except ValueError:
            if header_seen:
                raise
            names = [c.strip().lower() for c in cells]
            for cand in ('x', 'x_m'):
                if cand in names:
                    xi = names.index(cand)
            for cand in ('y', 'y_m'):
                if cand in names:
                    yi = names.index(cand)
            for cand in ('heading', 'psi', 'yaw'):
                if cand in names:
                    hi = names.index(cand)
            header_seen = True
            out.append(ln)
            continue
        p = R @ np.array([vals[xi], vals[yi]]) + t
        cells[xi], cells[yi] = f'{p[0]:.6f}', f'{p[1]:.6f}'
        if hi is not None:
            cells[hi] = f'{float(wrap(vals[hi] + th)):.6f}'
        out.append(','.join(cells))
    if lines and lines[0].startswith('#'):
        out.insert(1, f'# {note}')
    with open(dst, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out) + '\n')


def cmd_apply(a):
    import json
    import shutil
    import draw_track as dt
    T = load_T(a.align)
    th, t = T['theta'], T['t']
    if a.to == 'map':                          # 역변환
        th, t = -th, -(rot(-th) @ t)
    frame = 'mocap_world' if a.to == 'mocap' else 'map'
    note = (f'map_mocap_align.py apply → {frame} '
            f'({os.path.basename(a.align)}, {time.strftime("%Y-%m-%d %H:%M")})')

    if os.path.isfile(a.track):                # 파일 하나
        dst = a.as_ or (os.path.splitext(a.track)[0] + f'_{a.to}.csv')
        transform_csv(a.track, dst, th, t, note)
        print(f'✓ {dst}')
        return

    name, new = a.track, a.as_ or f'{a.track}_{a.to}'
    sdir, ddir = os.path.join(dt.OUTDIR, name), os.path.join(dt.OUTDIR, new)
    wp_s = os.path.join(dt.WAYPOINTS, f'{name}.csv')
    if not os.path.isdir(sdir) and not os.path.isfile(wp_s):
        print(f'✗ 트랙을 못 찾았습니다: {sdir} / {wp_s}')
        sys.exit(1)
    if os.path.exists(ddir) or os.path.exists(
            os.path.join(dt.WAYPOINTS, f'{new}.csv')):
        print(f'✗ 이미 있습니다: {new} — --as 로 다른 이름을 주세요')
        sys.exit(1)
    made = []
    if os.path.isfile(wp_s):
        wp_d = os.path.join(dt.WAYPOINTS, f'{new}.csv')
        transform_csv(wp_s, wp_d, th, t, note)
        dt.link_into_workspace(wp_d)
        made.append(wp_d)
    if os.path.isdir(sdir):
        os.makedirs(ddir)
        for fn in sorted(os.listdir(sdir)):
            sp = os.path.join(sdir, fn)
            dn = fn.replace(name, new) if fn in (f'{name}.csv',
                                                  f'prep_{name}.csv') else fn
            dp = os.path.join(ddir, dn)
            if fn == 'curvature_profile.csv':
                shutil.copy(sp, dp)            # 회전·이동에 안 변한다
            elif fn.endswith('.csv'):
                transform_csv(sp, dp, th, t, note)
            elif fn == 'track.json':
                with open(sp, encoding='utf-8') as f:
                    meta = json.load(f)
                meta['name'] = new
                meta['coordinate_frame'] = {
                    'name': frame, 'unit': 'meter',
                    'note': note, 'from': meta.get('coordinate_frame')}
                with open(dp, 'w', encoding='utf-8') as f:
                    json.dump(meta, f, indent=2, ensure_ascii=False)
            else:
                continue                       # 그림 등은 옮기지 않는다
            made.append(dp)
    print(f'✓ {name} → {new} ({frame} 프레임)')
    for p in made:
        print(f'    {os.path.relpath(p, REPO)}')
    print(f'   ★ 이 트랙은 {frame} 프레임입니다. '
          + ('mocap 상태(mocap_bridge)로 달립니다.' if a.to == 'mocap'
             else '라이다 위치추정 상태로 달립니다.'))


# ─────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description='라이다 맵 ↔ mocap 좌표계',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = p.add_subparsers(dest='cmd')
    sub.required = True

    r = sub.add_parser('record', help='mocap 과 Cartographer 자세를 같이 기록 (젯슨)')
    r.add_argument('out', help='CSV 경로')
    r.add_argument('--mocap-topic', default=None,
                   help='기본 mocap.yaml 의 mocap_topic (NatNet 원본)')
    r.add_argument('--lidar-topic', default='/tracked_pose')

    for name, h in (('fit', '기록으로 map→mocap 변환을 맞춘다'),
                    ('eval', '맞춘 변환으로 다른 기록의 위치추정 오차를 잰다')):
        s = sub.add_parser(name, help=h)
        s.add_argument('log')
        if name == 'fit':
            s.add_argument('out', help='출력 이름 (예: maps/lab1 → '
                                       'maps/lab1.mocap.yaml)')
            s.add_argument('--no-lever', action='store_true',
                           help='base_link 위치 차이(d)를 풀지 않는다')
            s.add_argument('--lag', type=float, default=None,
                           help='지연을 직접 준다 [s]')
            s.add_argument('--max-lag', type=float, default=0.5)
        else:
            s.add_argument('align', help='fit 결과 .mocap.yaml')
        s.add_argument('--mocap-yaml', default=MOCAP_YAML)
        s.add_argument('--no-offset', action='store_true',
                       help='mocap.yaml offset 을 적용하지 않는다')

    s = sub.add_parser('apply', help='트랙을 map ↔ mocap 프레임으로 옮긴다')
    s.add_argument('align', help='fit 결과 .mocap.yaml')
    s.add_argument('track', help='트랙 이름 (waypoints/ + lab_map/data/tracks/) '
                                 '또는 CSV 파일 하나')
    s.add_argument('--to', choices=('mocap', 'map'), required=True)
    s.add_argument('--as', dest='as_', default=None, help='새 이름 / 출력 경로')

    a = p.parse_args()
    {'record': cmd_record, 'fit': cmd_fit, 'eval': cmd_eval,
     'apply': cmd_apply}[a.cmd](a)


if __name__ == '__main__':
    main()
