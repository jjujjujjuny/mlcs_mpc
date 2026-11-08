#!/usr/bin/env python3
"""
test_map_tools.py — track_from_map / map_mocap_align 회귀 시험 (차·ROS 불필요)

  python3 tools/test_map_tools.py

■ 왜 있나

  두 도구는 대회장과 연구실에서 처음 실제 맵을 만난다. 그때 틀리면 고칠
  시간이 없다. 정답을 아는 합성 맵·합성 궤적으로 미리 확인한다.

  ① 경기장 모양 트랙(중심선을 안다)을 pgm 으로 그리고, 벽 틈·미지 잡음·
     기둥을 넣어 중심선이 정답에서 얼마나 벗어나는지 본다.
  ② 알고 있는 변환·base_link 오차·지연으로 라이다 자세를 만들고 되찾는다.
  ③ 트랙 파일을 mocap 으로 옮겼다가 되돌리면 원래대로 오는지 본다.
"""
import math
import os
import sys
import tempfile
import warnings

import numpy as np

warnings.filterwarnings('ignore')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import track_from_map as tfm                                # noqa: E402
import map_mocap_align as mma                               # noqa: E402
import draw_track as dt                                     # noqa: E402

FAIL = []
TMP = tempfile.mkdtemp(prefix='test_map_tools_')


def check(cond, name, detail=''):
    print(f'  {"✓" if cond else "✗"} {name}{"   " + detail if detail else ""}')
    if not cond:
        FAIL.append(name)


# 경기장 모양 중심선: 직선 ±2 m (y = ±1.5), 양끝 반원 반지름 1.5 m. 반시계.
SL, RAD = 2.0, 1.5


def stadium(n_per_m=100):
    pts = []
    for x in np.linspace(-SL, SL, int(2 * SL * n_per_m), endpoint=False):
        pts.append((x, -RAD))
    for a in np.linspace(-np.pi / 2, np.pi / 2, int(np.pi * RAD * n_per_m),
                         endpoint=False):
        pts.append((SL + RAD * np.cos(a), RAD * np.sin(a)))
    for x in np.linspace(SL, -SL, int(2 * SL * n_per_m), endpoint=False):
        pts.append((x, RAD))
    for a in np.linspace(np.pi / 2, 3 * np.pi / 2, int(np.pi * RAD * n_per_m),
                         endpoint=False):
        pts.append((-SL + RAD * np.cos(a), RAD * np.sin(a)))
    return np.array(pts)


def dist_stadium(P):
    """점들 → 경기장 중심선까지 해석적 거리."""
    x, y = P[:, 0], P[:, 1]
    cx = np.clip(x, -SL, SL)
    return np.abs(np.hypot(x - cx, y) - RAD)


def write_map(name, res=0.05, half=0.5, wall=0.10, gap=False, specks=False,
              pole=None):
    """경기장 트랙 맵. 벽 밖은 기본 미지(205), gap=True 면 빈 방 + 바깥벽에 틈."""
    ox, oy = -5.0, -4.0
    W, H = int(10.0 / res), int(8.0 / res)
    cols, rows = np.meshgrid(np.arange(W), np.arange(H))
    X = ox + (cols + 0.5) * res
    Y = oy + (H - 0.5 - rows) * res
    d = dist_stadium(np.c_[X.ravel(), Y.ravel()]).reshape(H, W)
    img = np.full((H, W), 205, np.uint8)
    if gap:
        img[:] = 254                                    # 바깥이 열린 방
    img[d < half + wall] = 0
    img[d < half] = 254
    if gap:
        # 바깥벽(위쪽 직선 y = 1.5 + 0.5 ~ 0.6) 에 10 cm 틈
        k = (np.abs(X) < 0.05) & (Y > RAD + half - 0.01) & (Y < RAD + half + wall + 0.01)
        img[k] = 254
    if specks:
        rng = np.random.default_rng(1)
        fr = np.argwhere(d < half - 0.1)
        for r, c in fr[rng.choice(len(fr), 40, replace=False)]:
            img[r, c] = 205                             # 미지 점 하나씩
    if pole is not None:
        # 위 직선에 지름 10 cm 기둥. pole = 기둥 가장자리에서 바깥벽까지 [m]
        k = np.hypot(X - 0.0, Y - (RAD + half - pole - 0.05)) < 0.05
        img[k] = 0
    pgm = os.path.join(TMP, f'{name}.pgm')
    with open(pgm, 'wb') as f:
        f.write(f'P5\n# test\n{W} {H}\n255\n'.encode())
        f.write(img.tobytes())
    yml = os.path.join(TMP, f'{name}.yaml')
    with open(yml, 'w') as f:
        f.write(f'image: {name}.pgm\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return yml


def centerline(yml, seal=0.10, start=(0.0, -RAD), yaw=0.0, reverse=False):
    m = tfm.load_map(yml)
    ring = tfm.extract_ring(m, start, seal)
    if ring['err']:
        return m, ring, None
    cpx = tfm.center_contour(ring)
    c = tfm.pix_to_world(m, cpx[:, 0], cpx[:, 1])
    return m, ring, tfm.orient(tfm.resample_poly(c, 0.05), start, yaw, reverse)


# ─────────────────────────────────────────────────────────────────────
print('① 맵 읽기와 좌표 변환')
yml = write_map('plain')
m = tfm.load_map(yml)
r, c = tfm.world_to_pix(m, np.array([[0.0, -RAD], [1.23, 0.45]]))
back = tfm.pix_to_world(m, r, c)
check(np.allclose(back, [[0.0, -RAD], [1.23, 0.45]]), '픽셀 ↔ 좌표 왕복')
check(m['free'][int(round(r[0])), int(round(c[0]))], '출발점이 빈칸')
check(not m['free'][0, 0] and not m['occ'][0, 0], '바깥은 미지 (빈칸도 벽도 아님)')

print('\n② 중심선 — 깨끗한 맵')
m, ring, cl = centerline(yml)
check(cl is not None, '고리 찾음', ring['err'] or '')
if cl is not None:
    e = dist_stadium(cl)
    check(e.max() < 0.04, '정답 중심선에서 최대 4 cm 이내',
          f'평균 {e.mean()*100:.1f} cm, 최대 {e.max()*100:.1f} cm')
    L = np.sum(np.linalg.norm(np.diff(np.vstack([cl, cl[:1]]), axis=0), axis=1))
    check(abs(L - (4 * SL + 2 * np.pi * RAD)) < 0.15, '길이',
          f'{L:.2f} m (정답 {4*SL + 2*np.pi*RAD:.2f})')
    check(np.hypot(*cl[0] - [0, -RAD]) < 0.06, '0 번 점이 출발점')
    check(cl[1, 0] > cl[0, 0], '출발 방향 +x 로 진행')
    check(dt.shoelace(cl) > 0 and np.cross(cl[5] - cl[0], cl[10] - cl[5]) >= -1e-9,
          '반시계 (아래 직선에서 +x 로 가면 왼쪽으로 돈다)')
    _, _, rv = centerline(yml, reverse=True)
    check(rv is not None and rv[1, 0] < rv[0, 0], '--reverse 는 반대로 돈다')
    blocked = tfm.wall_mask(m, 0.01)
    W = tfm.ray_widths(m, blocked, cl, 2.0)
    check(np.all(np.abs(W - 0.5) < 0.08), '좌우 폭 0.5 m',
          f'오른쪽 {W[:,0].min():.2f}~{W[:,0].max():.2f}, '
          f'왼쪽 {W[:,1].min():.2f}~{W[:,1].max():.2f}')
    clr = tfm.clearance(m, blocked, cl)
    check(abs(clr.min() - 0.5) < 0.06, '벽 여유 0.5 m', f'{clr.min():.3f}')

print('\n③ 바깥벽에 10 cm 틈 + 바깥이 빈 방')
yml_g = write_map('gap', gap=True)
_, ring0, cl0 = centerline(yml_g, seal=0.0)
ok0 = cl0 is not None and dist_stadium(cl0).max() < 0.04
check(not ok0, '메우지 않으면 틀린다 (틈으로 방과 이어짐)',
      ring0['err'] or (f'최대 {dist_stadium(cl0).max():.2f} m 벗어남'
                       if cl0 is not None else ''))
_, ring1, cl1 = centerline(yml_g)
check(cl1 is not None and dist_stadium(cl1).max() < 0.05,
      '기본 --seal 0.10 이면 바로잡힌다',
      ring1['err'] or f'최대 {dist_stadium(cl1).max()*100:.1f} cm')

print('\n④ 미지 잡음과 기둥 (벽에서 30 cm)')
yml_s = write_map('specks', specks=True, pole=0.30)
m_s, ring_s, cl_s = centerline(yml_s)
check(cl_s is not None, '고리 찾음', ring_s['err'] or '')
if cl_s is not None:
    check(len(ring_s['islands']) == 1, '기둥만 장애물로 알린다 (잡음 점은 조용히)',
          f'{len(ring_s["islands"])}개')
    check(dist_stadium(cl_s).max() < 0.05, '중심선은 기둥·잡음에 안 끌린다',
          f'최대 {dist_stadium(cl_s).max()*100:.1f} cm')
    raw = ~m_s['free']
    wm = tfm.wall_mask(m_s, 0.01)
    c_raw = tfm.clearance(m_s, raw, cl_s).min()
    c_wm = tfm.clearance(m_s, wm, cl_s).min()
    check(c_wm > c_raw + 0.05, '미지 점은 벽으로 안 친다 (남는 건 기둥까지 약 10 cm)',
          f'그대로 {c_raw:.3f} m → 정리 {c_wm:.3f} m')
    check(c_wm < 0.30, '기둥(점유)은 그대로 벽', f'기둥 쪽 여유 {c_wm:.3f} m')

yml_p = write_map('pole_near', pole=0.10)
m_p, ring_p, cl_p = centerline(yml_p)
check(cl_p is not None and dist_stadium(cl_p).max() < 0.15,
      '벽에 붙은 기둥(10 cm) — 중심선은 조금만 밀린다',
      f'최대 편차 {dist_stadium(cl_p).max()*100:.1f} cm' if cl_p is not None else '')
if cl_p is not None:
    cp = tfm.clearance(m_p, tfm.wall_mask(m_p, 0.01), cl_p).min()
    check(cp < 0.45, '그 기둥도 벽 여유에 들어간다', f'{cp:.3f} m')

print('\n⑤ 고리가 아닌 맵은 거절')
yml_n = write_map('plain2')
m_n = tfm.load_map(yml_n)
m_n['free'][:] = m_n['free'] | (dist_stadium(np.c_[
    tfm.pix_to_world(m_n, *np.nonzero(np.ones_like(m_n['free'])))]).reshape(
    m_n['free'].shape) < 1.6)                        # 가운데 섬을 지운다
ring_n = tfm.extract_ring(m_n, (0.0, -RAD), 0.0)
check(ring_n['err'] is not None, '섬이 없으면 오류', ring_n['err'] or '')

# ─────────────────────────────────────────────────────────────────────
print('\n⑥ map ↔ mocap 변환 되찾기')
TRUE = dict(theta=0.3, t=np.array([1.2, -0.7]), d=np.array([0.03, -0.01]),
            delta=0.02, lag=0.04)
rng = np.random.default_rng(0)
path = stadium(200)
s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
v = 1.5
tm = np.arange(0.0, 3 * s[-1] / v, 0.01)            # mocap 100 Hz, 3바퀴
ss = (tm * v) % s[-1]
xm = np.interp(ss, s, path[:, 0])
ym = np.interp(ss, s, path[:, 1])
hd = np.unwrap(np.arctan2(np.gradient(ym), np.gradient(xm)))
# 정답 mocap → 라이다 (map 프레임, base_link 오차 포함, 늦게 찍힘)
psi_map = hd - TRUE['theta'] - TRUE['delta']
pm = (np.c_[xm, ym] - TRUE['t']) @ mma.rot(-TRUE['theta']).T
pm = pm - np.einsum('nij,j->ni', np.stack([mma.rot(a) for a in psi_map]),
                    TRUE['d'])
tl_idx = np.arange(0, len(tm), 4)                   # 라이다 25 Hz
log = os.path.join(TMP, 'align.csv')
with open(log, 'w') as f:
    f.write('src,stamp,recv,x,y,yaw\n')
    for i in range(len(tm)):
        f.write(f'm,{tm[i]+100:.6f},0,{xm[i]+rng.normal(0,0.001):.5f},'
                f'{ym[i]+rng.normal(0,0.001):.5f},{mma.wrap(hd[i]):.6f}\n')
    for i in tl_idx:
        f.write(f'l,{tm[i]+100+TRUE["lag"]:.6f},0,'
                f'{pm[i,0]+rng.normal(0,0.01):.5f},{pm[i,1]+rng.normal(0,0.01):.5f},'
                f'{mma.wrap(psi_map[i]+rng.normal(0,0.005)):.6f}\n')
cfg0 = dict(topic='/car/pose', off_x=0.0, off_y=0.0, yaw_off=0.0)
M, L = mma.load_log(log, cfg0)
lag, rho, ok = mma.estimate_lag(M, L)
check(ok and abs(lag - TRUE['lag']) < 0.008, '지연',
      f'{lag*1000:.1f} ms (정답 {TRUE["lag"]*1000:.0f}), 상관 {rho:.3f}')
pr = mma.pair(M, L, lag)
T, keep, thr = mma.robust_fit(pr, lever=True)
check(abs(T['theta'] - TRUE['theta']) < 2e-3, 'θ',
      f'{T["theta"]:.4f} (정답 {TRUE["theta"]})')
check(np.linalg.norm(T['t'] - TRUE['t']) < 0.005, 't',
      f'{T["t"].round(4)} (정답 {TRUE["t"]})')
check(np.linalg.norm(T['d'] - TRUE['d']) < 0.006, 'base_link 위치 차이 d',
      f'{(T["d"]*100).round(2)} cm (정답 {TRUE["d"]*100})')
check(abs(T['delta'] - TRUE['delta']) < 2e-3, 'base_link yaw 차이 δ',
      f'{T["delta"]:.4f} (정답 {TRUE["delta"]})')
E = mma.errors(T, pr, True)
check(np.sqrt(np.mean(E['pos'] ** 2)) < 0.02, '보정 후 위치 RMSE ≈ 잡음 1 cm',
      f'{np.sqrt(np.mean(E["pos"]**2))*100:.2f} cm')
pr0 = mma.pair(M, L, 0.0)
E0 = mma.errors(T, pr0, False)
check(np.sqrt(np.mean(E0['lon'] ** 2)) > np.sqrt(np.mean(E0['lat'] ** 2)),
      '지연을 안 빼면 오차가 앞뒤로 쏠린다',
      f'앞뒤 {np.sqrt(np.mean(E0["lon"]**2))*100:.1f} vs '
      f'좌우 {np.sqrt(np.mean(E0["lat"]**2))*100:.1f} cm')

print('\n⑦ mocap offset 적용이 mocap_bridge 와 같다')
cfg1 = dict(topic='/car/pose', off_x=-0.1, off_y=-0.034, yaw_off=0.062)
M1, _ = mma.load_log(log, cfg1)
y0 = M['yaw'][123] + 0.062
exp = np.array([M['x'][123] + math.cos(y0) * -0.1 - math.sin(y0) * -0.034,
                M['y'][123] + math.sin(y0) * -0.1 + math.cos(y0) * -0.034])
check(np.allclose([M1['x'][123], M1['y'][123]], exp, atol=1e-9) and
      abs(M1['yaw'][123] - y0) < 1e-9, 'offset 공식')

print('\n⑧ 트랙 파일 옮기기 왕복')
src = os.path.join(TMP, 'prep_t.csv')
with open(src, 'w') as f:
    f.write('s,x,y,heading,curvature,track_width\n')
    for i, (x, y) in enumerate(path[::50]):
        f.write(f'{i*0.2:.3f},{x:.6f},{y:.6f},{0.1*i:.6f},0.5,1.0\n')
fwd = os.path.join(TMP, 'fwd.csv')
bwd = os.path.join(TMP, 'bwd.csv')
th, t = TRUE['theta'], TRUE['t']
mma.transform_csv(src, fwd, th, t, 'test')
mma.transform_csv(fwd, bwd, -th, -(mma.rot(-th) @ t), 'test')
A = np.loadtxt(src, delimiter=',', skiprows=1)
F = np.loadtxt(fwd, delimiter=',', skiprows=1)
B = np.loadtxt(bwd, delimiter=',', skiprows=1)
check(np.allclose(F[:, 1:3], A[:, 1:3] @ mma.rot(th).T + t, atol=1e-5),
      'x,y 가 변환대로')
check(np.allclose(mma.wrap(F[:, 3] - A[:, 3] - th), 0, atol=1e-5),
      'heading 열도 같이 돈다')
check(np.allclose(F[:, [0, 4, 5]], A[:, [0, 4, 5]]), 's·곡률·폭은 그대로')
check(np.allclose(B[:, 1:3], A[:, 1:3], atol=1e-5) and
      np.allclose(mma.wrap(B[:, 3] - A[:, 3]), 0, atol=1e-5), '왕복하면 원래대로')
wp = os.path.join(TMP, 'wp.csv')
with open(wp, 'w') as f:
    f.write('# x,y   test\n1.0,2.0\n3.0,4.0\n')
mma.transform_csv(wp, wp + '.o', th, t, 'note')
lines = open(wp + '.o').read().splitlines()
check(lines[0].startswith('# x,y') and lines[1].startswith('# note'),
      '# x,y 머리줄 유지 + 변환 기록')
check(np.allclose([float(v) for v in lines[2].split(',')],
                  mma.rot(th) @ [1.0, 2.0] + t, atol=1e-5), '머리줄 없는 x,y')

print()
if FAIL:
    print(f'✗ {len(FAIL)}건 실패: {", ".join(FAIL)}')
    sys.exit(1)
print('✓ 전부 통과')
