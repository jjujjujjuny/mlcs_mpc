# O자 트랙 실차 ↔ 시뮬 궤적 비교 (2026-11-27). 위치는 모두 앞바퀴 축 (실차 11-26 로그 기준점).
#   실차: 11-26 연구실 mocap, sensor 속도 상태 (바퀴·자이로), 끊김 없는 깨끗한 랩만
#   시뮬: for v in 1.0 1.5 2.0 2.5 3.0; do main.py --track o_track --laps 4 --set v_max=$v wall_margin=0.10 --car friction=0.41
#           --config tools/analysis/jetson_hmpc_params.yaml --lowlevel speed_scale=1.01 --realtime --estimator gyro --vy zero --log SIM/sensor_$v.csv; done
#   python3 docs/notes/figs/make_otrack_fig.py RUNS SIM docs/notes/figs/otrack_real_vs_sim.png docs/notes/figs/otrack_v2.0.gif
import sys, numpy as np, pandas as pd, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm, animation
from scipy.signal import savgol_filter
sys.path.insert(0, 'src/mlcs_mpc')
from mlcs_mpc.env.track.track import Track
from mlcs_mpc.env.track import find
fs = [f for f in fm.findSystemFonts() if 'Nanum' in f or ('Noto' in f and 'Sans' in f and 'CJK' in f)]
fm.fontManager.addfont(fs[0]); plt.rcParams['font.family'] = fm.FontProperties(fname=fs[0]).get_name()
plt.rcParams['axes.unicode_minus'] = False

RUNS, SIM, PNG, GIF = sys.argv[1:5]
L = 0.33                                   # 휠베이스 (시뮬 pose 는 뒤축 → 앞축 = +L)
REAL = {'1.0': ['202342', '211043'], '1.5': ['211859'], '2.0': ['212405'], '2.5': ['212844'], '3.0': ['213107']}
TR = Track(find('o_track')); LEN = TR.length
sg = np.linspace(0, LEN, 600)
wl = np.interp(sg, np.r_[TR.s, LEN], np.r_[TR.w_left, TR.w_left[0]])
wr = np.interp(sg, np.r_[TR.s, LEN], np.r_[TR.w_right, TR.w_right[0]])
LX, LY = TR.xy(sg, wl); RX, RY = TR.xy(sg, -wr); CX, CY = TR.xy(sg, 0 * sg)


def prep(t, x, y, yaw, cmd_speed):
    """100 Hz 로 다시 뽑고 s·n·속도 (위치 미분) 를 붙인다."""
    tt = np.arange(t[0], t[-1], 0.01)
    yw = np.interp(tt, t, np.unwrap(yaw)); X = np.interp(tt, t, x); Y = np.interp(tt, t, y)
    cs = np.interp(tt, t, cmd_speed)
    TR.reset(); f = np.array([TR.frenet(a, b, c) for a, b, c in zip(X, Y, yw)])
    vx = savgol_filter(X, 21, 2, deriv=1, delta=0.01); vy = savgol_filter(Y, 21, 2, deriv=1, delta=0.01)
    return dict(t=tt, x=X, y=Y, yaw=yw, s=f[:, 0], n=f[:, 1], mu=f[:, 2], v=np.hypot(vx, vy), cmd=cs)


def laps(d, vmax):
    """s 가 LEN 배수를 지나는 곳으로 나눈다. 완주하고 멈칫 (속도 < 0.5 v_max, 명령 < 0.3) 이 없는 랩만."""
    m0 = np.ceil(d['s'][0] / LEN); m1 = np.floor(d['s'][-1] / LEN)
    idx = [int(np.searchsorted(d['s'], m * LEN)) for m in np.arange(m0, m1 + 1)]
    out = []
    for a, b in zip(idx[:-1], idx[1:]):
        if a == 0 or b >= len(d['t']) - 1: continue
        if d['v'][a:b].min() < 0.5 * vmax or d['cmd'][a:b].min() < 0.3: continue
        if not np.all(np.diff(d['s'][a:b]) > -0.05): continue
        out.append((a, b))
    return out


def real(v):
    res = []
    for rid in REAL[v]:
        q = pd.read_csv(f'{RUNS}/drive_20261126_{rid}_mocap.csv')
        q = q[q.cmd_speed > 0.05]
        d = prep(q.time.values, q.x.values, q.y.values, q.yaw.values, q.cmd_speed.values)
        res += [(d, a, b) for a, b in laps(d, float(v))]
    return res


def sim(v):
    q = pd.read_csv(f'{SIM}/sensor_{v}.csv')
    x = q.x.values + L * np.cos(q.yaw.values); y = q.y.values + L * np.sin(q.yaw.values)
    d = prep(q.t.values, x, y, q.yaw.values, q.cmd_speed.values)
    return [(d, a, b) for a, b in laps(d, float(v))]      # 정지 출발 랩은 a == 0 이라 빠진다


def stats(L_):
    lt = [d['t'][b] - d['t'][a] for d, a, b in L_]
    n = np.concatenate([d['n'][a:b] for d, a, b in L_]); mu = np.concatenate([d['mu'][a:b] for d, a, b in L_])
    return lt, n.mean() * 100, np.sqrt((n ** 2).mean()) * 100, np.degrees(np.sqrt((mu ** 2).mean()))


def draw_track(ax):
    ax.fill(np.r_[LX, RX[::-1]], np.r_[LY, RY[::-1]], c='0.93', zorder=0)
    ax.plot(LX, LY, c='0.3', lw=1.2); ax.plot(RX, RY, c='0.3', lw=1.2)
    ax.plot(CX, CY, ':', c='0.6', lw=0.8)
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])


V = ['1.0', '1.5', '2.0', '2.5', '3.0']
R = {v: real(v) for v in V}; Sm = {v: sim(v) for v in V}
fig = plt.figure(figsize=(17, 11.5))
gs = fig.add_gridspec(3, 6, height_ratios=[1, 1, 0.9])
rows = []
for k, v in enumerate(V):
    ax = fig.add_subplot(gs[k // 3, 2 * (k % 3):2 * (k % 3) + 2]); draw_track(ax)
    for j, (d, a, b) in enumerate(R[v]):
        ax.plot(d['x'][a:b], d['y'][a:b], c='k', lw=1.0, alpha=0.55, label='실차' if j == 0 else None)
    for j, (d, a, b) in enumerate(Sm[v]):
        ax.plot(d['x'][a:b], d['y'][a:b], c='tab:blue', lw=1.2, alpha=0.8, label='시뮬' if j == 0 else None)
    rl, rn, rr, rh = stats(R[v]); sl, sn, sr, sh = stats(Sm[v])
    rows.append((v, len(rl), min(rl), max(rl), rn, rr, rh, len(sl), np.mean(sl), sn, sr, sh))
    ax.set_title(f'v_max {v} m/s — 랩 실차 {min(rl):.2f}~{max(rl):.2f} s ({len(rl)}) / 시뮬 {np.mean(sl):.2f} s', fontsize=10.5)
    if k == 0:
        ax.legend(loc='center', fontsize=10)
        ax.annotate('', xy=(CX[8], CY[8]), xytext=(CX[0], CY[0]), arrowprops=dict(arrowstyle='->', color='tab:red', lw=2))
        ax.text(CX[0], CY[0], '  출발선·방향 (시계)', color='tab:red', fontsize=9)
# 표
ax = fig.add_subplot(gs[1, 4:]); ax.axis('off')
cell = [[v, f'{a:.2f}~{b:.2f}', f'{sl:.2f}', f'{rn:+.0f} / {rr:.0f}', f'{sn:+.0f} / {sr:.0f}', f'{rh:.0f}° / {sh:.0f}°']
        for v, nr, a, b, rn, rr, rh, ns, sl, sn, sr, sh in rows]
tb = ax.table(cellText=cell, colLabels=['v_max', '랩 실차 (s)', '랩 시뮬', '횡오차 실차\n평균/rms cm', '횡오차 시뮬', '헤딩 rms\n실차/시뮬'],
              loc='center', cellLoc='center'); tb.auto_set_font_size(False); tb.set_fontsize(9.5); tb.scale(1, 1.9)
ax.set_title('깨끗한 랩 비교 (앞축 기준, 횡오차 + = 왼쪽 = 바깥)', fontsize=10.5)
# v2.0 트랙 위치별 속도·횡오차
for col, (key, lab, sc) in enumerate((('v', '속도 [m/s] (위치 미분)', 1), ('n', '횡오차 n [cm] (+ 왼쪽 = 바깥)', 100))):
    ax = fig.add_subplot(gs[2, 3 * col:3 * col + 3])
    for v, c in (('2.0', 'k'),):
        for j, (d, a, b) in enumerate(R[v]):
            ax.plot(np.mod(d['s'][a:b], LEN), d[key][a:b] * sc, c='k', lw=0.9, alpha=0.5, label='실차 v2.0' if j == 0 else None)
        for j, (d, a, b) in enumerate(Sm[v]):
            ax.plot(np.mod(d['s'][a:b], LEN), d[key][a:b] * sc, c='tab:blue', lw=1.1, label='시뮬 v2.0' if j == 0 else None)
    ax2 = ax.twinx(); ax2.fill_between(sg, 0, TR.kappa_at(sg), color='tab:orange', alpha=0.12); ax2.set_ylabel('곡률 κ (주황)', color='tab:orange')
    ax.set_xlabel('트랙 위치 s [m]'); ax.set_ylabel(lab); ax.set_xlim(0, LEN); ax.legend(loc='lower left', fontsize=9)
    ax.set_title(['② v2.0 트랙 위치별 속도', '③ v2.0 트랙 위치별 횡오차'][col], fontsize=10.5)
fig.suptitle('O자 트랙 (12.3 m, 폭 0.5 m) — 실차 11-26 (속도 상태 = 바퀴·자이로) vs 시뮬 (젯슨 MPC 설정 그대로·여유 0.10, 서보 20 ms + 2.5 rad/s, 바퀴 τ 0.4·실제/명령 1.01, 마찰 0.41, 실차 비동기 타이밍)', fontsize=11.5)
fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(PNG, dpi=120)
for r in rows: print(r)

# ── 움짤: v2.0 깨끗한 랩 하나씩, 같은 출발선에서 동시에 ─────────────────────
_lt = [d['t'][b] - d['t'][a] for d, a, b in R['2.0']]; dr, ar, br = R['2.0'][int(np.argsort(_lt)[len(_lt) // 2])]   # 랩 시간 중앙값 랩
ds, as_, bs = Sm['2.0'][-1]
Tr, Ts = dr['t'][br] - dr['t'][ar], ds['t'][bs] - ds['t'][as_]
FPS = 20; T = max(Tr, Ts) + 0.6; frames = int(T * FPS)
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5.6), gridspec_kw=dict(width_ratios=[1.25, 1]))
draw_track(a1)
a1.plot(dr['x'][ar:br], dr['y'][ar:br], c='k', lw=0.6, alpha=0.15); a1.plot(ds['x'][as_:bs], ds['y'][as_:bs], c='tab:blue', lw=0.6, alpha=0.15)
tr_r, = a1.plot([], [], c='k', lw=2, label=f'실차 (랩 {Tr:.2f} s)'); tr_s, = a1.plot([], [], c='tab:blue', lw=2, label=f'시뮬 (랩 {Ts:.2f} s)')
car_r, = a1.plot([], [], c='k', lw=4, solid_capstyle='round'); car_s, = a1.plot([], [], c='tab:blue', lw=4, solid_capstyle='round')
a1.legend(loc='center', fontsize=10); clock = a1.text(0.02, 0.97, '', transform=a1.transAxes, va='top', fontsize=11)
a1.set_title('v_max 2.0 m/s — 같은 출발선에서 동시 출발 (앞축 위치, 막대 = 차 몸체)', fontsize=10)
a2.set_xlim(0, LEN); a2.set_ylim(1.6, 2.2); a2.set_xlabel('트랙 위치 s [m]'); a2.set_ylabel('속도 [m/s]')
a2b = a2.twinx(); a2b.fill_between(sg, 0, TR.kappa_at(sg), color='tab:orange', alpha=0.12); a2b.set_ylabel('곡률 κ (주황)', color='tab:orange')
vr, = a2.plot([], [], c='k', lw=1.5, label='실차'); vs, = a2.plot([], [], c='tab:blue', lw=1.5, label='시뮬'); a2.legend(loc='lower left')
a2.set_title('트랙 위치별 속도', fontsize=10)


def at(d, a, b, tq):
    i = min(a + int(round(tq / 0.01)), b); return i


def upd(f):
    tq = f / FPS; out = []
    for d, a, b, trl, car, vl in ((dr, ar, br, tr_r, car_r, vr), (ds, as_, bs, tr_s, car_s, vs)):
        i = at(d, a, b, tq); j = max(a, i - 120)
        trl.set_data(d['x'][j:i + 1], d['y'][j:i + 1])
        x, y, w = d['x'][i], d['y'][i], d['yaw'][i]
        car.set_data([x, x - L * np.cos(w)], [y, y - L * np.sin(w)])
        vl.set_data(np.mod(d['s'][a:i + 1], LEN), d['v'][a:i + 1])
    clock.set_text(f't = {tq:4.2f} s')
    return tr_r, tr_s, car_r, car_s, vr, vs, clock


fig.tight_layout()
animation.FuncAnimation(fig, upd, frames=frames, interval=1000 / FPS, blit=True).save(GIF, writer=animation.PillowWriter(fps=FPS), dpi=80)
print('gif', Tr, Ts, frames)
