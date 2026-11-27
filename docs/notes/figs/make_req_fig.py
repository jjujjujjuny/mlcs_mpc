# 연구원님 요청 1~3 (2026-11-27) 그림. DIR = 시뮬 기록 (ll_{full,none,none101,vesc}_v, c_{o_track,biglab}_v, + 논문 가중치 O자 = ll_full_v)
#   python3 docs/notes/figs/make_req_fig.py DIR docs/notes/figs/req_lowlevel_center_speed.png
import sys, re, os, numpy as np, pandas as pd, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
sys.path.insert(0, 'src/mlcs_mpc')
from mlcs_mpc.env.track.track import Track
from mlcs_mpc.env.track import find
fs = [f for f in fm.findSystemFonts() if 'Noto' in f and 'Sans' in f and 'CJK' in f]
fm.fontManager.addfont(fs[0]); plt.rcParams['font.family'] = fm.FontProperties(fname=fs[0]).get_name(); plt.rcParams['axes.unicode_minus'] = False
D, OUT = sys.argv[1:3]; L = 0.33


def ana(tag, track):
    d = pd.read_csv(f'{D}/{tag}.csv'); txt = open(f'{D}/{tag}.txt').read()
    laps = [float(x) for x in re.findall(r'랩 \d: ([\d.]+) s', txt)]
    m = d.t > laps[0]; tr = Track(find(track)); f = []
    for x, y, w in zip(d.x, d.y, d.yaw):
        f.append(tr.frenet(x + L * np.cos(w), y + L * np.sin(w), w))
    f = np.array(f)[m]
    return dict(laps=laps[1:], n_mean=f[:, 1].mean() * 100, n_rms=np.sqrt((f[:, 1] ** 2).mean()) * 100,
                head=np.degrees(np.sqrt((f[:, 2] ** 2).mean())), lim=(d.cmd_steer[m].abs() > 0.399).mean() * 100, d=d[m])


REAL = {'1.0': (11.28, 11.92, -10, 10, 13, 19), '1.5': (7.62, 7.99, -9, 9, 13, 28), '2.0': (5.84, 6.10, -6, 7, 11, 44),
        '2.5': (5.34, 5.49, 3, 8, 12, 55), '3.0': (5.30, 5.30, 4, 8, 13, 63)}
LL = [('full', '로우레벨 전부 (서보 20 ms+2.5 rad/s, VESC 램프, 바퀴 1.01)', 'tab:blue'),
      ('none', '로우레벨 없음 (MPC 모델과 같은 1차 지연)', 'tab:gray'),
      ('none101', '없음 + 바퀴 1.01 만', 'tab:olive'),
      ('vesc', 'VESC 펌웨어 모델 (속도 PID·모터 전류)', 'tab:red')]
fig = plt.figure(figsize=(17, 10.5)); gs = fig.add_gridspec(2, 3)
# ① 1번: 랩 · 조향 한계 비율 vs 실차
a1 = fig.add_subplot(gs[0, 0]); a2 = fig.add_subplot(gs[0, 1]); a3 = fig.add_subplot(gs[0, 2])
V = list(REAL); xv = np.array([float(v) for v in V])
mid = np.array([np.mean(REAL[v][:2]) for v in V])
a1.fill_between(xv, [(REAL[v][0] / m - 1) * 100 for v, m in zip(V, mid)], [(REAL[v][1] / m - 1) * 100 for v, m in zip(V, mid)], color='k', alpha=0.15, label='실차 범위')
err = {}
for k, (tag, lab, c) in enumerate(LL):
    r = [ana(f'll_{tag}_{v}', 'o_track') for v in V]
    a1.errorbar(xv + (k - 1.5) * 0.03, [(np.mean(q['laps']) / m - 1) * 100 for q, m in zip(r, mid)],
                yerr=[np.ptp(q['laps']) / 2 / m * 100 for q, m in zip(r, mid)], fmt='o-', c=c, ms=4, label=lab)
    a2.plot(xv, [q['lim'] for q in r], 'o-', c=c, ms=4)
    e = []
    for q, v in zip(r, V):
        R = REAL[v]; lap = np.mean(q['laps']); le = 0 if R[0] <= lap <= R[1] else min(abs(lap - R[0]), abs(lap - R[1]))
        e.append([le / np.mean(R[:2]) * 100, abs(q['n_mean'] - R[2]), abs(q['n_rms'] - R[3]), abs(q['head'] - R[4]), abs(q['lim'] - R[5]) / 5])
    err[tag] = np.mean(e, 0)
a2.plot(xv, [REAL[v][5] for v in V], 'ks--', lw=2, label='실차')
a1.set_xlabel('v_max [m/s]'); a1.set_ylabel('랩 − 실차 중앙 [%]'); a1.axhline(0, c='k', lw=0.6); a1.legend(fontsize=7.5, loc='upper left')
a1.set_title('① 1번 — O자, 실차 가중치: 랩 시간 (막대 = 랩 2~4 범위)', fontsize=10.5)
a2.set_xlabel('v_max [m/s]'); a2.set_ylabel('조향 한계 (±0.4) 비율 [%]'); a2.legend(fontsize=9); a2.set_title('② 1번 — 조향이 한계에 걸린 시간', fontsize=10.5)
w = 0.2; lab = ['랩 범위 밖 %', 'n 평균 cm', 'n rms cm', '헤딩 °', '조향 한계 %p ÷5']
for k, (tag, l, c) in enumerate(LL):
    a3.bar(np.arange(5) + (k - 1.5) * w, err[tag], w, color=c, label=tag)
a3.set_xticks(range(5)); a3.set_xticklabels(lab, fontsize=8.5); a3.set_ylabel('|시뮬 − 실차| 평균 (v1.0~3.0)'); a3.legend(fontsize=9)
a3.set_title('③ 1번 — 실차와 차이 (작을수록 실차와 같다)', fontsize=10.5)
# ④ 2·3번: 랩 vs v_max (중앙선) + O자 논문 값
a4 = fig.add_subplot(gs[1, 0]); VV = ['1.0', '1.5', '2.0', '2.5', '3.0', '3.5', '4.0']
for tr, c in (('o_track', 'tab:blue'), ('biglab', 'tab:green')):
    r = [ana(f'c_{tr}_{v}', tr) for v in VV]
    a4.errorbar([float(v) for v in VV], [np.mean(q['laps']) for q in r], yerr=[np.ptp(q['laps']) / 2 for q in r], fmt='o-', c=c, label=f'{tr} 중앙선 (q_n 5, q_mu 1)')
r = [ana(f'll_full_{v}', 'o_track') for v in V]
a4.plot(xv, [np.mean(q['laps']) for q in r], 'o--', c='tab:blue', alpha=0.5, label='o_track 실차 가중치 (논문 값)')
a4.fill_between(xv, [REAL[v][0] for v in V], [REAL[v][1] for v in V], color='k', alpha=0.15, label='o_track 실차 (11-26)')
a4.set_yscale('log'); a4.set_yticks([5, 6, 8, 10, 12]); a4.set_yticklabels(['5', '6', '8', '10', '12'])
a4.set_xlabel('v_max [m/s]'); a4.set_ylabel('랩 [s]'); a4.legend(fontsize=8.5); a4.set_title('④ 2·3번 — 랩 시간 (v3.0 넘으면 줄지 않고 흩어짐)', fontsize=10.5)
# ⑤ biglab 궤적 v2.0 / v4.0 중앙선
a5 = fig.add_subplot(gs[1, 1]); tr = Track(find('biglab'))
sg = np.linspace(0, tr.length, 600)
for sgn in (1, -1):
    wl = np.interp(sg, np.r_[tr.s, tr.length], np.r_[tr.w_left if sgn > 0 else tr.w_right, (tr.w_left if sgn > 0 else tr.w_right)[0]])
    X, Y = tr.xy(sg, sgn * wl); a5.plot(X, Y, c='0.3', lw=1.2)
for v, c in (('3.0', 'tab:blue'), ('4.0', 'tab:red')):
    q = ana(f'c_biglab_{v}', 'biglab')['d']
    a5.plot(q.x + L * np.cos(q.yaw), q.y + L * np.sin(q.yaw), c=c, lw=1, label=f'v_max {v}')
a5.set_aspect('equal'); a5.legend(fontsize=9); a5.set_title('⑤ biglab 궤적 (중앙선 가중치, 반시계, 앞축, 랩 2~4)', fontsize=10.5)
# ⑥ biglab v4.0 속도 프로파일
a6 = fig.add_subplot(gs[1, 2])
for v, c in (('2.0', 'tab:blue'), ('3.0', 'tab:orange'), ('4.0', 'tab:red')):
    q = ana(f'c_biglab_{v}', 'biglab')['d']; tr.reset()
    s = np.array([tr.frenet(x + L * np.cos(w), y + L * np.sin(w), w)[0] for x, y, w in zip(q.x, q.y, q.yaw)]) % tr.length
    a6.plot(s, q.vx, '.', ms=1.5, c=c, label=f'v_max {v}')
a6b = a6.twinx(); a6b.fill_between(sg, 0, tr.kappa_at(sg), color='tab:orange', alpha=0.12); a6b.set_ylabel('곡률 κ (주황)')
a6.set_xlabel('트랙 위치 s [m]'); a6.set_ylabel('vx [m/s]'); a6.legend(fontsize=9, markerscale=6); a6.set_title('⑥ biglab 속도 — 코너 속도가 상한 (마찰 0.41)', fontsize=10.5)
fig.suptitle('연구원님 요청 1~3 (시뮬, 실차 sensor 추정·비동기 타이밍·마찰 0.41)', fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(OUT, dpi=115)
for k, v in err.items(): print(k, np.round(v, 2))
