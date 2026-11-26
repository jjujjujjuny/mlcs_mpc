# 폐루프 그림 (2026-11-26): main.py --log 의 cl_kf.csv·cl_gyro_zero.csv (DIR) 와 11-10 실차 로그를 겹친다
#   for e in "kf --vy kf" "gyro --vy zero"; do main.py --track 1108_7_track_mocap --laps 4 --set v_max=2.0 wall_margin=0.28 --car friction=0.41 --realtime --estimator $e --log DIR/cl_<kf|gyro_zero>.csv; done
#   python3 docs/notes/figs/make_closedloop_fig.py DIR docs/notes/figs/closedloop_real_vs_sim.png
import sys, numpy as np, pandas as pd, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
sys.path.insert(0,'tools/analysis'); import sim_fidelity as F
fs=[f for f in fm.findSystemFonts() if 'Nanum' in f or ('Noto' in f and 'CJK' in f)]
fm.fontManager.addfont(fs[0]); plt.rcParams['font.family']=fm.FontProperties(fname=fs[0]).get_name(); plt.rcParams['axes.unicode_minus']=False
S=sys.argv[1]
tr=pd.read_csv('src/mlcs_mpc/mlcs_mpc/env/track/data/1108_7_track_mocap/1108_7_track_mocap.csv'); cx,cy=tr.x_m.values,tr.y_m.values
cs=np.r_[0,np.cumsum(np.hypot(np.diff(cx),np.diff(cy)))]
def sproj(x,y):
    i=np.argmin((x[:,None]-cx[None])**2+(y[:,None]-cy[None])**2,1); return cs[i]
d=F.load('/home/user/mlcs_data/runs_20261110_mocap_HvsC/drive_20261110_135701_mocap.csv')
t=d.stamp.values-d.stamp.values[0]; m=(t>13)&(t<75)
x,y,v=d.x.values[m],d.y.values[m],np.hypot(d.vx_t.values,d.vy_t.values)[m]; tt=t[m]; s=sproj(x,y)
# 랩 나누기 (s 가 크게 줄면 새 랩)
cut=np.r_[0,np.where(np.diff(s)<-5)[0]+1,len(s)]
laps=[(cut[k],cut[k+1]) for k in range(1,len(cut)-2)]
lt=[tt[b]-tt[a] for a,b in laps]
fig,(a1,a2)=plt.subplots(1,2,figsize=(15,6.6),gridspec_kw=dict(width_ratios=[1,1.35]))
a1.plot(cx,cy,':',c='0.55',lw=1,label='트랙 센터라인')
for k,(a,b) in enumerate(laps): a1.plot(x[a:b],y[a:b],c='k',lw=1.0,alpha=0.35,label=f'실차 MPC ({len(laps)} 랩)' if k==0 else None)
for tag,c,lab in (('kf','tab:red','시뮬 MPC — 지금 추정 (mocap KF)'),('gyro_zero','tab:blue','시뮬 MPC — 바퀴 속도·자이로 (KF 없음)')):
    q=pd.read_csv(f'{S}/cl_{tag}.csv'); q=q[q.t>6.5]
    a1.plot(q.x,q.y,c=c,lw=1.4,label=lab)
a1.set_aspect('equal'); a1.legend(loc='center',fontsize=9); a1.set_title('① 궤적 — 실차 vs 시뮬 (같은 MPC, 같은 트랙)'); a1.set_xlabel('x [m]'); a1.set_ylabel('y [m]')
for k,(a,b) in enumerate(laps): a2.plot(s[a:b],v[a:b],c='k',lw=0.8,alpha=0.3,label='실차' if k==0 else None)
for tag,c,lab in (('kf','tab:red','시뮬 (kf)'),('gyro_zero','tab:blue','시뮬 (바퀴·자이로)')):
    q=pd.read_csv(f'{S}/cl_{tag}.csv'); q=q[q.t>6.5]; sq=sproj(q.x.values,q.y.values); o=np.argsort(sq)
    a2.plot(sq[o],np.hypot(q.vx,q.vy).values[o],'.',ms=2,c=c,label=lab)
a2.set_xlabel('트랙 위치 s [m]'); a2.set_ylabel('속도 [m/s]'); a2.set_title('② 트랙 위치별 속도'); a2.legend(fontsize=9,markerscale=5,loc='lower left'); a2.set_ylim(0.8,2.3)
txt=(f"실차: 2026-11-10 연구실 mocap, MPCC v_max 2.0, 랩 {min(lt):.2f}~{max(lt):.2f} s ({len(lt)} 랩)\n"
     "시뮬: 같은 트랙·v_max·벽 여유 0.28, 차 마찰 0.41, 서보 τ 0.03, 실차 비동기 타이밍\n"
     "      랩 (2~4 랩) — kf 5.10~5.13 s, 바퀴·자이로 5.19~5.29 s, 이상적 상태 5.14 s. 모두 벽 0 회")
a2.text(0.02,0.98,txt,transform=a2.transAxes,va='top',fontsize=9.5,bbox=dict(fc='white',ec='0.7'))
fig.tight_layout(); fig.savefig(sys.argv[2],dpi=130); print(lt)
