#!/usr/bin/env python3
"""
plot_real_vs_sim — 실제 mocap 궤적과, 같은 /drive 명령을 시뮬 (원저자 ODE + 서보·바퀴 지연) 에 넣은 궤적을 겹쳐 그린다.

  ① 1 s 마다 실제 상태에서 다시 시작 (짧은 구간 일치)  ② 한 랩 개루프 (되맞춤 없음)  ③ vx·r 시계열
  재생 방식은 sim_fidelity.run 과 같다 (cmd_recv 시각 그대로, 시작 조향은 지난 명령을 서보로 거른 값).

  python3 tools/analysis/plot_real_vs_sim.py [drive_*.csv] [--out docs/notes/figs/traj_real_vs_sim.png]
  기본 로그: ~/mlcs_data/runs_20261110_mocap_HvsC/drive_20261110_135701_mocap.csv (v_max 2.0, 랩 5.1~5.3 s)
"""
import argparse
import os
import math
import sys

import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_fidelity as F
from sim_fidelity import *

def rollout(d, i0, T, var={'friction':0.41}):
    m, sim = F.make(var); ll = sim.lowlevel
    tau = ll.cfg.steer_tau or m.steering_tau; dly = ll.cfg.steer_delay
    st = d.stamp.values
    ev_t, k = np.unique(d.cmd_recv.values, return_index=True); ok = np.isfinite(ev_t); ev_t, k = ev_t[ok], k[ok]
    ev_s, ev_v = d.cmd_steer.fillna(0).values[k], d.cmd_speed.fillna(0).values[k]
    phys = [command_to_steer(s, MAP_CMD, MAP_ACT, None) for s in ev_s]
    t0 = st[i0]; de, tp, dref = 0.0, None, 0.0
    sel = (ev_t > t0-1.5) & (ev_t <= t0-dly)
    for tt, a in zip(ev_t[sel], np.array(phys)[sel]):
        if tp is not None: de += (1-math.exp(-(tt-tp)/tau))*(dref-de)
        dref, tp = a, tt
    if tp is not None: de += (1-math.exp(-(t0-dly-tp)/tau))*(dref-de)
    sim.reset(d.x[i0], d.y[i0], d.yaw[i0])
    sim.x[3:8] = [d.vx_t[i0], d.vy_t[i0]+m.lr*d.r_t[i0], d.r_t[i0], d.motor_rpm[i0]/4781.5*ll.cfg.speed_scale, de]
    sim.t = t0; ll.reset()
    jn = np.searchsorted(ev_t, t0, side='right')
    if jn > 0:
        ll.d_buf.cur = phys[jn-1] if dly == 0 else de
        ll.w_buf.cur = ev_v[jn-1]*ll.cfg.speed_scale
    out = []
    while sim.t < t0+T-1e-9:
        nxt = min(sim.t+0.01, ev_t[jn] if jn < len(ev_t) else 1e18)
        sim.step(None, None, dt=max(nxt-sim.t, 1e-4))
        if jn < len(ev_t) and sim.t >= ev_t[jn]-1e-9:
            sim.command(ev_s[jn], ev_v[jn]); jn += 1
        px, py, pyaw = sim.pose(); tw = sim.twist()
        out.append((sim.t-t0, px, py, pyaw, tw[0], tw[2]))
    return np.array(out)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=os.path.expanduser('~/mlcs_data/runs_20261110_mocap_HvsC/drive_20261110_135701_mocap.csv'))
    ap.add_argument('--out', default=os.path.join(F.ROOT, 'docs', 'notes', 'figs', 'traj_real_vs_sim.png'))
    ap.add_argument('--start', type=float, default=20.0, help='그릴 구간 시작 (로그 처음부터 s)')
    a = ap.parse_args()
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    import pandas as pd
    from matplotlib import font_manager as fm
    fs=[f for f in fm.findSystemFonts() if 'Nanum' in f or 'Noto' in f and 'CJK' in f]
    if fs: fm.fontManager.addfont(fs[0]); plt.rcParams['font.family']=fm.FontProperties(fname=fs[0]).get_name()
    plt.rcParams['axes.unicode_minus']=False
    d=load(a.log)
    st=d.stamp.values; t=st-st[0]
    tr=pd.read_csv(os.path.join(F.ROOT,'src')+'/mlcs_mpc/mlcs_mpc/env/track/data/1108_7_track_mocap/1108_7_track_mocap.csv')
    cx,cy=tr.x_m.values,tr.y_m.values; dx,dy=np.gradient(cx),np.gradient(cy); n=np.hypot(dx,dy); nx,ny=-dy/n,dx/n
    def track(ax):
        ax.plot(cx,cy,':',c='0.55',lw=1.0,label='트랙 센터라인')
        ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    A,B=a.start,a.start+10.0   # 두 랩쯤
    ia,ib=np.searchsorted(t,A),np.searchsorted(t,B)
    segs=[]; T=1.0; s=A
    while s+T<=B+1e-9:
        segs.append(rollout(d,np.searchsorted(t,s),T)); s+=T
    full=rollout(d,ia,5.2)       # 한 랩, 되맞춤 없이
    fig=plt.figure(figsize=(14,10))
    ax1=fig.add_subplot(2,2,1); track(ax1)
    ax1.plot(d.x[ia:ib],d.y[ia:ib],'k',lw=2.2,label='실제 (mocap)')
    for k,sg in enumerate(segs):
        ax1.plot(sg[:,1],sg[:,2],c='tab:red',lw=1.4,label='시뮬 (1 s 마다 실제 상태로 다시 시작)' if k==0 else None)
        ax1.plot(sg[0,1],sg[0,2],'o',c='tab:red',ms=3)
    ax1.set_title(f'① 실제 명령을 시뮬에 그대로 넣기 — 1 s 구간 ({A:.0f}~{B:.0f} s)')
    ax1.legend(loc='center',fontsize=9)
    ax2=fig.add_subplot(2,2,2); track(ax2)
    j=np.searchsorted(t,A+5.2)
    ax2.plot(d.x[ia:j],d.y[ia:j],'k',lw=2.2,label='실제 (mocap)')
    ax2.plot(full[:,1],full[:,2],c='tab:blue',lw=1.6,label='시뮬 (한 번 시작, 되맞춤 없이 5.2 s)')
    for h in (1,2,3,4,5):
        q=full[np.searchsorted(full[:,0],h)]; r=np.searchsorted(t,A+h)
        ax2.plot([q[1],d.x[r]],[q[2],d.y[r]],c='0.5',lw=0.8); ax2.text(q[1],q[2],f' {h}s',fontsize=8,c='tab:blue')
    ax2.set_title('② 한 랩 개루프 (피드백 없이) — 시간 지날수록 벌어짐'); ax2.legend(loc='center',fontsize=9)
    # 시계열
    ax3=fig.add_subplot(4,2,5); ax4=fig.add_subplot(4,2,7,sharex=ax3)
    tt=t[ia:ib]
    ax3.plot(tt,d.vx_t[ia:ib],'k',lw=1.6,label='실제'); ax4.plot(tt,d.r_t[ia:ib],'k',lw=1.6)
    ax3.plot(tt,d.cmd_speed[ia:ib],c='tab:green',lw=1,ls='--',label='명령 바퀴 속도')
    for k,sg in enumerate(segs):
        t0=A+k*T; ax3.plot(t0+sg[:,0],sg[:,4],c='tab:red',lw=1.2,label='시뮬' if k==0 else None); ax4.plot(t0+sg[:,0],sg[:,5],c='tab:red',lw=1.2)
    ax3.set_ylabel('vx [m/s]'); ax4.set_ylabel('yaw rate r [rad/s]'); ax4.set_xlabel('time [s]'); ax3.legend(fontsize=8,ncol=3)
    ax3.set_title('③ 속도·회전 속도 (① 과 같은 구간)')
    # 오차 통계 (전 주행, 1 s)
    mv=t[d.vx_t.values>0.5]; s0,s1=mv[0]+0.5,mv[-1]-1.0
    errs=[]; s=s0
    while s+1<=s1:
        sg=rollout(d,np.searchsorted(t,s),1.0); r=np.searchsorted(t,s+1.0)
        errs.append(np.hypot(sg[-1,1]-d.x[r],sg[-1,2]-d.y[r])); s+=0.5
    errs=np.array(errs)*100
    full_e=[np.hypot(full[np.searchsorted(full[:,0],h)-1,1]-d.x[np.searchsorted(t,A+h)],full[np.searchsorted(full[:,0],h)-1,2]-d.y[np.searchsorted(t,A+h)])*100 for h in (1,2,3,4,5)]
    ax5=fig.add_subplot(2,2,4); ax5.axis('off')
    txt=(f"주행: 2026-11-10 연구실 mocap, 1108_7 트랙 (가상 벽), MPCC v_max 2.0 m/s, 랩 5.1~5.3 s\n"
         f"시뮬: HyperMPC 원저자 ODE (Pacejka 단일 궤적) + 서보 1차 τ 0.03 s + 바퀴 속도 1차 지연, 마찰 0.41\n"
         f"입력: 실차가 받은 /drive (조향·바퀴 속도) 를 받은 시각 그대로\n\n"
         f"1 s 뒤 위치 오차 ({s0:.0f}~{s1:.0f} s, 0.5 s 간격 {len(errs)} 구간)\n"
         f"   중앙 {np.median(errs):.1f} cm,  90 % {np.percentile(errs,90):.1f} cm,  최대 {errs.max():.1f} cm\n\n"
         f"② 되맞춤 없는 한 랩 개루프 위치 오차\n   "+',  '.join(f'{h} s {e:.0f} cm' for h,e in zip((1,2,3,4,5),full_e))+
         "\n\n→ 짧은 구간에서는 시뮬이 실제 궤적을 잘 따라간다.\n   한 랩 개루프는 작은 차이가 쌓여 벌어진다 (실차는 MPC 피드백으로 보정).")
    ax5.text(0,0.95,txt,va='top',fontsize=10.5,linespacing=1.6)
    fig.tight_layout(); fig.savefig(a.out,dpi=130)
    print(a.out, np.median(errs),np.percentile(errs,90),full_e)

if __name__ == '__main__':
    main()
