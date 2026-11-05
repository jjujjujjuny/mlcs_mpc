#!/usr/bin/env python3
"""
check_kinematic_fit.py — 주행 데이터에 운동학 모델이 얼마나 맞는지 잰다.

    python3 tools/check_kinematic_fit.py
    python3 tools/check_kinematic_fit.py ~/mlcs_data/hpm/train.csv

  입력은 prep_hpm_dataset.py 출력이다. 인자가 없으면 train, val 을 읽는다.
  test 는 나중 비교용으로 남겨 둔다.

  delta 는 측정값이 아니라 명령이다 (±0.40 제한, 3.2 rad/s 속도 제한 적용).
  v_y 는 무게중심 기준이라 뒤축 기준으로 되돌려 미끄러짐을 본다.

  수동 주행이라 과도 구간이 섞이고, 조향 맵은 |ay| 로 구간을 나눠 결과값으로
  고르는 편향이 있다. 조향 맵 자체는 원선회(./drive.sh rally)로 재야 한다.
"""
import argparse
import os

import numpy as np
import pandas as pd

ap = argparse.ArgumentParser()
ap.add_argument('csv', nargs='*', default=[
    os.path.expanduser(f'~/mlcs_data/hpm/{n}.csv') for n in ('train', 'val')])
a = ap.parse_args()

L = 0.33
lr = 0.1498
d = pd.concat([pd.read_csv(f).assign(split=f) for f in a.csv])
d['ay']=d.v_x*d.r
print('rows',len(d),'runs',d.groupby(['split','run_id']).ngroups)
# speed: wheel vs vx
m=(d.r.abs()<0.3)&(d.delta.abs()<0.05)&(d.v_x>0.5)&(d.Iq.abs()<10)
print('\n[speed] straight, low current n=',m.sum())
s=np.sum(d.omega_wheels[m]*d.v_x[m])/np.sum(d.omega_wheels[m]**2)
print(' v_x = %.3f * omega_wheels; resid std %.3f'%(s,(d.v_x[m]-s*d.omega_wheels[m]).std()))
for lo,hi in [(0.5,1),(1,2),(2,3),(3,4.5)]:
    q=m&(d.v_x>=lo)&(d.v_x<hi)
    if q.sum()>50: print('  vx %.1f-%.1f n=%d ratio vx/omega med=%.3f'%(lo,hi,q.sum(),(d.v_x[q]/d.omega_wheels[q]).median()))
# delay: per run xcorr of r vs vx tan(delta)/L
print('\n[delay] r vs vx*tan(delta)/L')
best={}
for lag in range(0,41,2):
    a=[];b=[]
    for _,g in d.groupby(['split','run_id']):
        if len(g)<300: continue
        pred=(g.v_x*np.tan(g.delta)/L).values; r=g.r.values
        if lag: a.append(pred[:-lag]);b.append(r[lag:])
        else: a.append(pred);b.append(r)
    a=np.concatenate(a);b=np.concatenate(b)
    best[lag]=np.corrcoef(a,b)[0,1]
for k,v in best.items(): print('  lag %3d ms corr %.4f'%(k*10,v))
lag=max(best,key=best.get); print(' best lag',lag*10,'ms')
# steering map with lag: curvature vs delta
rows=[]
for _,g in d.groupby(['split','run_id']):
    if len(g)<=lag+1: continue
    g2=g.iloc[lag:].copy(); g2['delta_l']=g.delta.values[:-lag] if lag else g.delta.values
    g2['delta_prev']=g.delta.values[:-lag] if lag else g.delta.values
    rows.append(g2)
e=pd.concat(rows)
e['kappa']=e.r/e.v_x
print('\n[steer map] kappa=r/vx by commanded delta, by |ay| band (kinematic kappa=tan(d)/L)')
bins=[-0.41,-0.35,-0.25,-0.15,-0.05,0.05,0.15,0.25,0.35,0.41]
for name,mask in [('|ay|<1',e.ay.abs()<1),('|ay| 1-2',(e.ay.abs()>=1)&(e.ay.abs()<2)),('|ay|>=2',e.ay.abs()>=2)]:
    q=e[mask&(e.v_x>0.5)]
    t=q.groupby(pd.cut(q.delta_l,bins),observed=True).agg(n=('kappa','size'),k=('kappa','median'),d=('delta_l','median'))
    t['kin']=np.tan(t.d)/L; t['ratio']=t.k/t.kin
    print(name); print(t.round(3).to_string())
q=e[(e.ay.abs()<1)&(e.v_x>0.5)&(e.delta_l.abs()>0.03)]
g=np.sum(q.kappa*np.tan(q.delta_l)/L)/np.sum((np.tan(q.delta_l)/L)**2)
print('\nlow-ay gain kappa/kin=%.3f (n=%d)'%(g,len(q)))
for sg,nm in [(1,'left d>0'),(-1,'right d<0')]:
    w=q[np.sign(q.delta_l)==sg]; print(' ',nm,'%.3f'%(np.sum(w.kappa*np.tan(w.delta_l)/L)/np.sum((np.tan(w.delta_l)/L)**2)),len(w))
z=e[(e.delta_l.abs()<0.01)&(e.v_x>0.8)]
print(' centre: delta~0 n=%d median kappa %.4f -> steering offset approx %.4f rad'%(len(z),z.kappa.median(),np.arctan(z.kappa.median()*L)))
# rear-axle lateral velocity (kinematic no-slip)
e['vy_rear']=e.v_y-lr*e.r
print('\n[no-slip] rear-axle vy by |ay|')
for lo,hi in [(0,0.5),(0.5,1),(1,2),(2,3),(3,20)]:
    w=e[(e.ay.abs()>=lo)&(e.ay.abs()<hi)&(e.v_x>0.5)]
    print('  |ay| %.1f-%.1f n=%6d share=%.0f%%  |vy_rear| med=%.3f p90=%.3f  slip |vy_rear/vx| med=%.1f deg'%(lo,hi,len(w),100*len(w)/len(e),w.vy_rear.abs().median(),w.vy_rear.abs().quantile(.9),np.degrees(np.arctan((w.vy_rear/w.v_x).abs())).median()))
# overall kinematic yaw rate prediction error
for lo,hi in [(0.5,1.5),(1.5,2.5),(2.5,5)]:
    w=e[(e.v_x>=lo)&(e.v_x<hi)]
    p=w.v_x*np.tan(w.delta_l)/L
    print('  vx %.1f-%.1f n=%d r RMSE=%.3f rad/s, R2=%.3f'%(lo,hi,len(w),np.sqrt(((w.r-p)**2).mean()),1-((w.r-p)**2).sum()/((w.r-w.r.mean())**2).sum()))
