# 제어 구조도 (2026-11-26, 서용화 연구원님 요청): python3 docs/notes/figs/make_control_diagram.py
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import FancyBboxPatch
fs=[f for f in fm.findSystemFonts() if 'Nanum' in f or ('Noto' in f and 'CJK' in f)]
fm.fontManager.addfont(fs[0]); plt.rcParams['font.family']=fm.FontProperties(fname=fs[0]).get_name()
fig,ax=plt.subplots(figsize=(14,6.0)); ax.set_xlim(0,14.2); ax.set_ylim(0.6,7.2); ax.axis('off')
def box(x,y,w,h,title,body,fc):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.05,rounding_size=0.15',fc=fc,ec='0.25',lw=1.3))
    ax.text(x+w/2,y+h-0.22,title,ha='center',va='top',fontsize=12.5,weight='bold')
    ax.text(x+w/2,y+h-0.62,body,ha='center',va='top',fontsize=10,linespacing=1.5)
def arr(x1,y1,x2,y2,lab='',dy=0.15,c='0.2'):
    ax.annotate('',(x2,y2),(x1,y1),arrowprops=dict(arrowstyle='->',lw=1.6,color=c))
    if lab: ax.text((x1+x2)/2,(y1+y2)/2+dy,lab,ha='center',va='bottom',fontsize=9.5,color=c)
B='#dbe8f7'; G='#e2f2df'; O='#fbe6d0'; P='#efe2f5'
box(0.3,4.6,3.6,2.2,'MPCC (상위 제어, Jetson)','30 Hz, 예측 N 60 × 33 ms (2 s)\n모델: 원저자 ODE\n(Pacejka 단일 궤적 + 서보·바퀴 1차 지연)\n풀이 acados, 약 24 ms',B)
box(5.2,4.6,3.6,2.2,'제어 입력 /drive','조향각 명령 δ_cmd [rad]\n바퀴 속도 명령 v_cmd [m/s]\n(AckermannDrive, 30 Hz)\n상태→명령 지연 약 34 ms',O)
box(10.1,5.4,3.6,1.4,'VESC — 바퀴 속도','v_cmd → ERPM (×4781.5)\nVESC 내부 속도 PID → 모터 전류',G)
box(10.1,3.7,3.6,1.4,'서보 — 조향','δ_cmd → 서보 위치 (PWM, VESC 가 냄)\n서보 자체 위치 제어 (τ ≈ 0.03 s)',G)
box(10.1,0.8,3.6,2.3,'차량','실제 조향각 δ, 바퀴 속도 ws\n→ 타이어 힘 → vx, vy, r\n→ 위치 x, y, yaw',P)
arr(3.95,5.7,5.15,5.7,'명령'); arr(8.85,6.1,10.05,6.1,'v_cmd'); arr(8.85,5.0,10.05,4.4,'δ_cmd')
ax.plot([13.75,13.9,13.9],[6.1,6.1,2.2],c='0.2',lw=1.6); arr(13.9,2.2,13.75,2.2,''); ax.text(13.95,4.0,'바퀴',rotation=90,fontsize=9.5,va='center')
arr(11.9,3.65,11.9,3.15,'조향',dy=-0.1)
ax.text(12.0,5.02,'',fontsize=1)
box(0.3,0.8,3.6,3.3,'상태 추정 → MPC 입력','x, y, yaw : mocap\nvx, vy, r : mocap 칼만 필터\n(지금, 80~90 ms 늦음)\n\n바꿀 예정 (연구원님 결정):\nvx = 바퀴 속도, r = 자이로',B)
box(5.2,0.8,3.6,3.3,'센서','OptiTrack mocap 100 Hz\n  위치·자세 (NatNet)\n\nVESC 50 Hz\n  바퀴 속도 (ERPM)\n  IMU 자이로 (VESC 내장)',O)
arr(10.05,2.0,8.85,2.0,'측정'); arr(5.15,2.4,3.95,2.4,'')
arr(2.1,4.15,2.1,4.55,'',dy=0.0); ax.text(2.25,4.3,'상태 (매 풀이마다 최신값)',fontsize=9.5,va='center')
ax.text(7.0,7.05,'제어 구조 — 상위 MPCC 가 조향·바퀴 속도를 내고, 저수준 (VESC·서보) 이 따라간다',ha='center',fontsize=13.5,weight='bold')
fig.tight_layout(); import os; fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)),'control_diagram.png'),dpi=130)
