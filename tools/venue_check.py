#!/usr/bin/env python3
"""
venue_check.py — 대회장 연습 주행 로그로 바닥 마찰·조향 지연·모델 예측 오차를 몇 분 안에 본다 (메인 PC 전용).

  ~/hpm-venv/bin/python tools/venue_check.py drive_A_race.csv [drive_B_race.csv ...]
      [--model hmpc]           비교할 상수 파라미터: hmpc = 저장소 hmpc.yaml (기본) / 파일 경로 (.npy, 30개)
      [--quick]                마찰·지연 격자를 줄인다 (약 1 분)

  필요: 라이다 주행 로그 (data_logger) + odom_* 열 (wheel_odom, loc_odom ekf 또는 늘 띄움). mocap 불필요.

■ 무엇을 하나 (2026-11-12 로비 데이터로 검증한 방식)
  ① clean_drive_log 와 같은 불량 제거 (추적 끊김 검사 끔, 뒤 0.2 s, 시간 구멍 0.1 s 까지 이음)
  ② vx·vy = 라이다 x,y 의 SG 미분 (비인과라 KF 지연 없음), r = 자이로 (odom_yaw 미분), δ = 명령→조향 맵→3.2 rad/s
  ③ 모델로 1 s 굴려서
       · 마찰 friction 0.30~0.50 중 오차가 가장 작은 값        → race.yaml hmpc.friction 후보
       · 조향 순수 지연 D (0~60 ms) 중 가장 작은 값             → 15 ms 근처면 로비와 같음
       · 속도 구간 (≤2 / 2~3 / 3+) 별 오차와 회전·속도 편향
  로비 1111_3~5 기준값: 마찰 최적 0.41~0.44 (0.36 은 회전을 덜 예측), 지연 15 ms, 3 m/s 이상 1 s 오차 0.04.
"""
import argparse, contextlib, io, os, sys, tempfile, types

import numpy as np
import pandas as pd
import yaml
from scipy.signal import savgol_filter

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for n in ('robot_model.acrobot', 'robot_model.acrobot.acrobot_model',
          'robot_model.acrobot.observation_preprocesor', 'robot_model.acrobot.acrobot_params'):
    sys.modules[n] = types.ModuleType(n)
sys.path[:0] = [os.path.expanduser('~/hypermpc_code'), REPO + '/hypermpc/hpm', REPO + '/tools']
import torch                                                                     # noqa: E402
import clean_drive_log as CL                                                     # noqa: E402
import prep_hpm_dataset as P                                                     # noqa: E402
from dataset_reader.f1tenth_dataset_new import F1tenthHyperDataset2 as DS        # noqa: E402
from mlcs_car_model import MlcsSingleTrack                                       # noqa: E402
from hyper_prediction_models.const_param_interpolation import ConstParamInterpolation as CI  # noqa: E402
from hyper_prediction_models.rollout_model import RolloutModel                   # noqa: E402

DT = 0.01
dev = 'cuda' if torch.cuda.is_available() else 'cpu'


def load_params(spec):
    if spec == 'hmpc':
        h = yaml.safe_load(open(REPO + '/src/mlcs_mpc/config/hmpc.yaml'))
        h = h.get('hmpc_node', h).get('ros__parameters', h) if isinstance(h, dict) else h

        def find(d, k):
            if isinstance(d, dict):
                if k in d:
                    return d[k]
                for v in d.values():
                    r = find(v, k)
                    if r is not None:
                        return r
            return None
        p = find(h, 'vehicle_params') + find(h, 'tire_front') + find(h, 'tire_rear')
    else:
        p = list(np.load(spec).ravel())
    assert len(p) == 30, f'파라미터 30개가 아님: {len(p)}'
    return torch.tensor(p, dtype=torch.float32, device=dev)


def build(paths):
    ca = types.SimpleNamespace(freeze_frames=10 ** 9, max_speed=6.0, max_yaw_rate=15.0, max_gap=0.1,
                               pad_before=0.1, pad_after=0.2, min_segment=1.5)
    pa = types.SimpleNamespace(min_vx=0.25, min_run=1.5, keep_reverse_wheel=True, drop_impacts=True)
    runs, rid = [], 1
    for f in paths:
        df = pd.read_csv(f)
        if 'odom_yaw' not in df or df['odom_yaw'].notna().mean() < 0.5:
            print(f'  ✗ {os.path.basename(f)}: odom_yaw 가 없음 (wheel_odom 안 떠 있었음) — 건너뜀')
            continue
        out, _ = CL.clean(df, ca)
        st = P.log_stamp(f)
        for _, s in out.dropna(subset=['odom_yaw']).groupby('segment'):
            t = np.arange(s.time.iloc[0], s.time.iloc[-1], DT)
            if len(t) <= 31:
                continue
            g = lambda v: np.interp(t, s.time.to_numpy(), v)  # noqa: E731
            sg = lambda v, d=0: savgol_filter(v, 31, 2, deriv=d, delta=DT)  # noqa: E731
            x, y = g(s.x.to_numpy()), g(s.y.to_numpy())
            yaw = sg(g(np.unwrap(s.yaw.to_numpy())))
            r = sg(g(np.unwrap(s.odom_yaw.to_numpy())), 1)
            dx, dy = sg(x, 1), sg(y, 1)
            o = pd.DataFrame({'time': t, 'x': x, 'y': y, 'yaw': yaw,
                              'v_x': dx * np.cos(yaw) + dy * np.sin(yaw),
                              'v_y': -dx * np.sin(yaw) + dy * np.cos(yaw) + 0.1537 * r, 'r': r,
                              'omega_wheels': g(s.motor_rpm.to_numpy()) / 4781.5, 'friction': 0.41,
                              'delta': P.rate_limit(P.steer_from_cmd(g(s.cmd_steer.to_numpy()), st), 3.2),
                              'Iq': g(s.motor_iq.to_numpy())}).iloc[15:-15]
            for run in P.forward_runs(o, pa):
                runs.append(run.assign(run_id=rid))
                rid += 1
    if not runs:
        sys.exit('쓸 수 있는 전진 구간이 없습니다.')
    return pd.concat(runs, ignore_index=True)


def shift(d, ms):
    if ms == 0:
        return d
    d = d.copy(); out = d['delta'].to_numpy().copy(); t = d['time'].to_numpy()
    for _, ix in d.groupby('run_id', sort=False).indices.items():
        out[ix] = np.interp(t[ix] - ms / 1000.0, t[ix], out[ix])
    d['delta'] = out
    return d


car = MlcsSingleTrack().to(dev)
ci = CI(param_ctrl_point=1, prediction_horizon=100, integration_method='rk4', device=dev).to(dev)
ro = RolloutModel(car, 'rk4', False, True, 10, 0.01).to(dev)
TMP = os.path.join(tempfile.gettempdir(), f'venue_check_{os.getpid()}.csv')


def windows(d, p):
    d.to_csv(TMP, index=False)
    with contextlib.redirect_stdout(io.StringIO()):
        dl = torch.utils.data.DataLoader(DS(dataset_path=TMP, observation_window=25, prediction_horizon=100,
                                            stride=4, device=dev), batch_size=1024, shuffle=False)
    R = []
    with torch.no_grad():
        for M, Pp, U, X0, X in dl:
            M, Pp, U, X0, X = [z.to(dev) for z in (M, Pp, U, X0, X)]
            v = ro(X0, U, ci(p.view(1, 1, -1).expand(X.shape[0], -1, -1)), 100)
            e = (v - X)[..., :3]
            R.append(torch.stack([X[..., 0].mean(1), e.pow(2).sum(-1).mean(1) / 4,
                                  (torch.sign(X[..., 2]) * e[..., 2]).mean(1), e[..., 0].mean(1)], 1).cpu())
    return pd.DataFrame(torch.cat(R).numpy(), columns=['v', 'L', 'b_r', 'b_vx'])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('logs', nargs='+')
    ap.add_argument('--model', default='hmpc')
    ap.add_argument('--quick', action='store_true')
    a = ap.parse_args()
    p = load_params(a.model)
    d = build(a.logs)
    v = d.v_x
    print(f'■ 전진 데이터 {len(d) / 6000:.1f} 분 (≤2: {(v <= 2).mean():.0%}, 2~3: {((v > 2) & (v <= 3)).mean():.0%}, 3+: {(v > 3).mean():.0%}), 최고 {v.max():.2f} m/s')
    if len(d) < 3000:
        print('  ⚠ 30 초 미만 — 결과가 흔들릴 수 있음. 2~3 분 이상 달린 로그를 권장')

    Fs = (0.33, 0.38, 0.41, 0.44, 0.48) if a.quick else (0.30, 0.33, 0.36, 0.38, 0.41, 0.44, 0.48, 0.52)
    Ds = (0, 15, 40) if a.quick else (0, 10, 15, 20, 30, 45, 60)
    d15 = shift(d, 15)
    print('\n■ 바닥 마찰 (지연 15 ms 가정)        오차 L   큰 회전 편향(+면 과대)')
    best = None
    for f in Fs:
        w = windows(d15.assign(friction=f), p)
        print(f'   friction {f:.2f}                   {w.L.mean():.4f}   {w.b_r.mean():+.3f}')
        best = (w.L.mean(), f) if best is None or w.L.mean() < best[0] else best
    fb = best[1]
    print(f'   → 가장 맞는 마찰 {fb:.2f}  (race.yaml hmpc.friction — 바꿀지는 젯슨과)')

    print(f'\n■ 조향 순수 지연 (friction {fb:.2f})')
    bd = None
    for D in Ds:
        L = windows(shift(d, D).assign(friction=fb), p).L.mean()
        print(f'   D {D:3d} ms   {L:.4f}')
        bd = (L, D) if bd is None or L < bd[0] else bd
    print(f'   → 가장 맞는 지연 {bd[1]} ms  (로비 15 ms. 70 ms 근처면 젯슨에 알릴 것)')

    w = windows(shift(d, bd[1]).assign(friction=fb), p)
    w['구간'] = pd.cut(w.v, [0, 2, 3, 9], labels=['≤2', '2~3', '3+'])
    print('\n■ 속도 구간별 1 s 예측 (b_vx −: 모델이 속도를 낮게 예측)')
    print(w.groupby('구간', observed=True).agg(창=('L', 'size'), 오차=('L', 'mean'), 회전편향=('b_r', 'mean'),
                                              속도편향=('b_vx', 'mean')).round(3).to_string())
    os.remove(TMP)


if __name__ == '__main__':
    main()
