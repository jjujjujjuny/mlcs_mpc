#!/usr/bin/env python3
"""
actuator_id.py — 속도 명령 → 바퀴 속도 응답을 잰다. MPC 의 바퀴 액추에이터 값을 정한다.

  python3 tools/actuator_id.py ~/mlcs_data/drive_*_rally.csv
  python3 tools/actuator_id.py drive_A.csv drive_B.csv --erpm-gain 4781.5

■ 왜 있나

  hmpc 는 바퀴 속도를 ws' = (ws_ref − ws) / wheel_speed_tau 로 본다. 저자 값 0.046 s 를
  그대로 쓰는데, 2026-11-08 race 로그로 재면 실제는 0.3~0.7 s 였고 감속은 ~1.3 m/s² 로
  굴러가기만 했다 (VESC speed PID 의 Allow Braking 이 꺼져 있었다). MPC 는 언제든 바로
  설 수 있다고 믿고 늦게 감속해 앞 벽에 박았다.

  tau 를 0.66 으로 올리면 QP 가 자주 실패했으므로 (config/hmpc.yaml 주석), tau 는 작게
  두고 바퀴 가속도를 [−ws_dec_max, ws_acc_max] 로 묶는 쪽을 쓴다 (offline_ocp 의 선형
  제약). 이 도구는 두 모델을 다 맞춰 본다.

    1차      ws' = (u − ws) / tau
    제한     ws' = clip((u − ws) / tau, −a_dec, +a_acc)

■ 출력

  ① 일정 속도 오차 — 명령이 1.5 s 이상 거의 같을 때 (명령 − 바퀴) 평균. VESC Ki 확인용
  ② 감속·가속 사건 — 명령이 바퀴보다 0.5 m/s 이상 낮/높을 때 바퀴 가속도, Iq
  ③ 맞춤 — 2 s 창마다 측정 바퀴 속도에서 출발해 명령만으로 적분한 오차 (RMSE)
     · 자유 tau + 제한  (실제 차에 가장 가까운 값)
     · tau 고정(--mpc-tau, 기본 hmpc.yaml 값) + 제한  → race.yaml 의 ws_acc_max / ws_dec_max

■ 주의

  · 충돌은 바퀴가 갑자기 멈춰 감속이 크게 나온다. 바퀴 감속 8 m/s² 넘는 곳 앞뒤 0.5 s 를 뺀다.
  · 후진(명령 < 0)과 정지 근처(바퀴 < 0.3 m/s)는 뺀다.
  · 바퀴 속도는 motor_rpm / erpm_gain — 바퀴 '선속도' 다 (prep_hpm_dataset 과 같음).
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

WIN = 2.0          # 맞춤 창 [s]
DT = 0.01          # 다시 샘플 격자 [s]


def load(path, gain):
    d = pd.read_csv(path).dropna(subset=['cmd_speed', 'motor_rpm']).sort_values('stamp')
    t = d['stamp'].to_numpy()
    tg = np.arange(t[0], t[-1], DT)
    hold = lambda v: v[np.clip(np.searchsorted(t, tg, side='right') - 1, 0, len(t) - 1)]  # noqa: E731
    u = hold(d['cmd_speed'].to_numpy())                 # 명령은 영차 유지
    w = np.interp(tg, t, d['motor_rpm'].to_numpy() / gain)
    iq = np.interp(tg, t, d['motor_iq'].to_numpy()) if 'motor_iq' in d else np.full_like(tg, np.nan)
    return tg - tg[0], u, w, iq


def crash_mask(w):
    """바퀴 감속이 8 m/s² 를 넘는 곳 앞뒤 0.5 s."""
    a = np.gradient(w, DT)
    k = int(0.5 / DT)
    return np.convolve((a < -8.0).astype(int), np.ones(2 * k + 1), 'same') > 0


def steady(t, u, w, ok):
    """명령이 1.5 s 이상 ±0.05 안에서 같을 때, 뒤 1 s 의 (명령 − 바퀴)."""
    out = []
    i = 0
    n = len(u)
    while i < n:
        j = i
        while j + 1 < n and abs(u[j + 1] - u[i]) < 0.05 and ok[j + 1]:
            j += 1
        if (j - i) * DT >= 1.5 and u[i] > 0.5 and ok[i]:
            k = j - int(1.0 / DT)
            out.append((u[i], np.mean(u[k:j] - w[k:j])))
        i = j + 1
    return out


def events(u, w, iq, ok, sign):
    """sign=-1: 명령 < 바퀴 − 0.5 (감속), +1: 명령 > 바퀴 + 0.5 (가속)."""
    m = ok & ((u < w - 0.5) if sign < 0 else (u > w + 0.5)) & (w > 0.3)
    a = np.gradient(w, DT)
    return dict(sec=m.sum() * DT, a=a[m], iq=iq[m])


def windows(logs):
    """2 s 창 (명령, 측정 바퀴) — 창 안이 전부 쓸 수 있을 때만."""
    L = int(WIN / DT)
    U, W = [], []
    for t, u, w, iq, ok in logs:
        for i in range(0, len(u) - L, L // 2):
            if ok[i:i + L].all() and w[i] > 0.3:
                U.append(u[i:i + L])
                W.append(w[i:i + L])
    return np.array(U), np.array(W)


def simulate(U, W0, tau, a_acc, a_dec):
    """창 K 개를 한꺼번에 적분 (오일러, DT)."""
    ws = W0.copy()
    out = np.empty_like(U)
    for k in range(U.shape[1]):
        out[:, k] = ws
        ws = ws + DT * np.clip((U[:, k] - ws) / tau, -a_dec, a_acc)
    return out


def fit(U, W, taus, accs, decs):
    best = None
    for tau in taus:
        for aa in accs:
            for ad in decs:
                e = np.sqrt(np.mean((simulate(U, W[:, 0], tau, aa, ad) - W) ** 2))
                if best is None or e < best[0]:
                    best = (e, tau, aa, ad)
    return best


def main():
    p = argparse.ArgumentParser(description='속도 명령 → 바퀴 속도 응답')
    p.add_argument('logs', nargs='+', help='drive_*.csv (data_logger 형식)')
    p.add_argument('--erpm-gain', type=float, default=4781.5)
    p.add_argument('--mpc-tau', type=float, default=0.0461, help='MPC 에 둘 tau (hmpc.yaml)')
    a = p.parse_args()

    logs = []
    for f in a.logs:
        t, u, w, iq = load(f, a.erpm_gain)
        cm = crash_mask(w)
        ok = (u >= 0) & ~cm
        logs.append((t, u, w, iq, ok))
        print(f'{os.path.basename(f)}: {t[-1]:.0f} s, 바퀴 최대 {w.max():.2f} m/s, '
              f'충돌 의심 {cm.sum() * DT:.1f} s 제외')

    print('\n① 일정 속도 오차 (명령 − 바퀴)  — 0 에 가까워야 한다. 크면 VESC speed Ki')
    st = [s for L in logs for s in steady(L[0], L[1], L[2], L[4])]
    if st:
        st = np.array(st)
        for lo, hi in ((0.5, 1.5), (1.5, 2.25), (2.25, 2.75), (2.75, 9)):
            m = (st[:, 0] >= lo) & (st[:, 0] < hi)
            if m.any():
                print(f'   명령 {lo:.2f}~{hi:.2f} m/s  {m.sum():3d}구간  '
                      f'오차 평균 {st[m, 1].mean():+.3f} m/s')
    else:
        print('   일정 속도 구간이 없습니다 (명령을 1.5 s 이상 유지한 곳)')

    print('\n② 사건')
    for nm, sg in (('감속', -1), ('가속', +1)):
        ev = [events(L[1], L[2], L[3], L[4], sg) for L in logs]
        A = np.concatenate([e['a'] for e in ev])
        Q = np.concatenate([e['iq'] for e in ev])
        sec = sum(e['sec'] for e in ev)
        if len(A) < 10:
            print(f'   {nm}: 거의 없음')
            continue
        print(f'   {nm} {sec:5.1f} s   바퀴 가속도 중앙값 {np.median(A):+.2f}  '
              f'p10 {np.quantile(A, .1):+.2f}  p90 {np.quantile(A, .9):+.2f} m/s²   '
              f'Iq 중앙값 {np.nanmedian(Q):+.1f} A' + (f'  최소 {np.nanmin(Q):+.1f}' if sg < 0 else ''))
        if sg < 0 and np.nanmedian(Q) > -2:
            print('   ★ 감속 중 Iq 가 0 근처 — VESC 가 제동하지 않습니다 (Speed PID Allow Braking, '
                  '배터리 회생 한도 확인)')

    U, W = windows(logs)
    print(f'\n③ 맞춤  ({len(U)}개 창, 창 {WIN:.0f} s, 측정 바퀴에서 출발해 명령만으로 적분)')
    if len(U) < 5:
        print('   창이 부족합니다')
        return 1
    accs = np.r_[np.arange(0.5, 6.01, 0.25), 8, 12, 50]
    decs = accs
    one = fit(U, W, np.r_[0.03, 0.0461, np.arange(0.1, 1.01, 0.05)], [1e9], [1e9])
    free = fit(U, W, np.r_[0.0461, np.arange(0.1, 1.01, 0.05)], accs, decs)
    mpc = fit(U, W, [a.mpc_tau], accs, decs)
    now = np.sqrt(np.mean((simulate(U, W[:, 0], a.mpc_tau, 1e9, 1e9) - W) ** 2))
    print(f'   지금 MPC (tau {a.mpc_tau:.4f}, 제한 없음)  RMSE {now:.3f} m/s')
    print(f'   1차만      tau {one[1]:.3f}                         RMSE {one[0]:.3f}')
    print(f'   자유+제한  tau {free[1]:.3f}  가속 {free[2]:.2f}  감속 {free[3]:.2f} m/s²   RMSE {free[0]:.3f}')
    print(f'   MPC용      tau {a.mpc_tau:.4f} 고정  가속 {mpc[2]:.2f}  감속 {mpc[3]:.2f} m/s²   RMSE {mpc[0]:.3f}')
    print('\n   → race.yaml hmpc:  '
          f'ws_acc_max: {mpc[2]:.2f}   ws_dec_max: {mpc[3]:.2f}   (wheel_speed_tau {a.mpc_tau:g} 그대로)')
    if mpc[0] > 1.3 * free[0]:
        print('   ★ tau 를 MPC 값으로 묶으면 오차가 꽤 커집니다. 제한만으로는 응답 지연을 다 못 담습니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
