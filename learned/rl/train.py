#!/usr/bin/env python3
"""
train.py — 강화학습 레이싱 정책 학습 (arXiv:2504.02420 재현, 차 모델은 우리 것 — env/vehicle_model.py).

  python3 learned/rl/train.py                                  rl17_track, 400 환경, 120M 스텝, friction σ 0.02, 저수준 = VESC 모델 (1 kHz)
  python3 learned/rl/train.py --actuator lag                   논문 그대로 (저수준 = 1차 지연)
  python3 learned/rl/train.py --total-steps 5e6 --tag quick    짧게
  python3 learned/rl/train.py --dr all --dr-sigma 0.05         랜덤화 실험 (논문 Fig. 3)
  python3 learned/rl/train.py --actuator none                  액추에이터 모델 없이 (논문 Table I 'RL w/o am')
  python3 learned/rl/train.py --profile fast --actuator vesc --dr full --curriculum adaptive
                                                               대량 병렬 (MPS, 16384 환경) + VESC 저수준 모델 + 랜덤화 커리큘럼

■ 프로파일 — paper: 논문 그대로 (CPU 400 환경 × 1024, 배치 1024, 갱신당 SGD 4000 번 → 맥북에서 120M 에 약 4.6 시간)
              fast : MPS 16384 환경 × 64 = 갱신당 1M 샘플, 배치 32768, 에포크 5 (Rudin 2021 식 대량 병렬, 120M 에 약 20~40 분)
■ 랜덤화 커리큘럼 (--curriculum) — 처음엔 랜덤화 없이 (dr_level 0) 배우고 조건을 넘으면 수준을 올린다 (docs/RL_RACING.md)
    adaptive  수준마다 최소 갱신 수를 채우고, 벽 비율 ≤ --crash-thr 이고 보상/스텝이 기준 (수준 0 최고의 --keep 배)
              이상이면 +--level-step. 수준 0 은 보상이 정체 (최근 5 회 개선 < 2 %) 해야 넘어간다.
              --patience 갱신 넘게 못 올라가면 그냥 올린다 (학습 예산 안에 수준 1 에 닿게)
    linear    --curriculum-start ~ --curriculum-end (전체 스텝 비율) 에서 0 → 1 로 선형
    none      처음부터 --dr-level (기본 1)

  결과 logs/rl/<시각>_<tag>/   policy.pt (마지막), best.pt (평균 보상 최고), train.csv, config.json
  평가: python3 learned/rl/evaluate.py logs/rl/<…>/best.pt
  끊기면: python3 learned/rl/train.py --resume logs/rl/<…>     같은 설정·같은 갱신부터 (가중치·Adam·학습률 위치·커리큘럼·난수)

  torch 가 필요하다 (macOS: conda 환경 mlcs_ros 에 pytorch 설치됨). 젯슨에서는 학습하지 않는다.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'src/mlcs_mpc'))
sys.path.insert(0, HERE)

from mlcs_mpc.env.vec.race_env import RaceEnvConfig, VecRaceEnv  # noqa: E402
from ppo import ActorCritic, PPOConfig, Rollout, update  # noqa: E402


def parse(argv=None):
    ap = argparse.ArgumentParser(description='PPO 레이싱 정책 학습 (arXiv:2504.02420)')
    ap.add_argument('--track', default='rl17_track')
    ap.add_argument('--profile', default='paper', choices=['paper', 'fast'])
    ap.add_argument('--num-envs', type=int, default=0)
    ap.add_argument('--n-steps', type=int, default=0, help='환경당 롤아웃 길이')
    ap.add_argument('--batch-size', type=int, default=0)
    ap.add_argument('--epochs', type=int, default=0)
    ap.add_argument('--total-steps', type=float, default=120e6)
    ap.add_argument('--actuator', default='vesc', choices=['lag', 'vesc', 'none'],
                    help='저수준 제어기 — vesc (기본: 실차 VESC 펌웨어 속도 PID 를 1 kHz 로 환경 안에서 적분) | '
                         'lag (논문 식 1 의 1차 지연 근사) | none')
    ap.add_argument('--vesc-fast', action='store_true',
                    help='vesc 1 ms 단계를 오일러 + 해석 dR 로 (1.6 배 빠름, 펌웨어 시뮬레이터 대비 vx 0.05 m/s·위치 3 cm)')
    ap.add_argument('--no-motor-limit', action='store_true', help='lag 에 모터 가속·감속 한계를 씌우지 않는다')
    ap.add_argument('--dr', default='friction', choices=['none', 'friction', 'all', 'full'])
    ap.add_argument('--dr-sigma', type=float, default=0.02)
    ap.add_argument('--dr-level', type=float, default=1.0, help='curriculum none 일 때 배율')
    ap.add_argument('--dr-nominal-frac', type=float, default=-1.0, help='랜덤화 없는 환경 비율 (기본: 커리큘럼이면 0.25)')
    ap.add_argument('--dr-drop', type=float, default=0.0, help='명령 끊김 시작 확률 / 주기 (수준 1, 예 0.03)')
    ap.add_argument('--dr-drop-len', type=float, default=3.0, help='끊김 평균 길이 (주기, 3 = 150 ms)')
    ap.add_argument('--start-spread', type=float, default=0.0,
                    help='시작 조건 커리큘럼 (1 = 횡 ±0.6 W, 헤딩 ±0.35 rad, 속도 +2 m/s, 10 %% 정지 출발 — 수준을 곱함)')
    ap.add_argument('--curriculum', default='none', choices=['none', 'adaptive', 'linear'])
    ap.add_argument('--level-step', type=float, default=0.25)
    ap.add_argument('--dr-max', type=float, default=1.0, help='커리큘럼 최대 수준 (낮추면 빠르지만 덜 강건 — 논문 Fig. 3)')
    ap.add_argument('--min-updates', type=int, default=5, help='수준마다 최소 갱신 수')
    ap.add_argument('--crash-thr', type=float, default=2e-3, help='스텝당 벽 비율 상한 (2e-3 = 25 s 에 한 번, 확률적 정책 기준)')
    ap.add_argument('--patience', type=int, default=12, help='한 수준에 이만큼 머물면 조건과 관계없이 올린다 (0 = 끔)')
    ap.add_argument('--keep', type=float, default=0.9, help='보상/스텝 ≥ keep × 수준 0 최고')
    ap.add_argument('--curriculum-start', type=float, default=0.2)
    ap.add_argument('--curriculum-end', type=float, default=0.7)
    ap.add_argument('--car', nargs='*', default=[], metavar='KEY=VALUE', help='차 값 덮어쓰기 (friction=0.35 …)')
    ap.add_argument('--obs-vel', default='true', choices=['true', 'delay', 'kf', 'gyro'],
                    help='관측 속도: true 참값 | delay 늦은 참값 | kf 실차 mocap_bridge 칼만 필터 (80~90 ms) | gyro r 자이로·vx 바퀴 속도')
    ap.add_argument('--obs-vy', default='kf', choices=['kf', 'rot'], help='kf·gyro 의 vy: kf 지금 실차 | rot 지연 yaw 로 돌림')
    ap.add_argument('--obs-delay', nargs='*', default=[], metavar='KEY=VALUE', help='obs_vel 지연 s (vx vy r gyro, 예 gyro=0.02)')
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--threads', type=int, default=0, help='torch CPU 스레드 (0 = 기본)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--det-eval-every', type=int, default=0,
                    help='이 갱신마다 결정적 정책 평가 (정지 출발 --det-starts 곳 × --det-time s, CPU) → best_det.pt (0 = 끔)')
    ap.add_argument('--sim-check', action='store_true',
                    help='결정적 평가 때 따로 구현한 casadi Simulator (vesc 면 펌웨어 경로) 에서도 정지 출발 2 랩 폐루프 + 궤적 그림 checks/')
    ap.add_argument('--det-starts', type=int, default=8)
    ap.add_argument('--det-time', type=float, default=30.0)
    ap.add_argument('--resume', default='', help='끊긴 학습 폴더 — config.json 의 설정과 state.pt 로 이어서')
    ap.add_argument('--init', default='', help='이 체크포인트의 가중치에서 시작 (이어 학습 — 관측·행동 차원이 같아야 한다)')
    ap.add_argument('--lr-start', type=float, default=0.0, help='0 = 기본 1e-3 (이어 학습이면 작게, 예 3e-4)')
    ap.add_argument('--tag', default='')
    ap.add_argument('--out', default='')
    return ap.parse_args(argv)


def save_atomic(obj, path):
    """저장 도중 끊겨도 이전 파일이 깨지지 않게 (임시 파일 → 이름 바꾸기)."""
    tmp = path + '.tmp'
    torch.save(obj, tmp)
    os.replace(tmp, path)


def main(argv=None):
    a = parse(argv)
    resume = a.resume
    if resume:
        saved = json.load(open(os.path.join(resume, 'config.json')))['args']
        dev_cli = a.device
        a = argparse.Namespace(**saved)
        a.resume, a.out = resume, resume
        # 다른 기계에서 이어 하기 (맥 mps → PC cuda): --device 를 주면 그걸, 저장된 장치가 없으면 있는 것으로
        if dev_cli != 'cpu':
            a.device = dev_cli
        elif a.device.startswith('cuda') and not torch.cuda.is_available() or \
                a.device == 'mps' and not torch.backends.mps.is_available():
            a.device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
            print(f'  ↳ 저장된 장치 {saved["device"]} 가 없어 {a.device} 로 이어 간다')
    if a.threads > 0:
        torch.set_num_threads(a.threads)
    torch.manual_seed(a.seed)
    car = {k: float(v) for k, v in (kv.split('=', 1) for kv in a.car)}
    prof = dict(paper=dict(num_envs=400, n_steps=1024, batch_size=1024, epochs=10, device='cpu'),
                fast=dict(num_envs=16384, n_steps=64, batch_size=32768, epochs=5,
                          device='cuda' if torch.cuda.is_available()
                          else 'mps' if torch.backends.mps.is_available() else 'cpu'))[a.profile]
    for k in ('num_envs', 'n_steps', 'batch_size', 'epochs'):
        if getattr(a, k):
            prof[k] = getattr(a, k)
    if a.device != 'cpu' or a.profile == 'paper':
        prof['device'] = a.device
    a.num_envs, a.n_steps, a.device = prof['num_envs'], prof['n_steps'], prof['device']
    nominal = a.dr_nominal_frac if a.dr_nominal_frac >= 0 else (0.25 if a.curriculum != 'none' else 0.0)
    level0 = 0.0 if a.curriculum != 'none' else a.dr_level
    ecfg = RaceEnvConfig(track=a.track, num_envs=a.num_envs, device=a.device, actuator=a.actuator,
                         motor_limit=not a.no_motor_limit, dr=a.dr, dr_sigma=a.dr_sigma, dr_level=level0,
                         dr_nominal_frac=nominal, car=car, start_spread=a.start_spread,
                         vesc_body='euler' if getattr(a, 'vesc_fast', False) else 'rk2',
                         vesc_dR='approx' if getattr(a, 'vesc_fast', False) else 'fd',
                         dr_terms=dict(drop=a.dr_drop, drop_len=a.dr_drop_len) if a.dr_drop > 0 else {},
                         obs_vel=a.obs_vel, obs_vy=a.obs_vy,
                         obs_delay=dict(RaceEnvConfig().obs_delay, **{k: float(v) for k, v in (kv.split('=') for kv in a.obs_delay)}))
    pcfg = PPOConfig(n_steps=a.n_steps, batch_size=prof['batch_size'], epochs=prof['epochs'])
    env = VecRaceEnv(ecfg, seed=a.seed)
    env.start_level = level0 if a.curriculum != 'none' else 1.0
    dev = env.device
    if a.lr_start > 0:
        pcfg.lr_start = a.lr_start
    ac = ActorCritic(env.obs_dim, env.act_dim, log_std0=pcfg.log_std0).to(dev)
    if a.init:
        ck0 = torch.load(a.init, map_location='cpu', weights_only=False)
        ac.load_state_dict(ck0['model'])
        print(f'  ↳ 가중치 {a.init} 에서 시작 ({ck0["meta"]["env"]["actuator"]}, 갱신 {ck0.get("update")})')
    opt = torch.optim.Adam(ac.parameters(), lr=pcfg.lr_start)
    out = a.out or os.path.join(ROOT, 'logs/rl', time.strftime('%Y%m%d_%H%M%S') + (f'_{a.tag}' if a.tag else ''))
    os.makedirs(out, exist_ok=True)
    meta = dict(env=ecfg.to_dict(), ppo=pcfg.to_dict(), obs_dim=env.obs_dim, act_dim=env.act_dim,
                max_steer=env.max_steer, v_max=env.v_max, args=vars(a))
    if not resume:
        json.dump(meta, open(os.path.join(out, 'config.json'), 'w'), indent=2, ensure_ascii=False)
    per_update = a.num_envs * a.n_steps
    n_updates = max(1, int(math.ceil(a.total_steps / per_update)))
    print(f'■ {out}')
    print(f'  트랙 {a.track} ({env.track.length:.2f} m)  환경 {a.num_envs}  롤아웃 {a.n_steps}  갱신 {n_updates} 회 '
          f'({n_updates * per_update / 1e6:.1f}M 스텝)  액추에이터 {a.actuator}  랜덤화 {a.dr} σ {a.dr_sigma} '
          f'(커리큘럼 {a.curriculum}, 무랜덤 {nominal:.0%})  관측 {env.obs_dim}  δ_max {env.max_steer:.3f} rad  '
          f'ω_ref ≤ {env.v_max:.2f} m/s  장치 {dev}  배치 {pcfg.batch_size}  에포크 {pcfg.epochs}')
    cur = dict(level=level0, at_level=0, ref=-1e9, hist=[])
    best = (-1.0, -1e9)
    upd0, elapsed0 = 0, 0.0
    state_path = os.path.join(out, 'state.pt')
    if resume:
        if not os.path.isfile(state_path):
            sys.exit(f'✗ {state_path} 가 없습니다 (이 기능 전에 시작한 학습은 --init {out}/policy.pt 로 가중치만 이어 갈 수 있다)')
        stt = torch.load(state_path, map_location='cpu', weights_only=False)
        ac.load_state_dict(stt['model'])
        opt.load_state_dict(stt['opt'])
        upd0, cur, best, elapsed0 = stt['update'], stt['cur'], tuple(stt['best']), stt['elapsed']
        env.set_dr_level(stt['dr_level'])
        env.start_level = stt['start_level']
        torch.set_rng_state(stt['rng_torch'])
        env.gen.set_state(stt['rng_env'])
        print(f'  ↻ {out} 갱신 {upd0}/{n_updates} 부터 이어서 (랜덤화 수준 {env.dr_level:.2f}, 지난 학습 {elapsed0:.0f} s)')
    new_csv = not (resume and os.path.isfile(os.path.join(out, 'train.csv')))
    log_f = open(os.path.join(out, 'train.csv'), 'a' if not new_csv else 'w', newline='')
    log = csv.writer(log_f)
    if new_csv:
        log.writerow(['update', 'env_steps', 'dr_level', 'reward_per_step', 'terminations', 'crash_rate', 'laps',
                      'lap_med', 'lap_best', 'lr', 'pg', 'vf', 'kl', 'clipfrac', 'std0', 'std1', 'sec'])
    buf = Rollout(a.n_steps, a.num_envs, env.obs_dim, env.act_dim, dev)
    obs = env.reset()
    t0 = time.time() - elapsed0
    det_best = tuple(stt.get('det_best', (2.0, 1e9))) if resume else (2.0, 1e9)

    def det_eval():
        """결정적 정책 (tanh 평균) 으로 정지 출발 끝까지 도는가 — eval_vec 과 같은 계산, 학습 모델 복사본을 CPU 로."""
        import copy
        from eval_vec import run_batch
        ac_c = copy.deepcopy(ac).to('cpu').eval()
        L = env.track.length
        grid = [(1.0, 0.0, L * j / a.det_starts) for j in range(a.det_starts)]
        r = run_batch(ac_c, dict(env=ecfg.to_dict()), a.actuator, grid, a.det_time, 'cpu')
        fail = float(r['failed'].mean())
        lm = float(np.nanmean(r['lap_mean'])) if np.isfinite(r['lap_mean']).any() else float('nan')
        return fail, lm

    def sim_check(tag):
        """학습 코드와 따로 쓴 Simulator 에서 실차 명령 경로로 폐루프 (evaluate.run_sim) → 결과 + 앞축 궤적 그림."""
        import copy
        from evaluate import run_sim
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        for f in ('AppleGothic', 'NanumGothic', 'Noto Sans CJK KR', 'Noto Sans CJK JP', 'Malgun Gothic'):
            if any(f == x.name for x in font_manager.fontManager.ttflist):
                plt.rcParams['font.family'] = f
                break
        ac_c = copy.deepcopy(ac).to('cpu').eval()
        wheel = 'vesc' if a.actuator == 'vesc' else 'lag'
        summ, tr = run_sim(ac_c, dict(env=ecfg.to_dict()), 2, wheel=wheel, max_time=25.0, stop_on_out=True)
        trk = env.track.track
        h = trk.heading
        fig, ax = plt.subplots(figsize=(5, 5))
        for sgn, w in ((1, trk.w_left), (-1, trk.w_right)):
            ax.plot(np.r_[trk.x - sgn * np.sin(h) * w, trk.x[0] - sgn * np.sin(h[0]) * w[0]],
                    np.r_[trk.y + sgn * np.cos(h) * w, trk.y[0] + sgn * np.cos(h[0]) * w[0]], 'k-', lw=0.8)
        sc = ax.scatter(tr['fx'], tr['fy'], c=tr['vx'], s=3, cmap='viridis', vmin=0, vmax=4)
        if summ['crash_t'] is not None:
            ax.plot(tr['fx'][-1], tr['fy'][-1], 'rx', ms=12, mew=3)
        fig.colorbar(sc, ax=ax, shrink=0.7, label='vx')
        ax.set_aspect('equal')
        res = (f"{summ['crash_why']} {summ['crash_t']:.1f}s" if summ['crash_t'] is not None
               else f"완주 {summ['laps'] + 1}랩 최고 {summ['best']:.2f}s")
        ax.set_title(f'{tag}  sim/{wheel}: {res}', fontsize=9)
        os.makedirs(os.path.join(out, 'checks'), exist_ok=True)
        png = os.path.join(out, 'checks', f'{tag}.png')
        fig.savefig(png, dpi=90)
        plt.close(fig)
        return wheel, res, png

    for upd in range(upd0, n_updates):
        frac = upd / max(n_updates - 1, 1)
        lr = pcfg.lr_start + (pcfg.lr_end - pcfg.lr_start) * frac
        laps, terms = [], 0
        lvl_now = env.dr_level
        for t in range(a.n_steps):
            act, u, logp = ac.act(obs)
            with torch.no_grad():
                val = ac.value(obs)
            nobs, rew, term, trunc, info = env.step(act)
            done = term | trunc
            with torch.no_grad():
                nval = ac.value(nobs)           # final obs (리셋 전)
            buf.obs[t], buf.u[t], buf.logp[t], buf.val[t] = obs, u, logp, val
            buf.rew[t], buf.next_val[t] = rew, nval
            buf.term[t], buf.done[t] = term.float(), done.float()
            lt = info['lap_time']
            if bool((~torch.isnan(lt)).any()):
                laps += lt[~torch.isnan(lt)].tolist()
            terms += info['out'].sum()          # 벽 (멈춤 종료는 세지 않는다)
            obs = env.reset(done.nonzero().squeeze(1)) if done.any() else nobs
        buf.gae(pcfg.gamma, pcfg.gae_lambda)
        st = update(ac, opt, buf, pcfg, lr)
        rps = float(buf.rew.mean())
        terms = int(terms)
        crash = terms / per_update
        std = ac.log_std.exp().tolist()
        # ── 랜덤화 커리큘럼 ──
        if a.curriculum == 'linear':
            f = ((upd + 1) / n_updates - a.curriculum_start) / max(a.curriculum_end - a.curriculum_start, 1e-9)
            env.set_dr_level(min(max(f, 0.0), 1.0) * a.dr_max)
        elif a.curriculum == 'adaptive' and env.dr_level < a.dr_max - 1e-9:
            cur['at_level'] += 1
            cur['hist'].append(rps)
            if env.dr_level == 0.0:
                cur['ref'] = max(cur['ref'], rps)
            h = cur['hist']
            if env.dr_level == 0.0:
                ready = len(h) >= max(a.min_updates, 6) and max(h[-5:]) <= h[-6] * 1.02 + 1e-4 and rps > 0
            else:
                ready = cur['at_level'] >= a.min_updates and np.mean(h[-3:]) >= a.keep * cur['ref']
            forced = a.patience > 0 and cur['at_level'] >= a.patience
            if (ready and crash <= a.crash_thr) or forced:
                env.set_dr_level(min(env.dr_level + a.level_step, a.dr_max))
                cur['at_level'], cur['hist'] = 0, []
                print(f'  ▲ 랜덤화 수준 {env.dr_level:.2f}{" (patience)" if forced and not ready else ""} (벽 비율 {crash:.1e}, 보상/스텝 {rps:+.4f}, 기준 {cur["ref"]:+.4f})')
        row = [upd + 1, (upd + 1) * per_update, lvl_now, rps, terms, crash, len(laps),
               float(np.median(laps)) if laps else '', float(np.min(laps)) if laps else '',
               lr, st['pg'], st['vf'], st['kl'], st['clipfrac'], std[0], std[1], round(time.time() - t0, 1)]
        log.writerow(row)
        log_f.flush()
        ckpt = dict(model=ac.state_dict(), meta=meta, update=upd + 1, reward_per_step=rps)
        save_atomic(ckpt, os.path.join(out, 'policy.pt'))
        # best = 가장 높은 랜덤화 수준에서 보상/스텝 최고 (커리큘럼이면 수준이 오를수록 보상이 내려가므로)
        if (lvl_now, rps) > best:
            best = (lvl_now, rps)
            save_atomic(ckpt, os.path.join(out, 'best.pt'))
        if a.det_eval_every > 0 and ((upd + 1) % a.det_eval_every == 0 or upd + 1 == n_updates):
            te = time.time()
            fail, lm = det_eval()
            key = (fail, lm if np.isfinite(lm) else 1e9)
            mark = ''
            if key < det_best:
                det_best = key
                save_atomic(ckpt, os.path.join(out, 'best_det.pt'))
                mark = '  ★ best_det.pt'
            print(f'  ◆ 결정적 평가 (정지 출발 {a.det_starts} 곳 × {a.det_time:.0f} s): 실패 {fail:.0%}  평균 랩 {lm:.2f} s'
                  f'  ({time.time() - te:.0f} s){mark}', flush=True)
            if a.sim_check:
                try:
                    te = time.time()
                    wheel, res, png = sim_check(f'upd_{upd + 1:04d}')
                    print(f'  ◇ 폐루프 Simulator/{wheel} (s 0 정지 출발): {res}  → {os.path.relpath(png, ROOT)}  '
                          f'({time.time() - te:.0f} s)', flush=True)
                except Exception as e:          # 확인이 실패해도 학습은 계속
                    print(f'  ◇ 폐루프 확인 실패: {type(e).__name__}: {e}', flush=True)
        # 이어 하기용 상태 (매 갱신)
        save_atomic(dict(model=ac.state_dict(), opt=opt.state_dict(), update=upd + 1, cur=cur, best=list(best),
                         det_best=list(det_best),
                         dr_level=env.dr_level, start_level=env.start_level, elapsed=time.time() - t0,
                         rng_torch=torch.get_rng_state(), rng_env=env.gen.get_state()), state_path)
        lap_s = f'랩 {len(laps)} 개 중앙 {np.median(laps):.2f} 최고 {np.min(laps):.2f} s' if laps else '랩 없음'
        print(f'[{upd + 1}/{n_updates}] {(upd + 1) * per_update / 1e6:6.1f}M  DR {lvl_now:.2f}  보상/스텝 {rps:+.4f}  '
              f'벽 {terms} ({crash:.1e}/스텝)  {lap_s}  '
              f'lr {lr:.1e}  kl {st["kl"]:.4f}  std {std[0]:.2f}/{std[1]:.2f}  {time.time() - t0:.0f}s', flush=True)
    print(f'✓ {out}/best.pt (랜덤화 수준 {best[0]:.2f}, 보상/스텝 {best[1]:+.4f})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
