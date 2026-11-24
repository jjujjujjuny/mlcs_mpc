# 강화학습 레이싱 정책 (arXiv:2504.02420 재현)

논문: G. Czechmanowski 외, "On learning racing policies with reinforcement learning", IROS 2025 (arXiv:2504.02420v2).
F1TENTH 차에서 PPO 정책이 MPC 와 사람 선수보다 빨랐다. 핵심은 두 가지다:
**① 저수준 제어기(액추에이터) 모델을 시뮬레이션 환경에 넣는 것**, **② 도메인 랜덤화(DR)를 적당히**.

여기서는 차 모델을 **우리 것**(`env/vehicle_model.py`, MPC 와 같은 식·같은 동정값)으로 바꾸고,
벡터화(배치 torch)는 [codemoo/vectorized-vehicle-control](https://github.com/codemoo/vectorized-vehicle-control) 방식을 따랐다.

## 구성

| 파일 | 역할 |
|---|---|
| `src/mlcs_mpc/mlcs_mpc/env/vec/vehicle.py` | `VehicleModel` 의 torch 배치판. 환경마다 p[30]·마찰·질량·시정수가 달라도 된다 (DR) |
| `src/mlcs_mpc/mlcs_mpc/env/vec/track.py` | 배치 Frenet (`Track.frenet` 과 같은 규칙), 앞쪽 곡률·벽 폭 |
| `src/mlcs_mpc/mlcs_mpc/env/vec/race_env.py` | 논문의 RL 환경 (관측·행동·보상·리셋·DR) |
| `learned/rl/ppo.py`, `train.py` | PPO (논문 하이퍼파라미터) |
| `learned/rl/evaluate.py` | 논문 지표 (최고 랩·평균 랩·E_off) — 학습 환경 + **따로 쓴 casadi 시뮬레이터** (sim-to-sim) |
| `tools/track/track_from_paper_2504_02420.py` | 논문 트랙을 Fig. 6 에서 뽑는다 → `env/track/data/rl17_track/` |
| `test/test_vec_vehicle.py` | torch == casadi (미분식 7e-15, 1 초 적분 4e-16), Frenet, 환경 API |

벡터화에서 따른 것: 상태 `(환경 수, 8)` 텐서, 배치 전체에 고정 간격 RK4, 환경은 스스로 리셋하지 않는다
(`step` 이 final obs 를 돌려주고 `reset(ids)` 를 따로 부른다 — 시간 제한 부트스트랩에 final obs 가 필요), PPO 는
tanh 로 묶은 가우시안에서 잠재 u 를 저장해 다시 계산한다 (atanh 로 되돌리지 않는다). 같은 코드가 CPU·CUDA·MPS 에서 돈다.

## 논문 설정과 우리 설정

| | 논문 | 여기 |
|---|---|---|
| 차 모델 | 단일 궤적 + MF6.1 타이어 | 단일 궤적 + Pacejka (HyperMPC 식, 우리 동정값) |
| 저수준 제어기 | 1차 지연 (식 1: 조향 T_δ, ESC 속도 PID 를 T_ω 로 근사) | 같다 — `steering_tau` 0.13, `wheel_speed_tau` 0.4 (mpcc.yaml, 실측) |
| 주기 | 0.05 s | 같다 (RK4 5 ms × 10) |
| 행동 | [δ_ref, ω̇_ref] ∈ [−1,1] × [0.5 rad, 5 m/s²], ω_ref 는 적분 | 같다. 단 δ_max = 우리 조향 한계 0.378 rad, ω_ref ≤ 4.86 m/s (VESC 모터 상한) |
| 관측 (식 2) | vx, vy, μ, n, r, δ, δ_ref, ω̇_ref, ω_ref, ω, c(N), w(N) / 최댓값 | 같다. N = 20 (0.3 m 간격, 논문에 N 없음), 폭은 왼쪽·오른쪽 따로 (2N) → 관측 70 |
| 보상 (식 3) | Δs, 경계를 넘으면 −1 → 센터라인 임의 위치로 리셋 | 같다 (무게중심이 벽 폭을 넘으면) |
| PPO | 400 환경, 롤아웃 1024, 배치 1024, γ 0.99, lr 1e-3→1e-4, 120M 스텝 | 같다. GAE λ 0.95·clip 0.2·에포크 10 은 논문에 없어 stable-baselines3 기본값 |
| 네트워크 | actor 2×256, critic 2×512, LeakyReLU 0.2 | 같다 |
| DR | friction / all, σ ∈ {0, 0.02, 0.05, 0.1}, 기본 정책 friction 0.02 | 같다. all = p[30] 전부 + 질량 + 마찰 + 두 시정수 |
| 트랙 | L자 17 m × 1 m | `rl17_track` — Fig. 6 에서 뽑음, 17.40 m, 폭 1.0 m |

## 저수준 제어기 (actuator) — 논문의 첫째 축

| | 바퀴 | 조향 | 쓰임 |
|---|---|---|---|
| `lag` (논문) | 1차 지연 τ_ω 0.4 s (실측) + **모터 한계**: 구동 iq ≤ min(60 A, 60 A/mod_q), 제동 \|iq\| ≤ min(60 A, 5 A/mod_q) → 가속도 = 0.545 N/A·iq/(m + m_eq). 빠를수록 회생 −5 A 때문에 제동이 약하다 | 1차 지연 τ_δ 0.13 s | 논문 식 (1) 과 같은 근사 |
| `vesc` (**기본**, `train.py`) | 실차 경로 그대로 — 50 ms 제어 주기 안에서 **1 ms (펌웨어 PID 주기, `sp_pid_loop_rate`) 마다** PID 갱신 → 전류 → 바퀴·차체 적분: 정지 제동 패치 → VESC 펌웨어 속도 PID 1 kHz (램프 5.23 m/s²) → 전류 제한 → 바퀴 관성 m_eq (반암시적). τ 0.4 보다 **빠르다** | 1차 지연 | `env/vec/drivetrain.py` = `env/drivetrain.py` 의 torch 판 (시뮬레이터와 차 0.002 m/s, test ⑥) |
| `none` | 명령 = 실제 | 명령 = 실제 | 논문 Table I 'RL w/o am' |

최고 속도는 ERPM 상한 (vesc.yaml speed_max 23250 → 4.86 m/s) 에서 막힌다 (`lag`·`vesc` 둘 다 직선 전속 4.85 m/s).
모터 전압 한계 (V_bus/√3 ÷ λ → 약 18 m/s) 는 이보다 훨씬 위라 걸리지 않는다.

## 랜덤화 (dr) — 논문의 둘째 축, 커리큘럼

항목마다 수준 1 의 크기를 두고 `dr_level` ℓ ∈ [0, 1] 을 곱한다 (`--dr full`).

| 항목 | 수준 1 | 근거 |
|---|---|---|
| 마찰 | × N(1, 0.05) | 논문 σ 0.02~0.1 (0.02 가 기본 정책) |
| 차량 p[0:10]·질량, 타이어 p[10:30] | × N(1, 0.02) | 질량·관성 랜덤화는 이득이 작고 느려진다 (Xie 2020, 논문 'all' 이 더 느림) |
| 시정수 τ_δ, τ_ω | × N(1, 0.1) | Tan 2018 모터 강도 ±20 % |
| 조향 이득 / 오프셋 | × N(1, 0.03) / N(0, 0.02 rad) | Werner 2023 ±0.03 rad (서보 캘리브레이션 오차) |
| VESC PID 이득, m_eq | × N(1, 0.1) | (`vesc` 일 때) |
| 지연 | U(0, 40 ms) | Tan 2018 0~40 ms. Peng 2018: 지연 고정 시 실기 성공률 0.89 → 0.29 |
| 모캡 노이즈 | 앞축 점 위치 2 mm, yaw 0.005 rad, 속도 0.03 m/s, 요레이트 0.05 rad/s, 바퀴 0.03 m/s | Peng 2018: 관측 노이즈 없으면 0.89 → 0.25 |

**모캡 기준점** — 강체 기준점은 앞축 중심 위 Y자 마커다 (`config/mocap.yaml`, `mocap_bridge` 가 offset_x −0.33 로 뒤축으로).
그래서 노이즈를 **앞축 점과 yaw** 에 넣고 같은 변환으로 무게중심을 추정한다 — yaw 노이즈가 (휠베이스 − l_r) 만큼 위치 노이즈로 번진다.
동역학은 무게중심 기준 그대로, 보상·경계는 참값. 영상의 ● 와 궤적도 앞축 점 (실차 로그와 같은 점) 이다.

**커리큘럼** (`--curriculum adaptive`, 기본 무랜덤 비율 25 %)
1. ℓ = 0 (랜덤화 없음, 액추에이터 모델은 켠 채) 에서 배운다. 보상이 정체 (최근 5 회 개선 < 2 %) 하고 벽 비율 ≤ 2e-3/스텝 이면
2. ℓ += 0.25. 이후 수준마다 최소 5 갱신, 보상/스텝 ≥ 0.9 × (수준 0 최고), 벽 비율 ≤ 2e-3 이면 다음 수준.
   12 갱신 (≈ 12M 스텝) 안에 못 넘으면 그냥 올린다 (patience — 예산 안에 수준 1 에 닿게)
3. 매 리셋 25 % 는 ℓ = 0 으로 (Rudin 2021 의 레벨 되돌림·ADR 의 망각 방지 — 속도 손실도 줄인다)

`--curriculum linear` 는 전체 스텝의 20 %~70 % 에서 ℓ 을 0 → 1 로 선형 (RMA 식).

### 관련 연구 (정량)

| 논문 | 무엇을 | 결과 |
|---|---|---|
| OpenAI ADR (arXiv:1910.07113) | 성능 임계로 분포를 넓히고 좁히는 자동 커리큘럼 | 실기 연속 성공 Manual DR 2.7 → ADR 13.3 (같은 학습량), 장기 32.0 |
| DORAEMON (arXiv:2311.01885) | 성공률 ≥ α 제약 아래 엔트로피 최대화 | 실기 PandaPush 성공률 Fixed-DR 0 %, No-DR 13 %, AutoDR 27 %, DORAEMON 60 % |
| Peng 2018 (arXiv:1710.06537) | 동역학·타이밍·관측 노이즈 랜덤화 ablation | 실기 성공률 전체 0.89 / 타임스텝 고정 0.29 / 관측 노이즈 없음 0.25 / 마찰 고정 0.48 / 질량 고정 0.64 |
| Xie 2020 (arXiv:2011.02404) | 무엇을 랜덤화할지 | 지연 랜덤화로 견디는 지연 17 → 32 ms. 질량·관성·P이득 랜덤화는 강건성 이득 없이 보수적 (최고속 1.1 → 0.9 m/s) |
| Hwangbo 2019 (arXiv:1901.08652) | 액추에이터 모델 | 이상 모델 정책은 실기에서 넘어짐. 토크 오차 3.55 → 0.74 Nm |
| Rudin 2021 (arXiv:2109.11978) | 대량 병렬 (4096 환경) + 게임식 지형 커리큘럼 | 약 20 분 학습. 최고 레벨은 임의 레벨로 되돌려 망각 방지 |
| Chisari 2021 (arXiv:2011.13332) | 1:43 레이싱, 1-step 모델 오차 80 % 를 덮는 랜덤화 | 랜덤화 없으면 실차 완주 못함. 랩 SAC+DR 13.53 s → 출력 정규화 11.43 s (MPC 10.57 s) |
| RLPP (arXiv:2501.17311) | F1TENTH, 마찰 0.5 + N(0, 0.15) | sim→real 랩 갭 순수 RL 17.5 % / PP 4.2 % / RLPP 2.1 % |
| 2504.02420 (이 논문) | 마찰·전체, σ 0~0.1 | 액추에이터 모델 없으면 최고 랩 5.561 → 6.430 s (+15.6 %). 마찰 σ 가 크면 랩↑·E_off↓ |

레이싱에서 커리큘럼 대 고정 랜덤화를 직접 비교한 논문은 찾지 못했다 — 그래서 아래 실험 D (고정) 를 같이 돌린다.

## 학습 속도 (맥북 Air, M 시리즈 CPU 4 스레드)

| | 환경 | SGD | 120M 스텝 |
|---|---|---|---|
| `--profile paper` (CPU, 400 × 1024, 배치 1024, 에포크 10) | 55k 스텝/s | 97k 샘플/s (배치 1024) | 약 4.6 시간 |
| `--profile fast` (MPS, 16384 × 64, 배치 32768, 에포크 5) | 520k 스텝/s (lag) | 650k 샘플/s | 약 35 분 |

`vesc` 는 바퀴·차체를 1 ms 로 적분해야 해서 (위 표) 환경이 약 4 배 느리다.

### GPU 장비 (CUDA) 로 옮기기

학습은 torch 텐서만 쓴다 — acados·ROS 는 필요 없다. `--profile fast` 는 CUDA → MPS → CPU 순으로 장치를 고른다.

```bash
git clone <이 저장소> && cd mlcs_mpc-main
python -m venv .venv && . .venv/bin/activate
pip install torch numpy scipy pyyaml casadi matplotlib        # casadi 는 차 파라미터를 읽는 VehicleModel 용
python test/test_vec_vehicle.py                                # torch == casadi 대조 (CPU)
python learned/rl/train.py --profile fast --actuator vesc --dr full --curriculum adaptive --num-envs 65536 --n-steps 32
```

맥북의 병목은 MPS 커널 호출 오버헤드 (vesc 는 제어 주기마다 1 ms × 50 단계) 다. CUDA 에서는 같은 배치에서 오버헤드가 작고,
맥에서 실패한 `torch.compile` (MPS Inductor 커널 컴파일 오류) 로 1 ms 단계를 묶을 수 있다 — 아직 해 보지 않았다.

## 학습에서 배운 것

- **멈춤 종료에는 페널티가 있어야 한다.** 처음에는 멈춘 차가 보상 0 으로 100 s 를 버텨서 정책이 점점 느려졌다
  (보상/스텝 0.0146 → 0.0066). 1 초 넘게 0.3 m/s 아래면 종료하게 했더니, 페널티 없는 종료가 **어려운 헤어핀에서 서서
  종료당하는 탈출구**가 되어 결정적 정책이 헤어핀에서 섰다. 멈춤도 벽과 같은 −1 로 바꿨다.
- 확률적 정책 (학습 중 랩) 과 결정적 정책 (평가) 은 다르다 — 평가는 항상 `evaluate.py` 로 (s = 0 정지 출발 포함).

## 실행

```bash
conda activate mlcs_ros
python learned/rl/train.py                                   # 논문 기본 (120M 스텝)
python learned/rl/train.py --total-steps 5e6 --tag quick     # 짧게
python learned/rl/train.py --actuator none --tag no_am        # Table I 'RL w/o am'
python learned/rl/train.py --dr all --dr-sigma 0.05 --tag all005   # Fig. 3
python learned/rl/train.py --profile fast --actuator vesc --dr full --curriculum adaptive   # 대량 병렬 + VESC + 커리큘럼
python learned/rl/evaluate.py logs/rl/<run>/best.pt --laps 20 [--wheel vesc] [--vec-actuator lag] [--car friction=0.38]
python learned/rl/render.py A/best.pt B/best.pt --labels lag vesc --sim vesc --laps 2 -o run.mp4      # 영상 (고스트 비교)
python learned/rl/eval_vec.py <ckpt>... --actuators vesc --dr-scales 0 0.5 1 1.5 2 --drop 0 0.05 --starts 8 --reps 16 --device cuda:0   # DR 강건성
python learned/rl/train.py --resume logs/rl/<run> --device cuda:0   # 다른 기계 (맥 → PC) 에서 이어 학습
bash logs/rl/exp/run_all.sh                                   # 실험 A~D 를 차례로 (T=60e6)
```

`evaluate.py` 의 sim 은 실차와 같은 명령 길을 거친다: δ_ref → 서보 명령 (조향 맵 역) → VESC 서보 한계 → 조향 맵,
ω_ref → 속도 명령 → VESC ERPM 상한. `--wheel vesc` 면 바퀴는 1차 지연 대신 VESC 펌웨어 속도 PID 모델
(`env/drivetrain.py`, `docs/VEHICLE_DYNAMICS.md`) — 학습에 쓴 1차 지연과 다른 저수준 모델로 옮겨 보는 시험이다.

## 트랙 (`rl17_track`)

논문 Fig. 6 왼쪽 그림은 래스터라 픽셀에서 쟀다: 격자선으로 축척 (103.83 px/m, 1 px = 9.6 mm), 회색 트랙 영역 분리,
그림 축이 트랙 위·아래를 잘라 바깥 경계 일부가 안 보이므로 **안쪽 경계에서 0.5 m 등거리선**을 센터라인으로.
보이는 곳의 바깥 경계까지 거리 중앙값 0.535 m (전폭 1.03 m ≈ 논문 1 m). 길이 17.40 m, 진행 표시 0~17 의 위치가 논문 그림과 맞는다.
가장 급한 곳 (s ≈ 12 m 헤어핀) 센터라인 반경 0.52 m — 우리 차 최소 회전 반경 ≈ 0.83 m 라 폭을 써서 돌아야 한다.

## 결과

(학습 결과는 아래에 쌓는다. 날짜·명령·지표.)

### 2026-11-21 — VESC 저수준 + DR 수준 비교 (PC, RTX 3060)

학습은 `logs/rl/exp/run_dr.sh`, `run_I.sh`, 평가는 `run_eval_dr.sh` (모두 `--profile fast`, 65536 환경 × 32, 에포크 3, 배치 65536).
E·F 는 맥 (MPS) 에서 시작해 PC 에서 `--resume <폴더> --device cuda:0` 로 이었다. 갱신 (2.1M 스텝) 당 맥 MPS 44 s → 3060 15 s
(한 프로세스로는 GPU 약 60 % — 커널 호출 병목이라 세 개를 같이 돌리면 GPU 100 %, 각 19~30 s).

| 정책 | 출발 | 학습 | 스텝 |
|---|---|---|---|
| A | 처음부터 | lag, DR full 커리큘럼 | 60M |
| E | A | vesc, DR 0.5 고정 + 시작 분산 + 끊김 0.03 | 120M |
| F | B (vesc 처음부터 60M) | E 와 같음 | 120M |
| G | E | vesc, **DR 1.0** | 40M |
| H | E | vesc, **DR 없음** (끊김·시작 분산도 수준 0) | 40M |
| I | F | vesc, **DR 1.0** (G 와 같은 설정) | 40M |

평가 (`eval_vec.py --dr-scales`, vec/vesc, 결정적 정책 `best_det.pt` — A 는 `best.pt`): 학습 전체 랜덤화 (DR_FULL) 를 배율 ×0~2 로
환경마다 뽑고, 정지 출발 8 곳 × 반복 16 = 조건마다 128 표본, 40 s. 실패 = 벽 밖 또는 1 s 멈춤. `logs/rl/exp/eval_dr.md`.

| 실패율 (끊김 0 / 끊김 0.05) | DR ×0 | ×0.5 | ×1 | ×1.5 | ×2 | 평균 랩 ×0 |
|---|---|---|---|---|---|---|
| A (lag 학습) | 100 / 100 % | 95 / 98 | 91 / 96 | 98 / 99 | 99 / 99 | (6.43) |
| E (DR 0.5) | 12 / 31 | 14 / 26 | 16 / 37 | 35 / 63 | 68 / 78 | 6.47 s |
| F (vesc 만, DR 0.5) | 0 / 20 | 7 / 23 | 11 / 41 | 38 / 65 | 67 / 77 | 6.31 s |
| G (E → DR 1.0) | 0 / 20 | 9 / 20 | 8 / 30 | 30 / 55 | 61 / 67 | 6.46 s |
| H (E → DR 0) | 12 / 32 | 13 / 35 | 19 / 37 | 35 / 66 | 66 / 76 | 6.40 s |
| I (F → DR 1.0, 마지막 `policy.pt`) | **0** / **16** | **6** / **18** | **6** / **26** | **30** / 59 | **60** / 76 | 6.33 s |
| I (`best_det.pt` = 갱신 5) | 0 / 20 | 6 / 22 | 6 / 32 | 32 / 63 | 62 / 77 | 6.31 s |

- **lag 로 배운 정책 (A) 은 VESC 에서 거의 다 실패한다** — 저수준 모델이 학습에 들어가야 한다 (논문 ①).
- **같은 출발 (E), 같은 40M 에서 DR 1.0 (G) 이 DR 없음 (H) 보다 모든 배율에서 실패가 적다** (×1: 8 vs 19 %, 끊김 포함 30 vs 37 %).
  값은 공칭 랩 +0.06 s (+1 %). H 는 E 의 실패 시작점 (공칭 12 %) 을 못 고쳤고 G 는 고쳤다.
- F 가 공칭에서 가장 빠르고 (6.31 s) ×1 까지 강하지만 끊김이 섞이면 G 보다 약하다 (41 vs 30 %).
  **F 에 DR 1.0 을 40M 더 (I) 하면 속도는 거의 그대로 (6.33 s) 끊김 포함 ×1 실패 41 → 26 % — 현재 가장 좋은 정책.**
  `best_det.pt` 는 공칭 정지 출발 8 곳만 보고 고르므로 DR 강건성은 마지막 정책이 더 낫다 (DR 미세조정은 `policy.pt` 를 쓸 것).
- ×1.5 이상 (학습보다 큰 오차) 에서는 모두 무너지고, 실패 평균 시각이 3 s 대 — **정지 출발 직후** 실패가 대부분이다.
- 모두 시뮬 (vec) 결과다. 실차는 아직 (README 표).

