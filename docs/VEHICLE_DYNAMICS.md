# 차 동역학과 저수준 제어 — 우리 모델, f1tenth_gym, 실차

이 문서는 세 가지를 나란히 놓는다: 실차의 명령 경로(`/drive`부터 바퀴까지), 우리 시뮬레이터
(`env/simulator.py`, 바퀴 모델 `lag`·`vesc`), f1tenth_gym(`f1tenth_gym_ros`가 쓰는 시뮬레이터, `f110_gym/envs/dynamic_models.py`·`base_classes.py`·`f110_env.py`, main 브랜치).
특히 **제어 입력**과 **제동 방식**의 차이를 정리한다. 수식과 값은 모두 코드에서 옮겼다.

## 1. 실차의 명령 경로

```
mpcc_node ── /drive (AckermannDriveStamped: speed m/s, steering_angle rad, 30 Hz)
  → ackermann_mux
  → ackermann_to_vesc  (vesc_ackermann + third_party/patches/vesc-stop-brake.patch)
       속도: erpm = 4781.5·speed  → commands/motor/speed
             단, |speed| < 0.05 이고 (바퀴 < 0.5 m/s 또는 VESC 상태가 0.2 s 넘게 끊김) 이면
             commands/motor/brake = 10 A  (한 번 넘어가면 출발 명령까지 유지)
       조향: servo = −0.8845·δ_cmd + 0.5077  → commands/servo/position
  → vesc_driver        erpm 을 [−23250, 23250] 으로 자름 (≈ ±4.86 m/s), 서보를 [0.08, 0.87] 로 자름
  → VESC 펌웨어 (FOC, 센서리스)
       속도 모드: 램프 → 속도 PID (1 kHz) → q축 전류 목표 → 전류 제한 → 전류 루프
       제동 전류 모드: iq = −sign(rpm)·10 A
  → 모터 → 기어 → 4륜 (Traxxas 4x4, 앞뒤 같은 바퀴 속도)
  서보: 위치 제어 서보 (내부 루프는 모름) → 실제 조향은 조향 맵 (steer_map) 으로 명령보다 작다
```

VESC 설정은 `docs/vesc/vesc6_f1tenth_1124.xml`(2026-11-24 최신: 속도 PID 다시 맞춤, 모터 재측정, 개루프 출발 보조. 앱 설정은 `docs/vesc/vesc_app_f1tenth_1124.xml`)이다.
이전 `docs/vesc/vesc6_upennmy_foc.xml`(11-08: 제동 켬, 회생 −5 A, Ki 0.002) 은 비교용으로 남겨 둔다.

| 이름 | 값 | 의미 |
|---|---|---|
| `s_pid_kp / ki / kd` | 0.003 / 0.0002 / 0.0002 (11-08: 0.004 / 0.002 / 0.0008) | 속도 PID. 펌웨어에서 모두 1/20 을 곱한다 |
| `s_pid_kd_filter` | 0.1 | D 항 EMA |
| `sp_pid_loop_rate` | 5 → 1000 Hz | PID 주기 |
| `s_pid_ramp_erpms_s` | 25000 | 목표 erpm 램프 → **5.23 m/s²** 바퀴 가속도 상한 (명령 쪽) |
| `s_pid_min_erpm` | 500 (≈ 0.105 m/s) | 이 아래 목표면 PID 끄고 duty 0 (상 단락 제동) |
| `s_pid_allow_braking` | 1 | 감속 때 반대 전류 허용 (11-08 이전엔 0 → 감속 명령에도 관성 주행) |
| `l_current_max / min` | 60 / −60 A | 모터 전류 |
| `l_in_current_max / min` | 60 / **−5 A** | 배터리 전류. −5 A 가 고속 제동을 묶는다 |
| `foc_motor_flux_linkage`, `foc_motor_r` | 0.728 mWb, **7.6 mΩ** (11-08: 0.726, 11.1) | 토크·단락 제동 (R 이 작아져 duty 0 단락 제동이 약 1.46 배) |
| `foc_motor_l`, `foc_current_kp / ki` | 2.89 µH, 0.0058 / 15.2 (11-08: 3.36 µH, 0.0067 / 22.28) | 전류 루프 — 모델은 전류 루프를 이상적으로 봐서 쓰지 않는다 |
| `foc_openloop_rpm`, `foc_sl_openloop_time / hyst`, `foc_sl_openloop_boost_q` | 1800, 0.1 / 0.1 s, **5 A** (11-08: 1200, 0.01 / 0.01, 0) | 센서리스 개루프 출발 — 저속이 hyst 를 넘으면 그 시간 동안 개루프, 구동 전류 ≥ 5 A (출발 떨림 대책). 모델은 보조 전류만 (★ 근사) |

시뮬레이터 (`wheel_model vesc`, 직진) 에서 두 설정 비교 (2026-11-24):

| | 11-08 | 11-24 |
|---|---|---|
| 0 → 2 m/s 90 % / 초과 | 0.64 s / +0.069 m/s | 0.69 s / −0.001 |
| 2 → 0 정지 | 0.76 s | 0.78 s |
| 0.3 m/s 출발 1.5 s 뒤 / 최대 전류 | **0.481 m/s** / 60 A | **0.303 m/s** / 11.8 A |

### 펌웨어 속도 PID (bldc master `motor/foc_math.c` `foc_run_pid_control_speed`)

1 kHz 마다:

1. `set_rpm` 을 `command_rpm` 쪽으로 `25000·dt` 만큼 옮기고 `[l_min_erpm, l_max_erpm]` 로 자른다. 속도 모드가 아니었거나 모터가 멈춰 있었으면, 명령이 올 때 `set_rpm` 을 지금 rpm 에서 시작한다.
2. `error = set_rpm − rpm` (rpm 은 PLL 추정값).
3. `|set_rpm| < 500` 이면 i·이전 오차를 초기화하고 `iq = 0`. 전류 루프는 이때 **duty 0** 으로 묶는다(상 단락, 속도에 비례하는 제동).
4. `p = error·kp/20`, `d = (error − prev)·(kd/dt)/20` → EMA(0.1).
5. `out = clip(p + i + d, ±1)`. 여기서 i 는 **갱신 전** 값이다.
6. `i += error·ki·dt/20`, `clip ±1`.
7. 제동을 끈 설정이면 회전 반대 방향 출력을 0 으로 만든다.
8. `iq_set = out·60 A`.

### 전류 루프 제한 (`motor/mcpwm_foc.c`)

- 제동 전류 모드: `iq = −sign(rpm)·min(|I|, 60)`. 방향이 바뀌는 순간에는 duty 0 으로 단락한다.
- 입력 전류: `mod_q = 1.5·vq/V_bus` 로 `iq ∈ [I_in_min, I_in_max]/mod_q`. 2 m/s 에서 −5 A 한계는 약 −52 A, 4 m/s 에서는 약 −26 A(≈ −14 N, −4 m/s²)라서 **빠를수록 회생 제동이 약하다**.
- 상 전류: `[−60, 60]`.

## 2. 우리 차 모델 (MPC 와 시뮬이 같은 식)

`env/vehicle_model.py`. HyperMPC 원저자 식과 같다.

- 상태(몸체): `vx, vy, r`(무게중심). 액추에이터 상태: `ws`(바퀴 속도 m/s), `δ`(실제 조향).
- 타이어: Pacejka 합성 슬립(종·횡 함께). 슬립률 `(ws − v)/(v + ε)` 와 슬립각으로 앞·뒤 각각 `Fx, Fy` 를 낸다. 하중은 정적 배분(하중 이동 없음)이고, 마찰 `friction` 을 곱한다.
- 항력: `Cd0·sign(vx) + Cd1·vx + Cd2·vx²`.
- **입력은 `ws_ref`(바퀴 속도)와 `δ_ref`**. `ws' = (ws_ref − ws)/τ_ws`(0.4 s 실측), `δ' = (δ_ref − δ)/τ_δ`(0.13 s).
- 앞뒤 바퀴 속도가 같다(4륜, 잠긴 디퍼렌셜). 가속과 제동 모두 이 한 식이다. 바퀴 속도가 차보다 느리면 음의 슬립이 생겨 타이어가 차를 감속시킨다. 즉 **제동은 "바퀴 속도를 낮춘다"는 것 하나뿐**이다.

### 시뮬레이터의 바퀴 모델 두 가지

| | `lag` (기본, MPC 와 같다) | `vesc` (`--wheel vesc`, sim_node `wheel_model: vesc`) |
|---|---|---|
| 입력 | `/drive` speed → VESC erpm 상한으로 자름 → `ws_ref` | `/drive` speed → `AckermannToVesc`(정지 제동 패치까지) → 펌웨어 PID → iq |
| 바퀴 속도 | `ws' = (ws_ref − ws)/0.4` (+ 선택 가속도 한계 `ws_acc/ws_dec`) | `m_eq·ws' = 0.545·iq − (Fx_앞 + Fx_뒤)` |
| 가속 한계 | 없음(지연만) | 명령 램프 5.23 m/s², 모터 60 A ≈ 33 N, 타이어 마찰 |
| 제동 | 같은 1차 지연(대칭) | 빠를 때: 속도 0 → 램프로 내려가는 PID 제동(회생 −5 A 한계). 0.5 m/s 아래: 제동 전류 10 A. 목표 < 0.105 m/s: duty 0 단락 |
| 정지 유지 | 바퀴 0 → 타이어가 붙잡음 | 제동 전류 10 A(회전 반대, 0 근처는 부드럽게) |
| 적분 | RK4 1 ms | 1 ms 마다 PID → 전류 → `ws` 선형화 후진 오일러(타이어 반력이 딱딱함) → 나머지 RK4 |

`vesc` 모드에서 `ws` 를 미는 반력은, 저속 운동학 모델과 섞을 때 시뮬레이터와 같은 무게 `w` 로 `m·(ws − vx)/0.05` 를 쓴다.

## 3. f1tenth_gym

`vehicle_dynamics_st`(CommonRoad 단일 궤적)이다. 상태는 `[x, y, δ, v, ψ, ψ̇, β]`.

| 항목 | f1tenth_gym | 우리 |
|---|---|---|
| **모델 입력** | 조향 **속도** `sv` + 종 **가속도** `a` | 바퀴 속도 `ws_ref` + 조향 `δ_ref` |
| `/drive` → 입력 | `pid()`: 조향은 **뱅뱅** `sv = sign(δ_cmd − δ)·3.2 rad/s`(차이 > 1e-4). 속도는 **P 제어**: 전진 중 가속이면 `kp = 10·a_max/v_max = 4.755`, 전진 중 감속이면 `kp = 10·a_max/|v_min| = 19.02`(4배 세다), 후진 중이면 `2·a_max/…` | 조향은 서보 맵 → 1차 지연 0.13 s. 속도는 1차 지연 0.4 s(`lag`) 또는 VESC PID(`vesc`) |
| 조향 지연 | `steer_buffer_size = 2` → 명령이 **2 스텝(20 ms) 늦게** 들어감 | 기본 없음 (서보 1차 지연만). `config/lowlevel.yaml` `steer_delay` 로 넣을 수 있다 |
| 가속 제한 | `accl_constraints`: `|a| ≤ a_max = 9.51`, `v > v_switch = 7.319` 면 양의 한계가 `a_max·v_switch/v`, `v ∈ [−5, 20]` | `lag`: 없음. `vesc`: 램프·전류·회생 한계 |
| 제동 | 감속도 = P 제어 가속도, 대칭 한계 9.51 m/s². 바퀴·타이어 종방향 힘 없음 | 바퀴 속도를 낮춰 종방향 타이어 힘으로. `vesc` 는 실제 VESC 순서 그대로 |
| 종방향 | `v̇ = a`(점질량). 항력·구름저항·슬립률·바퀴 속도 상태 없음 | 종방향 슬립과 항력이 있음. 바퀴 속도가 상태 |
| 타이어 | **선형** 횡 강성 `C_Sf = 4.718`, `C_Sr = 5.4562`(1/rad, `μ·m·g` 배). 하중 이동 `h = 0.074` 를 가속도로 | Pacejka 합성 슬립(포화), 하중 이동 없음 |
| 마찰 | `mu = 1.0489` | `friction = 0.41`(동정값, 타이어 D 와 곱) |
| 저속 | `|v| < 0.5` 면 운동학 모델로 **전환**(불연속) | `|vx|` 0.2~0.4 에서 운동학과 **섞음** |
| 차 값 | `m = 3.74`, `I = 0.04712`, `lf = 0.15875`, `lr = 0.17145`, `s ∈ ±0.4189` | `m = 3.284`, `I_z = 0.1439`, `L = 0.33`, `lr = 0.1537`, 실제 조향 ≤ 0.378 |
| 적분 | `Simulator` 기본 RK4, 0.01 s 한 스텝(`RaceCar` 단독 기본은 Euler) | RK4 1 ms substep |

### 우리 시뮬레이터에서 gym 저수준 재현 (`env/lowlevel.py`, 2026-11-24)

시뮬레이터는 `/drive` 목표 → 명령 지연 → 저수준 (δ̇, ω̇) → 동역학 순서로 적분한다 (RK4 단계마다 저수준을 다시 계산).
`--lowlevel gym` 이면 위 표의 gym 쪽 (뱅뱅 3.2 rad/s, 조향 지연 20 ms, P 4.755 / 19.02, 가속 한계 9.51, v_switch 7.319) 과 같은 법칙이다.
다만 차 동역학은 우리 것 그대로라 gym 의 점질량 종방향·선형 타이어는 재현하지 않는다. 그리고 P 제어 대상이 차 속도가 아니라 바퀴 속도다.
기본 (`config/lowlevel.yaml`) 은 실차 값: 서보·바퀴 1차 지연, VESC 가속 램프 5.23 m/s², 감속 2.5 (mpcc.yaml), 속도 환산 ×1.044.

### 이 차이가 실제로 뜻하는 것

1. **gym 의 제동은 가속보다 4배 세고, 9.51 m/s² 까지 바로 낸다.** 실차는 빠를 때 회생 −5 A 한계 때문에 오히려 약하다(실측 감속 중앙값 −2.21 m/s², p10 −3.51). gym 에서 맞춘 제동 거리는 실차에서 모자란다.
2. **gym 의 조향은 2 스텝 버퍼와 일정 속도 3.2 rad/s 뱅뱅이다.** 실차 서보는 1차 지연에 가깝다(0.13 s). 조향 맵 때문에 명령 0.4 rad 가 실제 0.378 rad 가 되는 것도 gym 에는 없다.
3. **gym 은 바퀴 슬립이 없다.** `v̇ = a` 라서 가속도가 타이어 마찰에 묶이지 않는다. 우리 모델은 마찰이 종·횡을 함께 묶는다.
4. 같은 `/drive` 메시지를 받지만, gym 은 speed 를 "목표 차 속도"로, 실차는 "목표 바퀴(모터) 속도"로 쓴다. 미끄러질 때 둘이 다르다.

## 4. 실측과 비교 (`vesc` 모드 검증)

계단 응답(직진, 0 → 2 m/s → 0, `step.py` 방식):

| | 실측 (11-08 최종 VESC, `actuator_id`) | `lag` | `vesc` |
|---|---|---|---|
| 감속 중앙값 | −2.21 m/s² | −1.02 | −2.48 |
| 63 % 도달 | τ 0.40~0.45 s | 0.41 s | 0.33 s |
| 정지 | 튀지 않고 섬 | 지수적으로 0 | 제동 전류로 정지, 그 자리 유지 |

로그 재생(`data/runs/runs_20261108_vesc.tar.gz` rally 2개, 2 s 창, 실제 cmd_speed·cmd_steer 를 넣고 바퀴 속도 비교, 후진 창 제외):

| | 바퀴 속도 2 s 예측 RMSE | 평균 오차 | 모터 전류 |
|---|---|---|---|
| `lag` | 0.306 m/s | +0.040 | — |
| `vesc` | 0.349 m/s | +0.157 | 모델 평균 +1.2 A / 실측 +7.1 A, 상관 0.69, RMSE 15.7 A |

**`vesc` 가 아직 `lag` 보다 덜 맞는다.** 이유는 다음과 같다.
- 실측 평균 전류가 6 A 더 크다. 차를 일정 속도로 미는 데 드는 종방향 저항(구름·구동계 손실·코너 저항)이 동정한 `Cd0~Cd2` 보다 훨씬 크다. 그래서 모델은 같은 전류로 더 빨리 가고(+0.157 m/s), 감속도 늦다.
- rally 로그에서 일정 명령 구간의 (명령 − 바퀴)가 0.75~1.0 m/s 다. 손으로 낸 명령이라 이 값은 거칠지만, 실차가 명령 속도에 못 미치는 경우가 많다는 뜻이다. PI 로는 정상상태 오차가 0 이어야 하므로, 센서리스 속도 추정이나 다른 한계가 있을 수 있다(미확인).
- 구동계 등가 질량 `m_eq = 0.3 kg` 은 **추정**이다(회전자 ~3e-5 kg·m², 바퀴 4개). 재지 않았다.

다음 단계 후보: 일정 속도 직진 구간의 전류로 종방향 저항을 다시 동정한다. 그 뒤 `m_eq` 를 계단 응답에 맞춘다.

## 5. 모델에 없는 것

- 센서리스 관측기·PLL 의 저속 추정(`foc_sl_erpm` 2500 ≈ 0.52 m/s 아래), 개루프의 각도 오차 (보조 전류만 넣었다). 모델은 rpm 을 정확히 안다.
- 온도·전력 감액(`l_temp_*`, `l_watt_*`), d축 전류·약계자, 전류 루프 동특성, 배터리 전압 처짐(`V_bus` 11.4 V 고정).
- VESC 명령 timeout(명령이 끊기면 VESC 가 놓거나 제동). 시뮬은 명령을 계속 준다.
- 후진: 공용 모델의 슬립 `(ws − vx)/(vx + ε)` 가 vx > 0 에서만 맞다. 후진하면 `vesc` 모드에서 바퀴가 발산한다.
- `ackermann_to_vesc` 판단은 시뮬 step 마다 한다(실차는 `/drive` 가 올 때, 30 Hz). 정지 제동으로 넘어가는 순간이 최대 33 ms 다를 수 있다.
- 펌웨어는 bldc **master** 코드를 기준으로 옮겼다. 차의 펌웨어 버전이 다르면 PID 갱신 순서 등이 다를 수 있다(XML 에 `s_pid_speed_source` 가 있어 6.x 로 본다).

## 코드

- `src/mlcs_mpc/mlcs_mpc/env/drivetrain.py`: `AckermannToVesc`, `VescSpeedController`, `FocConfig`(XML), `Drivetrain`, `wheel_forces`
- `src/mlcs_mpc/mlcs_mpc/env/simulator.py`: `wheel_model='vesc'`
- `python3 src/mlcs_mpc/main.py --wheel vesc [--drive-mass 0.3]`. sim_node 는 `wheel_model: vesc`, `drive_mass`, `vesc_xml` 파라미터(`MLCS_SIM_PARAMS`). `vesc` 모드에서는 `/sensors/core` 에 `current_motor` 도 낸다.
- `test/test_drivetrain.py`
