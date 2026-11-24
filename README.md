# mlcs_mpc — F1TENTH MPC 주행 스택 (모션캡처 위치추정)

Jetson Orin Nano / Ubuntu 22.04 (Jammy) / ROS 2 Humble 대상.
라이다 없이 **OptiTrack 모션캡처**로 위치를 잡고 MPC 로 경로를 추종한다.

목표는 [HyperMPC](https://arxiv.org/abs/2508.06181) 를 실차에 올리는 것이고,
이 저장소는 그 **1단계 — 기본 주행 환경** 이다. → [docs/ROADMAP.md](docs/ROADMAP.md)

---

## 상태 — 무엇이 검증됐고 무엇이 안 됐나

솔직하게 적는다. 이 구분이 디버깅 시간을 좌우한다.

| | 상태 |
|---|---|
| MPC 알고리즘 (iLQR) | ✅ 시뮬 검증 — Levine 트랙 90초 주행, 벽 충돌 0, 횡오차 평균 3.9cm |
| 해석적 야코비안 | ✅ 수치미분과 대조 검증 (오차 2e-8) |
| 속도 추정 칼만 필터 | ✅ 단순 차분 대비 노이즈 11.6배 감소 |
| 경로 관리 (폐곡선/곡률) | ✅ 원 궤적으로 검증 (곡률 오차 0.1%) |
| 캘리브레이션 원 피팅 | ✅ 합성 데이터 검증 (반경 오차 0.5mm) |
| 설정 파일 ↔ 노드 연결 | ✅ 전 키 자동 검증 |
| 수동/자율 조정 로직 | ✅ 우선순위 검증 — safety 정지는 LB 로 안 풀린다 |
| Stanley 컨트롤러 | ✅ 시뮬 검증 — track_5 에서 횡오차 평균 0.3cm (노이즈·지연 포함) |
| 조이스틱 실동작 | ✅ 젯슨 실차 확인 (2026-09-17) — 수동 주행 성공 |
| 젯슨 환경 구축 | ✅ 젯슨 실기 확인 — ROS 2 Humble + f1tenth_stack 빌드·구동 |
| macOS 개발 환경 | ✅ 맥북 (arm64) 확인 (2026-11-20) — acados MPC 폐루프·`./sim.sh` (ROS 2 RoboStack) 랩 완주 (`docs/SETUP_MAC.md`). RViz 창은 미확인 |
| 벡터화 차 모델 (torch) | ✅ casadi 식과 대조 — 미분식 7e-15, 1 초 적분 4e-16 (`test/test_vec_vehicle.py`) |
| VESC 저수준 모델 (`wheel_model: vesc`) | ⚠️ 구현·단위 시험만 — 실로그 바퀴 속도 RMSE 0.349 m/s 로 1차 지연 (0.306) 보다 아직 나쁨 (`docs/VEHICLE_DYNAMICS.md`) |
| RL 정책 (arXiv:2504.02420) | ❌ **실차 미검증** — 시뮬 학습만 (`docs/RL_RACING.md`) |
| **mocap 실연동** | ❌ **미검증** — 실제 NatNet 스트림으로 테스트 안 됨 |
| **실차 주행** | ❌ **미검증** |
| **VESC 캘리브레이션** | ❌ **미측정** — 값이 전부 잠정치 |
| acados 백엔드 | ❌ 미구현 (자리만 있음) |

> **개발 환경 주의:** 이 코드는 Ubuntu 20.04 / ROS 1 Noetic PC 에서 작성됐다.
> ROS 2 노드는 순수 파이썬 문법 검사와 알고리즘 단위 검증만 거쳤고,
> `rclpy` 로 실제 기동한 적은 없다. 젯슨에서 처음 돌릴 때 사소한 문제가
> 나올 수 있다 — 나오면 고치고 이 표를 갱신하세요.

---

## 빠른 시작

### 시뮬 (차 없이) — MPC 튜닝용

차 모델은 MPC 와 같은 식(`env/vehicle_model.py`)·같은 값(`config/mpcc.yaml`)이고, `env/simulator.py` 가
ODE 로 굴린다. 컨트롤러(`mpcc_node`)는 실차와 같은 토픽만 보므로 코드가 한 줄도 다르지 않다.

```bash
./sim.sh build                 # 처음 한 번 (~/mlcs_sim_ws, ROS 2 Foxy/Humble)
./sim.sh                       # race.yaml 의 track·mpcc 설정으로 (PC 에서 튜닝 → 젯슨에서 ./race.sh)
./sim.sh track s_track 0.8     # 트랙과 v_max 만
./sim.sh status                # 랩 시간·벽 충돌
```

### 실차

**순서를 지킬 것.** → [docs/SETUP_JETSON.md](docs/SETUP_JETSON.md)

```bash
# ① mocap
ros2 launch natnet_ros2 natnet_ros2.launch.py server_address:=<MotivePC> client_address:=<젯슨>

# ② VESC
ros2 launch f1tenth_stack bringup_launch.py

# ③ 거치대 위에서 조향만 확인 (바퀴가 땅에 안 닿게)
ros2 launch mlcs_mpc car_mpc.launch.py waypoints:=<경로.csv> bench:=true
ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: true}"

# ④ 캘리브레이션 → docs/CALIBRATION.md

# ⑤ 저속 실주행
ros2 launch mlcs_mpc car_mpc.launch.py waypoints:=<경로.csv> target_speed:=1.0
```

**차는 기본적으로 출발하지 않는다.** `/mpc/enabled` 를 받아야 움직인다.

```bash
ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: true}"   # 출발
ros2 topic pub --once /mpc/enabled std_msgs/msg/Bool "{data: false}"  # 정지
```

### 수동 주행 (조이스틱)

로지텍 F710 — 뒷면 **Mode 버튼 OFF**, 앞면 스위치 **X**.

`drive.sh` 가 브링업까지 한 번에 띄웁니다:

```bash
./drive.sh joy                 # 조이스틱 수동 조종
./drive.sh bench               # 거치대 위 (속도 0 고정) — 첫 확인용
MLCS_SPEED=2.0 ./drive.sh joy  # 속도 상한 조절
```

| 명령 | 하는 일 |
|---|---|
| `./drive.sh joy` | 브링업 + 조이스틱 수동 조종 |
| `./drive.sh rally [태그]` | 원선회용 수동 조종 + 기록 — 십자키로 최대 속도·조향 조절 |
| `./drive.sh bench` | 거치대 모드 — 속도 0 고정, 조향만 |
| `./drive.sh mpc <csv>` | MPC 자율주행 + 조이스틱 (LB 로 전환) |
| `./drive.sh stanley <csv>` | **Stanley 자율주행** (배관 검증용) |
| `./drive.sh record <이름>` | 조이스틱으로 몰면서 웨이포인트 기록 |
| `./drive.sh cal <모드>` | 캘리브레이션 (neutral/speed/steer) |
| `./drive.sh log [태그]` | 주행 + **HyperPM 학습 데이터 기록** |
| `./drive.sh mocapcal [spin\|straight]` | 마커 오프셋 측정 |
| `./drive.sh rviz` | **RViz 시각화** (젯슨에서 — NoMachine 으로) |
| `./drive.sh topics` | 토픽 상태 점검 |
| `./drive.sh stop` | 비상 정지 + 전부 종료 |

> `drive.sh` 가 **bringup 의 `joy_teleop` 을 죽입니다.** f1tenth_stack 의
> bringup 은 자기 `joy_teleop` 을 같이 띄우는데, 그게 **LB(버튼4)를 데드맨으로
> 쓰고 scale 5.0** 이라 우리 모드 토글과 정면충돌합니다. 직접 launch 를 쓸
> 때는 이 처리가 없으니 주의하세요.

런치를 직접 쓰려면:

```bash
ros2 launch mlcs_mpc joystick.launch.py joy:=false   # joy_node 중복 방지
```

| 조작 | 동작 |
|---|---|
| **RT** 당김 | 전진 (당긴 만큼 선형, 놓으면 즉시 정지) |
| **RB** + RT | 후진 |
| **L스틱** 좌/우 | 조향 |
| **LB** | 수동 ↔ 자율 토글 |
| **START** | 비상 정지 |

★ 기동 직후 0.4초간 트리거 휴지값을 자동으로 잽니다 — **RT 에서 손을 떼고**
계세요. 이게 틀리면 RT 를 안 눌러도 차가 나갑니다.

자율 주행과 같이 쓰려면 터미널 두 개로:

```bash
ros2 launch mlcs_mpc car_mpc.launch.py waypoints:=<경로.csv>   # A
ros2 launch mlcs_mpc joystick.launch.py joy:=false             # B
```

LB 로 오갈 수 있습니다. 수동일 때는 mux 우선순위(joystick 100 > navigation 10)로
사람 입력이 MPC 를 덮어씁니다.

### 주행 화면 보기 (RViz)

트랙 좌표가 **mocap 글로벌 좌표계** 기준이라 별도 변환 없이 그대로 그려집니다.

```bash
# ★ 젯슨에서 실행하고 NoMachine 으로 봅니다
MLCS_TRACK=/경로/deepracer_mpc_low_level/tracks/track_5 ./drive.sh rviz
```

> ⚠ **메인 PC 에서는 안 됩니다.** Ubuntu 20.04 + ROS 1 Noetic 이라 rviz2 가
> 없고, ROS 1 과 ROS 2 는 애초에 통신하지 않습니다 (TCPROS vs DDS).
> 젯슨에서 띄우고 원격 데스크톱으로 화면만 가져오는 것이 가장 간단합니다.

| 화면 요소 | 색 |
|---|---|
| 트랙 면 / 좌우 경계 | 회색 / 흰색 |
| 중심선 | 초록 |
| 차량 (실측 치수) + 앞방향 | 파랑 + 주황 화살표 |
| 실제 주행 자취 | 노랑 |
| 컨트롤러 참조 경로 | 하늘 |
| MPC 예측 궤적 | 빨강 |
| safety 경계 | 빨간 사각형 |

> `MLCS_TRACK` 을 주면 **좌우 경계와 트랙 면**까지 그립니다. 폭 0.6m 트랙에
> 폭 0.27m 차량이면 좌우 여유가 16cm 뿐이라, 면으로 봐야 이탈이 보입니다.

### 경로 만들기

```bash
# 차를 손/조이스틱으로 몰면서 기록 (Ctrl+C 로 저장)
# (조이스틱으로 몰면서 — 위 joystick.launch.py 를 같이 띄워두세요)
ros2 run mlcs_mpc waypoint_logger --ros-args -p output:=track1.csv

# 다듬기 — ★ 생략하지 마세요 (곡률이 튀면 속도가 들쭉날쭉해집니다)
ros2 run mlcs_mpc smooth_path --ros-args -p input:=track1.csv -p output:=track1_smooth.csv
```

---

## 구조

최상위에는 실행 진입점만 둔다.

```
mlcs_mpc/
├── drive.sh  race.sh  sim.sh  bat.sh  race.yaml   실행 진입점 / 레이스 설정 (실차·시뮬 공통)
├── src/mlcs_mpc/        ROS 2 패키지
│   ├── main.py  main_sim.py   ROS 없는 튜닝 루프 (MPC + ODE 시뮬, main_sim 은 실시간 + RViz)
│   ├── config/          mpcc.yaml (★ MPC·차 모델 값), lowlevel.yaml (시뮬 차의 저수준), mocap·safety·vehicle …
│   ├── launch/  waypoints/
│   └── mlcs_mpc/        파이썬 패키지 (아래)
├── learned/             학습
│   ├── hyperpm/         HyperPM: numpy 추론, export_hyper, train/ (학습)
│   └── rl/              PPO 레이싱 정책 (arXiv:2504.02420) — train, evaluate, eval_vec (DR 강건성), compare,
│                        render (영상), experiments/ (실험 묶음 스크립트)
├── tools/
│   ├── track/           트랙 만들기 — draw_track, track_from_map, track_from_lap, track_from_paper_2504_02420,
│   │                    prep_track, make_s_track, preview_track, map_mocap_align
│   ├── analysis/        주행 로그 분석·학습 데이터 — race_report, race_to_mocap_log,
│   │                    clean_drive_log, prep_hpm_dataset, actuator_id, check_kinematic_fit
│   └── hw/              차 점검 — check_state, servo_sweep
├── test/                시험 전부 (python3 test/<이름>.py)
├── data/
│   ├── maps/            Cartographer 맵 (.pbstream/.pgm/.yaml, .mocap.yaml) — 연구실 1108_*, 대회장 track1*·track2*
│   ├── runs/            주행 로그 묶음 (tar.gz)
│   └── lab_map/         실험실 경계 19각형 + 옛 DeepRacer 트랙 도구
├── docs/                설명서 (아래 "문서")
│   ├── REFERENCES.txt   참고 문헌·코드 목록      REQUIREMENTS.txt  요구사항·실차 값·환경
│   ├── vesc/            VESC 설정 XML (11-24 최신, 11-08 비교용, 앱 설정)
│   ├── references/      논문 PDF, HyperMPC 저자 정식화 원본, 다른 팀 VESC 튜닝표
│   ├── competition_tracks/  대회장 트랙 README·그림
│   └── notes/           날짜별 기록 (옛 경로 그대로 둔다)
├── third_party/         patches/ (서드파티 수정본), vesc_msgs/ (시뮬 PC 용 메시지)
└── logs/                (git 제외) sim/, tune/, rl/ — 학습·평가 결과
```

ROS 패키지 내부 (`src/mlcs_mpc/mlcs_mpc/`):

```
env/     ROS 없는 핵심 — 차·트랙·MPC·시뮬레이터·RL 환경
  config.py          MpccConfig — N, dt, 입력·트랙 경계, 비용 가중치 (값은 config/mpcc.yaml)
  vehicle_model.py   ★ 공통 차 모델 (Pacejka 단일 궤적 + 액추에이터 1차 지연). MPC·시뮬·RL 이 같은 식
  vesc.py            차의 vesc.yaml (erpm gain·상한, 서보 gain·offset·한계)
  lowlevel.py        시뮬 차의 저수준 제어기 — 명령 지연, 조향 lag|gym + 각속도 한계, 속도 lag|gym + 가속 한계
  drivetrain.py      VESC 펌웨어 경로 (정지 제동 패치 → 속도 PID 1 kHz → 전류 제한·개루프 출발 → 바퀴 힘)
  simulator.py       ODE 시뮬레이터 — /drive → VESC·서보 맵 → 저수준 → vehicle_model 을 RK4 1 ms 로
  racing.py          main.py / main_sim.py 공통 설정 (MPC + 시뮬, ROS 경로와 같은 계산)
  track/             닫힌 트랙 (Frenet, 지점별 벽, κ(s)) + data/<이름>/ 트랙 파일
  mpc/               MPCC — acados_ocp, acados_solver, bounds, core, mpcc, param_model, worker, offline (README.md)
  vec/               배치 torch 판 — vehicle (== vehicle_model), drivetrain (== drivetrain.py), track, race_env (RL 환경)
  opponent.py        상대 차 예측 → 스테이지별 통로 (추월/뒤따르기)
  path_manager.py    웨이포인트 → 참조 궤적 (옛 컨트롤러·시각화)
  legacy/            옛 iLQR 의 운동학 모델·솔버
control/ 컨트롤러 노드
  mpcc_node.py       ★ MPCC 컨트롤러 (풀이는 env/mpc/worker.py 프로세스)
  legacy/            ilqr_node (mpc_node), stanley_node — 베이스라인·배관 검증
io/      위치추정 → /mpc/state
  mocap_bridge.py    NatNet pose → 상태 추정 (칼만 필터)
  lidar_bridge.py    Cartographer pose → 같은 인터페이스
  sim_node.py        env/simulator → 같은 인터페이스 (+ /sensors/core)
  sim_bridge.py      f1tenth_gym odom → 같은 인터페이스 (옛 시뮬)
  mocap_health.py    mocap 프레임을 믿을 만한가 판정
  relocalize.py      Cartographer 시작 위치 잡기 + 스캔 매칭으로 다시 맞춤
ui/      사람 입력·화면
  joystick_teleop.py 로지텍 F710 수동 조종 (+ /joy 워치독)
  rally_limits.py    랠리 모드 십자키 조종 상한 (원선회용)
  track_viz.py       트랙·차량·궤적 RViz 시각화
utils/   안전·캘리브레이션·기록·실행 환경
  safety_node.py     경계 감시 / 비상 정지
  runenv.py          acados·ROS 2 환경 잡기 (ensure_acados / ensure_ros2), 플랫폼별 라이브러리 변수
  track_info.py      트랙 기하 + 안전 여유 점검
  calibrate.py       mocap 으로 VESC 값 측정
  mocap_calibrate.py 마커 지그 → 뒤축 중심 오프셋 측정
  data_logger.py     학습 데이터 수집 (100Hz, mocap 클럭)
  waypoint_logger.py 주행 경로 기록
  smooth_path.py     경로 평활화
  import_track.py    DeepRacer 트랙 → 웨이포인트 (+주행가능성 검사)
  sim_report.py  sim_rviz.py  sim_viz.py   시뮬 결과 요약·시각화
```

노드 이름(`ros2 run mlcs_mpc <노드>`)은 폴더와 상관없이 `setup.py` 의 entry point 가 잇는다.

### 토픽

```
/natnet_ros/car/pose  ─→ mocap_bridge ─→ /mpc/state  ─→ mpcc_node ─→ /drive
   (PoseStamped)                          (Odometry)                (Ackermann)
                                       /mocap/valid ↗           ↑
                                                      /mpc/enabled
```

시뮬에서는 `mocap_bridge`(와 VESC) 자리에 `sim_node` 가 들어가고 **나머지는 동일하다.**
컨트롤러가 시뮬/실차용으로 갈리지 않는 것이 이 설계의 요점이다.

---

## 안전

1. **하드웨어 킬스위치를 손에 들고 실험하세요.** 아래는 전부 소프트웨어이고
   노드가 죽으면 같이 죽습니다.
2. mocap 이 끊기면 차는 위치를 **전혀** 모른다 (라이다가 없다) → 즉시 정지
3. `safety_node` 가 실험 공간 경계를 감시 — `config/safety.yaml` 의 좌표를
   **실측값으로 바꿔야 한다** (기본값은 예시)
4. 첫 주행은 `bench:=true` 로 거치대 위에서
5. **조이스틱이 끊기면 속도 0 을 강제 발행한다** — VESC 는 새 명령이 없으면
   마지막 속도를 유지하므로, 발행을 멈추는 것만으로는 차가 서지 않는다
   (26-jetson 이 이 구조로 하루에 세 번 폭주를 겪었다)

---

## 튜닝

`config/vehicle.yaml` 에서. 증상별 대응:

| 증상 | 손볼 것 |
|---|---|
| 조향이 떨린다 | `w_ddelta` ↑ |
| 코너에서 경로를 벗어난다 | `w_pos` ↑, `target_speed` ↓ |
| 가감속이 급하다 | `w_a` ↑ |
| 먼 코너를 늦게 본다 | `N` ↑ (계산시간 확인) |
| 계산이 느리다 | `N` ↓ 또는 `max_iter` ↓ |

계산 시간은 실시간으로 볼 수 있다:

```bash
ros2 topic echo /mpc/solve_time      # 50ms(20Hz) 안에 들어와야 한다
```

> `max_iter` 는 3 이 기본이다. warm start 덕분에 8 로 올려도 결과가 같고
> 시간만 2.5배 든다 (Levine 한 바퀴로 측정).

---

## 참고한 것

전체 목록 (논문·코드·어디에 썼나) 은 [docs/REFERENCES.txt](docs/REFERENCES.txt).

- [HyperMPC](https://github.com/hyper-mpc/hypermpc_code) (arXiv:2508.06181) — 최종 목표, OCP 정식화의 출발점
- arXiv:2504.02420 (IROS 2025) — RL 레이싱 환경 (`learned/rl`, `env/vec`)
- [f1tenth_system](https://github.com/f1tenth/f1tenth_system) — VESC/센서 배관
- [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) / [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) — 시뮬레이터 (저수준 구조 비교)
- `26-jetson-f1tenth-car1-car2merge` — 같은 연구실 pure-pursuit 스택.
  캘리브레이션 실패 사례, DDS 함정, 안전장치 설계가 이 저장소 곳곳에 반영돼 있다.

---

## 문서

- [docs/SETUP_JETSON.md](docs/SETUP_JETSON.md) — 젯슨 환경 구축 (여기부터)
- [docs/MOCAP_SETUP.md](docs/MOCAP_SETUP.md) — 모션캡처 연동 (RigidBody → 토픽)
- [docs/CALIBRATION.md](docs/CALIBRATION.md) — VESC 캘리브레이션
- [data/lab_map/](data/lab_map/) — 실험실 경계(19각형) + 옛 트랙 생성 도구 (메인 PC 에서 가져옴)
- [docs/TRACKS.md](docs/TRACKS.md) — 트랙 출처와 **주행 가능성** (★ 주행 전 확인)
- [docs/ROADMAP.md](docs/ROADMAP.md) — HyperMPC 까지 가는 길 (논문 분석 포함)
- [docs/REQUIREMENTS.txt](docs/REQUIREMENTS.txt) — 요구사항·실차 파라미터 반영 현황·소프트웨어 환경
- [docs/REFERENCES.txt](docs/REFERENCES.txt) — 참고 문헌·코드
- [docs/VEHICLE_DYNAMICS.md](docs/VEHICLE_DYNAMICS.md) — 차 모델·저수준 (VESC, f1tenth_gym 비교, 실측)
- [docs/RL_RACING.md](docs/RL_RACING.md) — 강화학습 (VESC 저수준, 도메인 랜덤화, 결과)
- [docs/SETUP_MAC.md](docs/SETUP_MAC.md) — 맥북 환경
- [docs/competition_tracks/](docs/competition_tracks/) — 대회장 트랙 (MAGP 2026)

> ★ **데이터는 지금부터 모으세요.** HyperPM 학습에 36분이 필요한데
> (논문 기준), 나중에 몰아서 모으기 어렵습니다. 차를 몰 때마다
> `./drive.sh log` 를 쓰면 됩니다.
