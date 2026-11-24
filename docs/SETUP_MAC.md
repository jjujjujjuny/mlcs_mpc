# macOS (Apple Silicon) 개발 환경

맥북에서 acados MPC·ROS 2 시뮬·RL 학습을 돌린다. 실차(젯슨)와 같은 코드·같은 `race.yaml`·`config/mpcc.yaml` 이다.
2026-11-20 macOS 15.7 (arm64) 에서 확인.

| 무엇 | 어떻게 |
|---|---|
| ROS 2 Humble | RoboStack conda 환경 `mlcs_ros` (`/opt/ros` 없음) |
| acados | `~/acados` v0.6.1 소스 빌드 (`libacados.dylib`), `acados_template` 은 conda 환경에 editable 설치 |
| 파이썬 | conda 환경의 python 3.12 — ★ 시스템 `/usr/bin/python3` 는 쓰지 않는다 (SIP 가 `DYLD_LIBRARY_PATH` 를 지운다) |
| numpy | conda 환경은 **2.x** (RoboStack 패키지가 numpy 2 로 빌드돼 있다). 1.26 고정은 우분투·젯슨 이야기다 |
| torch | conda-forge pytorch (RL 학습, CPU·MPS) |

## 설치 (처음 한 번)

```bash
brew install --cask miniforge
mamba create -n mlcs_ros -c conda-forge -c robostack-humble \
    ros-humble-desktop ros-humble-ackermann-msgs colcon-common-extensions compilers cmake pkg-config make ninja \
    casadi scipy matplotlib pyyaml "numpy>=2" "setuptools<80" pip pytorch pymupdf
conda activate mlcs_ros

git clone --recursive https://github.com/acados/acados.git ~/acados
cd ~/acados && git checkout v0.6.1 && git submodule update --init --recursive
mkdir -p build && cd build && cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_DAQP=ON .. && make install -j8
python -m pip install -e ~/acados/interfaces/acados_template
# tera_renderer (osx-arm64) 는 첫 OCP 생성 때 ~/acados/bin 에 자동으로 받는다
```

- `setuptools<80` — 80 이상이면 `colcon build --symlink-install` 이 `option --editable not recognized` 로 실패한다.
- `numpy>=2` — 1.26 이면 colcon 이 `numpy/_core/include` 를 못 찾는다 (RoboStack 메시지 패키지가 numpy 2 경로를 가리킨다).
- `cartographer_ros_msgs` 는 RoboStack 에 없다 — 라이다 재위치 (`relocalize`) 만 쓰므로 시뮬에는 상관없다.

## 실행

```bash
conda activate mlcs_ros            # sim.sh 는 /opt/ros 가 없으면 이 환경을 스스로 켠다 (MLCS_CONDA_ENV)
python src/mlcs_mpc/main.py --laps 2           # ROS 없는 MPC 폐루프 (acados)
./sim.sh build && ./sim.sh                     # sim_node + mpcc_node + 워커 + RViz
python test/test_stage_params.py               # acados 빌드·풀이 확인
python learned/rl/train.py                     # RL (docs/RL_RACING.md)
```

## 맥에서 고친 것 (리눅스 동작은 그대로)

| 곳 | 문제 | 고침 |
|---|---|---|
| `utils/runenv.py`, `control/mpcc_node.py`, `sim.sh` | acados 라이브러리 경로를 `LD_LIBRARY_PATH` 로만 줬다 → `libqpOASES_e.dylib` 를 못 찾음 | macOS 는 `DYLD_LIBRARY_PATH` (`runenv.LIB_PATH_VAR`) |
| `env/mpc/acados_solver.py` | 빌드 캐시가 `*.so` 만 찾음 → 매번 다시 빌드 | macOS 는 `*.dylib` (`LIB_EXT`) |
| `env/mpc/worker.py` | `os.sched_setaffinity` 가 macOS 에 없어 워커가 죽음 | 있을 때만 |
| `sim.sh` | `/opt/ros` 만 찾음, `timeout` 없음, 설정 읽는 `python3` 가 시스템 것 | conda 환경 자동 활성화, `timeout`/`gtimeout` 없으면 생략, ROS 환경을 먼저 켠다 |

## 확인한 것 (2026-11-20)

- `main.py` comp1_track v_max 1.0: 2 바퀴 벽 0, 풀이 평균 3.5 ms (젯슨 p95 25 ms)
- `./sim.sh` (RViz 없이): sim_node + mpcc_node + acados 워커, 랩 11.87 s 벽 0
- 테스트: vehicle_model 4, simulator 4, mpcc_core 13, opponent 18, ocp_kappa 2, stage_params 1, algorithms 29, hyperpm, vec_vehicle 5 통과.
  `test_map_tools`·`test_draw_track` 은 창을 띄우는 도구라 DISPLAY 검사에서 멈추고, `test_clean_drive_log` 는 pandas 가 필요하다 (`mamba install pandas`).
- RViz 창은 띄워 보지 않았다.
