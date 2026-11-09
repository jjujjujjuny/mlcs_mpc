# env/mpc — MPCC (Frenet 컨투어링 MPC)

[HyperMPC](https://arxiv.org/abs/2508.06181) 논문의 정식화(비용·동역학)를 출발점으로 한 MPC.
학습 모델(HyperPM 등)은 여기 들어 있지 않다 — 스테이지별 동역학 파라미터를 주는 쪽으로
`learned/` 에서 붙는다.

## 파일

| 파일 | 하는 일 |
|---|---|
| `acados_ocp.py` | OCP 정의 — 상태·입력·비용·경계. 곡률 κ(s) 는 트랙을 B-스플라인으로 넣어 OCP 가 상태 s 로 찾는다 |
| `acados_solver.py` | 솔버 옵션 + 생성 코드 빌드/재사용 (`~/mpcc_build/<태그>_trk<트랙해시>/`) |
| `bounds.py` | 실행 중 스테이지별 경계 (벽, 입력 상한), 초기 추측 |
| `core.py` | ROS 없는 노드 계산 — `/mpc/state` → OCP x, 해 검사, 실제 조향 ↔ 서보 명령 |
| `worker.py` | 풀이만 하는 프로세스 (`control/mpcc_node.py` 가 띄우고 감시한다) |
| `offline.py` | 차 없이 풀어보기 (직선·곡선·트랙, `--plant` 폐루프, 가상 상대) |

설정은 전부 `config/mpcc.yaml` 에서 온다 (`env/config.py` 의 `MpccConfig` 가 이름·기본값).
차 모델은 `env/vehicle_model.py` — 시뮬레이터(`env/simulator.py`)와 같은 식이다.

## 실행

```bash
source ~/hmpc-venv/bin/activate          # casadi + acados_template
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib

python3 src/mlcs_mpc/mlcs_mpc/env/mpc/offline.py                      # mpcc.yaml 값으로 직선·곡선·트랙
python3 src/mlcs_mpc/mlcs_mpc/env/mpc/offline.py --scenario track --plant \
    --track data/tracks/comp1_track/prep_comp1_track.csv --v-max 1.0   # 같은 모델로 폐루프
```

트랙마다 첫 빌드가 있다 (이 PC 약 4초, 젯슨 약 20초). 원저자 저장소(`~/hypermpc_code`)는 필요 없다.

## 상태·입력·파라미터

```
x = [ s, n, mu,            Frenet 포즈 (진행, 횡편차, 헤딩편차)
      vx, vy, r, friction, 동역학 (env/vehicle_model.py)
      wheel_speed, delta ] 액추에이터 (1차 지연)     → 9
u = [ wheel_speed_ref, delta_ref ]                   → 2
p = [ 차량 10 + 앞타이어 10 + 뒤타이어 10 ]          → 30, 매 풀이마다 스테이지 k = 0..N 별로 넣는다
κ(s)                                                  → 생성 코드 안 (트랙 B-스플라인)
```

## 트랙 폭 — 0.25 는 반폭이다 (2026-10-01 확인)

원저장소를 받아 확인했다. `conf_mpc/config_car.yaml` 의 `track_width: 0.25` 가
`robot_model/car/casadi_car_model.py:106` 에서 이렇게 쓰인다:

```python
h_left  = cfg.track_width - n        #  n ≤ +0.25
h_right = cfg.track_width + n        #  n ≥ -0.25
track_soft_constraints_cost = soft_constraint(h_left) + soft_constraint(h_right)
```

**`/2` 가 없으므로 반폭**이다 — 전폭 0.50 m. 같은 저장소의
`casadi_car_model_drift_parking.py:107` 은 `width_s(s)/2 - n` 으로 전폭을
받으므로 이름이 파일마다 다른 뜻이다. 차량 모델 쪽이 반폭이고, 그것이
논문 결과를 낸 설정이다.

### ⚠ 하드 제약이 아니다

```python
def soft_constraint(h, lambda_=cfg.soft_constraint_lambda):   # 200.0
    return cs.log(1 + cs.exp(-lambda_ * h))
```

비용에 더하는 배리어이고, `mpc_formulation_car.py` 의
`ocp.constraints.lbx` 는 **비어 있다** — 상태에 하드 상한이 아예 없다.
즉 0.25 는 "벽" 이 아니라 "중심선에서 이만큼 벗어나면 비용이 물린다" 는
튜닝값이다. 저자들 트랙 중 `lab_curvy_v1` 은 반폭 0.125 로 **차폭
0.27 m 보다도 좁다** — 벽으로 읽으면 말이 안 되는 값이다.

### 저자들 트랙 (`mpc/tracks/*.csv`, `x_m,y_m,w_tr_right_m,w_tr_left_m`)

| 트랙 | 반폭 | 전폭 | 크기 |
|---|---|---|---|
| `lab_curvy_v1` | 0.125 | 0.25 m | 3.73 × 6.19 m |
| `lab_monza` | 0.230~0.510 | 0.46~1.02 m | 4.20 × 5.76 m |
| `lab_usa_v3` | 0.550 | 1.10 m | 2.59 × 5.61 m |

CSV 의 폭 열은 차량 모델이 **읽지 않는다** — 스칼라 `cfg.track_width` 만
쓴다. 폭 열을 쓰는 것은 drift-parking 모델뿐이다.

실험실 크기가 우리와 비슷하다 (우리 19각형은 5.78 × 6.74 m).

→ `tools/track/draw_track.py` 의 `--half-width` 기본값을 0.25 로 맞췄다.
  `track_5` 는 0.30 으로 만들어져 있어 다르다.

## 트랙 파이프라인 — 저자들과 같은 2단계

```
<이름>.csv        x_m,y_m,w_tr_right_m,w_tr_left_m      ← 원본 (한쪽씩 반폭)
   ↓  mpc/tracks/track_preprocesor.py : TrackReader
prep_<이름>.csv   s,x,y,heading,curvature,track_width   ← MPC 가 읽는 것
   ↓  mpc/tracks/map_reader.py : getTrackCustom
```

우리 쪽 도구가 두 단계를 그대로 따른다. **곡률·heading·s 를 우리가 다시
계산하지 않는다** — 저자 코드를 import 해서 돌린다.

```bash
./drive.sh drawtrack mytrack     # 손클릭 → 원본 CSV (1 cm 간격)
./drive.sh preptrack mytrack     # 저자 TrackReader → prep_mytrack.csv
./drive.sh preptrack mytrack --install    # 원저장소 mpc/tracks/ 에도 복사
```

`drawtrack` 은 그리는 동안 **저자 전처리 결과를 같이 띄운다** (주황 파선).
두 스플라인이 다르기 때문이다 — 우리 것은 `k=3`, 5 cm 이고 저자 것은
`k=5, s=2.0`, 20 cm 다. 곡률이 **양방향으로** 달라진다:

| 트랙 | 우리 R_min | 저자 R_min | |
|---|---|---|---|
| 넉넉 | 1.074 m | 1.274 m | 전처리가 완만하게 |
| 보통 | 0.684 m | 0.921 m | 우리만 불합격 |
| 급함 | 0.406 m | 0.510 m | 둘 다 불합격 |

한쪽만 보면 "그려서 ✓ → `preptrack` 에서 ✗" 가 난다. 그래서 저장을
**둘 다** 통과해야 되게 막았다. 미리보기는 `preptrack` 과 같은 값을 낸다
(검증: R_min 1.274 일치). 비용은 1 cm 1000점에 12 ms 라 클릭마다 돌린다.

### 저자 설정 (`track_preprocesor.py` 그대로)

| | 값 |
|---|---|
| 스플라인 | `splprep(k=5, s=2.0, w=1/전폭, per=True)` |
| 재샘플 | `points_per_meter = 5` → 20 cm 간격 |
| 폭 평활 | `savgol_filter(w_r, 10, 3)` — 오른쪽만 |
| 전폭 | `track_width = w_r + w_l` |
| 폭 보정 | `(전폭/2 − e)·2` — 스플라인 오차만큼 좁힌다 |

### ★ 걸리는 것 세 가지

**① 진행 방향이 뒤집힌다.** `TrackReader.__init__` 이 `reverse=False`
(기본값) 일 때 `np.flip(data, axis=0)` 을 한다. CCW 로 그린 트랙이 CW 가
되고 곡률 부호가 반대로 나온다. `--reverse` 로 끈다 (실측: ∮κ ds
−6.3077 → +6.3077).

**② `s=2.0` 은 점 개수에 걸린다.** scipy 의 `s` 는 가중 잔차 제곱합이라
점이 적으면 점당 허용 오차가 커진다. 저자들 원본은 **1.1 cm 간격**이다:

| 트랙 | 점 | 길이 | 간격 |
|---|---|---|---|
| `lab_curvy_v1` | 1703 | 19.25 m | 1.13 cm |
| `lab_monza` | 1662 | 18.68 m | 1.12 cm |
| `lab_usa_v3` | 1367 | 14.80 m | 1.08 cm |

우리 5 cm 를 그냥 넣으면 과평활해진다 (track_5 로 실측):

| 원본 간격 | RMSE | R_min |
|---|---|---|
| 5.00 cm (220점) | 0.048 m | 0.966 m |
| 2.00 cm (548점) | 0.030 m | 0.989 m |
| 1.00 cm | 0.022 m | — |

그래서 `draw_track.py` 가 주행용은 5 cm, **HyperMPC 원본은 1 cm** 로
따로 깐다 (`--raw-spacing`).

**③ `import scipy` 만 하고 `scipy.signal` 을 쓴다.** 저자 환경에서는 다른
import 가 먼저 올려 줘서 가려졌던 문제다. `prep_track.py` 가 서브모듈을
먼저 올린다 — 저자 코드는 손대지 않는다.

## ★ 저자 전처리는 설계 R_min 의 0.72~0.85 만 남긴다

트랙을 새로 만들 때 가장 중요한 숫자다. 저자 `TrackReader` 의 `s=2.0` 은
가중 잔차 제곱합이고 가중치가 `1/전폭 = 2` 이므로, 허용 RMS 편차가
약 **2 cm** 다. 그 편차를 실제로 다 쓰면서 곡선에 기복이 생기고, 그것이
**코너를 더 조인다.**

S자 트랙(클로소이드 포함, 1 cm 원본)으로 실측했다:

| 설계 R_min | 전처리 후 | 비 | RMSE |
|---|---|---|---|
| 0.90 m | 0.652 m | 0.72 | 0.0195 |
| 1.00 m | 0.761 m | 0.76 | 0.0188 |
| 1.10 m | 0.868 m | 0.79 | 0.0181 |
| 1.30 m | 1.077 m | 0.83 | 0.0170 |
| 1.45 m | 1.225 m | 0.85 | 0.0163 |

**우리 차 한계가 0.781 m 이므로 설계 R_min 은 1.05 m 이상이어야 한다.**
0.90 으로 설계하면 전처리 후 0.65 로 떨어져 못 돈다 — 그린 쪽에서는
"✓ 통과" 로 보이는데 MPC 가 받는 트랙은 불가다. `drawtrack` 이 전처리
결과를 같이 띄우고 `make_s_track` 이 그걸 제약으로 거는 이유다.

클로소이드(곡률 1차 램프)를 넣어도 이 축소는 사라지지 않는다. 전이 길이를
0.2 → 0.8 m 로 늘려 봐도 전처리 후 R_min 은 0.86 → 0.92 로 조금만 좋아진다.

## 지금 MPC 와의 차이

|  | 현재 `mpc_node` | 이 정식화 |
|---|---|---|
| 상태 | 운동학 자전거 | 9개 (동역학 + Pacejka) |
| 경로 | 참조 궤적 추종 | Frenet 컨투어링 |
| 예측 | N=20, dt=0.05 → 1.0 s | N=80, dt=0.033 → 2.64 s |
| 솔버 | iLQR (numpy) | acados SQP_RTI + HPIPM |
| 트랙 | x,y 중심선 | Frenet + 경계 폭 |

제어 아키텍처 전환이지 솔버 교체가 아니다.

## ⚠ 아직 논문 차량 값이다

`casadi_car_model_naive.py` 에 `m = 5.1`, `L = 0.33` 이 하드코딩돼 있다.
휠베이스는 우리와 같지만 **질량은 다르다** — 우리 차는 2026-11-05 저울
실측(라이다 포함)으로 **3.284 kg**, 논문 차량의 64% 다. 타이어 하중에 그대로 들어가므로
동역학 모델로 갈 때 바꿔야 한다.

    mass  3.284   앞축 1529 g / 뒤축 1753 g
    l_f   0.1763  l_r 0.1537   (합 = 0.3300 = L ✓)
    무게중심은 뒤쪽 — 앞 46.6% : 뒤 53.4%

`I_z` 와 `lr` 은 우리 차 값으로 바꿨다 (`offline_ocp.py`):

    I_z  0.46  → 0.089   추정. 논문 값은 회전반경 0.300 m 로 물리적
                         최대(0.253)를 넘는다 — 측정치가 아니라 학습
                         파라미터 초기값이다.
    lr   0.115 → 0.1537  실측

Pacejka 계수는 아직 논문 차량 값이다.

## 막힐 만한 곳

- **학습 가중치를 못 받을 수 있다.** 원본 `conf_mpc/config_car.yaml` 이
  W&B 아티팩트(`hpm_car/model:v23679`)를 가리키는데 저자들 프로젝트라
  접근이 안 될 가능성이 높다. 그러면 직접 학습해야 하고, 그것이 36분
  데이터가 필요한 이유다 (`docs/ROADMAP.md` 2단계).
- **l4casadi** 가 가장 어려운 부품이다 — libtorch C++ 에 대해 컴파일한다.
  다행히 위 표의 "HyperPM" 단계 전까지는 필요 없다.
