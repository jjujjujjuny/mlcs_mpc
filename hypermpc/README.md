# hypermpc — HyperMPC 정식화를 이 차에 올리기

HyperMPC 논문([arXiv:2508.06181](https://arxiv.org/abs/2508.06181))의 정식화를
우리 차량에 적용하는 작업. 원본 코드는
[hyper-mpc/hypermpc_code](https://github.com/hyper-mpc/hypermpc_code).

## 지금 어디까지

| 단계 | 상태 |
|---|---|
| pip + casadi (venv) | ✅ casadi 3.8.1 |
| acados 소스 빌드 (aarch64) | ✅ 최소 OCP 0.659 ms |
| **오프라인 OCP** | ⏳ **코드 생성·빌드까지 확인, 풀이 미검증** |
| 트랙 변환 (track_5 → Frenet) | 대기 |
| ROS 노드 | 대기 |
| 파라미터 동정 | 대기 |
| HyperPM (torch + l4casadi) | 대기 |

## 실행

```bash
source ~/hmpc-venv/bin/activate
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib

git clone https://github.com/hyper-mpc/hypermpc_code.git ~/hypermpc_code
python3 hypermpc/offline_ocp.py --hypermpc ~/hypermpc_code
```

**첫 빌드는 10분을 넘긴다.** Pacejka 가 들어간 9상태 모델에 `EXACT` 헤시안이라
심볼릭 미분이 무겁다. 생성된 코드는 남으므로 두 번째부터는 빠르다.
(2026-09-30 시도: `.so` 까지 나왔고 10분 제한에 잘렸다 — 실패가 아니라 시간 초과)

## 왜 이 스크립트가 필요한가

`docs/mpc_formulation_car.py` 는 원본의 `mpc/mpc_formulation_car.py` **한 파일**이고,
동역학·비용·파라미터를 전부 `model` 객체에서 받아쓴다. **그 model 파일이 없다.**

그래서 여기서는 정식화를 직접 세운다. 동역학만 원본
`robot_model/car/casadi_car_model_naive.py` 를 그대로 import 한다 —
`casadi` + `numpy` 만 쓰는 유일한 경로다 (다른 모델 파일은 `torch` 와
`l4casadi` 를 import 한다).

## 상태·입력

```
x = [ s, n, mu,            Frenet 포즈 (진행, 횡편차, 헤딩편차)
      vx, vy, r, friction, 동역학  ← naive 모델
      wheel_speed, delta ] 액추에이터 (1차 지연)     → 9
u = [ wheel_speed_ref, delta_ref ]                   → 2
p = [ 차량 10 + 앞타이어 10 + 뒤타이어 10 + 곡률 1 ] → 31
```

파라미터 기본값은 원본 `single_track_params.py` / `pacejka_params.py` 에서 가져왔다.

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

→ `tools/draw_track.py` 의 `--half-width` 기본값을 0.25 로 맞췄다.
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
휠베이스는 우리와 같지만 **질량은 다르다** (우리 `vehicle.yaml` 은 3.5 로
가정 — 미측정). 저울에 올려 재고 바꿔야 예측이 맞는다.

`I_z`, `lr`, Pacejka 계수도 전부 논문 차량 값이다.

## 막힐 만한 곳

- **학습 가중치를 못 받을 수 있다.** 원본 `conf_mpc/config_car.yaml` 이
  W&B 아티팩트(`hpm_car/model:v23679`)를 가리키는데 저자들 프로젝트라
  접근이 안 될 가능성이 높다. 그러면 직접 학습해야 하고, 그것이 36분
  데이터가 필요한 이유다 (`docs/ROADMAP.md` 2단계).
- **l4casadi** 가 가장 어려운 부품이다 — libtorch C++ 에 대해 컴파일한다.
  다행히 위 표의 "HyperPM" 단계 전까지는 필요 없다.
